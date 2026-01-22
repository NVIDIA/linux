// SPDX-License-Identifier: GPL-2.0
/*
 * Remote I2C FRU Bridge Target (Client side)
 *
 * Registers as an I2C slave/target on a link I2C bus at a fixed address.
 * Receives Remote I2C frames, executes i2c_transfer() on a downstream adapter,
 * and returns a framed response when the Host reads from this slave address.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>
#include <linux/crc16.h>
#include <linux/printk.h>
#include <linux/delay.h>
#include <linux/minmax.h>

#define DRV_NAME "virtual-mux-client"

/* Protocol constants (must match host) */
#define RI_MAGIC_REQ  cpu_to_le16(0x4952) /* 'R''I' */
#define RI_MAGIC_RESP cpu_to_le16(0x4F52) /* 'R''O' */
/* Protocol version matched with host:
 * v3 removes host-selected bus_id; client enforces mapping from DT
 */
#define RI_VER        0x03

#define RI_MSG_SUBMIT_XFER 0x01
#define RI_MSG_XFER_RESULT 0x81
#define RI_MSG_NOT_READY   0x82

#define RI_FLAG_CRC16      BIT(0)

#define RI_MAX_MSGS        4
#define RI_MAX_REQ_BYTES   2048
#define RI_MAX_RESP_BYTES  4096
#define RI_MSG_DESC_LEN    4 /* addr(1) + flags(1) + len(2) */
#define RI_DOWNSTREAM_RETRIES 5
#define RI_SLEEP_MS        5

struct __packed ri_hdr {
	__le16 magic;
	u8     version;
	u8     msg_type;
	u8     seq;
	u8     header_len;
	__le16 total_len;
	__le16 flags;
	__le16 reserved;
};

struct bridge_target {
	struct device *dev;
	struct i2c_client *slave;          /* link bus slave client */

	/* RX accumulation */
	u8  *rx_buf;
	u16  rx_len;
	/* (rx accumulation size hint removed; we parse in worker) */

	/* Response */
	u8  *tx_buf;                        /* primary tx buffer */
	u8  *tx_active;                     /* pointer to buffer currently served (tx_buf) */
	u16  tx_len;                        /* length of active buffer */
	bool tx_ready;
	u16  tx_idx;
	/* Static NOT_READY frame */
	u8  *tx_not_ready;
	u16  tx_not_ready_len;

	/* Workspaces to avoid per-transfer allocations */
	u8  *payload_ws;                    /* workspace for building read payloads */
	u8  *read_ws;                       /* workspace backing READ i2c_msg buffers */

	/* Current READ serving pointer (decoupled from tx_active when serving NOT_READY) */
	const u8 *rd_ptr;
	u16  rd_len;
	u16  rd_idx;
	bool rd_is_result;

	/* State */
	bool use_crc;

	/* Async execution */
	struct work_struct work;

	spinlock_t lock;                    /* protects tx/rx indices/flags */

	/* Virtual address map (whitelist) built from DT children */
	bool vmap_valid[128];
	u8   vmap_down_addr[128];
	u32  vmap_down_bus[128];
};

static u16 ri_crc16(const void *buf, size_t len)
{
	return crc16(0, buf, len);
}


static s16 ri_exec_downstream(struct bridge_target *b,
			      struct i2c_msg *msgs, int nmsgs,
			      u32 exec_bus)
{
	struct i2c_adapter *exec_adap = NULL;
	int ret, i, tries;

	if (exec_bus == (u32)-1)
		return -ENODEV;

	exec_adap = i2c_get_adapter(exec_bus);
	if (!exec_adap) {
		dev_dbg(b->dev, "no adapter for bus %u\n", exec_bus);
		return -ENODEV;
	}

	/* Submit downstream request */
	dev_dbg(b->dev, "downstream submit: bus=%u nmsgs=%u\n", exec_bus, nmsgs);
	for (i = 0; i < nmsgs; i++) {
		dev_dbg(b->dev, "  dmsg[%d]: %s addr=0x%02x len=%u\n", i,
			(msgs[i].flags & I2C_M_RD) ? "RD" : "WR",
			msgs[i].addr, msgs[i].len);
	}

	/* Rely on i2c_transfer() for proper bus locking; do not double-lock */
	for (tries = 0; tries < RI_DOWNSTREAM_RETRIES; tries++) {
		ret = i2c_transfer(exec_adap, msgs, nmsgs);
		if (ret != -EBUSY)
			break;
		/* Allow bus to become idle (covers Tbuf) */
		msleep(RI_SLEEP_MS);
	}
	if (ret == -EBUSY)
		dev_dbg(b->dev, "downstream busy after retries (bus=%u)\n", exec_bus);

	i2c_put_adapter(exec_adap);

	if (ret < 0) {
		dev_dbg(b->dev, "xfer error: %d\n", ret);
		return (s16)ret;
	}
	if (ret != nmsgs) {
		dev_dbg(b->dev, "short xfer: %d/%u\n", ret, nmsgs);
		return -EIO;
	}

	dev_dbg(b->dev, "downstream complete: ret=%d\n", ret);
	return 0;
}

static void ri_build_static_not_ready(struct bridge_target *b)
{
	struct ri_hdr *hdr;
	u8 *buf;
	size_t off = 0;

	buf = b->tx_not_ready;
	memset(buf, 0, RI_MAX_RESP_BYTES);

	hdr = (struct ri_hdr *)buf;
	hdr->magic = RI_MAGIC_RESP;
	hdr->version = RI_VER;
	hdr->msg_type = RI_MSG_NOT_READY;
	hdr->seq = 0; /* don't care for NOT_READY */
	hdr->header_len = sizeof(*hdr);
	hdr->reserved = 0;

	/* Always header-only for NOT_READY; no CRC, no payload */
	hdr->flags = cpu_to_le16(0);

	off = sizeof(*hdr);
	hdr->total_len = cpu_to_le16(off);

	b->tx_not_ready_len = off;
}

static s16 ri_parse_submit_and_execute(struct bridge_target *b,
				       const u8 *buf, size_t len,
				       u8 *out_seq,
				       u8 *read_payloads, size_t *read_payloads_len)
{
	const struct ri_hdr *hdr;
	u16 total, flags;
	size_t off;
	u8 nmsgs;
	int i, nread = 0;
	struct i2c_msg msgs[RI_MAX_MSGS];
	size_t read_ws_off = 0;
	int ret;
	u32 exec_bus = (u32)-1;
	u8  mapped_addr = 0;

	*read_payloads_len = 0;

	if (len < sizeof(*hdr))
		return -EMSGSIZE;

	hdr = (const struct ri_hdr *)buf;
	if (hdr->magic != RI_MAGIC_REQ || hdr->version != RI_VER || hdr->msg_type != RI_MSG_SUBMIT_XFER)
		return -EPROTO;

	total = le16_to_cpu(hdr->total_len);
	flags = le16_to_cpu(hdr->flags);

	if (total > len || total < sizeof(*hdr))
		return -EMSGSIZE;

	if (flags & RI_FLAG_CRC16) {
		u16 crc_frame, crc_calc;

		if (total < sizeof(*hdr) + 2)
			return -EMSGSIZE;
		crc_frame = le16_to_cpu(*(__le16 *)&buf[total - 2]);
		crc_calc = ri_crc16(buf, total - 2);
		if (crc_frame != crc_calc) {
			dev_dbg(b->dev, "crc mismatch: frame=0x%04x calc=0x%04x total=%u\n",
				crc_frame, crc_calc, total);
			return -EBADMSG;
		}
	}

	*out_seq = hdr->seq;
	dev_dbg(b->dev, "rx submit: seq=%u total=%u flags=0x%x ver=0x%02x\n",
		hdr->seq, total, flags, hdr->version);

	off = sizeof(*hdr);
	if (off + 1 + 1 + 2 + 4 > total)
		return -EMSGSIZE;

	nmsgs = buf[off++];
	off++;      /* retry_hint */
	off += 2;   /* timeout hint */
	off += 4;   /* client_cookie */

	if (nmsgs == 0 || nmsgs > RI_MAX_MSGS)
		return -EINVAL;

	memset(msgs, 0, sizeof(msgs));

	for (i = 0; i < nmsgs; i++) {
		u8 addr, mflags;
		u16 mlen;
		bool is_read;

		if (off + RI_MSG_DESC_LEN > total)
			return -EMSGSIZE;

		addr = buf[off++];
		mflags = buf[off++];
		mlen = le16_to_cpu(*(__le16 *)&buf[off]); off += 2;

		is_read = !!(mflags & 0x01);

		/* Map virtual address -> downstream bus/addr */
		if (addr >= 0x80 || !b->vmap_valid[addr]) {
			/* not in whitelist */
			dev_dbg(b->dev, "deny: virt=0x%02x not in whitelist\n", addr);
			return -EPERM;
		}
		mapped_addr = b->vmap_down_addr[addr] & 0x7f;
		if (exec_bus == (u32)-1) {
			exec_bus = b->vmap_down_bus[addr];
		} else if (exec_bus != b->vmap_down_bus[addr]) {
			/* mixed target buses in single transfer */
			dev_dbg(b->dev, "deny: mixed buses virt=0x%02x bus=%u!=%u\n",
				addr, b->vmap_down_bus[addr], exec_bus);
			return -EXDEV;
		}

		msgs[i].addr = mapped_addr;
		msgs[i].len = mlen;
		dev_dbg(b->dev, "msg[%d]: %s virt=0x%02x -> bus=%u addr=0x%02x len=%u\n",
			i, is_read ? "RD" : "WR", addr, exec_bus, mapped_addr, mlen);

		if (is_read) {
			msgs[i].flags = I2C_M_RD;

			/* Slice from preallocated read workspace */
			if (read_ws_off + mlen > RI_MAX_RESP_BYTES)
				return -EMSGSIZE;

			msgs[i].buf = &b->read_ws[read_ws_off];
			read_ws_off += mlen;
			nread++;
		} else {
			msgs[i].flags = 0;
			if (off + mlen > total)
				return -EMSGSIZE;
			/* Point directly into request buffer (safe during this function) */
			msgs[i].buf = (u8 *)&buf[off];
			off += mlen;
		}
	}

	/* Execute downstream transfer (adapter selected by whitelist mapping) */
	ret = ri_exec_downstream(b, msgs, nmsgs, exec_bus);
	if (ret)
		return (s16)ret;

	/* Build read payloads: for each READ msg in order: u16 len + data */
	{
		size_t woff = 0;

		for (i = 0; i < nmsgs; i++) {
			if (!(msgs[i].flags & I2C_M_RD))
				continue;

			if (woff + 2 + msgs[i].len > RI_MAX_RESP_BYTES)
				return -EMSGSIZE;

			*(__le16 *)&read_payloads[woff] = cpu_to_le16(msgs[i].len);
			woff += 2;

			memcpy(&read_payloads[woff], msgs[i].buf, msgs[i].len);
			woff += msgs[i].len;
		}

		*read_payloads_len = woff;
	}

	dev_dbg(b->dev, "xfer ok: seq=%u nmsgs=%u bus=%u\n", *out_seq, nmsgs, exec_bus);

	return 0;
}

static void bridge_work_fn(struct work_struct *work)
{
	struct bridge_target *b = container_of(work, struct bridge_target, work);
	u8 seq = 0;
	s16 status = -EIO;
	unsigned long irqflags;
	u8 *payload;
	size_t payload_len = 0;
	u8 nread_msgs = 0;

	payload = b->payload_ws;

	/* Parse and execute using the captured rx_buf */
	/* No deferred READ logging; worker runs only for SubmitXfer processing */

	status = ri_parse_submit_and_execute(b, b->rx_buf, b->rx_len,
					     &seq, payload, &payload_len);

	/* Count how many read payload blocks exist (u16 len + data...) */
	if (status == 0) {
		size_t off = 0;

		while (off + 2 <= payload_len) {
			u16 l = le16_to_cpu(*(__le16 *)&payload[off]);

			off += 2;
			if (off + l > payload_len)
				break;
			off += l;
			nread_msgs++;
		}
	}

	/* Build full response frame */
	spin_lock_irqsave(&b->lock, irqflags);
	{
		struct ri_hdr *hdr;
		u8 *buf;
		size_t off = 0;
		u16 flags = 0;

		/* Single-buffer build */
		buf = b->tx_buf;
		memset(buf, 0, RI_MAX_RESP_BYTES);

		hdr = (struct ri_hdr *)buf;
		hdr->magic = RI_MAGIC_RESP;
		hdr->version = RI_VER;
		hdr->msg_type = RI_MSG_XFER_RESULT;
		hdr->seq = seq;
		hdr->header_len = sizeof(*hdr);
		hdr->reserved = 0;

		if (b->use_crc) {
			flags |= RI_FLAG_CRC16;
			hdr->flags = cpu_to_le16(flags);
		}

		off = sizeof(*hdr);

		*(__le16 *)&buf[off] = cpu_to_le16((u16)status); off += 2;
		buf[off++] = nread_msgs;
		buf[off++] = 0;
		*(__le16 *)&buf[off] = cpu_to_le16(0); off += 2;
		*(__le16 *)&buf[off] = cpu_to_le16(0); off += 2;

		if (status == 0) {
			if (off + payload_len > RI_MAX_RESP_BYTES) {
				/* override with EMSGSIZE */
				off = sizeof(*hdr);
				*(__le16 *)&buf[off] = cpu_to_le16((u16)-EMSGSIZE); off += 2;
				buf[off++] = 0; buf[off++] = 0;
				*(__le16 *)&buf[off] = cpu_to_le16(0); off += 2;
				*(__le16 *)&buf[off] = cpu_to_le16(0); off += 2;
			} else {
				memcpy(&buf[off], payload, payload_len);
				off += payload_len;
			}
		}

		if (b->use_crc) {
			u16 crc;

			if (off + 2 <= RI_MAX_RESP_BYTES) {
				hdr->total_len = cpu_to_le16(off + 2);
				crc = ri_crc16(buf, off);
				*(__le16 *)&buf[off] = cpu_to_le16(crc);
				off += 2;
			} else {
				hdr->total_len = cpu_to_le16(off);
			}
		} else {
			hdr->total_len = cpu_to_le16(off);
		}

		/* Single-buffer model: publish immediately */
		b->tx_active = b->tx_buf;
		b->tx_len = off;
		b->tx_ready = true;
		b->tx_idx = 0;
		/* Log the prepared RESULT frame summary */
		{
			const struct ri_hdr *ph = (const struct ri_hdr *)b->tx_buf;
			u16 total_dbg = le16_to_cpu(ph->total_len);

			dev_dbg(b->dev, "result prepared: total=%u status=%d nread=%u\n",
				 (unsigned int)total_dbg, (int)status, (unsigned int)nread_msgs);
		}
	}
	spin_unlock_irqrestore(&b->lock, irqflags);

	/* Clear RX capture and allow new frames */
	spin_lock_irqsave(&b->lock, irqflags);
	b->rx_len = 0;
	spin_unlock_irqrestore(&b->lock, irqflags);

	/* payload_ws is persistent; no free */
}

static int bridge_slave_cb(struct i2c_client *client,
			   enum i2c_slave_event event, u8 *val)
{
	struct bridge_target *b = i2c_get_clientdata(client);
	unsigned long flags;
	/* no header peeking in callback; worker validates frame */

	switch (event) {
	case I2C_SLAVE_WRITE_RECEIVED:
		spin_lock_irqsave(&b->lock, flags);

		if (b->rx_len < RI_MAX_REQ_BYTES)
			b->rx_buf[b->rx_len++] = *val;
		spin_unlock_irqrestore(&b->lock, flags);
		break;

	case I2C_SLAVE_STOP:
		/* If we captured a full frame, schedule work */
		spin_lock_irqsave(&b->lock, flags);
		if (b->rd_idx == 0 && b->rx_len > 0) {
			/* 1) Host sent SubmitXfer (WRITE complete, nothing read yet) */
			dev_dbg(b->dev, "host submit received: len=%u\n", b->rx_len);
			print_hex_dump_debug("host submit rx: ", DUMP_PREFIX_OFFSET, 16, 1,
					     b->rx_buf, min_t(u16, b->rx_len, 128), false);
			/* New SubmitXfer arrived: cancel any stale outgoing response state */
			b->tx_ready = false;
			b->tx_idx = 0;
			b->tx_len = 0;
			b->tx_active = b->tx_buf;
			/* Prefer fresh work: cancel pending (non-blocking) and schedule new */
			cancel_work(&b->work);
			schedule_work(&b->work);
		}
		/* End of current transaction: allow pending swap */
		{
			bool consumed_payload = (b->rd_is_result && b->rd_idx > sizeof(struct ri_hdr));
			/* Log end of transaction */
			dev_dbg(b->dev, "host read finished: sent=%u bytes\n", b->tx_idx);
			/*
			 * If we served a RESULT and the master read past the header,
			 * assume it will not come back for more; clear the current result.
			 */
			if (consumed_payload) {
				b->tx_ready = false;
				b->tx_idx = 0;
				b->tx_len = 0;
				b->tx_active = b->tx_buf;
			}
			/* Clear read serving state */
			b->rd_ptr = NULL;
			b->rd_len = 0;
			b->rd_idx = 0;
			b->rd_is_result = false;
		}
		spin_unlock_irqrestore(&b->lock, flags);
		break;

	case I2C_SLAVE_READ_REQUESTED:
		spin_lock_irqsave(&b->lock, flags);
		/* Start of READ: always choose which response to serve */
		if (b->tx_ready) {
			b->rd_ptr = b->tx_active + b->tx_idx;
			b->rd_len = b->tx_len - b->tx_idx;
			b->rd_idx = 0;
			b->rd_is_result = true;
		} else {
			b->rd_ptr = b->tx_not_ready;
			b->rd_len = b->tx_not_ready_len;
			b->rd_idx = 0;
			b->rd_is_result = false;
		}
		if (b->rd_idx < b->rd_len) {
			*val = b->rd_ptr[b->rd_idx++];
			if (b->rd_is_result && b->tx_idx < b->tx_len)
				b->tx_idx++;
		} else {
			*val = 0x00;
		}
		spin_unlock_irqrestore(&b->lock, flags);
		break;

	case I2C_SLAVE_READ_PROCESSED:
		spin_lock_irqsave(&b->lock, flags);
		/* READ_PROCESSED: just continue streaming bytes */
		if (b->rd_idx < b->rd_len) {
			*val = b->rd_ptr[b->rd_idx++];
			if (b->rd_is_result && b->tx_idx < b->tx_len)
				b->tx_idx++;
		} else {
			*val = 0x00;
		}
		spin_unlock_irqrestore(&b->lock, flags);
		break;

	default:
		break;
	}

	return 0;
}

static int bridge_probe(struct i2c_client *client)
{
	struct bridge_target *b;
	int ret;
	struct device_node *child;
	u32 reg, down_bus, down_addr;
	unsigned int count = 0;

	b = devm_kzalloc(&client->dev, sizeof(*b), GFP_KERNEL);
	if (!b)
		return -ENOMEM;

	b->dev = &client->dev;
	b->slave = client;
	spin_lock_init(&b->lock);
	INIT_WORK(&b->work, bridge_work_fn);
	dev_dbg(&client->dev, "client probe: addr=%02x\n", client->addr);

	b->rx_buf = devm_kzalloc(&client->dev, RI_MAX_REQ_BYTES, GFP_KERNEL);
	b->tx_buf = devm_kzalloc(&client->dev, RI_MAX_RESP_BYTES, GFP_KERNEL);
	b->tx_not_ready = devm_kzalloc(&client->dev, RI_MAX_RESP_BYTES, GFP_KERNEL);
	b->payload_ws = devm_kzalloc(&client->dev, RI_MAX_RESP_BYTES, GFP_KERNEL);
	b->read_ws = devm_kzalloc(&client->dev, RI_MAX_RESP_BYTES, GFP_KERNEL);
	if (!b->rx_buf || !b->tx_buf || !b->tx_not_ready || !b->payload_ws || !b->read_ws)
		return -ENOMEM;
	b->tx_active = b->tx_buf;
	b->tx_len = 0;
	b->rx_len = 0;
	b->rd_ptr = NULL;
	b->rd_len = 0;
	b->rd_is_result = false;

	/* Prebuild static NOT_READY frame */
	ri_build_static_not_ready(b);

	/* Build whitelist mapping from DT children: map@XX nodes */
	memset(b->vmap_valid, 0, sizeof(b->vmap_valid));
	for_each_available_child_of_node(client->dev.of_node, child) {
		if (of_property_read_u32(child, "reg", &reg))
			continue;
		if (of_property_read_u32(child, "downstream-bus", &down_bus))
			continue;
		if (of_property_read_u32(child, "downstream-addr", &down_addr))
			continue;
		if (reg < 0x80) {
			b->vmap_valid[reg] = true;
			b->vmap_down_bus[reg] = down_bus;
			b->vmap_down_addr[reg] = (u8)(down_addr & 0x7f);
			dev_dbg(&client->dev, "map: virt=0x%02x -> bus=%u addr=0x%02x\n",
				(u32)reg, down_bus, down_addr);
			count++;
		}
	}

	b->use_crc = of_property_read_bool(client->dev.of_node, "use-crc16");
	dev_dbg(&client->dev, "client: use-crc16=%d\n", b->use_crc);

	i2c_set_clientdata(client, b);

	/* Mark client as a slave before registering; core will fail if unsupported */
	client->flags |= I2C_CLIENT_SLAVE;

	/* Register as I2C slave/target */
	ret = i2c_slave_register(client, bridge_slave_cb);
	if (ret) {
		dev_err(&client->dev, "i2c_slave_register failed: %d\n", ret);
		return ret;
	}

	/* Prime response as NOT_READY */
	b->tx_ready = false;
	b->tx_idx = 0;
	b->tx_len = 0;

	dev_info(&client->dev, "virtual-mux-client up @%02x (maps=%u, crc=%d)\n",
		 client->addr, count, b->use_crc);
	if (!client->dev.of_node && count == 0)
		dev_warn(&client->dev, "no DT mapping found (sysfs new_device). All requests will be denied.\n");

	return 0;
}

static void bridge_remove(struct i2c_client *client)
{
	struct bridge_target *b = i2c_get_clientdata(client);

	i2c_slave_unregister(client);
	cancel_work_sync(&b->work);
}

static const struct of_device_id bridge_of_match[] = {
	{ .compatible = "virtual-mux-client" },
	{ }
};
MODULE_DEVICE_TABLE(of, bridge_of_match);

/* Allow manual binding via sysfs new_device */
static const struct i2c_device_id bridge_id[] = {
	{ "virtual-mux-client", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, bridge_id);

static struct i2c_driver bridge_driver = {
	.driver = {
		.name = DRV_NAME,
		.of_match_table = bridge_of_match,
	},
	.id_table = bridge_id,
	.probe = bridge_probe,
	.remove = bridge_remove,
};

module_i2c_driver(bridge_driver);

MODULE_AUTHOR("Radivoje (Ogi) Jovanovic");
MODULE_DESCRIPTION("Virtual I2C Mux Client (I2C slave, executes downstream i2c_transfer())");
MODULE_LICENSE("GPL");


