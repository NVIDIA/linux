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
#include <linux/jiffies.h>
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

#define RI_FLAG_CRC16        BIT(0)
/*
 * Capability flag, host -> client: "host will read the response in a single
 * I2C transaction starting at offset 0". When set, we serve every
 * READ_REQUESTED from offset 0 (no cross-STOP cursor) and echo the flag
 * back in the response so the host knows we honored it. When absent, we
 * fall back to the legacy cursor-across-STOP behavior so this client
 * stays compatible with old hosts.
 */
#define RI_FLAG_OFFSET0_READ BIT(1)

#define RI_MAX_MSGS        4
#define RI_MAX_REQ_BYTES   2048
#define RI_MAX_RESP_BYTES  4096
#define RI_MSG_DESC_LEN    4 /* addr(1) + flags(1) + len(2) */
/*
 * Defaults for the downstream retry budget (overridable via DT
 * downstream-retry-max-ms / downstream-retry-sleep-ms). The worker
 * retries -EBUSY / -EAGAIN from i2c_transfer() until either a
 * non-transient outcome or this deadline. Sized to fit inside the BMC
 * host's per-attempt budget so the host doesn't time out mid-xfer; if
 * the host now uses xfer-timeout-ms=2500 (DTS) and host-retry-max-ms=8000,
 * 1500 ms here leaves comfortable headroom for transport plus polling.
 */
#define RI_DEFAULT_DOWNSTREAM_MAX_MS         1500
#define RI_DEFAULT_DOWNSTREAM_RETRY_SLEEP_MS 20

/*
 * Sanity ceiling for DT-supplied timing values (ms). A typo'd value
 * shouldn't be able to push the downstream retry wallclock or sleep
 * interval into pathological territory.
 */
#define RI_DT_TIMING_MAX_MS                  60000

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
	/* WRITE_REQUESTED seen, awaiting first WRITE_RECEIVED */
	bool rx_write_pending;
	/* bumped on each new submit STOP; worker drops stale publish */
	u32  rx_gen;

	/* Response */
	u8  *tx_buf;                        /* primary tx buffer */
	u8  *tx_active;                     /* current served buffer */
	u16  tx_len;                        /* length of active buffer */
	bool tx_ready;
	u16  tx_idx;
	/* serve current response from offset 0 each READ_REQUESTED */
	bool tx_offset0;
	/* Static NOT_READY frame */
	u8  *tx_not_ready;
	u16  tx_not_ready_len;

	/* Workspaces to avoid per-transfer allocations */
	u8  *payload_ws;                    /* read-payload build buffer */
	u8  *read_ws;                       /* READ i2c_msg backing storage */
	/* per-worker snapshot of rx_buf; owned by worker after snapshot */
	u8  *rx_local_buf;

	/* Current READ serving pointer (decoupled from tx_active when serving NOT_READY) */
	const u8 *rd_ptr;
	u16  rd_len;
	u16  rd_idx;
	bool rd_is_result;

	/* State */
	bool use_crc;
	u32 downstream_max_ms;              /* DT: downstream-retry-max-ms */
	u32 downstream_retry_sleep_ms;      /* DT: downstream-retry-sleep-ms */

	/* Async execution */
	struct workqueue_struct *wq;        /* private ordered wq; one submit in flight */
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
			      u32 exec_bus, u8 seq)
{
	struct i2c_adapter *exec_adap = NULL;
	int ret, i;
	unsigned int retries = 0;
	unsigned long deadline;
	u8 addr0 = nmsgs > 0 ? (msgs[0].addr & 0x7f) : 0xff;

	if (exec_bus == (u32)-1)
		return -ENODEV;

	exec_adap = i2c_get_adapter(exec_bus);
	if (!exec_adap) {
		dev_err(b->dev, "no adapter for bus %u\n", exec_bus);
		return -ENODEV;
	}

	/* Submit downstream request */
	dev_dbg(b->dev, "downstream submit: bus=%u nmsgs=%u\n", exec_bus, nmsgs);
	for (i = 0; i < nmsgs; i++) {
		dev_dbg(b->dev, "  dmsg[%d]: %s addr=0x%02x len=%u\n", i,
			(msgs[i].flags & I2C_M_RD) ? "RD" : "WR",
			msgs[i].addr, msgs[i].len);
	}

	/*
	 * Wallclock-bounded retry loop for transient downstream errors. -EBUSY
	 * and -EAGAIN are both "try again" conditions (bus contended,
	 * arbitration loss, downstream NACK with controller retries=0, etc.);
	 * either can be cleared by waiting a moment for the bus to become
	 * idle. Anything else (hard error, success, partial xfer) breaks out
	 * immediately. The retry sleep is clamped to the remaining budget
	 * so we don't overshoot downstream-retry-max-ms by a full sleep
	 * interval on the last attempt; the in-flight i2c_transfer is always
	 * allowed to finish, since aborting mid-xfer is what we explicitly
	 * do not want.
	 */
	deadline = jiffies + msecs_to_jiffies(b->downstream_max_ms);
	while (1) {
		unsigned long left_jif;
		u32 sleep_ms;

		ret = i2c_transfer(exec_adap, msgs, nmsgs);
		if (ret != -EBUSY && ret != -EAGAIN)
			break;
		if (time_after_eq(jiffies, deadline))
			break;
		retries++;
		left_jif = deadline - jiffies;
		sleep_ms = b->downstream_retry_sleep_ms;
		if (jiffies_to_msecs(left_jif) < sleep_ms)
			sleep_ms = jiffies_to_msecs(left_jif);
		if (sleep_ms)
			msleep(sleep_ms);
	}

	i2c_put_adapter(exec_adap);

	if (ret < 0) {
		dev_err(b->dev,
			"downstream xfer error: seq=%u bus=%u addr=0x%02x ret=%d retries=%u\n",
			seq, exec_bus, addr0, ret, retries);
		return (s16)ret;
	}
	if (ret != nmsgs) {
		dev_err(b->dev,
			"downstream short xfer: seq=%u bus=%u addr=0x%02x got=%d want=%d\n",
			seq, exec_bus, addr0, ret, nmsgs);
		return -EIO;
	}
	if (retries)
		dev_info(b->dev,
			 "downstream recovered: seq=%u bus=%u addr=0x%02x retries=%u\n",
			 seq, exec_bus, addr0, retries);

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
				       u8 *out_seq, bool *out_seq_valid,
				       u8 *read_payloads, size_t *read_payloads_len,
				       bool *out_offset0)
{
	const struct ri_hdr *hdr;
	u16 total, flags;
	size_t off;
	u8 nmsgs;
	int i, nread = 0;
	struct i2c_msg msgs[RI_MAX_MSGS];
	size_t read_ws_off = 0;
	size_t resp_payload_off = 0;
	int ret;
	u32 exec_bus = (u32)-1;
	u8  mapped_addr = 0;

	*read_payloads_len = 0;
	*out_offset0 = false;
	*out_seq_valid = false;

	if (len < sizeof(*hdr)) {
		dev_err_ratelimited(b->dev, "parse: short submit len=%zu (need >=%zu)\n",
				    len, sizeof(*hdr));
		return -EMSGSIZE;
	}

	hdr = (const struct ri_hdr *)buf;
	if (hdr->magic != RI_MAGIC_REQ || hdr->version != RI_VER ||
	    hdr->msg_type != RI_MSG_SUBMIT_XFER) {
		dev_err_ratelimited(b->dev,
				    "parse: bad header magic=0x%04x ver=0x%02x type=0x%02x len=%zu\n",
				    (u32)le16_to_cpu(hdr->magic), hdr->version,
				    hdr->msg_type, len);
		return -EPROTO;
	}

	/*
	 * From this point hdr->seq is trustworthy: publish-with-correct-seq
	 * lets the host correlate any error response to its submit. The
	 * worker checks *out_seq_valid before publishing so the two
	 * pre-seq error paths above (short submit, bad header) do NOT emit
	 * a bogus seq=0 response.
	 */
	*out_seq = hdr->seq;
	*out_seq_valid = true;

	total = le16_to_cpu(hdr->total_len);
	flags = le16_to_cpu(hdr->flags);

	if (total > len || total < sizeof(*hdr)) {
		dev_err_ratelimited(b->dev,
				    "parse: total/len mismatch seq=%u total=%u len=%zu\n",
				    hdr->seq, total, len);
		return -EMSGSIZE;
	}

	if (flags & RI_FLAG_CRC16) {
		u16 crc_frame, crc_calc;

		if (total < sizeof(*hdr) + 2) {
			dev_err_ratelimited(b->dev,
					    "parse: short for CRC trailer seq=%u total=%u\n",
					    hdr->seq, total);
			return -EMSGSIZE;
		}
		crc_frame = le16_to_cpu(*(__le16 *)&buf[total - 2]);
		crc_calc = ri_crc16(buf, total - 2);
		if (crc_frame != crc_calc) {
			dev_err_ratelimited(b->dev,
					    "crc mismatch: frame=0x%04x calc=0x%04x total=%u\n",
					    crc_frame, crc_calc, total);
			return -EBADMSG;
		}
	}

	*out_offset0 = !!(flags & RI_FLAG_OFFSET0_READ);
	dev_dbg(b->dev, "rx submit: seq=%u total=%u flags=0x%x ver=0x%02x\n",
		hdr->seq, total, flags, hdr->version);

	off = sizeof(*hdr);
	if (off + 1 + 1 + 2 + 4 > total) {
		dev_err_ratelimited(b->dev,
				    "parse: short payload hdr seq=%u off=%zu total=%u\n",
				    hdr->seq, off, total);
		return -EMSGSIZE;
	}

	nmsgs = buf[off++];
	off++;      /* retry_hint */
	off += 2;   /* timeout hint */
	off += 4;   /* client_cookie */

	if (nmsgs == 0 || nmsgs > RI_MAX_MSGS) {
		dev_err_ratelimited(b->dev, "parse: bad nmsgs=%u seq=%u\n",
				    nmsgs, hdr->seq);
		return -EINVAL;
	}

	memset(msgs, 0, sizeof(msgs));

	for (i = 0; i < nmsgs; i++) {
		u8 addr, mflags;
		u16 mlen;
		bool is_read;

		if (off + RI_MSG_DESC_LEN > total) {
			dev_err_ratelimited(b->dev,
					    "parse: truncated msg desc seq=%u i=%d off=%zu total=%u\n",
					    hdr->seq, i, off, total);
			return -EMSGSIZE;
		}

		addr = buf[off++];
		mflags = buf[off++];
		mlen = le16_to_cpu(*(__le16 *)&buf[off]); off += 2;

		is_read = !!(mflags & 0x01);

		/* Map virtual address -> downstream bus/addr */
		if (addr >= 0x80 || !b->vmap_valid[addr]) {
			/* not in whitelist */
			dev_warn(b->dev, "deny: virt=0x%02x not in whitelist\n", addr);
			return -EPERM;
		}
		mapped_addr = b->vmap_down_addr[addr] & 0x7f;
		if (exec_bus == (u32)-1) {
			exec_bus = b->vmap_down_bus[addr];
		} else if (exec_bus != b->vmap_down_bus[addr]) {
			/* mixed target buses in single transfer */
			dev_warn(b->dev, "deny: mixed buses virt=0x%02x bus=%u!=%u\n",
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
			if (read_ws_off + mlen > RI_MAX_RESP_BYTES) {
				dev_err_ratelimited(b->dev,
						    "parse: read ws overflow seq=%u i=%d off=%zu mlen=%u\n",
						    hdr->seq, i, read_ws_off, mlen);
				return -EMSGSIZE;
			}

			/*
			 * Pre-validate the encoded response payload size
			 * (u16 len + data per read msg) before we run
			 * ri_exec_downstream. Failing this check after the
			 * downstream transfer has already executed would leave
			 * the host to retry, which on a SubmitXfer containing
			 * downstream WRITE messages would re-execute them - the
			 * "execute and then bail" hazard. The check here is
			 * structurally equivalent to the one this replaces in
			 * the post-execute payload-build loop.
			 */
			if (resp_payload_off + 2 + mlen > RI_MAX_RESP_BYTES) {
				dev_err_ratelimited(b->dev,
						    "parse: encoded resp overflow seq=%u i=%d off=%zu mlen=%u\n",
						    hdr->seq, i, resp_payload_off, mlen);
				return -EMSGSIZE;
			}

			msgs[i].buf = &b->read_ws[read_ws_off];
			read_ws_off += mlen;
			resp_payload_off += 2 + mlen;
			nread++;
		} else {
			msgs[i].flags = 0;
			if (off + mlen > total) {
				dev_err_ratelimited(b->dev,
						    "parse: write data past total seq=%u i=%d off=%zu mlen=%u total=%u\n",
						    hdr->seq, i, off, mlen, total);
				return -EMSGSIZE;
			}
			/* Point directly into request buffer (safe during this function) */
			msgs[i].buf = (u8 *)&buf[off];
			off += mlen;
		}
	}

	/* Execute downstream transfer (adapter selected by whitelist mapping) */
	ret = ri_exec_downstream(b, msgs, nmsgs, exec_bus, hdr->seq);
	if (ret)
		return (s16)ret;

	/*
	 * Build read payloads: for each READ msg in order: u16 len + data.
	 * The total encoded size was already validated in the parse loop
	 * above (resp_payload_off check), so no per-iteration bounds check
	 * is needed here.
	 */
	{
		size_t woff = 0;

		for (i = 0; i < nmsgs; i++) {
			if (!(msgs[i].flags & I2C_M_RD))
				continue;

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
	bool seq_valid = false;
	s16 status = -EIO;
	unsigned long irqflags;
	u8 *payload;
	size_t payload_len = 0;
	u8 nread_msgs = 0;
	bool offset0 = false;
	u32 my_gen = 0;

	payload = b->payload_ws;

	/*
	 * Snapshot rx_buf into rx_local_buf under the lock. After this
	 * point the slave callback only writes to rx_buf (a new submit's
	 * WRITE_RECEIVED stream lands there); rx_local_buf is owned by
	 * this worker for the rest of its run, so msgs[].buf pointers
	 * we set up later for downstream i2c_transfer stay stable.
	 */
	{
		size_t snap_len;

		spin_lock_irqsave(&b->lock, irqflags);
		snap_len = b->rx_len;
		if (snap_len > RI_MAX_REQ_BYTES)
			snap_len = RI_MAX_REQ_BYTES;
		memcpy(b->rx_local_buf, b->rx_buf, snap_len);
		b->rx_len = 0;
		my_gen = b->rx_gen;
		spin_unlock_irqrestore(&b->lock, irqflags);

		status = ri_parse_submit_and_execute(b, b->rx_local_buf, snap_len,
						     &seq, &seq_valid,
						     payload, &payload_len,
						     &offset0);
	}

	/*
	 * Pre-seq parse failure (short submit or bad header): we have no
	 * trustworthy seq to put on the response, so the host wouldn't be
	 * able to correlate it anyway. Skip publishing entirely; the host
	 * will time out on its FETCH poll and retry with a fresh seq.
	 */
	if (!seq_valid)
		return;

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
	/*
	 * If a newer submit's STOP fired while this worker was running its
	 * downstream xfer, b->rx_gen has been bumped past my_gen. Drop this
	 * (now stale) publish; the worker queued by that newer STOP will
	 * snapshot the fresh data and publish the right response.
	 */
	if (b->rx_gen != my_gen) {
		spin_unlock_irqrestore(&b->lock, irqflags);
		return;
	}
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

		if (b->use_crc)
			flags |= RI_FLAG_CRC16;
		if (offset0)
			flags |= RI_FLAG_OFFSET0_READ;
		hdr->flags = cpu_to_le16(flags);

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
		b->tx_offset0 = offset0;
		/* Log the prepared RESULT frame summary */
		{
			const struct ri_hdr *ph = (const struct ri_hdr *)b->tx_buf;
			u16 total_dbg = le16_to_cpu(ph->total_len);

			dev_dbg(b->dev, "result prepared: total=%u status=%d nread=%u\n",
				 (unsigned int)total_dbg, (int)status, (unsigned int)nread_msgs);
		}
	}
	spin_unlock_irqrestore(&b->lock, irqflags);

	/*
	 * Do not clear b->rx_len here: a new SubmitXfer may already have
	 * started arriving on the link, and its WRITE_REQUESTED handler
	 * has already reset rx_len = 0 and subsequent WRITE_RECEIVED events
	 * are filling rx_buf from offset 0. Clearing it here would race with
	 * those writes.
	 */

	/* payload_ws is persistent; no free */
}

static int bridge_slave_cb(struct i2c_client *client,
			   enum i2c_slave_event event, u8 *val)
{
	struct bridge_target *b = i2c_get_clientdata(client);
	unsigned long flags;
	/* no header peeking in callback; worker validates frame */

	switch (event) {
	case I2C_SLAVE_WRITE_REQUESTED:
		/*
		 * Mark that a write is about to begin. We do NOT zero rx_len
		 * here because some I2C target back-ends (notably ast2600
		 * when STOP is coalesced with the next transaction's
		 * SLAVE_MATCH in one IRQ) fire WRITE_REQUESTED unconditionally
		 * even when the next transaction is actually a read. Deferring
		 * the rx_len reset to the first WRITE_RECEIVED makes such
		 * spurious WRITE_REQUESTED events harmless: if no data byte
		 * follows, rx_len keeps the previous submit's length and the
		 * worker still snapshots the right data.
		 */
		spin_lock_irqsave(&b->lock, flags);
		b->rx_write_pending = true;
		spin_unlock_irqrestore(&b->lock, flags);
		break;

	case I2C_SLAVE_WRITE_RECEIVED:
		spin_lock_irqsave(&b->lock, flags);
		if (b->rx_write_pending) {
			b->rx_len = 0;
			b->rx_write_pending = false;
		}
		if (b->rx_len < RI_MAX_REQ_BYTES)
			b->rx_buf[b->rx_len++] = *val;
		spin_unlock_irqrestore(&b->lock, flags);
		break;

	case I2C_SLAVE_STOP:
		spin_lock_irqsave(&b->lock, flags);
		if (b->rd_idx == 0 && b->rx_len > 0) {
			/* Host sent a SubmitXfer; cancel stale result and run worker */
			b->tx_ready = false;
			b->tx_idx = 0;
			b->tx_len = 0;
			b->tx_active = b->tx_buf;
			b->tx_offset0 = false;
			b->rx_gen++;
			cancel_work(&b->work);
			queue_work(b->wq, &b->work);
		} else if (b->rd_idx == 0 && b->rx_write_pending) {
			/*
			 * Address-matched write with no data byte accumulated:
			 * either an explicit zero-length resync from the host, or
			 * a SubmitXfer whose WRITE_RECEIVED stream got dropped by
			 * the i2c-target subsystem (master saw all bytes ACKed but
			 * slave_cb never received them). Drop any published result
			 * so the next READ_REQUESTED serves NOT_READY, ensuring
			 * the host's next submit is not shadowed by a frame from
			 * a previous seq.
			 */
			b->tx_ready = false;
			b->tx_idx = 0;
			b->tx_len = 0;
			b->tx_active = b->tx_buf;
			b->tx_offset0 = false;
		} else {
			bool consumed_payload;

			if (b->tx_offset0)
				consumed_payload = (b->rd_is_result && b->rd_idx >= b->tx_len);
			else
				consumed_payload = (b->rd_is_result && b->tx_idx >= b->tx_len);

			if (consumed_payload) {
				b->tx_ready = false;
				b->tx_idx = 0;
				b->tx_len = 0;
				b->tx_active = b->tx_buf;
				b->tx_offset0 = false;
			}
		}
		/* Reset per-transaction state for next transaction */
		b->rx_write_pending = false;
		b->rd_ptr = NULL;
		b->rd_len = 0;
		b->rd_idx = 0;
		b->rd_is_result = false;
		spin_unlock_irqrestore(&b->lock, flags);
		break;

	case I2C_SLAVE_READ_REQUESTED:
		spin_lock_irqsave(&b->lock, flags);
		/*
		 * Two serving modes:
		 *  - tx_offset0 (new host advertised RI_FLAG_OFFSET0_READ):
		 *    start from offset 0 of the active buffer on every
		 *    READ_REQUESTED; the host reads the full frame in one
		 *    transaction so no cross-STOP cursor is needed.
		 *  - legacy: start from tx_idx (cursor across STOPs) so old
		 *    hosts that stitch header+remainder still work.
		 */
		if (b->tx_ready) {
			if (b->tx_offset0) {
				b->rd_ptr = b->tx_active;
				b->rd_len = b->tx_len;
			} else {
				b->rd_ptr = b->tx_active + b->tx_idx;
				b->rd_len = b->tx_len - b->tx_idx;
			}
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
			if (!b->tx_offset0 && b->rd_is_result &&
			    b->tx_idx < b->tx_len)
				b->tx_idx++;
		} else {
			*val = 0x00;
		}
		spin_unlock_irqrestore(&b->lock, flags);
		break;

	case I2C_SLAVE_READ_PROCESSED:
		spin_lock_irqsave(&b->lock, flags);
		/* READ_PROCESSED: continue streaming bytes from rd_ptr */
		if (b->rd_idx < b->rd_len) {
			*val = b->rd_ptr[b->rd_idx++];
			if (!b->tx_offset0 && b->rd_is_result &&
			    b->tx_idx < b->tx_len)
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

	/*
	 * Private ordered workqueue so the long downstream xfer (up to
	 * downstream-retry-max-ms) does not block other system_wq
	 * consumers. WQ_MEM_RECLAIM keeps the queue drainable under
	 * memory pressure.
	 */
	b->wq = alloc_ordered_workqueue("%s", WQ_MEM_RECLAIM,
					dev_name(&client->dev));
	if (!b->wq)
		return -ENOMEM;

	b->rx_buf = devm_kzalloc(&client->dev, RI_MAX_REQ_BYTES, GFP_KERNEL);
	b->tx_buf = devm_kzalloc(&client->dev, RI_MAX_RESP_BYTES, GFP_KERNEL);
	b->tx_not_ready = devm_kzalloc(&client->dev, RI_MAX_RESP_BYTES, GFP_KERNEL);
	b->payload_ws = devm_kzalloc(&client->dev, RI_MAX_RESP_BYTES, GFP_KERNEL);
	b->read_ws = devm_kzalloc(&client->dev, RI_MAX_RESP_BYTES, GFP_KERNEL);
	b->rx_local_buf = devm_kzalloc(&client->dev, RI_MAX_REQ_BYTES, GFP_KERNEL);
	if (!b->rx_buf || !b->tx_buf || !b->tx_not_ready || !b->payload_ws ||
	    !b->read_ws || !b->rx_local_buf) {
		destroy_workqueue(b->wq);
		return -ENOMEM;
	}
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
	if (of_property_read_u32(client->dev.of_node, "downstream-retry-max-ms",
				 &b->downstream_max_ms))
		b->downstream_max_ms = RI_DEFAULT_DOWNSTREAM_MAX_MS;
	if (of_property_read_u32(client->dev.of_node, "downstream-retry-sleep-ms",
				 &b->downstream_retry_sleep_ms))
		b->downstream_retry_sleep_ms = RI_DEFAULT_DOWNSTREAM_RETRY_SLEEP_MS;
	/* Clamp DT values to a sane ceiling */
	if (b->downstream_max_ms > RI_DT_TIMING_MAX_MS)
		b->downstream_max_ms = RI_DT_TIMING_MAX_MS;
	if (b->downstream_retry_sleep_ms > RI_DT_TIMING_MAX_MS)
		b->downstream_retry_sleep_ms = RI_DT_TIMING_MAX_MS;
	if (b->downstream_retry_sleep_ms == 0)
		b->downstream_retry_sleep_ms = RI_DEFAULT_DOWNSTREAM_RETRY_SLEEP_MS;
	dev_dbg(&client->dev, "client: use-crc16=%d downstream-retry-max-ms=%u sleep-ms=%u\n",
		b->use_crc, b->downstream_max_ms, b->downstream_retry_sleep_ms);

	i2c_set_clientdata(client, b);

	/* Mark client as a slave before registering; core will fail if unsupported */
	client->flags |= I2C_CLIENT_SLAVE;

	/* Register as I2C slave/target */
	ret = i2c_slave_register(client, bridge_slave_cb);
	if (ret) {
		dev_err(&client->dev, "i2c_slave_register failed: %d\n", ret);
		destroy_workqueue(b->wq);
		return ret;
	}

	/* Prime response as NOT_READY */
	b->tx_ready = false;
	b->tx_idx = 0;
	b->tx_len = 0;

	dev_info(&client->dev,
		 "virtual-mux-client up @%02x maps=%u crc=%d downstream-retry-max-ms=%u sleep-ms=%u\n",
		 client->addr, count, b->use_crc,
		 b->downstream_max_ms, b->downstream_retry_sleep_ms);
	if (!client->dev.of_node && count == 0)
		dev_warn(&client->dev, "no DT mapping found (sysfs new_device). All requests will be denied.\n");

	return 0;
}

static void bridge_remove(struct i2c_client *client)
{
	struct bridge_target *b = i2c_get_clientdata(client);

	i2c_slave_unregister(client);
	cancel_work_sync(&b->work);
	destroy_workqueue(b->wq);
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


