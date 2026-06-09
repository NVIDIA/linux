// SPDX-License-Identifier: GPL-2.0
/*
 * Remote I2C FRU Host Adapter
 *
 * Registers a virtual I2C adapter. Forwards i2c_transfer() to a remote bridge
 * target over a link I2C bus using the Remote I2C FRU Bridge Protocol.
 *
 * Intended use: expose remote FRU EEPROM(s) to stock at24 and OpenBMC FRU/IPMI.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/crc16.h>
#include <linux/mutex.h>
#include <linux/printk.h>
#include <linux/minmax.h>

#define DRV_NAME "virtual-mux-host"

/* Protocol constants */
#define RI_MAGIC_REQ  cpu_to_le16(0x4952) /* 'R''I' */
#define RI_MAGIC_RESP cpu_to_le16(0x4F52) /* 'R''O' */
/* Protocol version:
 * 0x01 - original framing
 * 0x02 - host-selected bus_id (deprecated)
 * 0x03 - client-enforced mapping; host sends only virtual addresses
 */
#define RI_VER        0x03

#define RI_MSG_SUBMIT_XFER 0x01
#define RI_MSG_XFER_RESULT 0x81
#define RI_MSG_NOT_READY   0x82

#define RI_FLAG_CRC16        BIT(0)
/*
 * Capability flag, host -> client: "I will read the response in a single
 * I2C transaction starting at offset 0. Please serve every READ_REQUESTED
 * from offset 0 instead of advancing a cursor across STOPs."
 * The client echoes the flag in the response header if it honored the
 * request. The host then knows whether to issue the new single-transaction
 * full-frame read or fall back to the legacy header+remainder pattern.
 * Old clients ignore unknown flag bits and respond without it.
 */
#define RI_FLAG_OFFSET0_READ BIT(1)

#define RI_MAX_MSGS        4
#define RI_MAX_REQ_BYTES   2048
#define RI_MAX_RESP_BYTES  4096
#define RI_RESP_PAYLOAD_HDR_LEN 8 /* status(2) + nread(1) + rsvd(1) + rsvd2(4) */
#define RI_DEFAULT_POLL_SLEEP_MS   40
#define RI_DEFAULT_XFER_TIMEOUT_MS 2500

/*
 * Sanity ceiling for DT-supplied timing values (ms). A typo'd or
 * pathological value gets clamped here so we don't compute absurd
 * jiffy counts or wedge the FETCH poll loop with hundreds of millions
 * of iterations.
 */
#define RI_DT_TIMING_MAX_MS  60000

/*
 * Wallclock cap and inter-attempt sleep for the host's outer retry loop.
 * Symmetric with the client's downstream retry: on transient errors
 * (-EBUSY/-EAGAIN/-ETIMEDOUT/-EBADMSG/-EREMOTEIO) we resubmit with a fresh
 * seq until success or deadline. Sized below fru-device's 10s rescan future
 * cap so a stuck transaction can't take an entire bus offline.
 */
#define RI_DEFAULT_HOST_RETRY_MAX_MS    8000
#define RI_DEFAULT_HOST_RETRY_SLEEP_MS  20

/*
 * Stale-seq resync threshold. If the client serves a frame whose seq does
 * not match our outstanding submit for this many consecutive poll cycles,
 * the client is likely stuck on a previous result (its WRITE_RECEIVED
 * stream for our submit may have been dropped by the i2c-target
 * subsystem). Send a zero-length write to nudge the client into clearing
 * its tx_ready so it returns NOT_READY for the remainder of this xfer;
 * the outer retry loop then resubmits with a fresh seq.
 */
#define RI_STALE_RESYNC_THRESHOLD       3

/*
 * CP2112 reports address NACK as -ETIMEDOUT. Match virtual-mux-client:
 * fast ETIMEDOUT (< this ms) is treated as no device, not a link glitch.
 */
#define RI_NACK_FAST_FAIL_MS            800

/* Common header (packed) */
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

struct remote_host {
	struct device *dev;
	struct i2c_client *link_client;     /* The *link* bus client (bridge addr) */
	struct i2c_adapter virt_adap;        /* Virtual adapter presented to Linux */
	struct i2c_algorithm algo;
	struct mutex xfer_lock;             /* serialize transfers (FRU is fine) */
	u8 seq;
	bool use_crc;
	u32 poll_sleep_ms;                  /* DT: xfer-poll-sleep-ms */
	u32 xfer_timeout_ms;                /* DT: xfer-timeout-ms */
	u32 host_retry_max_ms;              /* DT: host-retry-max-ms */
	u32 host_retry_sleep_ms;            /* DT: host-retry-sleep-ms */
	u32 adapter_nr;                     /* optional forced bus number */
};

static int ri_build_submit_frame(struct remote_host *h,
				 struct i2c_msg *msgs, int num,
				 u8 **out_buf, size_t *out_len, u8 seq)
{
	u8 *buf;
	size_t off = 0, i;
	u16 flags = 0;
	struct ri_hdr *hdr;
	int read_count = 0;

	if (num <= 0 || num > RI_MAX_MSGS)
		return -EINVAL;

	buf = kzalloc(RI_MAX_REQ_BYTES, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	hdr = (struct ri_hdr *)buf;
	hdr->magic = RI_MAGIC_REQ;
	hdr->version = RI_VER;
	hdr->msg_type = RI_MSG_SUBMIT_XFER;
	hdr->seq = seq;
	hdr->header_len = sizeof(*hdr);
	hdr->flags = 0;
	hdr->reserved = 0;

	if (h->use_crc)
		flags |= RI_FLAG_CRC16;
	/* Advertise that we will single-transaction read responses */
	flags |= RI_FLAG_OFFSET0_READ;
	hdr->flags = cpu_to_le16(flags);

	off = sizeof(*hdr);

	/* Payload header */
	buf[off++] = (u8)num;     /* nmsgs */
	buf[off++] = 0;           /* retry_hint */
	*(__le16 *)&buf[off] = cpu_to_le16(0); off += 2;  /* timeout_ms_hint */
	*(__le32 *)&buf[off] = cpu_to_le32(0); off += 4;  /* client_cookie */

	dev_dbg(h->dev, "submit seq=%u nmsgs=%d\n", seq, num);

	/* Message descriptors */
	for (i = 0; i < num; i++) {
		u16 len = msgs[i].len;
		u8 addr = msgs[i].addr & 0x7f;  /* host sends virtual address only */
		bool is_read = !!(msgs[i].flags & I2C_M_RD);

		if (off + 1 + 1 + 2 > RI_MAX_REQ_BYTES) {
			kfree(buf);
			return -EMSGSIZE;
		}

		buf[off++] = addr;

		/* msg_flags: bit0=READ */
		buf[off++] = is_read ? 0x01 : 0x00;

		dev_dbg(h->dev, "  msg[%zu]: %s virt=0x%02x len=%u\n",
			i, is_read ? "RD" : "WR", addr, len);

		*(__le16 *)&buf[off] = cpu_to_le16(len);
		off += 2;

		if (!is_read) {
			if (off + len > RI_MAX_REQ_BYTES) {
				kfree(buf);
				return -EMSGSIZE;
			}
			memcpy(&buf[off], msgs[i].buf, len);
			off += len;
		} else {
			read_count++;
		}
	}

	/* Fill total_len and optional CRC */
	if (h->use_crc) {
		u16 crc;

		if (off + 2 > RI_MAX_REQ_BYTES) {
			kfree(buf);
			return -EMSGSIZE;
		}
		hdr->total_len = cpu_to_le16(off + 2);
		crc = crc16(0, buf, off);
		*(__le16 *)&buf[off] = cpu_to_le16(crc);
		off += 2;
	} else {
		hdr->total_len = cpu_to_le16(off);
	}

	*out_buf = buf;
	*out_len = off;
	return 0;
}

static int ri_parse_response(struct remote_host *h,
			     struct i2c_msg *msgs, int num,
			     const u8 *buf, size_t buflen, u8 seq)
{
	size_t cursor = 0;
	bool saw_not_ready = false;

	while (cursor + sizeof(struct ri_hdr) <= buflen) {
		const struct ri_hdr *hdr;
		size_t off;
		u16 total;
		u16 flags;
		s16 status;
		u8 nread_msgs;
		int i, read_idx = 0;

		hdr = (const struct ri_hdr *)(buf + cursor);

		dev_dbg(h->dev,
			"resp hdr: magic=0x%04x ver=0x%02x type=0x%02x seq=%u hlen=%u total=%u flags=0x%04x buflen=%zu cursor=%zu\n",
			(u32)le16_to_cpu(hdr->magic), hdr->version, hdr->msg_type, hdr->seq,
			hdr->header_len, (u32)le16_to_cpu(hdr->total_len), (u32)le16_to_cpu(hdr->flags),
			buflen, cursor);

		if (hdr->magic != RI_MAGIC_RESP || hdr->version != RI_VER) {
			dev_warn(h->dev,
				 "parse: bad hdr magic=0x%04x ver=0x%02x seq=%u (want seq=%u)\n",
				 (u32)le16_to_cpu(hdr->magic), hdr->version,
				 hdr->seq, seq);
			return -EPROTO;
		}

		total = le16_to_cpu(hdr->total_len);
		flags = le16_to_cpu(hdr->flags);

		if (total < sizeof(*hdr) || cursor + total > buflen) {
			dev_warn(h->dev,
				 "parse: bad total=%u seq=%u buflen=%zu\n",
				 total, hdr->seq, buflen);
			return -EMSGSIZE;
		}

		if (flags & RI_FLAG_CRC16) {
			u16 crc_calc, crc_frame;

			if (total < sizeof(*hdr) + 2) {
				dev_warn(h->dev,
					 "parse: short for CRC seq=%u total=%u\n",
					 hdr->seq, total);
				return -EMSGSIZE;
			}
			crc_frame = le16_to_cpu(*(__le16 *)&buf[cursor + total - 2]);
			crc_calc = crc16(0, buf + cursor, total - 2);
			if (crc_frame != crc_calc) {
				dev_warn(h->dev,
					 "parse: CRC mismatch seq=%u frame=0x%04x calc=0x%04x total=%u\n",
					 hdr->seq, crc_frame, crc_calc, total);
				return -EBADMSG;
			}
		}

		/* If this is NOT_READY, be tolerant of seq mismatches and continue scanning */
		if (hdr->msg_type == RI_MSG_NOT_READY) {
			saw_not_ready = true;
			cursor += total;
			continue;
		}

		/* For real responses, require matching seq */
		if (hdr->msg_type != RI_MSG_XFER_RESULT || hdr->seq != seq) {
			cursor += total;
			continue;
		}

		/* Parse the selected frame */
		off = cursor + sizeof(*hdr);

		/* Response payload: s16 status, u8 nread_msgs, u8 reserved, ... */
		if (off + RI_RESP_PAYLOAD_HDR_LEN > cursor + total)
			return -EMSGSIZE;

		status = (s16)le16_to_cpu(*(__le16 *)&buf[off]); off += 2;
		nread_msgs = buf[off++]; /* u8 */
		off++;                   /* reserved */
		off += 2;                /* reserved2 */
		off += 2;                /* reserved2 cont (we used 4 bytes total) */

		/* Log parsed structure summary before applying to msgs */
		dev_dbg(h->dev, "resp parsed: seq=%u status=%d nread=%u total=%u\n",
			 (unsigned int)seq, (int)status, (unsigned int)nread_msgs, (unsigned int)total);

		if (status < 0)
			return (int)status;

		dev_dbg(h->dev, "resp ok: seq=%u nread=%u\n", seq, nread_msgs);

		/* Copy read payloads into corresponding read msgs in original array */
		for (i = 0; i < num; i++) {
			if (!(msgs[i].flags & I2C_M_RD))
				continue;

			if (read_idx >= nread_msgs)
				return -EPROTO;

			if (off + 2 > cursor + total)
				return -EMSGSIZE;

			{
				u16 rlen = le16_to_cpu(*(__le16 *)&buf[off]);

				off += 2;

				if (rlen != msgs[i].len)
					return -EPROTO;

				if (off + rlen > cursor + total)
					return -EMSGSIZE;

				memcpy(msgs[i].buf, &buf[off], rlen);
				off += rlen;
			}

			read_idx++;
		}

		if (read_idx != nread_msgs)
			return -EPROTO;

		return 0;
	}

	/* No usable frame; if we saw NOT_READY earlier, ask caller to retry */
	return saw_not_ready ? -EAGAIN : -EPROTO;
}

static int ri_read_response_frame(struct remote_host *h,
				  const struct ri_hdr *hdr,
				  u8 *resp, u16 total)
{
	u16 remain;
	u8 *p;
	int r;
	bool offset0;

	/* Sanity on total size */
	if (total < sizeof(*hdr) || total > RI_MAX_RESP_BYTES)
		return -EMSGSIZE;

	/*
	 * If the client echoed RI_FLAG_OFFSET0_READ in the response header,
	 * it serves every READ_REQUESTED from offset 0 and we re-read the
	 * full frame (header included) in a single I2C transaction. This
	 * removes the cross-STOP cursor in the slave and is immune to
	 * end-of-xfer callback quirks in any particular slave back-end.
	 *
	 * If the flag is not echoed, the client is the legacy implementation
	 * that advances a cursor across STOPs; preserve the original
	 * memcpy-header + read-remainder pattern so this host stays
	 * compatible with old clients.
	 */
	offset0 = !!(le16_to_cpu(hdr->flags) & RI_FLAG_OFFSET0_READ);

	if (offset0) {
		remain = total;
		p = resp;
	} else {
		memcpy(resp, hdr, sizeof(*hdr));
		remain = total - sizeof(*hdr);
		p = resp + sizeof(*hdr);
	}

	while (remain) {
		r = i2c_master_recv(h->link_client, p, remain);
		if (r < 0)
			return r;
		if (r == 0)
			return -EMSGSIZE;
		p += r;
		remain -= r;
	}

	return 0;
}

static void ri_hex_dump(struct remote_host *h, const char *label,
			const void *buf, size_t len)
{
	char prefix[64];

	scnprintf(prefix, sizeof(prefix), "%s: %s: ", dev_name(h->dev), label);
	print_hex_dump_debug(prefix, DUMP_PREFIX_OFFSET, 16, 1,
			     buf, len, false);
}

static bool ri_xfer_retryable(int ret, unsigned int elapsed_ms)
{
	/*
	 * Class A (device gone / hard downstream): no retry this xfer.
	 * -ENXIO: target address NACK (no device), relayed by HMC client.
	 * -EIO: device responded but data xfer failed.
	 * -ENODEV: downstream bus unavailable.
	 * -EREMOTEIO: legacy NACK mapping from older HMC firmware.
	 *
	 * Class B (transient link/protocol): retry within host-retry-max-ms.
	 * -ETIMEDOUT: link or bus timeout. CP2112 reports address NACK as
	 * -ETIMEDOUT too; fast completion (< RI_NACK_FAST_FAIL_MS) is treated
	 * as Class A. Slow ETIMEDOUT stays retryable.
	 * -EPROTO / -EMSGSIZE: corrupted RI frame, usually clears on resubmit.
	 */
	if (ret == -ENXIO || ret == -EIO || ret == -ENODEV || ret == -EREMOTEIO)
		return false;
	if (ret == -ETIMEDOUT && elapsed_ms < RI_NACK_FAST_FAIL_MS)
		return false;

	return ret == -EBUSY || ret == -EAGAIN || ret == -ETIMEDOUT ||
	       ret == -EBADMSG || ret == -EPROTO || ret == -EMSGSIZE;
}

static int ri_do_one_xfer(struct remote_host *h, struct i2c_msg *msgs, int num)
{
	u8 *req = NULL;
	size_t req_len = 0;
	u8 *resp = NULL;
	int ret, tries, max_tries;
	unsigned int stale_polls = 0;
	u8 seq;

	seq = ++h->seq;
	dev_dbg(h->dev, "xfer start: seq=%u num=%d\n", seq, num);

	ret = ri_build_submit_frame(h, msgs, num, &req, &req_len, seq);
	if (ret)
		goto out;

	/* Dump request frame (first 128 bytes) */
	ri_hex_dump(h, "ri host req", req, min_t(size_t, req_len, 128));

	ret = i2c_master_send(h->link_client, req, req_len);
	if (ret < 0)
		goto out;
	if (ret != req_len) {
		ret = -EIO;
		goto out;
	}

	resp = kzalloc(RI_MAX_RESP_BYTES, GFP_KERNEL);
	if (!resp) {
		ret = -ENOMEM;
		goto out;
	}

	/* Poll FETCH: read header first, then exact remaining payload */
	max_tries = DIV_ROUND_UP(h->xfer_timeout_ms, h->poll_sleep_ms);
	max_tries = max_t(int, 1, max_tries);
	for (tries = 0; tries < max_tries; tries++) {
		struct ri_hdr hdr_local;
		int r;
		u16 total;
		u8 msg_type;

		/* Read exactly header */
		r = i2c_master_recv(h->link_client, (u8 *)&hdr_local, sizeof(hdr_local));
		if (r < 0) {
			dev_warn(h->dev,
				 "fetch hdr: recv error seq=%u ret=%d\n", seq, r);
			ret = r;
			goto out;
		}
		if (r != sizeof(hdr_local)) {
			dev_warn(h->dev,
				 "fetch hdr: short recv seq=%u got=%d want=%zu\n",
				 seq, r, sizeof(hdr_local));
			ret = -EMSGSIZE;
			goto out;
		}

		/* Dump raw header */
		ri_hex_dump(h, "ri host resp hdr", &hdr_local, sizeof(hdr_local));

		/* Basic validation */
		if (hdr_local.magic != RI_MAGIC_RESP || hdr_local.version != RI_VER) {
			dev_warn(h->dev,
				 "fetch hdr: bad magic=0x%04x ver=0x%02x seq=%u (want seq=%u)\n",
				 (u32)le16_to_cpu(hdr_local.magic),
				 hdr_local.version, hdr_local.seq, seq);
			ret = -EPROTO;
			goto out;
		}

		msg_type = hdr_local.msg_type;
		total = le16_to_cpu(hdr_local.total_len);

		/* NOT_READY: tolerate any seq and retry */
		if (msg_type == RI_MSG_NOT_READY) {
			dev_dbg(h->dev,
				"fetch: NOT_READY seq=%u (want %u) total=%u try=%d/%d\n",
				hdr_local.seq, seq, total, tries, max_tries);
			msleep(h->poll_sleep_ms);
			continue;
		}

		/* For real responses, require matching seq */
		if (msg_type != RI_MSG_XFER_RESULT || hdr_local.seq != seq) {
			dev_warn(h->dev,
				 "fetch: skip type=0x%02x seq=%u (want type=0x%02x seq=%u) total=%u\n",
				 msg_type, hdr_local.seq,
				 (u32)RI_MSG_XFER_RESULT, seq, total);
			/* Drain the rest of this frame if any to keep link sane */
			if (total > sizeof(hdr_local) && total <= RI_MAX_RESP_BYTES) {
				u16 drain = total - sizeof(hdr_local);
				u8 drain_buf[32];

				while (drain) {
					int chunk = min_t(int, drain, (int)sizeof(drain_buf));

					r = i2c_master_recv(h->link_client, drain_buf, chunk);
					if (r <= 0)
						break;
					drain -= r;
				}
			}
			if (++stale_polls == RI_STALE_RESYNC_THRESHOLD) {
				struct i2c_msg resync = {
					.addr  = h->link_client->addr,
					.flags = 0,
					.len   = 0,
					.buf   = NULL,
				};
				int rret;

				rret = i2c_transfer(h->link_client->adapter,
						    &resync, 1);
				if (rret < 0)
					dev_warn(h->dev,
						 "stale-seq resync send failed seq=%u after %u stale polls: ret=%d\n",
						 seq, stale_polls, rret);
				else
					dev_info(h->dev,
						 "stale-seq resync sent seq=%u after %u stale polls\n",
						 seq, stale_polls);
			}
			msleep(h->poll_sleep_ms);
			continue;
		}

		ret = ri_read_response_frame(h, &hdr_local, resp, total);
		if (ret) {
			dev_warn(h->dev,
				 "fetch body: ri_read_response_frame seq=%u total=%u ret=%d\n",
				 seq, total, ret);
			goto out;
		}

		/* Dump full frame (first 128 bytes) */
		ri_hex_dump(h, "ri host resp full", resp, min_t(u16, total, 128));

		/* Parse the complete frame */
		ret = ri_parse_response(h, msgs, num, resp, total, seq);
		if (ret == 0) {
			ret = num;
			dev_dbg(h->dev, "xfer done: seq=%u ok\n", seq);
		} else {
			dev_warn(h->dev,
				 "parse: ri_parse_response seq=%u total=%u ret=%d\n",
				 seq, total, ret);
		}
		goto out;
	}

	dev_warn(h->dev,
		 "fetch: poll deadline expired seq=%u timeout_ms=%u\n",
		 seq, h->xfer_timeout_ms);
	ret = -ETIMEDOUT;

out:
	kfree(resp);
	kfree(req);
	return ret;
}

static int remote_master_xfer(struct i2c_adapter *adap,
			      struct i2c_msg *msgs, int num)
{
	struct remote_host *h = i2c_get_adapdata(adap);
	unsigned long deadline;
	unsigned int retries = 0;
	int ret;

	mutex_lock(&h->xfer_lock);

	deadline = jiffies + msecs_to_jiffies(h->host_retry_max_ms);
	for (;;) {
		unsigned long left_jif;
		unsigned long attempt_start;
		unsigned int elapsed_ms;
		u32 sleep_ms;

		attempt_start = jiffies;
		ret = ri_do_one_xfer(h, msgs, num);
		elapsed_ms = jiffies_to_msecs(jiffies - attempt_start);
		if (!ri_xfer_retryable(ret, elapsed_ms))
			break;
		dev_warn(h->dev,
			 "xfer retry: ret=%d retries=%u (sleeping %ums)\n",
			 ret, retries, h->host_retry_sleep_ms);
		if (time_after_eq(jiffies, deadline))
			break;
		retries++;
		/*
		 * Clamp the retry sleep to the remaining wallclock budget so
		 * we don't overshoot host-retry-max-ms by a full sleep
		 * interval on the last cycle, then re-check the deadline
		 * before launching another attempt: msleep can still drift
		 * past the deadline and we don't want to spend another full
		 * xfer-timeout-ms past budget. The in-flight ri_do_one_xfer
		 * is always allowed to finish - aborting mid-xfer is what we
		 * explicitly do not want; worst-case total runtime is thus
		 * host-retry-max-ms + one xfer-timeout-ms.
		 */
		left_jif = deadline - jiffies;
		sleep_ms = h->host_retry_sleep_ms;
		if (jiffies_to_msecs(left_jif) < sleep_ms)
			sleep_ms = jiffies_to_msecs(left_jif);
		if (sleep_ms)
			msleep(sleep_ms);
		if (time_after_eq(jiffies, deadline))
			break;
	}

	/*
	 * Per-attempt failure events (bad header, CRC, parse error, retry,
	 * poll deadline, stale-seq skip) stay at dev_warn so a single log
	 * dump captures *why* a transfer needed retries; without that
	 * why-trail the iter-14-class "stuck on stale seq for 4s then
	 * recovered after 2 retries" symptom is invisible from a single
	 * outcome line. Per-poll NOT_READY status is dev_dbg (no event
	 * content; only useful with dynamic debug enabled). The single
	 * recovered/failed line below is the summary; the per-event lines
	 * above are the why-trail.
	 */
	if (retries && ret > 0)
		dev_info(h->dev, "xfer recovered: ret=%d retries=%u\n",
			 ret, retries);
	else if (ret < 0)
		dev_err(h->dev, "xfer failed: ret=%d retries=%u\n",
			ret, retries);

	mutex_unlock(&h->xfer_lock);
	return ret;
}

static u32 remote_func(struct i2c_adapter *adap)
{
	/*
	 * Support plain I2C and SMBus via emulation.
	 * The core will route SMBus ops through i2c_smbus_xfer_emulated(),
	 * which we can service over the remote I2C message path.
	 */
	return I2C_FUNC_I2C
		| I2C_FUNC_SMBUS_EMUL
		| I2C_FUNC_SMBUS_QUICK
		| I2C_FUNC_SMBUS_BYTE
		| I2C_FUNC_SMBUS_BYTE_DATA
		| I2C_FUNC_SMBUS_WORD_DATA
		| I2C_FUNC_SMBUS_PROC_CALL
		| I2C_FUNC_SMBUS_BLOCK_DATA
		| I2C_FUNC_SMBUS_I2C_BLOCK;
}

/* No DT child auto-creation; devices will be created manually by userspace */

static int ri_host_probe(struct i2c_client *client)
{
	struct remote_host *h;
	int ret;

	/* Loud boot-time marker to confirm protocol version at runtime */
	dev_info(&client->dev, "virtual-mux-host: protocol v3\n");

	h = devm_kzalloc(&client->dev, sizeof(*h), GFP_KERNEL);
	if (!h)
		return -ENOMEM;

	h->dev = &client->dev;
	h->link_client = client;
	mutex_init(&h->xfer_lock);

	/* DT optional: use-crc16 */
	h->use_crc = of_property_read_bool(client->dev.of_node, "use-crc16");
	if (of_property_read_u32(client->dev.of_node, "xfer-poll-sleep-ms", &h->poll_sleep_ms))
		h->poll_sleep_ms = RI_DEFAULT_POLL_SLEEP_MS;
	if (of_property_read_u32(client->dev.of_node, "xfer-timeout-ms", &h->xfer_timeout_ms))
		h->xfer_timeout_ms = RI_DEFAULT_XFER_TIMEOUT_MS;
	if (of_property_read_u32(client->dev.of_node, "host-retry-max-ms",
				 &h->host_retry_max_ms))
		h->host_retry_max_ms = RI_DEFAULT_HOST_RETRY_MAX_MS;
	if (of_property_read_u32(client->dev.of_node, "host-retry-sleep-ms",
				 &h->host_retry_sleep_ms))
		h->host_retry_sleep_ms = RI_DEFAULT_HOST_RETRY_SLEEP_MS;
	/* Clamp DT values to a sane ceiling before applying relationships */
	if (h->poll_sleep_ms > RI_DT_TIMING_MAX_MS)
		h->poll_sleep_ms = RI_DT_TIMING_MAX_MS;
	if (h->xfer_timeout_ms > RI_DT_TIMING_MAX_MS)
		h->xfer_timeout_ms = RI_DT_TIMING_MAX_MS;
	if (h->host_retry_max_ms > RI_DT_TIMING_MAX_MS)
		h->host_retry_max_ms = RI_DT_TIMING_MAX_MS;
	if (h->host_retry_sleep_ms > RI_DT_TIMING_MAX_MS)
		h->host_retry_sleep_ms = RI_DT_TIMING_MAX_MS;
	if (h->poll_sleep_ms == 0)
		h->poll_sleep_ms = RI_DEFAULT_POLL_SLEEP_MS;
	if (h->xfer_timeout_ms < h->poll_sleep_ms)
		h->xfer_timeout_ms = h->poll_sleep_ms;
	if (h->host_retry_sleep_ms == 0)
		h->host_retry_sleep_ms = RI_DEFAULT_HOST_RETRY_SLEEP_MS;
	if (h->host_retry_max_ms < h->xfer_timeout_ms)
		h->host_retry_max_ms = h->xfer_timeout_ms;
	dev_dbg(&client->dev, "host probe: addr=%02x use-crc16=%d\n",
		client->addr, h->use_crc);

	/* Virtual adapter setup */
	h->algo.master_xfer = remote_master_xfer;
	h->algo.functionality = remote_func;

	h->virt_adap.owner = THIS_MODULE;
	h->virt_adap.algo = &h->algo;
	h->virt_adap.dev.parent = &client->dev;
	h->virt_adap.dev.of_node = client->dev.of_node;

	snprintf(h->virt_adap.name, sizeof(h->virt_adap.name),
		 "virtual-mux-%s", dev_name(&client->dev));

	i2c_set_adapdata(&h->virt_adap, h);

	/* Optional fixed bus number: virtual-bus-num = <12>; */
	if (!of_property_read_u32(client->dev.of_node, "virtual-bus-num", &h->adapter_nr)) {
		/* Ensure the numbered adapter uses the requested bus number */
		h->virt_adap.nr = h->adapter_nr;
		ret = i2c_add_numbered_adapter(&h->virt_adap);
	} else {
		ret = i2c_add_adapter(&h->virt_adap);
	}

	if (ret) {
		dev_err(&client->dev, "failed to add virtual adapter: %d\n", ret);
		return ret;
	}
	dev_dbg(&client->dev, "host: virtual adapter created nr=%d\n", h->virt_adap.nr);

	i2c_set_clientdata(client, h);

	dev_info(&client->dev,
		 "virtual-mux-host registered as i2c-%d crc=%d xfer-timeout-ms=%u poll-sleep-ms=%u host-retry-max-ms=%u retry-sleep-ms=%u\n",
		 h->virt_adap.nr, h->use_crc,
		 h->xfer_timeout_ms, h->poll_sleep_ms,
		 h->host_retry_max_ms, h->host_retry_sleep_ms);

	return 0;
}

static void ri_host_remove(struct i2c_client *client)
{
	struct remote_host *h = i2c_get_clientdata(client);

	i2c_del_adapter(&h->virt_adap);
}

static const struct of_device_id ri_host_of_match[] = {
	{ .compatible = "virtual-mux-host" },
	{ }
};
MODULE_DEVICE_TABLE(of, ri_host_of_match);

/* Allow manual binding via sysfs new_device */
static const struct i2c_device_id ri_host_id[] = {
	{ "virtual-mux-host", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ri_host_id);

static struct i2c_driver ri_host_driver = {
	.driver = {
		.name = DRV_NAME,
		.of_match_table = ri_host_of_match,
	},
	.id_table = ri_host_id,
	.probe = ri_host_probe,
	.remove = ri_host_remove,
};

module_i2c_driver(ri_host_driver);

MODULE_AUTHOR("Radivoje (Ogi) Jovanovic");
MODULE_DESCRIPTION("Virtual I2C Mux Host (virtual adapter forwarding over I2C)");
MODULE_LICENSE("GPL");

