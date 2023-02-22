// SPDX-License-Identifier: GPL-2.0-only
/*
 * The driver for BMC side of SSIF interface
 *
 * Copyright (c) 2022, Ampere Computing LLC
 *
 */

#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/ipmi_ssif_bmc.h>
#include <linux/kfifo.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>

#include <linux/aspeed-2600-ara.h>
#include <linux/i2c-aspeed.h>

#define DEVICE_NAME                             "ipmi-ssif-host"
#ifdef CONFIG_SEPARATE_SSIF_POSTCODES
#define DEVICE_NAME_POST                        "ipmi-ssif-postcodes"
#define POST_CODE_SIZE 9
#define POST_CODE_OFFSET 3
#endif //CONFIG_SEPARATE_SSIF_POSTCODES

#define GET_8BIT_ADDR(addr_7bit)                (((addr_7bit) << 1) & 0xff)

/* A standard SMBus Transaction is limited to 32 data bytes */
#define MAX_PAYLOAD_PER_TRANSACTION             32
/* Transaction includes the address, the command, the length and the PEC byte */
#define MAX_TRANSACTION                         (MAX_PAYLOAD_PER_TRANSACTION + 4)

#define MAX_IPMI_DATA_PER_START_TRANSACTION     30
#define MAX_IPMI_DATA_PER_MIDDLE_TRANSACTION    31

#define SSIF_IPMI_SINGLEPART_WRITE              0x2
#define SSIF_IPMI_SINGLEPART_READ               0x3
#define SSIF_IPMI_MULTIPART_WRITE_START         0x6
#define SSIF_IPMI_MULTIPART_WRITE_MIDDLE        0x7
#define SSIF_IPMI_MULTIPART_WRITE_END           0x8
#define SSIF_IPMI_MULTIPART_READ_START          0x3
#define SSIF_IPMI_MULTIPART_READ_MIDDLE         0x9

#define BUFFER_SIZE 1024

struct ssif_part_buffer {
	u8 address;
	u8 smbus_cmd;
	u8 length;
	u8 payload[MAX_PAYLOAD_PER_TRANSACTION];
	u8 pec;
	u8 index;
};

/*
 * SSIF internal states:
 *   SSIF_READY         0x00 : Ready state
 *   SSIF_START         0x01 : Start smbus transaction
 *   SSIF_SMBUS_CMD     0x02 : Received SMBus command
 *   SSIF_REQ_RECVING   0x03 : Receiving request
 *   SSIF_RES_SENDING   0x04 : Sending response
 *   SSIF_ABORTING      0x05 : Aborting state
 */
enum ssif_state {
	SSIF_READY,
	SSIF_START,
	SSIF_SMBUS_CMD,
	SSIF_REQ_RECVING,
	SSIF_RES_SENDING,
	SSIF_ABORTING,
	SSIF_STATE_MAX
};

struct ssif_bmc_ctx {
	struct i2c_client       *client;
	struct miscdevice       miscdev;
	int                     msg_idx;
	bool                    pec_support;
	/* ssif bmc spinlock */
	spinlock_t              lock_rd;
	spinlock_t              lock_wr;
	wait_queue_head_t       wait_queue_rd;
	u8                      running;
	enum ssif_state         state;
	/* Timeout waiting for response */
	struct timer_list       response_timer;
	/* Flag to identify a Multi-part Read Transaction */
	bool                    is_singlepart_read;
	u8                      nbytes_processed;
	u8                      remain_len;
	u8                      recv_len;
	/* Block Number of a Multi-part Read Transaction */
	u8                      block_num;
	bool                    aborting;
	/* Buffer for SSIF Transaction part*/
	struct ssif_part_buffer part_buf;
	struct ipmi_ssif_msg    response;
	struct ipmi_ssif_msg    request;
	struct kfifo fifo;
	u8 msg_count;
	ktime_t msg_time;
	u32 timeout;
	unsigned long response_timeout;
	struct gpio_desc *alert;
	u32 pulse_width;
#ifdef CONFIG_SEPARATE_SSIF_POSTCODES
	struct miscdevice       miscdev_post;
	spinlock_t              lock_post_rd;
	wait_queue_head_t       wait_queue_post_rd;
	u8                      running_post;
	struct kfifo fifo_post;
#endif //CONFIG_SEPARATE_SSIF_POSTCODES
	struct ast2600_ara *ara;
};

void disable_ast2600_slave(struct i2c_client *client)
{
	u32 addr_reg_val;
	struct aspeed_i2c_bus *bus;

	if (!client)
		return;

	bus = i2c_get_adapdata(client->adapter);
	addr_reg_val = readl(bus->base + ASPEED_I2C_DEV_ADDR_REG);
	if ((addr_reg_val & 0x7F) == client->addr)
	{//THIS BIT CANNOT BE DISABLED WITH OLD REGISTER MODE
		addr_reg_val &= 0xFFFFFF7F;
		writel(addr_reg_val, bus->base + ASPEED_I2C_DEV_ADDR_REG);
	}
	if (((addr_reg_val >> 8) & 0x7F) == client->addr)
	{
		addr_reg_val &= 0xFFFF7FFF;
		writel(addr_reg_val, bus->base + ASPEED_I2C_DEV_ADDR_REG);
	}
	if (((addr_reg_val >> 16) & 0x7F) == client->addr)
	{
		addr_reg_val &= 0xFF7FFFFF;
		writel(addr_reg_val, bus->base + ASPEED_I2C_DEV_ADDR_REG);
	}
}

void enable_ast2600_slave(struct i2c_client *client)
{
	u32 addr_reg_val;
	struct aspeed_i2c_bus *bus;

	if (!client)
		return;

	bus = i2c_get_adapdata(client->adapter);
	addr_reg_val = readl(bus->base + ASPEED_I2C_DEV_ADDR_REG);
	if ((addr_reg_val & 0x7F) == client->addr)
	{
		addr_reg_val |= 0x80;
		writel(addr_reg_val, bus->base + ASPEED_I2C_DEV_ADDR_REG);
	}
	else if (((addr_reg_val >> 8) & 0x7F) == client->addr)
	{
		addr_reg_val |= 0x8000;
		writel(addr_reg_val, bus->base + ASPEED_I2C_DEV_ADDR_REG);
	}
	else if (((addr_reg_val >> 16) & 0x7F) == client->addr)
	{
		addr_reg_val |= 0x800000;
		writel(addr_reg_val, bus->base + ASPEED_I2C_DEV_ADDR_REG);
	}
}


static inline struct ssif_bmc_ctx *to_ssif_bmc(struct file *file)
{
	return container_of(file->private_data, struct ssif_bmc_ctx, miscdev);
}

static const char *state_to_string(enum ssif_state state)
{
	switch (state) {
	case SSIF_READY:
		return "SSIF_READY";
	case SSIF_START:
		return "SSIF_START";
	case SSIF_SMBUS_CMD:
		return "SSIF_SMBUS_CMD";
	case SSIF_REQ_RECVING:
		return "SSIF_REQ_RECVING";
	case SSIF_RES_SENDING:
		return "SSIF_RES_SENDING";
	case SSIF_ABORTING:
		return "SSIF_ABORTING";
	default:
		return "SSIF_STATE_UNKNOWN";
	}
}

static ssize_t ssif_bmc_read(struct file *file, char __user *buf, size_t count_in, loff_t *ppos)
{
	struct ssif_bmc_ctx *ssif_bmc = to_ssif_bmc(file);
	unsigned long flags;
	unsigned int count_out;
	ssize_t ret = 0;

	if (kfifo_is_empty(&ssif_bmc->fifo)) {
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;
		ret = wait_event_interruptible(ssif_bmc->wait_queue_rd, !kfifo_is_empty(&ssif_bmc->fifo));
		if (ret == -ERESTARTSYS)
			return ret; 
	}
	spin_lock_irqsave(&ssif_bmc->lock_rd, flags);
	ret = kfifo_to_user(&ssif_bmc->fifo, buf, count_in, &count_out);
	spin_unlock_irqrestore(&ssif_bmc->lock_rd, flags);
	return (ret < 0) ? ret : count_out;
}

#ifdef CONFIG_SEPARATE_SSIF_POSTCODES
static inline struct ssif_bmc_ctx *to_ssif_bmc_post(struct file *file)
{
	return container_of(file->private_data, struct ssif_bmc_ctx, miscdev_post);
}


static ssize_t ssif_bmc_read_post(struct file *file, char __user *buf, size_t count_in, loff_t *ppos)
{
	struct ssif_bmc_ctx *ssif_bmc = to_ssif_bmc_post(file);
	unsigned long flags;
	unsigned int count_out;
	ssize_t ret = 0;

	if (kfifo_is_empty(&ssif_bmc->fifo_post)) {
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;
		ret = wait_event_interruptible(ssif_bmc->wait_queue_post_rd, !kfifo_is_empty(&ssif_bmc->fifo_post));
		if (ret == -ERESTARTSYS)
			return ret;
	}
	spin_lock_irqsave(&ssif_bmc->lock_post_rd, flags);
	ret = kfifo_to_user(&ssif_bmc->fifo_post, buf, count_in, &count_out);
	spin_unlock_irqrestore(&ssif_bmc->lock_post_rd, flags);
	return (ret < 0) ? ret : count_out;
}

static int ssif_bmc_open_post(struct inode *inode, struct file *file)
{
	struct ssif_bmc_ctx *ssif_bmc = to_ssif_bmc_post(file);
	int ret = 0;

	if (!ssif_bmc->running_post)
		ssif_bmc->running_post = 1;
	else
		ret = -EBUSY;

	return ret;
}

static __poll_t ssif_bmc_poll_post(struct file *file, poll_table *wait)
{
	struct ssif_bmc_ctx *ssif_bmc = to_ssif_bmc_post(file);

	poll_wait(file, &ssif_bmc->wait_queue_post_rd, wait);
	if (!kfifo_is_empty(&ssif_bmc->fifo_post)) {
		return POLLIN | POLLRDNORM;
	}
	return 0;
}

static int ssif_bmc_release_post(struct inode *inode, struct file *file)
{
	struct ssif_bmc_ctx *ssif_bmc = to_ssif_bmc_post(file);

	ssif_bmc->running_post = 0;

	return 0;
}

static const struct file_operations ssif_bmc_post_fops = {
	.owner		= THIS_MODULE,
	.open		= ssif_bmc_open_post,
	.read		= ssif_bmc_read_post,
	.release	= ssif_bmc_release_post,
	.poll		= ssif_bmc_poll_post,
};

void send_post_code(struct ssif_bmc_ctx *ssif_bmc)
{
	unsigned long flags = 0;
	int rc = 0;

	if (kfifo_initialized(&ssif_bmc->fifo_post) && ssif_bmc->request.header.len >= (POST_CODE_SIZE + POST_CODE_OFFSET)) {
		ssize_t to_send = POST_CODE_SIZE;
		spin_lock_irqsave(&ssif_bmc->lock_post_rd, flags);
		if ((BUFFER_SIZE - kfifo_len(&ssif_bmc->fifo_post)) < to_send)
			kfifo_reset(&ssif_bmc->fifo_post);
		rc = kfifo_in(&ssif_bmc->fifo_post, &ssif_bmc->request.payload[POST_CODE_OFFSET], to_send);
		if (rc != to_send) {
			kfifo_reset(&ssif_bmc->fifo_post);
			spin_unlock_irqrestore(&ssif_bmc->lock_post_rd, flags);
			return;
		}
		spin_unlock_irqrestore(&ssif_bmc->lock_post_rd, flags);

		wake_up_all(&ssif_bmc->wait_queue_post_rd);
	}
}
#endif //CONFIG_SEPARATE_SSIF_POSTCODES

/* Handle SSIF message that is written by user */
static ssize_t ssif_bmc_write(struct file *file, const char __user *buf, size_t count,
			      loff_t *ppos)
{
	struct ssif_bmc_ctx *ssif_bmc = to_ssif_bmc(file);
	struct ipmi_ssif_msg msg;
	unsigned long flags;
	ssize_t ret;
	ktime_t delta;
	ktime_t now;
	s64 timer;

	if (count > sizeof(struct ipmi_ssif_msg))
		return -EINVAL;

	if (copy_from_user(&msg, buf, count))
		return -EFAULT;

	//the message count must match, otherwise it is wrong message
	//tell upper layers that all is fine even though the message was dropped by the user
	if (msg.header.msg_num != ssif_bmc->msg_count) {
		return count;
	}

	if (!msg.header.len || count < sizeof(struct ipmi_ssif_msg_header) + msg.header.len) {
		return -EINVAL;
	}
	now = ktime_get();
	delta = ktime_sub(now, ssif_bmc->msg_time);
	timer = ktime_to_ms(delta);
	if (timer > (s64)(ssif_bmc->timeout))
		return -EPERM;

	spin_lock_irqsave(&ssif_bmc->lock_wr, flags);
	memcpy(&ssif_bmc->response, &msg, count);
	ssif_bmc->is_singlepart_read = (msg.header.len <= MAX_PAYLOAD_PER_TRANSACTION);
	spin_unlock_irqrestore(&ssif_bmc->lock_wr, flags);

	del_timer_sync(&ssif_bmc->response_timer);
	enable_ast2600_slave(ssif_bmc->client);

	if (!IS_ERR(ssif_bmc->alert)) {
		enable_ast2600_ara(ssif_bmc->client);
		//if gpio is already asserted toggle it
		if (gpiod_get_value(ssif_bmc->alert))
		{
			gpiod_set_value(ssif_bmc->alert, 0);
			udelay(ssif_bmc->pulse_width);
		}

		gpiod_set_value(ssif_bmc->alert, 1);
	}

	return (ret < 0) ? ret : count;
}

static int ssif_bmc_open(struct inode *inode, struct file *file)
{
	struct ssif_bmc_ctx *ssif_bmc = to_ssif_bmc(file);
	int ret = 0;

	if (!ssif_bmc->running)
		ssif_bmc->running = 1;
	else
		ret = -EBUSY;

	return ret;
}

static __poll_t ssif_bmc_poll(struct file *file, poll_table *wait)
{
	struct ssif_bmc_ctx *ssif_bmc = to_ssif_bmc(file);

	poll_wait(file, &ssif_bmc->wait_queue_rd, wait);
	if (!kfifo_is_empty(&ssif_bmc->fifo)) {
		return POLLIN | POLLRDNORM;
	}
	return 0;
}

static int ssif_bmc_release(struct inode *inode, struct file *file)
{
	struct ssif_bmc_ctx *ssif_bmc = to_ssif_bmc(file);

	ssif_bmc->running = 0;

	return 0;
}

/*
 * System calls to device interface for user apps
 */
static const struct file_operations ssif_bmc_fops = {
	.owner		= THIS_MODULE,
	.open		= ssif_bmc_open,
	.read		= ssif_bmc_read,
	.write		= ssif_bmc_write,
	.release	= ssif_bmc_release,
	.poll		= ssif_bmc_poll,
};

/* Called with ssif_bmc->lock held. */
static void handle_request(struct ssif_bmc_ctx *ssif_bmc)
{
	unsigned long flags = 0;
	int rc = 0;
#ifdef CONFIG_SEPARATE_SSIF_POSTCODES
	int netfn = ssif_bmc->request.payload[0] >> 2;
	int cmd = ssif_bmc->request.payload[1];
	int group = ssif_bmc->request.payload[2];

	if (netfn == 0x2c && cmd == 0x2 && group == 0xAE)
		send_post_code(ssif_bmc);
	else
#endif //CONFIG_SEPARATE_SSIF_POSTCODES
	if (kfifo_initialized(&ssif_bmc->fifo)) {
		ssize_t to_send = sizeof(struct ipmi_ssif_msg_header) + ssif_bmc->request.header.len;
		if (to_send > BUFFER_SIZE) {
			return;
		}

		if ((BUFFER_SIZE - kfifo_len(&ssif_bmc->fifo)) < to_send)
			kfifo_reset(&ssif_bmc->fifo);

		spin_lock_irqsave(&ssif_bmc->lock_rd, flags);
		rc = kfifo_in(&ssif_bmc->fifo, &ssif_bmc->request.header,  sizeof(struct ipmi_ssif_msg_header));
		if (rc != sizeof(struct ipmi_ssif_msg_header)) {
			kfifo_reset(&ssif_bmc->fifo);
			spin_unlock_irqrestore(&ssif_bmc->lock_rd, flags);
			return;
		}
		to_send = min_t(ssize_t, ssif_bmc->request.header.len, IPMI_SSIF_PAYLOAD_MAX);
		rc = kfifo_in(&ssif_bmc->fifo, ssif_bmc->request.payload, to_send);
		spin_unlock_irqrestore(&ssif_bmc->lock_rd, flags);
		if (rc != to_send) {
			kfifo_reset(&ssif_bmc->fifo);
			return;
		}

		disable_ast2600_slave(ssif_bmc->client);
		mod_timer(&ssif_bmc->response_timer, jiffies + msecs_to_jiffies(ssif_bmc->response_timeout));

		memset(&ssif_bmc->response, 0, sizeof(struct ipmi_ssif_msg_header));
		ssif_bmc->msg_time = ktime_get();
		wake_up_all(&ssif_bmc->wait_queue_rd);
	}
}

static void calculate_response_part_pec(struct ssif_part_buffer *part)
{
	u8 addr = part->address;

	/* PEC - Start Read Address */
	part->pec = i2c_smbus_pec(0, &addr, 1);
	/* PEC - SSIF Command */
	part->pec = i2c_smbus_pec(part->pec, &part->smbus_cmd, 1);
	/* PEC - Restart Write Address */
	addr = addr | 0x01;
	part->pec = i2c_smbus_pec(part->pec, &addr, 1);
	part->pec = i2c_smbus_pec(part->pec, &part->length, 1);
	if (part->length)
		part->pec = i2c_smbus_pec(part->pec, part->payload, part->length);
}

static void set_singlepart_response_buffer(struct ssif_bmc_ctx *ssif_bmc)
{
	struct ssif_part_buffer *part = &ssif_bmc->part_buf;
	unsigned long flags;

	part->address = GET_8BIT_ADDR(ssif_bmc->client->addr);
	spin_lock_irqsave(&ssif_bmc->lock_wr, flags);
	part->length = (u8)ssif_bmc->response.header.len;

	/* Clear the rest to 0 */
	memset(part->payload + part->length, 0, MAX_PAYLOAD_PER_TRANSACTION - part->length);
	memcpy(&part->payload[0], &ssif_bmc->response.payload[0], part->length);
	spin_unlock_irqrestore(&ssif_bmc->lock_wr, flags);
}

static void set_multipart_response_buffer(struct ssif_bmc_ctx *ssif_bmc)
{
	struct ssif_part_buffer *part = &ssif_bmc->part_buf;
	unsigned long flags;
	u8 part_len = 0;

	part->address = GET_8BIT_ADDR(ssif_bmc->client->addr);
	switch (part->smbus_cmd) {
	case SSIF_IPMI_MULTIPART_READ_START:
		/*
		 * Read Start length is 32 bytes.
		 * Read Start transfer first 30 bytes of IPMI response
		 * and 2 special code 0x00, 0x01.
		 */
		ssif_bmc->nbytes_processed = 0;
		ssif_bmc->block_num = 0;
		part->length = MAX_PAYLOAD_PER_TRANSACTION;
		part_len = MAX_IPMI_DATA_PER_START_TRANSACTION;
		spin_lock_irqsave(&ssif_bmc->lock_wr, flags);
		ssif_bmc->remain_len = ssif_bmc->response.header.len - part_len;

		part->payload[0] = 0x00; /* Start Flag */
		part->payload[1] = 0x01; /* Start Flag */

		memcpy(&part->payload[2], &ssif_bmc->response.payload[0], part_len);
		spin_unlock_irqrestore(&ssif_bmc->lock_wr, flags);
		break;

	case SSIF_IPMI_MULTIPART_READ_MIDDLE:
		/*
		 * IPMI READ Middle or READ End messages can carry up to 31 bytes
		 * IPMI data plus block number byte.
		 */
		if (ssif_bmc->remain_len <= MAX_IPMI_DATA_PER_MIDDLE_TRANSACTION) {
			/*
			 * This is READ End message
			 *  Return length is the remaining response data length
			 *  plus block number
			 *  Block number 0xFF is to indicate this is last message
			 *
			 */
			/* Clean the buffer */
			memset(&part->payload[0], 0, MAX_PAYLOAD_PER_TRANSACTION);
			part->length = ssif_bmc->remain_len + 1;
			part_len = ssif_bmc->remain_len;
			ssif_bmc->block_num = 0xFF;
			part->payload[0] = ssif_bmc->block_num;
		} else {
			/*
			 * This is READ Middle message
			 *  Response length is the maximum SMBUS transfer length
			 *  Block number byte is incremented
			 * Return length is maximum SMBUS transfer length
			 */
			part->length = MAX_PAYLOAD_PER_TRANSACTION;
			part_len = MAX_IPMI_DATA_PER_MIDDLE_TRANSACTION;
			part->payload[0] = ssif_bmc->block_num;
			ssif_bmc->block_num++;
		}

		ssif_bmc->remain_len -= part_len;
		spin_lock_irqsave(&ssif_bmc->lock_wr, flags);
		memcpy(&part->payload[1], ssif_bmc->response.payload + ssif_bmc->nbytes_processed,
		       part_len);
		spin_unlock_irqrestore(&ssif_bmc->lock_wr, flags);
		break;

	default:
		/* Do not expect to go to this case */
		dev_err(&ssif_bmc->client->dev, "%s: Unexpected SMBus command 0x%x\n",
			__func__, part->smbus_cmd);
		break;
	}

	ssif_bmc->nbytes_processed += part_len;
}

static bool supported_read_cmd(u8 cmd)
{
	if (cmd == SSIF_IPMI_SINGLEPART_READ ||
	    cmd == SSIF_IPMI_MULTIPART_READ_START ||
	    cmd == SSIF_IPMI_MULTIPART_READ_MIDDLE)
		return true;

	return false;
}

static bool supported_write_cmd(u8 cmd)
{
	if (cmd == SSIF_IPMI_SINGLEPART_WRITE ||
	    cmd == SSIF_IPMI_MULTIPART_WRITE_START ||
	    cmd == SSIF_IPMI_MULTIPART_WRITE_MIDDLE ||
	    cmd == SSIF_IPMI_MULTIPART_WRITE_END)
		return true;

	return false;
}

/* Process the IPMI response that will be read by master */
static void handle_read_processed(struct ssif_bmc_ctx *ssif_bmc, u8 *val)
{
	struct ssif_part_buffer *part = &ssif_bmc->part_buf;

	/* msg_idx start from 0 */
	if (part->index < part->length)
		*val = part->payload[part->index];
	else if (part->index == part->length && ssif_bmc->pec_support)
		*val = part->pec;
	else
		*val = 0;

	part->index++;
}

static void handle_write_received(struct ssif_bmc_ctx *ssif_bmc, u8 *val)
{
	/*
	 * The msg_idx must be 1 when first enter SSIF_REQ_RECVING state
	 * And it would never exceeded 36 bytes included the 32 bytes max payload +
	 * the address + the command + the len and the PEC.
	 */
	if (ssif_bmc->msg_idx < 1  || ssif_bmc->msg_idx > MAX_TRANSACTION)
		return;

	if (ssif_bmc->msg_idx == 1) {
		ssif_bmc->part_buf.length = *val;
		ssif_bmc->part_buf.index = 0;
	} else {
		ssif_bmc->part_buf.payload[ssif_bmc->part_buf.index++] = *val;
	}

	ssif_bmc->msg_idx++;
}

static bool validate_request_part(struct ssif_bmc_ctx *ssif_bmc)
{
	struct ssif_part_buffer *part = &ssif_bmc->part_buf;
	bool ret = true;
	u8 cpec;
	u8 addr;

	if (part->index == part->length) {
		/* PEC is not included */
		ssif_bmc->pec_support = false;
		ret = true;
		goto exit;
	}

	if (part->index != part->length + 1) {
		ret = false;
		goto exit;
	}

	/* PEC is included */
	ssif_bmc->pec_support = true;
	part->pec = part->payload[part->length];
	addr = GET_8BIT_ADDR(ssif_bmc->client->addr);
	cpec = i2c_smbus_pec(0, &addr, 1);
	cpec = i2c_smbus_pec(cpec, &part->smbus_cmd, 1);
	cpec = i2c_smbus_pec(cpec, &part->length, 1);
	/*
	 * As SMBus specification does not allow the length
	 * (byte count) in the Write-Block protocol to be zero.
	 * Therefore, it is illegal to have the last Middle
	 * transaction in the sequence carry 32-byte and have
	 * a length of ‘0’ in the End transaction.
	 * But some users may try to use this way and we should
	 * prevent ssif_bmc driver broken in this case.
	 */
	if (part->length)
		cpec = i2c_smbus_pec(cpec, part->payload, part->length);

	if (cpec != part->pec)
		ret = false;

exit:
	return ret;
}

static void process_request_part(struct ssif_bmc_ctx *ssif_bmc)
{
	struct ssif_part_buffer *part = &ssif_bmc->part_buf;
	unsigned int len;

	switch (part->smbus_cmd) {
	case SSIF_IPMI_SINGLEPART_WRITE:
		/* save the whole part to request*/
		ssif_bmc->request.header.len = part->length;
		ssif_bmc->request.header.msg_num = ssif_bmc->msg_count;
		memcpy(ssif_bmc->request.payload, part->payload, part->length);

		break;
	case SSIF_IPMI_MULTIPART_WRITE_START:
		ssif_bmc->request.header.len = 0;
		ssif_bmc->request.header.msg_num = ssif_bmc->msg_count;

		fallthrough;
	case SSIF_IPMI_MULTIPART_WRITE_MIDDLE:
	case SSIF_IPMI_MULTIPART_WRITE_END:
		len = ssif_bmc->request.header.len + part->length;
		/* Do the bound check here, not allow the request len exceed 254 bytes */
		if (len > IPMI_SSIF_PAYLOAD_MAX) {
			dev_warn(&ssif_bmc->client->dev,
				 "Warn: Request exceeded 254 bytes, aborting");
			/* Request too long, aborting */
			ssif_bmc->aborting =  true;
		} else {
			memcpy(ssif_bmc->request.payload + ssif_bmc->request.header.len,
			       part->payload, part->length);
			ssif_bmc->request.header.len += part->length;
		}
		break;
	default:
		/* Do not expect to go to this case */
		dev_err(&ssif_bmc->client->dev, "%s: Unexpected SMBus command 0x%x\n",
			__func__, part->smbus_cmd);
		break;
	}
}

static void process_smbus_cmd(struct ssif_bmc_ctx *ssif_bmc, u8 *val)
{
	/* SMBUS command can vary (single or multi-part) */
	ssif_bmc->part_buf.smbus_cmd = *val;
	ssif_bmc->msg_idx = 1;
	memset(&ssif_bmc->part_buf.payload[0], 0, MAX_PAYLOAD_PER_TRANSACTION);

	if (*val == SSIF_IPMI_SINGLEPART_WRITE || *val == SSIF_IPMI_MULTIPART_WRITE_START) {
		if (!IS_ERR(ssif_bmc->alert))
			gpiod_set_value(ssif_bmc->alert, 0);
		/* This is new request, flip aborting flag if set */
		if (ssif_bmc->aborting)
			ssif_bmc->aborting = false;
		ssif_bmc->msg_count++;
	}
}

static void on_read_requested_event(struct ssif_bmc_ctx *ssif_bmc, u8 *val)
{
	if (ssif_bmc->state == SSIF_READY ||
	    ssif_bmc->state == SSIF_START ||
	    ssif_bmc->state == SSIF_REQ_RECVING ||
	    ssif_bmc->state == SSIF_RES_SENDING) {
		dev_warn(&ssif_bmc->client->dev,
			 "Warn: %s unexpected READ REQUESTED in state=%s\n",
			 __func__, state_to_string(ssif_bmc->state));
		ssif_bmc->state = SSIF_ABORTING;
		*val = 0;
		return;

	} else if (ssif_bmc->state == SSIF_SMBUS_CMD) {
		if (!supported_read_cmd(ssif_bmc->part_buf.smbus_cmd)) {
			dev_warn(&ssif_bmc->client->dev, "Warn: Unknown SMBus read command=0x%x",
				 ssif_bmc->part_buf.smbus_cmd);
			ssif_bmc->aborting = true;
		}

		if (ssif_bmc->aborting)
			ssif_bmc->state = SSIF_ABORTING;
		else
			ssif_bmc->state = SSIF_RES_SENDING;
	}

	ssif_bmc->msg_idx = 0;

	/* Send 0 if there is nothing to send */
	if (ssif_bmc->state == SSIF_ABORTING) {
		*val = 0;
		return;
	}

	if (ssif_bmc->is_singlepart_read)
		set_singlepart_response_buffer(ssif_bmc);
	else
		set_multipart_response_buffer(ssif_bmc);

	calculate_response_part_pec(&ssif_bmc->part_buf);
	ssif_bmc->part_buf.index = 0;
	*val = ssif_bmc->part_buf.length;
	if (!IS_ERR(ssif_bmc->alert))
		gpiod_set_value(ssif_bmc->alert, 0);
}

static void on_read_processed_event(struct ssif_bmc_ctx *ssif_bmc, u8 *val)
{
	if (ssif_bmc->state == SSIF_READY ||
	    ssif_bmc->state == SSIF_START ||
	    ssif_bmc->state == SSIF_REQ_RECVING ||
	    ssif_bmc->state == SSIF_SMBUS_CMD) {
		dev_warn(&ssif_bmc->client->dev,
			 "Warn: %s unexpected READ PROCESSED in state=%s\n",
			 __func__, state_to_string(ssif_bmc->state));
		ssif_bmc->state = SSIF_ABORTING;
		*val = 0;
		return;
	}

	/* Send 0 if there is nothing to send */
	if (ssif_bmc->state == SSIF_ABORTING) {
		*val = 0;
		return;
	}

	handle_read_processed(ssif_bmc, val);
}

static void on_write_requested_event(struct ssif_bmc_ctx *ssif_bmc, u8 *val)
{
	if (ssif_bmc->state == SSIF_READY || ssif_bmc->state == SSIF_SMBUS_CMD) {
		ssif_bmc->state = SSIF_START;

	} else if (ssif_bmc->state == SSIF_START ||
		   ssif_bmc->state == SSIF_REQ_RECVING ||
		   ssif_bmc->state == SSIF_RES_SENDING) {
		dev_warn(&ssif_bmc->client->dev,
			 "Warn: %s unexpected WRITE REQUEST in state=%s\n",
			 __func__, state_to_string(ssif_bmc->state));
		ssif_bmc->state = SSIF_ABORTING;
		return;
	}

	ssif_bmc->msg_idx = 0;
	ssif_bmc->part_buf.address = *val;
}

static void on_write_received_event(struct ssif_bmc_ctx *ssif_bmc, u8 *val)
{
	if (ssif_bmc->state == SSIF_READY ||
	    ssif_bmc->state == SSIF_RES_SENDING) {
		dev_warn(&ssif_bmc->client->dev,
			 "Warn: %s unexpected WRITE RECEIVED in state=%s\n",
			 __func__, state_to_string(ssif_bmc->state));
		ssif_bmc->state = SSIF_ABORTING;

	} else if (ssif_bmc->state == SSIF_START) {
		ssif_bmc->state = SSIF_SMBUS_CMD;

	} else if (ssif_bmc->state == SSIF_SMBUS_CMD) {
		if (!supported_write_cmd(ssif_bmc->part_buf.smbus_cmd)) {
			dev_warn(&ssif_bmc->client->dev, "Warn: Unknown SMBus write command=0x%x",
				 ssif_bmc->part_buf.smbus_cmd);
			ssif_bmc->aborting = true;
		}

		if (ssif_bmc->aborting)
			ssif_bmc->state = SSIF_ABORTING;
		else
			ssif_bmc->state = SSIF_REQ_RECVING;
	}

	/* This is response sending state */
	if (ssif_bmc->state == SSIF_REQ_RECVING)
		handle_write_received(ssif_bmc, val);
	else if (ssif_bmc->state == SSIF_SMBUS_CMD)
		process_smbus_cmd(ssif_bmc, val);
}

static void on_stop_event(struct ssif_bmc_ctx *ssif_bmc, u8 *val)
{
	if (ssif_bmc->state == SSIF_READY ||
	    ssif_bmc->state == SSIF_START ||
	    ssif_bmc->state == SSIF_SMBUS_CMD ||
	    ssif_bmc->state == SSIF_ABORTING) {
		dev_warn(&ssif_bmc->client->dev,
			 "Warn: %s unexpected SLAVE STOP in state=%s\n",
			 __func__, state_to_string(ssif_bmc->state));
		ssif_bmc->state = SSIF_READY;

	} else if (ssif_bmc->state == SSIF_REQ_RECVING) {
		if (validate_request_part(ssif_bmc)) {
			process_request_part(ssif_bmc);
			if (ssif_bmc->part_buf.smbus_cmd == SSIF_IPMI_SINGLEPART_WRITE ||
			    ssif_bmc->part_buf.smbus_cmd == SSIF_IPMI_MULTIPART_WRITE_END)
				handle_request(ssif_bmc);
			ssif_bmc->state = SSIF_READY;
		} else {
			/*
			 * A BMC that receives an invalid request drop the data for the write
			 * transaction and any further transactions (read or write) until
			 * the next valid read or write Start transaction is received
			 */
			dev_err(&ssif_bmc->client->dev, "Error: invalid pec\n");
			ssif_bmc->aborting = true;
		}
	} else if (ssif_bmc->state == SSIF_RES_SENDING) {
		ssif_bmc->state = SSIF_READY;
	}

	/* Reset message index */
	ssif_bmc->msg_idx = 0;
}

/*
 * Callback function to handle I2C slave events
 */
static int ssif_bmc_cb(struct i2c_client *client, enum i2c_slave_event event, u8 *val)
{
	struct ssif_bmc_ctx *ssif_bmc = i2c_get_clientdata(client);
	int ret = 0;

	switch (event) {
	case I2C_SLAVE_READ_REQUESTED:
		on_read_requested_event(ssif_bmc, val);
		break;

	case I2C_SLAVE_WRITE_REQUESTED:
		on_write_requested_event(ssif_bmc, val);
		break;

	case I2C_SLAVE_READ_PROCESSED:
		on_read_processed_event(ssif_bmc, val);
		break;

	case I2C_SLAVE_WRITE_RECEIVED:
		on_write_received_event(ssif_bmc, val);
		break;

	case I2C_SLAVE_STOP:
		on_stop_event(ssif_bmc, val);
		break;

	default:
		dev_warn(&ssif_bmc->client->dev, "Warn: Unknown i2c slave event\n");
		break;
	}

	return ret;
}

static void retry_timeout(struct timer_list *t)
{
	struct ssif_bmc_ctx *ssif_bmc = from_timer(ssif_bmc, t, response_timer);

	dev_warn(&ssif_bmc->client->dev, "Userspace did not respond in time. Force enable i2c target\n");
	enable_ast2600_slave(ssif_bmc->client);
}

static int ssif_bmc_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ssif_bmc_ctx *ssif_bmc;
	int ret;

	ssif_bmc = devm_kzalloc(&client->dev, sizeof(*ssif_bmc), GFP_KERNEL);
	if (!ssif_bmc)
		return -ENOMEM;

	/* Request GPIO for alerting the host that response is ready */
	ssif_bmc->alert = devm_gpiod_get(&client->dev, "alert", GPIOD_OUT_LOW);

	if (of_property_read_u32(client->dev.of_node, "timeout_ms", &ssif_bmc->timeout))
		ssif_bmc->timeout = 250;
	if (of_property_read_u32(client->dev.of_node, "pulse_width_us", &ssif_bmc->pulse_width))
		ssif_bmc->pulse_width = 5;

	ssif_bmc->response_timeout = ssif_bmc->timeout + 5;

	ret = kfifo_alloc(&ssif_bmc->fifo, BUFFER_SIZE, GFP_KERNEL);
	if (ret)
		return ret;

	ssif_bmc->msg_count = 0;
	ssif_bmc->running = 0;
	spin_lock_init(&ssif_bmc->lock_rd);
	spin_lock_init(&ssif_bmc->lock_wr);

	init_waitqueue_head(&ssif_bmc->wait_queue_rd);

	/* Register misc device interface */
	ssif_bmc->miscdev.minor = MISC_DYNAMIC_MINOR;
	ssif_bmc->miscdev.name = DEVICE_NAME;
	ssif_bmc->miscdev.fops = &ssif_bmc_fops;
	ssif_bmc->miscdev.parent = &client->dev;
	ret = misc_register(&ssif_bmc->miscdev);
	if (ret)
		return ret;

#ifdef CONFIG_SEPARATE_SSIF_POSTCODES
	ssif_bmc->running_post = 0;
	spin_lock_init(&ssif_bmc->lock_post_rd);
	init_waitqueue_head(&ssif_bmc->wait_queue_post_rd);
	ret = kfifo_alloc(&ssif_bmc->fifo_post, BUFFER_SIZE, GFP_KERNEL);
	if (ret) {
		misc_deregister(&ssif_bmc->miscdev);
		return ret;
	}
	ssif_bmc->miscdev_post.minor = MISC_DYNAMIC_MINOR;
	ssif_bmc->miscdev_post.name = DEVICE_NAME_POST;
	ssif_bmc->miscdev_post.fops = &ssif_bmc_post_fops;
	ssif_bmc->miscdev_post.parent = &client->dev;
	ret = misc_register(&ssif_bmc->miscdev_post);
	if (ret) {
		misc_deregister(&ssif_bmc->miscdev);
		return ret;
	}
#endif //CONFIG_SEPARATE_SSIF_POSTCODES
	ssif_bmc->client = client;
	ssif_bmc->client->flags |= I2C_CLIENT_SLAVE;

	timer_setup(&ssif_bmc->response_timer, retry_timeout, 0);
	/* Register I2C slave */
	i2c_set_clientdata(client, ssif_bmc);
	ret = i2c_slave_register(client, ssif_bmc_cb);
	if (ret) {
		misc_deregister(&ssif_bmc->miscdev);
#ifdef CONFIG_SEPARATE_SSIF_POSTCODES
		misc_deregister(&ssif_bmc->miscdev_post);
#endif //CONFIG_SEPARATE_SSIF_POSTCODES
	}
	ssif_bmc->ara = register_ast2600_ara(client);

	gpiod_set_value(ssif_bmc->alert, 0);
	return ret;
}

static int ssif_bmc_remove(struct i2c_client *client)
{
	struct ssif_bmc_ctx *ssif_bmc = i2c_get_clientdata(client);

	unregister_ast2600_ara(ssif_bmc->ara);
	kfifo_free(&ssif_bmc->fifo);
	i2c_slave_unregister(client);
	misc_deregister(&ssif_bmc->miscdev);
#ifdef CONFIG_SEPARATE_SSIF_POSTCODES
	kfifo_free(&ssif_bmc->fifo_post);
	misc_deregister(&ssif_bmc->miscdev_post);
#endif //CONFIG_SEPARATE_SSIF_POSTCODES

	return 0;
}

static const struct of_device_id ssif_bmc_match[] = {
	{ .compatible = "ssif-bmc" },
	{ },
};
MODULE_DEVICE_TABLE(of, ssif_bmc_match);

static const struct i2c_device_id ssif_bmc_id[] = {
	{ DEVICE_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, ssif_bmc_id);

static struct i2c_driver ssif_bmc_driver = {
	.driver         = {
		.name           = DEVICE_NAME,
		.of_match_table = ssif_bmc_match,
	},
	.probe          = ssif_bmc_probe,
	.remove         = ssif_bmc_remove,
	.id_table       = ssif_bmc_id,
};

module_i2c_driver(ssif_bmc_driver);

MODULE_AUTHOR("Quan Nguyen <quan@os.amperecomputing.com>");
MODULE_AUTHOR("Chuong Tran <chuong@os.amperecomputing.com>");
MODULE_DESCRIPTION("Linux device driver of the BMC IPMI SSIF interface.");
MODULE_LICENSE("GPL");
