// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2023 Aspeed Technology Inc.
 */

#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/module.h>
#include <linux/asn1_decoder.h>
#include <linux/scatterlist.h>
#include <linux/iopoll.h>
#include <linux/interrupt.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/vmalloc.h>
#include <crypto/ecdh.h>
#include <crypto/internal/sig.h>
#include <crypto/sha2.h>

#include "aspeed-ecdsa.h"

//#define ASPEED_ECDSA_IRQ_MODE

static int aspeed_ecdsa_complete(struct aspeed_ecdsa_dev *ecdsa_dev);

/************************************************************************/
/*                   Aspeed's ECDSA driver utility                      */
/************************************************************************/
static int aspeed_ecdsa_self_test(struct aspeed_ecdsa_dev *ecdsa_dev)
{
	u32 val;

	ast_write(ecdsa_dev, ECC_EN, ASPEED_ECC_CTRL_REG);
	val = ast_read(ecdsa_dev, ASPEED_ECC_CTRL_REG);
	if (val != ECC_EN)
		return -EIO;

	ast_write(ecdsa_dev, 0x0, ASPEED_ECC_CTRL_REG);
	val = ast_read(ecdsa_dev, ASPEED_ECC_CTRL_REG);
	if (val)
		return -EIO;

	return 0;
}

#ifdef CONFIG_CRYPTO_DEV_ASPEED_DEBUG
static void hexdump(const char *name, unsigned char *buf, unsigned int len)
{
	pr_info("%s\n", name);
	print_hex_dump(KERN_CONT, "", DUMP_PREFIX_OFFSET,
		       16, 1,
		       buf, len, false);
}
#else
static void hexdump(const char *name, unsigned char *buf, unsigned int len)
{
	/* empty */
}
#endif

static void buff_reverse(u8 *dst, u8 *src, int len)
{
	for (int i = 0; i < len; i++)
		dst[len - i - 1] = src[i];
}

static bool aspeed_ecdsa_need_fallback(struct aspeed_ecc_ctx *ctx, int d_len)
{
	int curve_id = ctx->curve_id;

	switch (curve_id) {
	case ECC_CURVE_NIST_P256:
		if (d_len != SHA256_DIGEST_SIZE)
			return true;
		break;
	case ECC_CURVE_NIST_P384:
		if (d_len != SHA384_DIGEST_SIZE)
			return true;
		break;
	}

	return false;
}

/************************************************************************/
/*                 Aspeed's ECDSA hardware operation                    */
/************************************************************************/
static void aspeed_ecdsa_done_task(struct aspeed_ecdsa_dev *ecdsa_dev)
{
	u32 ctrl;

	AST_DBG(ecdsa_dev, "\n");

	/* Reset engine */
	ctrl = ast_read(ecdsa_dev, ASPEED_ECC_CTRL_REG);
	ast_write(ecdsa_dev, 0, ASPEED_ECC_CTRL_REG);

	/* Memory barrier to ensure ecc ctrl is reset. */
	mb();
	ast_write(ecdsa_dev, ctrl, ASPEED_ECC_CTRL_REG);

	aspeed_ecdsa_complete(ecdsa_dev);
}

#ifndef ASPEED_ECDSA_IRQ_MODE
static int aspeed_ecdsa_wait_complete(struct aspeed_ecdsa_dev *ecdsa_dev)
{
	struct aspeed_engine_ecdsa *ecdsa_engine = &ecdsa_dev->ecdsa_engine;
	u32 sts;
	int ret;

	ret = readl_poll_timeout(ecdsa_dev->regs + ASPEED_ECC_STS_REG, sts,
				 ((sts & ECC_IDLE) == ECC_IDLE),
				 ASPEED_ECC_POLLING_TIME,
				 ASPEED_ECC_TIMEOUT * 10);
	if (ret) {
		dev_err(ecdsa_dev->dev, "ECC engine wrong status\n");
		return -EIO;
	}

	sts = ast_read(ecdsa_dev, ASPEED_ECC_STS_REG) & ECC_VERIFY_PASS;
	if (sts == ECC_VERIFY_PASS) {
		ecdsa_engine->results = 0;
		AST_DBG(ecdsa_dev, "Verify PASS !\n");

	} else {
		ecdsa_engine->results = -EKEYREJECTED;
		AST_DBG(ecdsa_dev, "Verify FAILED !\n");
	}

	/* Stop ECDSA engine */
	if (ecdsa_engine->flags & CRYPTO_FLAGS_BUSY)
		aspeed_ecdsa_done_task(ecdsa_dev);
	else
		dev_err(ecdsa_dev->dev, "ECDSA no active requests.\n");

	return ecdsa_engine->results;
}
#else
/* ecdsa interrupt service routine. */
static irqreturn_t aspeed_ecdsa_irq(int irq, void *dev)
{
	struct aspeed_ecdsa_dev *ecdsa_dev = (struct aspeed_ecdsa_dev *)dev;
	struct aspeed_engine_ecdsa *ecdsa_engine = &ecdsa_dev->ecdsa_engine;
	u32 sts;

	sts = ast_read(ecdsa_dev, ASPEED_ECC_INT_STS);
	ast_write(ecdsa_dev, sts, ASPEED_ECC_INT_STS);

	AST_DBG(ecdsa_dev, "irq sts:0x%x\n", sts);

	sts = ast_read(ecdsa_dev, ASPEED_ECC_STS_REG) & ECC_VERIFY_PASS;
	if (sts == ECC_VERIFY_PASS) {
		AST_DBG(ecdsa_dev, "Verify PASS !\n");

		ecdsa_engine->results = 0;
		/* Stop ECDSA engine */
		if (ecdsa_engine->flags & CRYPTO_FLAGS_BUSY)
			tasklet_schedule(&ecdsa_engine->done_task);
		else
			dev_err(ecdsa_dev->dev, "ECDSA no active requests.\n");

	} else {
		ecdsa_engine->results = -EKEYREJECTED;
		AST_DBG(ecdsa_dev, "Verify FAILED !\n");
	}

	return IRQ_HANDLED;
}
#endif

static int aspeed_hw_trigger(struct aspeed_ecdsa_dev *ecdsa_dev)
{
	AST_DBG(ecdsa_dev, "\n");

	ast_write(ecdsa_dev, 0x1, ASPEED_ECC_ECDSA_VERIFY);

	ast_write(ecdsa_dev, ECC_EN, ASPEED_ECC_CMD_REG);
	ast_write(ecdsa_dev, 0x0, ASPEED_ECC_CMD_REG);

#ifdef ASPEED_ECDSA_IRQ_MODE
	return 0;
#else
	return aspeed_ecdsa_wait_complete(ecdsa_dev);
#endif
}

static int _aspeed_ecdsa_init_ecc_curve(struct aspeed_ecc_ctx *ctx)
{
	void __iomem *base = ctx->ecdsa_dev->regs;
	int nbytes = ctx->curve->g.ndigits << ECC_DIGITS_TO_BYTES_SHIFT;
	u32 ctrl;
	u8 *data;

	AST_DBG(ctx->ecdsa_dev, "\n");

	switch (ctx->curve_id) {
	case ECC_CURVE_NIST_P256:
		AST_DBG(ctx->ecdsa_dev, "curve ECC_CURVE_NIST_P256\n");
		ctrl = ECDSA_256_EN;
		break;
	case ECC_CURVE_NIST_P384:
		AST_DBG(ctx->ecdsa_dev, "curve ECC_CURVE_NIST_P384\n");
		ctrl = ECDSA_384_EN;
		break;
	}

	ast_write(ctx->ecdsa_dev, ECC_EN | ctrl, ASPEED_ECC_CTRL_REG);

	/* Initial Curve: ecc point/p/a/n */
	data = vmalloc(nbytes);
	if (!data)
		return -ENOMEM;

	hexdump("Dump Gx:", (u8 *)ctx->curve->g.x, nbytes);
	buff_reverse(data, (u8 *)ctx->curve->g.x, nbytes);
	memcpy_toio(base + ASPEED_ECC_PAR_GX_REG, data, nbytes);

	hexdump("Dump Gy:", (u8 *)ctx->curve->g.y, nbytes);
	buff_reverse(data, (u8 *)ctx->curve->g.y, nbytes);
	memcpy_toio(base + ASPEED_ECC_PAR_GY_REG, data, nbytes);

	hexdump("Dump P:", (u8 *)ctx->curve->p, nbytes);
	buff_reverse(data, (u8 *)ctx->curve->p, nbytes);
	memcpy_toio(base + ASPEED_ECC_PAR_P_REG, data, nbytes);

	hexdump("Dump A:", (u8 *)ctx->curve->a, nbytes);
	buff_reverse(data, (u8 *)ctx->curve->a, nbytes);
	memcpy_toio(base + ASPEED_ECC_PAR_A_REG, data, nbytes);

	hexdump("Dump N:", (u8 *)ctx->curve->n, nbytes);
	buff_reverse(data, (u8 *)ctx->curve->n, nbytes);
	memcpy_toio(base + ASPEED_ECC_PAR_N_REG, data, nbytes);

	vfree(data);

	return 0;
}

static int _aspeed_ecdsa_set_pub_key(struct aspeed_ecc_ctx *ctx)
{
	void __iomem *base = ctx->ecdsa_dev->regs;
	u32 nbytes = ctx->curve->g.ndigits << ECC_DIGITS_TO_BYTES_SHIFT;
	u8 *data;

	/* Set public key: Qx/Qy */
	data = vmalloc(nbytes);
	if (!data)
		return -ENOMEM;

	hexdump("Dump Qx:", (u8 *)ctx->pub_key.x, nbytes);
	buff_reverse(data, (u8 *)ctx->pub_key.x, nbytes);
	memcpy_toio(base + ASPEED_ECC_PAR_QX_REG, data, nbytes);

	hexdump("Dump Qy:", (u8 *)ctx->pub_key.y, nbytes);
	buff_reverse(data, (u8 *)ctx->pub_key.y, nbytes);
	memcpy_toio(base + ASPEED_ECC_PAR_QY_REG, data, nbytes);

	vfree(data);

	return 0;
}

static int _aspeed_ecdsa_verify(struct aspeed_ecc_ctx *ctx, const u64 *hash,
				const u64 *r, const u64 *s)
{
	const struct ecc_curve *curve = ctx->curve;
	void __iomem *base = ctx->ecdsa_dev->regs;
	int nbytes = ctx->curve->g.ndigits << ECC_DIGITS_TO_BYTES_SHIFT;
	u8 ndigits = curve->g.ndigits;
	u8 *data, *buf;

	/* 0 < r < n  and 0 < s < n */
	if (vli_is_zero(r, ndigits) || vli_cmp(r, curve->n, ndigits) >= 0 ||
	    vli_is_zero(s, ndigits) || vli_cmp(s, curve->n, ndigits) >= 0)
		return -EBADMSG;

	/* hash is given */
	AST_DBG(ctx->ecdsa_dev, "hash : %016llx %016llx ... %016llx\n",
		hash[ndigits - 1], hash[ndigits - 2], hash[0]);

	data = vmalloc(nbytes);
	if (!data)
		return -ENOMEM;

	/* Initial signature/message and trigger ecdsa verification */
	buf = (u8 *)r;
	hexdump("Dump r:", buf, nbytes);

	buff_reverse(data, (u8 *)r, nbytes);
	memcpy_toio(base + ASPEED_ECC_SIGN_R_REG, data, nbytes);

	buf = (u8 *)s;
	hexdump("Dump s:", buf, nbytes);

	buff_reverse(data, (u8 *)s, nbytes);
	memcpy_toio(base + ASPEED_ECC_SIGN_S_REG, data, nbytes);

	buf = (u8 *)hash;
	hexdump("Dump m:", buf, nbytes);

	buff_reverse(data, (u8 *)hash, nbytes);
	memcpy_toio(base + ASPEED_ECC_MESSAGE_REG, data, nbytes);

	vfree(data);

	return aspeed_hw_trigger(ctx->ecdsa_dev);
}

static int aspeed_ecdsa_trigger(struct aspeed_ecc_ctx *ctx)
{
	u64 hash[ECC_MAX_DIGITS];
	size_t keylen = ctx->curve->g.ndigits * sizeof(u64);
	int diff;
	int ret;
	u8 rawhash[ECC_MAX_BYTES];

	AST_DBG(ctx->ecdsa_dev, "\n");

	if (unlikely(!ctx->pub_key_set))
		return -EINVAL;

	/* if the hash is shorter then we will add leading zeros to fit to ndigits */
	diff = keylen - ctx->sig_len;
	if (diff >= 0) {
		if (diff)
			memset(rawhash, 0, diff);
		memcpy(&rawhash[diff], ctx->digest, ctx->dig_len);
	} else if (diff < 0) {
		/* given hash is longer, we take the left-most bytes */
		memcpy(&rawhash, ctx->digest, keylen);
	}
	ecc_digits_from_bytes(rawhash, ctx->dig_len, hash,
			      ctx->curve->g.ndigits);

	/* Start ecdsa engine verification */
	mutex_lock(&ctx->ecdsa_dev->lock);

	ret = _aspeed_ecdsa_init_ecc_curve(ctx);
	if (ret)
		goto end;

	ret = _aspeed_ecdsa_set_pub_key(ctx);
	if (ret)
		goto end;

	ret = _aspeed_ecdsa_verify(ctx, hash, ctx->sig.r, ctx->sig.s);
	if (ret)
		goto end;

end:
	mutex_unlock(&ctx->ecdsa_dev->lock);
	return ret;
}

/************************************************************************/
/*                Aspeed's ECDSA crypto engine function                 */
/************************************************************************/
static int aspeed_ecdsa_do_request(struct aspeed_ecc_ctx *ctx)
{
	struct aspeed_ecdsa_dev *ecdsa_dev = ctx->ecdsa_dev;
	struct aspeed_engine_ecdsa *ecdsa_engine;

	AST_DBG(ctx->ecdsa_dev, "\n");

	ecdsa_engine = &ecdsa_dev->ecdsa_engine;
	ecdsa_engine->flags |= CRYPTO_FLAGS_BUSY;

	return aspeed_ecdsa_trigger(ctx);
}

static int aspeed_ecdsa_complete(struct aspeed_ecdsa_dev *ecdsa_dev)
{
	struct aspeed_engine_ecdsa *ecdsa_engine = &ecdsa_dev->ecdsa_engine;
	int results = ecdsa_engine->results;

	AST_DBG(ecdsa_dev, "\n");

	ecdsa_engine->flags &= ~CRYPTO_FLAGS_BUSY;

	return results;
}

static int aspeed_ecdsa_handle_queue(struct aspeed_ecc_ctx *ctx)
{
	int ret;

	if (aspeed_ecdsa_need_fallback(ctx, ctx->dig_len)) {
		AST_DBG(ctx->ecdsa_dev, "SW fallback\n");

		ret = crypto_sig_verify(ctx->fallback_tfm, &ctx->sig,
					ctx->sig_len, ctx->digest,
					ctx->dig_len);

		AST_DBG(ctx->ecdsa_dev, "SW verify...ret:0x%x\n", ret);

		return ret;
	}

	/* sig_alg does not support crypto engine queue now, do request directly */
	return aspeed_ecdsa_do_request(ctx);
}

/************************************************************************/
/*             Aspeed's ECDSA context operation function                */
/************************************************************************/
static int aspeed_ecdsa_ecc_ctx_init(struct crypto_sig *tfm,
				     unsigned int curve_id)
{
	struct aspeed_ecc_ctx *ctx = crypto_sig_ctx(tfm);
	struct sig_alg *alg = crypto_sig_alg(tfm);
	struct aspeed_ecdsa_alg *ecdsa_alg;
	const char *name = crypto_tfm_alg_name(&tfm->base);

	/* Get ecdsa device */
	ecdsa_alg = container_of(alg, struct aspeed_ecdsa_alg, sig_alg);
	ctx->ecdsa_dev = ecdsa_alg->ecdsa_dev;

	AST_DBG(ctx->ecdsa_dev, "\n");

	/* Get ecdsa fallback software */
	ctx->fallback_tfm = crypto_alloc_sig(name, 0, CRYPTO_ALG_NEED_FALLBACK);
	if (IS_ERR(ctx->fallback_tfm)) {
		dev_err(ctx->ecdsa_dev->dev,
			"ERROR: Cannot allocate fallback for %s %ld\n", name,
			PTR_ERR(ctx->fallback_tfm));
		return PTR_ERR(ctx->fallback_tfm);
	}

	/* Set ecdsa curve */
	ctx->curve_id = curve_id;
	ctx->curve = ecc_get_curve(curve_id);
	if (!ctx->curve)
		return -EINVAL;

	return 0;
}

static void aspeed_ecdsa_ecc_ctx_deinit(struct aspeed_ecc_ctx *ctx)
{
	ctx->pub_key_set = false;
}

static void aspeed_ecdsa_ecc_ctx_reset(struct aspeed_ecc_ctx *ctx)
{
	ctx->pub_key = ECC_POINT_INIT(ctx->x, ctx->y,
				      ctx->curve->g.ndigits);
}

/************************************************************************/
/*                Aspeed's ECDSA driver entry function                  */
/************************************************************************/
/*
 * Set the public key given the raw uncompressed key data from an X509
 * certificate. The key data contain the concatenated X and Y coordinates of
 * the public key.
 */
static int aspeed_ecdsa_set_pub_key(struct crypto_sig *tfm, const void *key,
				    unsigned int keylen)
{
	struct aspeed_ecc_ctx *ctx = crypto_sig_ctx(tfm);
	u32 ndigits, digitlen;
	const u8 *d = key;
	int ret;

	AST_DBG(ctx->ecdsa_dev, "\n");

	ret = crypto_sig_set_pubkey(ctx->fallback_tfm, key, keylen);
	if (ret)
		return ret;

	aspeed_ecdsa_ecc_ctx_reset(ctx);

	if (keylen < 1 || (((keylen - 1) >> 1) % sizeof(u64)) != 0)
		return -EINVAL;
	/* we only accept uncompressed format indicated by '4' */
	if (d[0] != 4)
		return -EINVAL;

	keylen--;
	digitlen = keylen >> 1;
	ndigits = digitlen / sizeof(u64);
	if (ndigits != ctx->curve->g.ndigits)
		return -EINVAL;

	d++;
	ecc_digits_from_bytes(d, digitlen, ctx->pub_key.x, ndigits);
	ecc_digits_from_bytes(&d[digitlen], digitlen, ctx->pub_key.y, ndigits);

	ret = ecc_is_pubkey_valid_full(ctx->curve, &ctx->pub_key);
	ctx->pub_key_set = ret == 0;

	return ret;
}

/*
 * Verify an ECDSA signature.
 */
static int aspeed_ecdsa_verify(struct crypto_sig *tfm, const void *src,
			       unsigned int slen, const void *digest,
			       unsigned int dlen)
{
	struct aspeed_ecc_ctx *ctx = crypto_sig_ctx(tfm);

	AST_DBG(ctx->ecdsa_dev, "\n");

	if (slen > ECC_MAX_BYTES * 2 || dlen > SHA512_DIGEST_SIZE)
		return -EINVAL;

	/* Prepare signature information */
	ctx->sig_len = slen;
	memcpy(&ctx->sig, src, slen);

	/* Prepare digest information */
	ctx->dig_len = dlen;
	memcpy(ctx->digest, digest, dlen);

	return aspeed_ecdsa_handle_queue(ctx);
}

static unsigned int aspeed_ecdsa_max_size(struct crypto_sig *tfm)
{
	struct aspeed_ecc_ctx *ctx = crypto_sig_ctx(tfm);

	return ctx->pub_key.ndigits << ECC_DIGITS_TO_BYTES_SHIFT;
}

static int aspeed_ecdsa_nist_p256_init_tfm(struct crypto_sig *tfm)
{
	return aspeed_ecdsa_ecc_ctx_init(tfm, ECC_CURVE_NIST_P256);
}

static int aspeed_ecdsa_nist_p384_init_tfm(struct crypto_sig *tfm)
{
	return aspeed_ecdsa_ecc_ctx_init(tfm, ECC_CURVE_NIST_P384);
}

static void aspeed_ecdsa_exit_tfm(struct crypto_sig *tfm)
{
	struct aspeed_ecc_ctx *ctx = crypto_sig_ctx(tfm);

	AST_DBG(ctx->ecdsa_dev, "\n");

	aspeed_ecdsa_ecc_ctx_deinit(ctx);

	crypto_free_sig(ctx->fallback_tfm);
}

static struct aspeed_ecdsa_alg aspeed_ecdsa_nist_p256 = {
	.sig_alg = {
		.set_pub_key = aspeed_ecdsa_set_pub_key,
		.verify = aspeed_ecdsa_verify,
		.key_size = aspeed_ecdsa_max_size,
		.init = aspeed_ecdsa_nist_p256_init_tfm,
		.exit = aspeed_ecdsa_exit_tfm,
		.base = {
			.cra_name = "ecdsa-nist-p256",
			.cra_driver_name = "aspeed-ecdsa-nist-p256",
			.cra_priority = 300,
			.cra_module = THIS_MODULE,
			.cra_ctxsize = sizeof(struct aspeed_ecc_ctx),
			.cra_flags = CRYPTO_ALG_KERN_DRIVER_ONLY |
				     CRYPTO_ALG_NEED_FALLBACK,
		},
	},
};

static struct aspeed_ecdsa_alg aspeed_ecdsa_nist_p384 = {
	.sig_alg = {
		.set_pub_key = aspeed_ecdsa_set_pub_key,
		.verify = aspeed_ecdsa_verify,
		.key_size = aspeed_ecdsa_max_size,
		.init = aspeed_ecdsa_nist_p384_init_tfm,
		.exit = aspeed_ecdsa_exit_tfm,
		.base = {
			.cra_name = "ecdsa-nist-p384",
			.cra_driver_name = "aspeed-ecdsa-nist-p384",
			.cra_priority = 300,
			.cra_module = THIS_MODULE,
			.cra_ctxsize = sizeof(struct aspeed_ecc_ctx),
			.cra_flags = CRYPTO_ALG_KERN_DRIVER_ONLY |
				     CRYPTO_ALG_NEED_FALLBACK,
		},
	},
};

/************************************************************************/
/*            Aspeed's ECDSA driver init/register function              */
/************************************************************************/
static int aspeed_ecdsa_register(struct aspeed_ecdsa_dev *ecdsa_dev)
{
	int rc;

	aspeed_ecdsa_nist_p256.ecdsa_dev = ecdsa_dev;
	rc = crypto_register_sig(&aspeed_ecdsa_nist_p256.sig_alg);
	if (rc)
		goto nist_p256_error;

	aspeed_ecdsa_nist_p384.ecdsa_dev = ecdsa_dev;
	rc = crypto_register_sig(&aspeed_ecdsa_nist_p384.sig_alg);
	if (rc)
		goto nist_p384_error;

	return 0;

nist_p384_error:
	crypto_unregister_sig(&aspeed_ecdsa_nist_p256.sig_alg);
nist_p256_error:
	return rc;
}

static void aspeed_ecdsa_unregister(struct aspeed_ecdsa_dev *ecdsa_dev)
{
	crypto_unregister_sig(&aspeed_ecdsa_nist_p256.sig_alg);
	crypto_unregister_sig(&aspeed_ecdsa_nist_p384.sig_alg);
}

static int aspeed_ecdsa_probe(struct platform_device *pdev)
{
	struct aspeed_ecdsa_dev *ecdsa_dev;
	struct device *dev = &pdev->dev;
	int rc;

	ecdsa_dev = devm_kzalloc(dev, sizeof(struct aspeed_ecdsa_dev),
				 GFP_KERNEL);
	if (!ecdsa_dev)
		return -ENOMEM;

	ecdsa_dev->dev = dev;

	platform_set_drvdata(pdev, ecdsa_dev);

	ecdsa_dev->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ecdsa_dev->regs))
		return PTR_ERR(ecdsa_dev->regs);

#ifdef ASPEED_ECDSA_IRQ_MODE
	/* Get irq number and register it */
	ecdsa_dev->irq = platform_get_irq(pdev, 0);
	if (ecdsa_dev->irq < 0)
		return -ENXIO;

	rc = devm_request_irq(dev, ecdsa_dev->irq, aspeed_ecdsa_irq, 0,
			      dev_name(dev), ecdsa_dev);
	if (rc) {
		dev_err(dev, "Failed to request irq.\n");
		return rc;
	}

	/* Enable interrupt */
	ast_write(ecdsa_dev, 0x1, ASPEED_ECC_INT_EN);
#endif

	rc = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));
	if (rc) {
		dev_warn(&pdev->dev, "No suitable DMA available\n");
		return rc;
	}

	ecdsa_dev->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(ecdsa_dev->clk)) {
		dev_err(dev, "Failed to get ecdsa clk\n");
		return PTR_ERR(ecdsa_dev->clk);
	}

	ecdsa_dev->rst = devm_reset_control_get_shared(dev, NULL);
	if (IS_ERR(ecdsa_dev->rst)) {
		dev_err(dev, "Failed to get ecdsa reset\n");
		return PTR_ERR(ecdsa_dev->rst);
	}

	rc = reset_control_deassert(ecdsa_dev->rst);
	if (rc) {
		dev_err(dev, "Deassert ecdsa reset failed\n");
		return rc;
	}

	/* Self-test */
	rc = aspeed_ecdsa_self_test(ecdsa_dev);
	if (rc)
		goto end;

	rc = aspeed_ecdsa_register(ecdsa_dev);
	if (rc) {
		dev_err(dev, "ECDSA algo register failed\n");
		return rc;
	}

	mutex_init(&ecdsa_dev->lock);

	dev_info(dev, "Aspeed ECDSA Hardware Accelerator successfully registered\n");

	return 0;

end:
	return rc;
}

static void aspeed_ecdsa_remove(struct platform_device *pdev)
{
	struct aspeed_ecdsa_dev *ecdsa_dev = platform_get_drvdata(pdev);

	aspeed_ecdsa_unregister(ecdsa_dev);
}

static const struct of_device_id aspeed_ecdsa_of_matches[] = {
	{
		.compatible = "aspeed,ast2700-ecdsa",
	},
	{},
};
MODULE_DEVICE_TABLE(of, aspeed_ecdsa_of_matches);

static struct platform_driver aspeed_ecdsa_driver = {
	.probe		= aspeed_ecdsa_probe,
	.remove		= aspeed_ecdsa_remove,
	.driver		= {
		.name   = KBUILD_MODNAME,
		.of_match_table = aspeed_ecdsa_of_matches,
	},
};

module_platform_driver(aspeed_ecdsa_driver);
MODULE_AUTHOR("Neal Liu <neal_liu@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED ECDSA algorithm driver acceleration");
MODULE_LICENSE("GPL");
