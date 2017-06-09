/*
 * Glue code for the SHA512 Secure Hash Algorithm assembly implementation
 * using optimized ARM assembler and NEON instructions.
 *
 * Copyright © 2015 Google Inc.
 *
 * This file is based on sha512_ssse3_glue.c:
 *   Copyright (C) 2013 Intel Corporation
 *   Author: Tim Chen <tim.c.chen@linux.intel.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 */

#include <common.h>
#include <digest.h>
#include <init.h>
#include <crypto/sha.h>
#include <crypto/internal.h>
#include <asm/byteorder.h>
#include <asm/unaligned.h>

void sha512_block_data_order_neon(u64 *digest, const void *data,
				      unsigned int num_blks);

static int sha512_init(struct digest *desc)
{
        struct sha512_state *sctx = digest_ctx(desc);
        sctx->state[0] = SHA512_H0;
        sctx->state[1] = SHA512_H1;
        sctx->state[2] = SHA512_H2;
        sctx->state[3] = SHA512_H3;
        sctx->state[4] = SHA512_H4;
        sctx->state[5] = SHA512_H5;
        sctx->state[6] = SHA512_H6;
        sctx->state[7] = SHA512_H7;
        sctx->count[0] = sctx->count[1] = 0;

        return 0;
}

static int sha384_init(struct digest *desc)
{
        struct sha512_state *sctx = digest_ctx(desc);
        sctx->state[0] = SHA384_H0;
        sctx->state[1] = SHA384_H1;
        sctx->state[2] = SHA384_H2;
        sctx->state[3] = SHA384_H3;
        sctx->state[4] = SHA384_H4;
        sctx->state[5] = SHA384_H5;
        sctx->state[6] = SHA384_H6;
        sctx->state[7] = SHA384_H7;
        sctx->count[0] = sctx->count[1] = 0;

        return 0;
}

int sha512_update(struct digest *desc, const void *data,
			     unsigned long len)
{
	struct sha512_state *sctx = digest_ctx(desc);
	unsigned int partial = sctx->count[0] % SHA512_BLOCK_SIZE;

	sctx->count[0] += len;
	if (sctx->count[0] < len)
		sctx->count[1]++;

	if (partial + len >= SHA512_BLOCK_SIZE) {
		int blocks;

		if(partial) {
			int p = SHA512_BLOCK_SIZE - partial;

			memcpy(sctx->buf + partial, data, p);
			data +=p;
			len -= p;

			sha512_block_data_order_neon(sctx->state, sctx->buf, 1);
		}

		blocks = len / SHA512_BLOCK_SIZE;
		len %= SHA512_BLOCK_SIZE;

		if(blocks) {
			sha512_block_data_order_neon(sctx->state, data, blocks);
			data += blocks * SHA512_BLOCK_SIZE;
		}
		partial = 0;
	}
	if (len)
		memcpy(sctx->buf + partial, data, len);

	return 0;
}

/* Add padding and return the message digest. */
static int sha512_final(struct digest *desc, u8 *out)
{
	const int bit_offset = SHA512_BLOCK_SIZE - sizeof(__be64[2]);
	struct sha512_state *sctx = digest_ctx(desc);
	__be64 *bits = (__be64 *)(sctx->buf + bit_offset);
	unsigned int partial = sctx->count[0] % SHA512_BLOCK_SIZE;
	__be64 *digest = (__be64 *)out;
	int i;
	unsigned int digest_size = desc->algo->length;

	sctx->buf[partial++] = 0x80;
	if (partial > bit_offset) {
		memset(sctx->buf + partial, 0x0, SHA512_BLOCK_SIZE - partial);
		partial = 0;

		sha512_block_data_order_neon(sctx->state, sctx->buf, 1);
	}

	memset(sctx->buf + partial, 0x0, bit_offset - partial);
	bits[0] = cpu_to_be64(sctx->count[1] << 3 | sctx->count[0] >> 61);
	bits[1] = cpu_to_be64(sctx->count[0] << 3);
	sha512_block_data_order_neon(sctx->state, sctx->buf, 1);

	for (i = 0; digest_size > 0; i++, digest_size -= sizeof(__be64))
		put_unaligned_be64(sctx->state[i], digest++);

	*sctx = (struct sha512_state){};

	return 0;
}

static struct digest_algo sha384 = {
	.base = {
		.name		=	"sha384",
		.driver_name 	=	"sha384-asm",
		.priority	=	150,
		.algo		=	HASH_ALGO_SHA384,
	},

	.length	=	SHA384_DIGEST_SIZE,
	.init	=	sha384_init,
	.update	=	sha512_update,
	.final	=	sha512_final,
	.digest	=	digest_generic_digest,
	.verify	=	digest_generic_verify,
	.ctx_length =	sizeof(struct sha512_state),
};

static int sha384_digest_register(void)
{
	return digest_algo_register(&sha384);
}
device_initcall(sha384_digest_register);

static struct digest_algo sha512 = {
	.base = {
		.name		=	"sha512",
		.driver_name 	=	"sha512-asm",
		.priority	=	150,
		.algo		=	HASH_ALGO_SHA512,
	},

	.length	=	SHA512_DIGEST_SIZE,
	.init	=	sha512_init,
	.update	=	sha512_update,
	.final	=	sha512_final,
	.digest	=	digest_generic_digest,
	.verify	=	digest_generic_verify,
	.ctx_length =	sizeof(struct sha512_state),
};

static int sha512_digest_register(void)
{
	return digest_algo_register(&sha512);
}
device_initcall(sha512_digest_register);
