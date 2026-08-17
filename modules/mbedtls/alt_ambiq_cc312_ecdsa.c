/*
 * Copyright (c) 2026 Ambiq Micro, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <string.h>

#include <mbedtls/ecdsa.h>
#include <mbedtls/ecp.h>
#include <mbedtls/platform_util.h>

#include <soc.h>

#define CC312_PD DEVICE_DT_GET(DT_NODELABEL(crypto_pd))

/* The HAL only carries the secp256r1 domain, and defining the ALT macros
 * removes Mbed TLS' own implementation, so there is nothing to fall back to
 * for other curves.
 */
#define P256_BYTES 32U
#define P256_WORDS (P256_BYTES / 4U)

static K_MUTEX_DEFINE(cc312_ecdsa_lock);

static void be_bytes_to_le_words(const uint8_t be[P256_BYTES], uint32_t out[P256_WORDS])
{
	for (size_t i = 0; i < P256_WORDS; i++) {
		out[P256_WORDS - 1U - i] = sys_get_be32(&be[i * 4U]);
	}
}

static int mpi_to_le_words(const mbedtls_mpi *x, uint32_t out[P256_WORDS])
{
	uint8_t be[P256_BYTES];
	int ret = mbedtls_mpi_write_binary(x, be, sizeof(be));

	if (ret != 0) {
		return ret;
	}

	be_bytes_to_le_words(be, out);

	return 0;
}

/* The point coordinates are MBEDTLS_PRIVATE() members, so go through the
 * public serialisation instead of reaching into the struct.
 */
static int point_to_le_words(const mbedtls_ecp_group *grp, const mbedtls_ecp_point *Q,
			     uint32_t qx[P256_WORDS], uint32_t qy[P256_WORDS])
{
	uint8_t pt[1U + (2U * P256_BYTES)];
	size_t olen;
	int ret = mbedtls_ecp_point_write_binary(grp, Q, MBEDTLS_ECP_PF_UNCOMPRESSED, &olen, pt,
						 sizeof(pt));

	if (ret != 0) {
		return ret;
	}

	if (olen != sizeof(pt)) {
		return MBEDTLS_ERR_ECP_BAD_INPUT_DATA;
	}

	be_bytes_to_le_words(&pt[1], qx);
	be_bytes_to_le_words(&pt[1U + P256_BYTES], qy);

	return 0;
}

static int le_words_to_mpi(const uint32_t in[P256_WORDS], mbedtls_mpi *x)
{
	uint8_t be[P256_BYTES];

	for (size_t i = 0; i < P256_WORDS; i++) {
		sys_put_be32(in[P256_WORDS - 1U - i], &be[i * 4U]);
	}

	return mbedtls_mpi_read_binary(x, be, sizeof(be));
}

static int cc312_acquire(const am_hal_cc312_ecc_domain_t **domain)
{
	*domain = am_hal_cc312_ecc_domain_secp256r1();
	if (*domain == NULL) {
		return MBEDTLS_ERR_ECP_FEATURE_UNAVAILABLE;
	}

	k_mutex_lock(&cc312_ecdsa_lock, K_FOREVER);

	if (pm_device_runtime_get(CC312_PD) < 0) {
		k_mutex_unlock(&cc312_ecdsa_lock);
		return MBEDTLS_ERR_ECP_BAD_INPUT_DATA;
	}

	return 0;
}

static void cc312_release(void)
{
	(void)pm_device_runtime_put(CC312_PD);
	k_mutex_unlock(&cc312_ecdsa_lock);
}

int mbedtls_ecdsa_sign(mbedtls_ecp_group *grp, mbedtls_mpi *r, mbedtls_mpi *s,
		       const mbedtls_mpi *d, const unsigned char *buf, size_t blen,
		       mbedtls_f_rng_t *f_rng, void *p_rng)
{
	const am_hal_cc312_ecc_domain_t *domain;
	uint32_t priv[P256_WORDS];
	uint32_t sig_r[P256_WORDS];
	uint32_t sig_s[P256_WORDS];
	size_t hlen = MIN(blen, P256_BYTES);
	int ret;

	ARG_UNUSED(f_rng);
	ARG_UNUSED(p_rng);

	if (grp->id != MBEDTLS_ECP_DP_SECP256R1) {
		return MBEDTLS_ERR_ECP_FEATURE_UNAVAILABLE;
	}

	ret = mpi_to_le_words(d, priv);
	if (ret != 0) {
		return ret;
	}

	ret = cc312_acquire(&domain);
	if (ret != 0) {
		goto out;
	}

	if (am_hal_cc312_ecdsa_sign_rnd(domain, priv, buf, (uint32_t)hlen, sig_r, sig_s) !=
	    AM_HAL_STATUS_SUCCESS) {
		cc312_release();
		ret = MBEDTLS_ERR_ECP_RANDOM_FAILED;
		goto out;
	}

	cc312_release();

	ret = le_words_to_mpi(sig_r, r);
	if (ret == 0) {
		ret = le_words_to_mpi(sig_s, s);
	}

out:
	mbedtls_platform_zeroize(priv, sizeof(priv));
	return ret;
}

int mbedtls_ecdsa_verify(mbedtls_ecp_group *grp, const unsigned char *buf, size_t blen,
			 const mbedtls_ecp_point *Q, const mbedtls_mpi *r, const mbedtls_mpi *s)
{
	const am_hal_cc312_ecc_domain_t *domain;
	uint32_t qx[P256_WORDS];
	uint32_t qy[P256_WORDS];
	uint32_t sig_r[P256_WORDS];
	uint32_t sig_s[P256_WORDS];
	size_t hlen = MIN(blen, P256_BYTES);
	int ret;

	if (grp->id != MBEDTLS_ECP_DP_SECP256R1) {
		return MBEDTLS_ERR_ECP_FEATURE_UNAVAILABLE;
	}

	/* Reject r or s outside [1, n-1] here: the HAL reports a rejected
	 * signature and a hardware fault with the same status.
	 */
	if (mbedtls_mpi_cmp_int(r, 1) < 0 || mbedtls_mpi_cmp_mpi(r, &grp->N) >= 0 ||
	    mbedtls_mpi_cmp_int(s, 1) < 0 || mbedtls_mpi_cmp_mpi(s, &grp->N) >= 0) {
		return MBEDTLS_ERR_ECP_VERIFY_FAILED;
	}

	ret = point_to_le_words(grp, Q, qx, qy);
	if (ret == 0) {
		ret = mpi_to_le_words(r, sig_r);
	}
	if (ret == 0) {
		ret = mpi_to_le_words(s, sig_s);
	}
	if (ret != 0) {
		return ret;
	}

	ret = cc312_acquire(&domain);
	if (ret != 0) {
		return ret;
	}

	if (am_hal_cc312_ecdsa_verify(domain, qx, qy, buf, (uint32_t)hlen, sig_r, sig_s) !=
	    AM_HAL_STATUS_SUCCESS) {
		ret = MBEDTLS_ERR_ECP_VERIFY_FAILED;
	}

	cc312_release();

	return ret;
}
