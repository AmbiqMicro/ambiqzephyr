/*
 * Copyright (c) 2026, Ambiq Micro Inc. <www.ambiq.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief PM Peripheral Demo — apollo330mP_evb
 *
 * Demonstrates PM-aware peripheral use on the Apollo330P SoC:
 *   - BLE: advertising as a connectable peripheral (IPC-based, no overlay)
 *   - Flash: erase / write / read on storage_partition
 *   - CRC32: hardware digest over flash read-back (ambiq,hw-crc32)
 *   - AES-128-ECB: hardware encrypt/decrypt via CC312 AES accelerator
 *     (ambiq,crypto-aes driver, cc312_aes@400c0000)
 *   - SPI: bus transfer on spi0 (IOM0)
 *   - I2C: bus scan on i2c1 (IOM1)
 *
 * Each peripheral runs in its own dedicated Zephyr thread.  Threads sleep
 * between iterations with staggered start times so the PM subsystem gets
 * varied idle windows and can exercise multiple suspend-to-idle substates.
 * When all threads are simultaneously idle, the deepest eligible power state
 * is entered and the system wakes via the next thread's timeout.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/crc.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/crypto/crypto.h>
#include <string.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pm_demo, CONFIG_LOG_DEFAULT_LEVEL);

/* ---- Device nodes ---- */
#if DT_HAS_CHOSEN(zephyr_crc)
#define CRC_NODE  DT_CHOSEN(zephyr_crc)
#endif
#if DT_NODE_EXISTS(DT_ALIAS(demo_spi))
#define SPI_NODE DT_ALIAS(demo_spi)
#endif
#if DT_NODE_EXISTS(DT_ALIAS(demo_i2c))
#define I2C_NODE DT_ALIAS(demo_i2c)
#endif

/* Flash partition */
#define STORAGE_PART_ID FIXED_PARTITION_ID(storage_partition)

/*
 * Write block size on apollo330P is 32 bytes (board DTS override).
 * Erase block size is 2048 bytes.
 * Keep the demo buffer at 64 bytes (2 × write-block, well within one erase
 * block) so erase / write / read are all properly aligned.
 */
#define FLASH_DEMO_BUF_SIZE 64
#define FLASH_DEMO_ERASE_SIZE 2048

/* Per-thread period and staggered boot delay (ms).  Staggering prevents all
 * threads from waking simultaneously, giving PM more varied idle windows.
 */
#define DEMO_PERIOD_MS      15000
#define STAGGER_ENTROPY_MS   5000
#define STAGGER_FLASH_CRC_MS 2000
#define STAGGER_AES_MS       4000
#define STAGGER_SPI_MS       6000
#define STAGGER_I2C_MS       8000

/* ---- AES-128-ECB test vectors (FIPS 197, first vector) ---- */
static const uint8_t aes_key[16] __aligned(4) = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
	0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
};
static uint8_t aes_plaintext[16] __aligned(4) = {
	0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
	0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff,
};
static const uint8_t aes_expected_ct[16] __aligned(4) = {
	0x69, 0xc4, 0xe0, 0xd8, 0x6a, 0x7b, 0x04, 0x30,
	0xd8, 0xcd, 0xb7, 0x80, 0x70, 0xb4, 0xc5, 0x5a,
};

/* ---- Entropy demo buffer ---- */
#define ENTROPY_BUF_SIZE 16
static uint8_t entropy_buf[ENTROPY_BUF_SIZE];

/* ---- Flash demo buffers ---- */
static const uint8_t flash_write_buf[FLASH_DEMO_BUF_SIZE] = {
	'A', 'm', 'b', 'i', 'q', ' ', 'P', 'M', ' ', 'D', 'e', 'm', 'o', '!',
	0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
	0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10,
	0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
	0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20,
	0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28,
	0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x30,
};
static uint8_t flash_read_buf[FLASH_DEMO_BUF_SIZE];

/* ---- SPI demo buffers ---- */
#if DT_NODE_EXISTS(DT_ALIAS(demo_spi))
static uint8_t spi_tx_buf[] = { 0xde, 0xad, 0xbe, 0xef };
static uint8_t spi_rx_buf[sizeof(spi_tx_buf)];
#endif

/* ---- BLE advertising data ---- */
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
	BT_DATA(BT_DATA_NAME_COMPLETE,
		CONFIG_BT_DEVICE_NAME,
		sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/* Deferred work so bt_conn is fully released before we re-advertise. */
static void ble_adv_restart_work_handler(struct k_work *work)
{
	int ret;

	ret = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), NULL, 0);
	if (ret) {
		LOG_ERR("bt_le_adv_start (reconnect): %d", ret);
	} else {
		LOG_INF("BLE: advertising as \"%s\"", CONFIG_BT_DEVICE_NAME);
	}
}

static K_WORK_DEFINE(ble_adv_restart_work, ble_adv_restart_work_handler);

static void ble_disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("BLE: disconnected (reason 0x%02x)", reason);
	k_work_submit(&ble_adv_restart_work);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.disconnected = ble_disconnected,
};

/* ================================================================
 * Entropy demo — collect random bytes from the hardware TRNG
 *                 (zephyr,entropy = &trng in board chosen)
 * ================================================================
 */
static int demo_thread_entropy(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_entropy));
	int ret;

	if (!device_is_ready(dev)) {
		LOG_ERR("Entropy device not ready");
		return -ENODEV;
	}

	ret = entropy_get_entropy(dev, entropy_buf, sizeof(entropy_buf));
	if (ret) {
		LOG_ERR("entropy_get_entropy: %d", ret);
		return ret;
	}

	LOG_INF("Entropy (TRNG): "
		"%02x %02x %02x %02x %02x %02x %02x %02x "
		"%02x %02x %02x %02x %02x %02x %02x %02x",
		entropy_buf[0],  entropy_buf[1],  entropy_buf[2],  entropy_buf[3],
		entropy_buf[4],  entropy_buf[5],  entropy_buf[6],  entropy_buf[7],
		entropy_buf[8],  entropy_buf[9],  entropy_buf[10], entropy_buf[11],
		entropy_buf[12], entropy_buf[13], entropy_buf[14], entropy_buf[15]);
	return 0;
}

/* ================================================================
 * Flash demo — erase a sector, write a fixed pattern, read it back
 * ================================================================
 */
static int demo_thread_flash(void)
{
	const struct flash_area *fa;
	int ret;

	ret = flash_area_open(STORAGE_PART_ID, &fa);
	if (ret) {
		LOG_ERR("flash_area_open: %d", ret);
		return ret;
	}

	ret = flash_area_erase(fa, 0, FLASH_DEMO_ERASE_SIZE);
	if (ret) {
		LOG_ERR("flash_area_erase: %d", ret);
		goto out;
	}

	ret = flash_area_write(fa, 0, flash_write_buf, sizeof(flash_write_buf));
	if (ret) {
		LOG_ERR("flash_area_write: %d", ret);
		goto out;
	}

	ret = flash_area_read(fa, 0, flash_read_buf, sizeof(flash_read_buf));
	if (ret) {
		LOG_ERR("flash_area_read: %d", ret);
		goto out;
	}

	if (memcmp(flash_write_buf, flash_read_buf, sizeof(flash_write_buf)) == 0) {
		LOG_INF("Flash: write/read verified OK (%zu bytes)", sizeof(flash_write_buf));
	} else {
		LOG_ERR("Flash: data mismatch after read-back");
		ret = -EIO;
	}

out:
	flash_area_close(fa);
	return ret;
}

#if DT_HAS_CHOSEN(zephyr_crc)
/* ================================================================
 * CRC32 demo — hardware digest over flash read-back buffer
 * ================================================================
 */
static int demo_thread_crc32(void)
{
	const struct device *dev = DEVICE_DT_GET(CRC_NODE);
	struct crc_ctx ctx = {
		.type       = CRC32_IEEE,
		.polynomial = CRC32_IEEE_POLY,
		.seed       = CRC32_IEEE_INIT_VAL,
		.reversed   = (CRC_FLAG_REVERSE_INPUT | CRC_FLAG_REVERSE_OUTPUT),
	};
	int ret;

	if (!device_is_ready(dev)) {
		LOG_ERR("CRC device not ready");
		return -ENODEV;
	}

	ret = crc_begin(dev, &ctx);
	if (ret) {
		LOG_ERR("crc_begin: %d", ret);
		return ret;
	}

	ret = crc_update(dev, &ctx, flash_read_buf, sizeof(flash_read_buf));
	if (ret) {
		LOG_ERR("crc_update: %d", ret);
		return ret;
	}

	ret = crc_finish(dev, &ctx);
	if (ret) {
		LOG_ERR("crc_finish: %d", ret);
		return ret;
	}

	LOG_INF("CRC32 (hardware): result=0x%08x over %zu bytes",
		(uint32_t)ctx.result, sizeof(flash_read_buf));
	return 0;
}
#endif /* DT_HAS_CHOSEN(zephyr_crc) */

/* ================================================================
 * AES demo — hardware AES-128-ECB via CC312 AES accelerator
 *            (ambiq,crypto-aes driver, cc312_aes DT node)
 * ================================================================
 */
static int demo_thread_aes(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(cc312_aes));
	struct cipher_ctx sess;
	struct cipher_pkt op;
	uint8_t encrypted[16] __aligned(4);
	uint8_t decrypted[16] __aligned(4);
	uint32_t cap_flags;
	int ret;

	if (!device_is_ready(dev)) {
		LOG_ERR("CC312 AES device not ready");
		return -ENODEV;
	}

	cap_flags = crypto_query_hwcaps(dev);
	if (!(cap_flags & CAP_RAW_KEY) || !(cap_flags & CAP_SYNC_OPS) ||
	    !(cap_flags & CAP_SEPARATE_IO_BUFS)) {
		LOG_ERR("Crypto device missing required caps (0x%08x)", cap_flags);
		return -ENOTSUP;
	}
	cap_flags = CAP_RAW_KEY | CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;

	/* Encrypt */
	sess = (struct cipher_ctx){
		.keylen         = sizeof(aes_key),
		.key.bit_stream = aes_key,
		.flags          = cap_flags,
	};
	ret = cipher_begin_session(dev, &sess, CRYPTO_CIPHER_ALGO_AES,
				   CRYPTO_CIPHER_MODE_ECB,
				   CRYPTO_CIPHER_OP_ENCRYPT);
	if (ret) {
		LOG_ERR("cipher_begin_session (enc): %d", ret);
		return ret;
	}

	op = (struct cipher_pkt){
		.in_buf      = aes_plaintext,
		.in_len      = sizeof(aes_plaintext),
		.out_buf     = encrypted,
		.out_buf_max = sizeof(encrypted),
	};
	ret = cipher_block_op(&sess, &op);
	cipher_free_session(dev, &sess);
	if (ret) {
		LOG_ERR("cipher_block_op (enc): %d", ret);
		return ret;
	}

	if (memcmp(encrypted, aes_expected_ct, sizeof(aes_expected_ct)) == 0) {
		LOG_INF("AES-128-ECB (sw): encrypt matches FIPS-197 vector");
	} else {
		LOG_WRN("AES-128-ECB (sw): ciphertext mismatch vs FIPS-197");
	}

	/* Decrypt */
	sess = (struct cipher_ctx){
		.keylen         = sizeof(aes_key),
		.key.bit_stream = aes_key,
		.flags          = cap_flags,
	};
	ret = cipher_begin_session(dev, &sess, CRYPTO_CIPHER_ALGO_AES,
				   CRYPTO_CIPHER_MODE_ECB,
				   CRYPTO_CIPHER_OP_DECRYPT);
	if (ret) {
		LOG_ERR("cipher_begin_session (dec): %d", ret);
		return ret;
	}

	op = (struct cipher_pkt){
		.in_buf      = encrypted,
		.in_len      = sizeof(encrypted),
		.out_buf     = decrypted,
		.out_buf_max = sizeof(decrypted),
	};
	ret = cipher_block_op(&sess, &op);
	cipher_free_session(dev, &sess);
	if (ret) {
		LOG_ERR("cipher_block_op (dec): %d", ret);
		return ret;
	}

	if (memcmp(decrypted, aes_plaintext, sizeof(aes_plaintext)) == 0) {
		LOG_INF("AES-128-ECB (sw): decrypt round-trip verified");
	} else {
		LOG_ERR("AES-128-ECB (sw): decrypt mismatch");
	}

	return 0;
}

#if DT_NODE_EXISTS(DT_ALIAS(demo_spi))
/* ================================================================
 * SPI demo — single transfer via demo-spi alias
 * ================================================================
 */
static int demo_thread_spi(void)
{
	const struct device *spi = DEVICE_DT_GET(SPI_NODE);
	const struct spi_config cfg = {
		.frequency = 1000000U,
		.operation = SPI_WORD_SET(8) | SPI_OP_MODE_MASTER,
	};
	struct spi_buf tx_buf = { .buf = spi_tx_buf, .len = sizeof(spi_tx_buf) };
	struct spi_buf rx_buf = { .buf = spi_rx_buf, .len = sizeof(spi_rx_buf) };
	struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1 };
	struct spi_buf_set rx_set = { .buffers = &rx_buf, .count = 1 };
	int ret;

	if (!device_is_ready(spi)) {
		LOG_ERR("SPI device not ready");
		return -ENODEV;
	}

	ret = spi_transceive(spi, &cfg, &tx_set, &rx_set);
	if (ret) {
		/* Expected without loopback hardware; driver / bus is exercised */
		LOG_WRN("SPI transceive: %d (no loopback device connected)", ret);
	} else {
		LOG_INF("SPI: transceive complete (0x%02x 0x%02x 0x%02x 0x%02x)",
			spi_rx_buf[0], spi_rx_buf[1],
			spi_rx_buf[2], spi_rx_buf[3]);
	}

	return 0;
}
#endif /* DT_NODE_EXISTS(DT_ALIAS(demo_spi)) */

#if DT_NODE_EXISTS(DT_ALIAS(demo_i2c))
/* ================================================================
 * I2C demo — bus scan via demo-i2c alias
 * ================================================================
 */
static int demo_thread_i2c(void)
{
	const struct device *i2c = DEVICE_DT_GET(I2C_NODE);
	uint8_t dummy;
	int found = 0;

	if (!device_is_ready(i2c)) {
		LOG_ERR("I2C device not ready");
		return -ENODEV;
	}

	LOG_INF("I2C: scanning bus 0x08..0x77 ...");
	for (uint8_t addr = 0x08U; addr < 0x78U; addr++) {
		if (i2c_read(i2c, &dummy, sizeof(dummy), addr) == 0) {
			LOG_INF("I2C: device found at 0x%02x", addr);
			found++;
		}
	}
	if (found == 0) {
		LOG_INF("I2C: no devices found (expected without external hardware)");
	} else {
		LOG_INF("I2C: scan complete, %d device(s) found", found);
	}

	return 0;
}
#endif /* DT_NODE_EXISTS(DT_ALIAS(demo_i2c)) */

/* ================================================================
 * Thread: entropy
 * ================================================================
 */
static void entropy_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

	k_msleep(STAGGER_ENTROPY_MS);
	while (true) {
		demo_thread_entropy();
		k_msleep(DEMO_PERIOD_MS);
	}
}

/* ================================================================
 * Thread: flash + CRC32
 * CRC32 digests the flash read-back buffer, so both run in one thread
 * to avoid a mutex on flash_read_buf.
 * ================================================================
 */
static void flash_crc_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

	k_msleep(STAGGER_FLASH_CRC_MS);
	while (true) {
		demo_thread_flash();
#if DT_HAS_CHOSEN(zephyr_crc)
		demo_thread_crc32();
#endif
		k_msleep(DEMO_PERIOD_MS);
	}
}

/* ================================================================
 * Thread: AES (software via mbedTLS shim)
 * ================================================================
 */
static void aes_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

	k_msleep(STAGGER_AES_MS);
	while (true) {
		demo_thread_aes();
		k_msleep(DEMO_PERIOD_MS);
	}
}

#if DT_NODE_EXISTS(DT_ALIAS(demo_spi))
/* ================================================================
 * Thread: SPI
 * ================================================================
 */
static void spi_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

	k_msleep(STAGGER_SPI_MS);
	while (true) {
		demo_thread_spi();
		k_msleep(DEMO_PERIOD_MS);
	}
}
#endif /* DT_NODE_EXISTS(DT_ALIAS(demo_spi)) */

#if DT_NODE_EXISTS(DT_ALIAS(demo_i2c))
/* ================================================================
 * Thread: I2C
 * ================================================================
 */
static void i2c_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

	k_msleep(STAGGER_I2C_MS);
	while (true) {
		demo_thread_i2c();
		k_msleep(DEMO_PERIOD_MS);
	}
}
#endif /* DT_NODE_EXISTS(DT_ALIAS(demo_i2c)) */

/* Thread definitions — created at startup, run forever */
K_THREAD_DEFINE(entropy_tid,   512,  entropy_thread_fn,   NULL, NULL, NULL, 7, 0, 0);
K_THREAD_DEFINE(flash_crc_tid, 1024, flash_crc_thread_fn, NULL, NULL, NULL, 7, 0, 0);
K_THREAD_DEFINE(aes_tid,       1536, aes_thread_fn,       NULL, NULL, NULL, 7, 0, 0);
#if DT_NODE_EXISTS(DT_ALIAS(demo_spi))
K_THREAD_DEFINE(spi_tid,        512, spi_thread_fn,       NULL, NULL, NULL, 7, 0, 0);
#endif
#if DT_NODE_EXISTS(DT_ALIAS(demo_i2c))
K_THREAD_DEFINE(i2c_tid,        768, i2c_thread_fn,       NULL, NULL, NULL, 7, 0, 0);
#endif

/* ================================================================
 * Entry point — BLE init only; peripheral work runs in threads above
 * ================================================================
 */
int main(void)
{
	int ret;

	LOG_INF("=== PM Peripheral Demo ===");
	LOG_INF("Board: %s", CONFIG_BOARD);

	/* BLE — IPC transport, no board overlay required */
	ret = bt_enable(NULL);
	if (ret) {
		LOG_ERR("bt_enable: %d", ret);
		return ret;
	}

	ret = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), NULL, 0);
	if (ret) {
		LOG_ERR("bt_le_adv_start: %d", ret);
		return ret;
	}

	LOG_INF("BLE: advertising as \"%s\"", CONFIG_BT_DEVICE_NAME);
	return 0;
}
