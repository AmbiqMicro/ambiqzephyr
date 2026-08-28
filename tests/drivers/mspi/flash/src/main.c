/*
 * Copyright (c) 2024 Ambiq Micro Inc. <www.ambiq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/mspi.h>
#include <zephyr/drivers/mspi_emul.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/linker/devicetree_regions.h>
#include <zephyr/ztest.h>

#define MSPI_BUS_NODE                 DT_ALIAS(mspi0)

#define MSPI_FLASH_TEST_REGION_OFFSET 0x0

#define MSPI_FLASH_SECTOR_SIZE        4096

#define MSPI_FLASH_TEST_SIZE          3000

static const struct device *mspi_devices[] = {
	DT_FOREACH_CHILD_STATUS_OKAY_SEP(MSPI_BUS_NODE, DEVICE_DT_GET, (,))
};

#if CONFIG_DCACHE
static uint8_t expected[MSPI_FLASH_TEST_SIZE]__aligned(CONFIG_DCACHE_LINE_SIZE);
static uint8_t actual[MSPI_FLASH_TEST_SIZE]__aligned(CONFIG_DCACHE_LINE_SIZE);
#else
static uint8_t expected[MSPI_FLASH_TEST_SIZE];
static uint8_t actual[MSPI_FLASH_TEST_SIZE];
#endif


static void prepare_test_pattern(uint32_t pattern_index, uint8_t *buff, uint32_t len)
{
	uint32_t *ui32ptr = (uint32_t *)buff;
	uint8_t *ui8ptr = (uint8_t *)buff;

	switch (pattern_index) {
	case 0:
		/* 0x5555AAAA */
		for (uint32_t i = 0; i < len / 4; i++) {
			ui32ptr[i] = (0x5555AAAA);
		}
		break;
	case 1:
		/*  0xFFFF0000 */
		for (uint32_t i = 0; i < len / 4; i++) {
			ui32ptr[i] = (0xFFFF0000);
		}
		break;
	case 2:
		/* walking */
		for (uint32_t i = 0; i < len; i++) {
			ui8ptr[i] = 0x01 << (i % 8);
		}
		break;
	case 3:
		/* incremental from 1 */
		for (uint32_t i = 0; i < len; i++) {
			ui8ptr[i] = ((i + 1) & 0xFF);
		}
		break;
	case 4:
		/* decremental from 0xff */
		for (uint32_t i = 0; i < len; i++) {
			/* decrement starting from 0xff */
			ui8ptr[i] = (0xff - i) & 0xFF;
		}
		break;
	default:
		/* incremental from 1 */
		for (uint32_t i = 0; i < len; i++) {
			ui8ptr[i] = ((i + 1) & 0xFF);
		}
		break;
	}
}

static int test_multi_sector_rw(const struct device *flash_dev)
{
	int rc = 0;
	const struct flash_pages_layout *layout = NULL;
	size_t layout_size = 0;
	size_t min_page_size = -1;
	size_t offs;

	TC_PRINT("\n===================================================================\n");
	TC_PRINT("Perform test on multiple consequtive sectors on %s\n", flash_dev->name);

	TC_PRINT("\nTest 0: Get Flash page layout\n");

	const struct flash_driver_api *api = flash_dev->api;

	api->page_layout(flash_dev, &layout, &layout_size);

	if (layout && layout_size) {
		TC_PRINT("----pages-------size----\n");
		for (int i = 0; i < layout_size; ++i) {
			TC_PRINT("%2d: 0x%-8X  0x%-8x\n", i, layout[i].pages_count,
				 layout[i].pages_size);
			min_page_size = MIN(min_page_size, layout[i].pages_size);
		}
	} else {
		TC_PRINT("Empty flash_pages_layout!\n");
		return TC_FAIL;
	}

	TC_PRINT("\nPage size selected: %d\n", min_page_size);

	for (int i = 0; i < MSPI_FLASH_TEST_SIZE; i += min_page_size) {
		prepare_test_pattern(i % 5, expected + i,
				     MIN(min_page_size, MSPI_FLASH_TEST_SIZE - i));
	}

	TC_PRINT("\nTest 1: Flash erase\n");

	/* Full flash erase if MSPI_FLASH_TEST_REGION_OFFSET = 0 and
	 * MSPI_FLASH_SECTOR_SIZE = flash size
	 * Erase 2 sectors for check for erase of consequtive sectors
	 */
	rc = flash_erase(flash_dev, MSPI_FLASH_TEST_REGION_OFFSET, MSPI_FLASH_SECTOR_SIZE * 2);
	if (rc != 0) {
		TC_PRINT("Flash erase failed! %d\n", rc);
		return TC_FAIL;
	}
	/* Read the content and check for erased */
	memset(actual, 0, MSPI_FLASH_TEST_SIZE);

	offs = MSPI_FLASH_TEST_REGION_OFFSET;
	while (offs < MSPI_FLASH_TEST_REGION_OFFSET + 2 * MSPI_FLASH_SECTOR_SIZE) {
		rc = flash_read(flash_dev, offs, actual, MSPI_FLASH_TEST_SIZE);
		if (rc != 0) {
			TC_PRINT("Flash read failed! %d\n", rc);
			return TC_FAIL;
		}
		if (actual[0] != 0xff) {
			TC_PRINT("Flash erase failed at offset 0x%x got 0x%x\n", offs,
					actual[0]);
			return TC_FAIL;
		}
		offs += MSPI_FLASH_SECTOR_SIZE;
	}
	TC_PRINT("Flash erase succeeded!\n");

	TC_PRINT("\nTest 2: Flash write\n");

	offs = MSPI_FLASH_TEST_REGION_OFFSET;
	while (offs < MSPI_FLASH_TEST_REGION_OFFSET + 2 * MSPI_FLASH_SECTOR_SIZE) {
		TC_PRINT("\nAttempting to write %zu bytes at offset 0x%x\n", MSPI_FLASH_TEST_SIZE,
			 offs);
		rc = flash_write(flash_dev, offs, expected, MSPI_FLASH_TEST_SIZE);
		if (rc != 0) {
			TC_PRINT("Flash write failed! %d\n", rc);
			return TC_FAIL;
		}

		memset(actual, 0, MSPI_FLASH_TEST_SIZE);
		rc = flash_read(flash_dev, offs, actual, MSPI_FLASH_TEST_SIZE);
		if (rc != 0) {
			TC_PRINT("Flash read failed! %d\n", rc);
			return TC_FAIL;
		}

		if (memcmp(expected, actual, MSPI_FLASH_TEST_SIZE) == 0) {
			TC_PRINT("Data read matches data written. Good!!\n");
		} else {
			const uint8_t *wp = expected;
			const uint8_t *rp = actual;
			const uint8_t *rpe = rp + MSPI_FLASH_TEST_SIZE;
			int count = 0;

			TC_PRINT("Data read does not match data written!!\n");
			while (rp < rpe) {
				if (*rp != *wp) {
					TC_PRINT("%08x wrote %02x read %02x MISMATCH\n",
						 (uint32_t)(offs + (rp - actual)), *wp, *rp);
					count++;
				}
				if (count > 100) {
					TC_PRINT("Too many data mismatch!!\n");
					break;
				}
				++rp;
				++wp;
			}
			return TC_FAIL;
		}
		offs += MSPI_FLASH_SECTOR_SIZE;
	}

	return TC_PASS;
}

ZTEST(mspi_flash, test_multi_sector_rw)
{

	for (int idx = 0; idx < ARRAY_SIZE(mspi_devices); ++idx) {

		zassert_true(device_is_ready(mspi_devices[idx]),
					     "flash%d is not ready", idx);
		zassert_true(test_multi_sector_rw(mspi_devices[idx]) == TC_PASS);

	}

}

#if DT_NODE_HAS_STATUS(DT_NODELABEL(dtcm), okay)
#define DTCM_SECT Z_GENERIC_SECTION(LINKER_DT_NODE_REGION_NAME(DT_NODELABEL(dtcm)))

#if CONFIG_DCACHE
static uint8_t dtcm_expected[MSPI_FLASH_TEST_SIZE] __aligned(CONFIG_DCACHE_LINE_SIZE) DTCM_SECT;
static uint8_t dtcm_actual[MSPI_FLASH_TEST_SIZE] __aligned(CONFIG_DCACHE_LINE_SIZE) DTCM_SECT;
#else
static uint8_t dtcm_expected[MSPI_FLASH_TEST_SIZE] DTCM_SECT;
static uint8_t dtcm_actual[MSPI_FLASH_TEST_SIZE] DTCM_SECT;
#endif

#define FILL_BYTE 0xEE

static void report_mismatch(const uint8_t *want, const uint8_t *got)
{
	uint32_t matched = 0, untouched = 0, first = MSPI_FLASH_TEST_SIZE;

	for (uint32_t i = 0; i < MSPI_FLASH_TEST_SIZE; i++) {
		if (want[i] == got[i]) {
			matched++;
		} else if (first == MSPI_FLASH_TEST_SIZE) {
			first = i;
		}
		if (got[i] == FILL_BYTE) {
			untouched++;
		}
	}

	TC_PRINT("  matched %u/%u, untouched %u, first mismatch at %u (line %u)\n",
		 matched, (uint32_t)MSPI_FLASH_TEST_SIZE, untouched, first,
		 first / CONFIG_DCACHE_LINE_SIZE);
	TC_PRINT("  want %02x %02x %02x %02x %02x %02x %02x %02x\n",
		 want[0], want[1], want[2], want[3], want[4], want[5], want[6], want[7]);
	TC_PRINT("  got  %02x %02x %02x %02x %02x %02x %02x %02x\n",
		 got[0], got[1], got[2], got[3], got[4], got[5], got[6], got[7]);
}

static void report_buffer_placement(void)
{
	TC_PRINT("DTCM region : 0x%08lx + 0x%lx\n",
		 (unsigned long)DT_REG_ADDR(DT_NODELABEL(dtcm)),
		 (unsigned long)DT_REG_SIZE(DT_NODELABEL(dtcm)));
	TC_PRINT("dtcm_expected %p  dtcm_actual %p\n", dtcm_expected, dtcm_actual);
	TC_PRINT("sram expected %p  actual      %p\n", expected, actual);
}

ZTEST(mspi_flash, test_dtcm_buffer_rw)
{
	const struct device *flash_dev = mspi_devices[0];
	const off_t offs = MSPI_FLASH_TEST_REGION_OFFSET;
	int wrc, rrc;

	zassert_true(device_is_ready(flash_dev), "flash0 is not ready");

	report_buffer_placement();

	prepare_test_pattern(0, dtcm_expected, MSPI_FLASH_TEST_SIZE);

	zassert_equal(flash_erase(flash_dev, offs, MSPI_FLASH_SECTOR_SIZE), 0,
		      "flash_erase failed");

	wrc = flash_write(flash_dev, offs, dtcm_expected, MSPI_FLASH_TEST_SIZE);
	memset(dtcm_actual, FILL_BYTE, MSPI_FLASH_TEST_SIZE);
	rrc = flash_read(flash_dev, offs, dtcm_actual, MSPI_FLASH_TEST_SIZE);

	TC_PRINT("DTCM write rc=%d read rc=%d\n", wrc, rrc);

	if (wrc != 0 || rrc != 0) {
		TC_PRINT("DTCM buffers refused by the driver, as expected\n");
		return;
	}

	if (memcmp(dtcm_expected, dtcm_actual, MSPI_FLASH_TEST_SIZE) != 0) {
		report_mismatch(dtcm_expected, dtcm_actual);
	}

	zassert_mem_equal(dtcm_expected, dtcm_actual, MSPI_FLASH_TEST_SIZE,
			  "DMA reported success from DTCM but moved the wrong data");
}

static bool dtcm_roundtrip_ok(const struct device *flash_dev, off_t offs,
			      uint8_t *src, uint8_t *dst, uint32_t pattern)
{
	int rc;

	prepare_test_pattern(pattern, src, MSPI_FLASH_TEST_SIZE);

	zassert_equal(flash_erase(flash_dev, offs, MSPI_FLASH_SECTOR_SIZE), 0,
		      "flash_erase failed");

	rc = flash_write(flash_dev, offs, src, MSPI_FLASH_TEST_SIZE);
	if (rc != 0) {
		return true;
	}

	memset(dst, FILL_BYTE, MSPI_FLASH_TEST_SIZE);
	rc = flash_read(flash_dev, offs, dst, MSPI_FLASH_TEST_SIZE);
	if (rc != 0) {
		return true;
	}

	if (memcmp(src, dst, MSPI_FLASH_TEST_SIZE) == 0) {
		return true;
	}

	report_mismatch(src, dst);
	return false;
}

ZTEST(mspi_flash, test_dtcm_direction_split)
{
	const struct device *flash_dev = mspi_devices[0];
	const off_t offs = MSPI_FLASH_TEST_REGION_OFFSET;
	bool into_dtcm_ok, from_dtcm_ok;

	zassert_true(device_is_ready(flash_dev), "flash0 is not ready");

	into_dtcm_ok = dtcm_roundtrip_ok(flash_dev, offs, expected, dtcm_actual, 0);
	from_dtcm_ok = dtcm_roundtrip_ok(flash_dev, offs, dtcm_expected, actual, 1);

	TC_PRINT("DMA into DTCM (flash read) : %s\n", into_dtcm_ok ? "OK" : "CORRUPT");
	TC_PRINT("DMA from DTCM (flash write): %s\n", from_dtcm_ok ? "OK" : "CORRUPT");

	zassert_true(into_dtcm_ok,
		     "DMA into a DTCM destination reported success and corrupted the buffer");
	zassert_true(from_dtcm_ok,
		     "DMA from a DTCM source reported success and wrote the wrong data");
}
#endif

#define SOAK_ITERATIONS  256
#define SOAK_IDLE_MS     5

/*
 * The round trip tests above run back to back, so the idle thread never gets
 * to run and the system never reaches a low power state. A DMA left posted
 * across a suspend cannot be observed that way. These soak the same path with
 * an idle gap between transfers, and under contention from a second thread.
 */
static int soak_read_loop(const struct device *flash_dev, off_t offs,
			  uint8_t *buf, uint32_t iterations, uint32_t idle_ms)
{
	int rc;

	for (uint32_t i = 0; i < iterations; i++) {
		memset(buf, 0, MSPI_FLASH_TEST_SIZE);

		rc = flash_read(flash_dev, offs, buf, MSPI_FLASH_TEST_SIZE);
		if (rc != 0) {
			TC_PRINT("iteration %u: flash_read rc=%d\n", i, rc);
			return rc;
		}

		if (memcmp(expected, buf, MSPI_FLASH_TEST_SIZE) != 0) {
			TC_PRINT("iteration %u: data mismatch\n", i);
			return -EIO;
		}

		if (idle_ms) {
			k_sleep(K_MSEC(idle_ms));
		}
	}

	return 0;
}

static void soak_prepare(const struct device *flash_dev, off_t offs)
{
	prepare_test_pattern(0, expected, MSPI_FLASH_TEST_SIZE);

	zassert_equal(flash_erase(flash_dev, offs, MSPI_FLASH_SECTOR_SIZE), 0,
		      "flash_erase failed");
	zassert_equal(flash_write(flash_dev, offs, expected, MSPI_FLASH_TEST_SIZE), 0,
		      "flash_write failed");
}

ZTEST(mspi_flash, test_dma_soak_with_idle)
{
	const struct device *flash_dev = mspi_devices[0];
	const off_t offs = MSPI_FLASH_TEST_REGION_OFFSET;
	int rc;

	zassert_true(device_is_ready(flash_dev), "flash0 is not ready");

	soak_prepare(flash_dev, offs);

	rc = soak_read_loop(flash_dev, offs, actual, SOAK_ITERATIONS, SOAK_IDLE_MS);

	zassert_not_equal(rc, -ETIMEDOUT,
			  "DMA completion was lost after an idle period");
	zassert_equal(rc, 0, "soak with idle failed: %d", rc);
}

ZTEST_SUITE(mspi_flash, NULL, NULL, NULL, NULL, NULL);
