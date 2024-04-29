/*
 * Copyright 2022 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/disk.h>
#include <zephyr/drivers/sdhc.h>
#include <zephyr/sd/mmc.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#define SECTOR_COUNT   32
#define SECTOR_SIZE    512 /* subsystem should set all cards to 512 byte blocks */
#define BUF_SIZE       (SECTOR_SIZE * SECTOR_COUNT)
#define EMMC_BLOCK_NUM (0x760000)

#define SPEED_START_INDEX (0)
#define SPEED_END_INDEX   (2)

#define WIDTH_START_INDEX (0)
#define WIDTH_END_INDEX   (3)

static const struct device *const sdhc_dev = DEVICE_DT_GET(DT_ALIAS(sdhc0));
static struct sd_card card;
static uint8_t buf[BUF_SIZE] __aligned(CONFIG_SDHC_BUFFER_ALIGNMENT);
static uint8_t check_buf[BUF_SIZE] __aligned(CONFIG_SDHC_BUFFER_ALIGNMENT);
static uint32_t sector_size;
static uint32_t sector_count;

#define MMC_UNALIGN_OFFSET 1

typedef struct {
	enum sdhc_bus_width width;
	const char *string;
} sdio_width_t;

typedef struct {
	const uint32_t speed;
	const char *string;
} sdio_speed_t;

sdio_speed_t sdio_test_speeds[] = {
	{96000000, "96MHz"}, {48000000, "48MHz"}, {24000000, "24MHz"}, {12000000, "12MHz"},
	{3000000, "3MHz"},   {750000, "750KHz"},  {375000, "375KHz"},
};

sdio_width_t sdio_test_widths[] = {
	{SDHC_BUS_WIDTH1BIT, "1bit"},
	{SDHC_BUS_WIDTH4BIT, "4bit"},
	{SDHC_BUS_WIDTH8BIT, "8bit"},
};

void prepare_data_pattern(uint32_t pattern_index, uint8_t *buff, uint32_t len)
{
	uint32_t *pui32TxPtr = (uint32_t *)buff;
	uint8_t *pui8TxPtr = (uint8_t *)buff;

	switch (pattern_index) {
	case 0:
		// 0x5555AAAA
		for (uint32_t i = 0; i < len / 4; i++) {
			pui32TxPtr[i] = (0x5555AAAA);
		}
		break;
	case 1:
		// 0xFFFF0000
		for (uint32_t i = 0; i < len / 4; i++) {
			pui32TxPtr[i] = (0xFFFF0000);
		}
		break;
	case 2:
		// walking
		for (uint32_t i = 0; i < len; i++) {
			pui8TxPtr[i] = 0x01 << (i % 8);
		}
		break;
	case 3:
		// incremental from 1
		for (uint32_t i = 0; i < len; i++) {
			pui8TxPtr[i] = ((i + 1) & 0xFF);
		}
		break;
	case 4:
		// decremental from 0xff
		for (uint32_t i = 0; i < len; i++) {
			// decrement starting from 0xff
			pui8TxPtr[i] = (0xff - i) & 0xFF;
		}
		break;
	default:
		// incremental from 1
		for (uint32_t i = 0; i < len; i++) {
			pui8TxPtr[i] = ((i) & 0xFF);
		}
		break;
	}
}

/*
 * Verify that SD stack can initialize an MMC card
 * This test must run first, to ensure the card is initialized.
 */
ZTEST(sd_stack, test_0_init)
{
	int ret;

	zassert_true(device_is_ready(sdhc_dev), "SDHC device is not ready");

	card.bus_width = sdio_test_widths[2].width;
	ret = sd_init(sdhc_dev, &card);

	zassert_equal(ret, 0, "Card initialization failed");
}

/* Verify that MMC stack returns valid IOCTL values */
ZTEST(sd_stack, test_ioctl)
{
	int ret;

	ret = mmc_ioctl(&card, DISK_IOCTL_GET_SECTOR_COUNT, &sector_count);
	zassert_equal(ret, 0, "IOCTL sector count read failed");
	TC_PRINT("SD card reports sector count of %d\n", sector_count);

	ret = mmc_ioctl(&card, DISK_IOCTL_GET_SECTOR_SIZE, &sector_size);
	zassert_equal(ret, 0, "IOCTL sector size read failed");
	TC_PRINT("SD card reports sector size of %d\n", sector_size);
}

/* Verify that SD stack can read from an SD card */
ZTEST(sd_stack, test_read)
{
	int ret;
	int block_addr = 0;

	/* Try simple reads from start of SD card */

	ret = mmc_read_blocks(&card, buf, block_addr, 1);
	zassert_equal(ret, 0, "Single block card read failed");

	ret = mmc_read_blocks(&card, buf, block_addr, SECTOR_COUNT / 2);
	zassert_equal(ret, 0, "Multiple block card read failed");

	/* Try a series of reads from the same block */
	block_addr = sector_count / 2;
	for (int i = 0; i < 10; i++) {
		ret = mmc_read_blocks(&card, buf, block_addr, SECTOR_COUNT);
		zassert_equal(ret, 0, "Multiple reads from same addr failed");
	}
	/* Verify that out of bounds read fails */
	block_addr = sector_count;
	ret = mmc_read_blocks(&card, buf, block_addr, 1);
	zassert_not_equal(ret, 0, "Out of bounds read should fail");

	block_addr = sector_count - 2;
	ret = mmc_read_blocks(&card, buf, block_addr, 2);
	zassert_equal(ret, 0, "Read from end of card failed");

	/* Verify that unaligned reads work */
	block_addr = 3;
	ret = mmc_read_blocks(&card, buf + MMC_UNALIGN_OFFSET, block_addr, SECTOR_COUNT - 1);
	zassert_equal(ret, 0, "Unaligned read failed");
}

/* Verify that SD stack can write to an SD card */
ZTEST(sd_stack, test_write)
{
	int ret;
	int block_addr = 0;

	/* Try simple writes from start of SD card */

	ret = mmc_write_blocks(&card, buf, block_addr, 1);
	zassert_equal(ret, 0, "Single block card write failed");

	ret = mmc_write_blocks(&card, buf, block_addr, SECTOR_COUNT / 2);
	zassert_equal(ret, 0, "Multiple block card write failed");

	/* Try a series of reads from the same block */
	block_addr = sector_count / 2;
	for (int i = 0; i < 10; i++) {
		ret = mmc_write_blocks(&card, buf, block_addr, SECTOR_COUNT);
		zassert_equal(ret, 0, "Multiple writes to same addr failed");
	}
	/* Verify that out of bounds write fails */
	block_addr = sector_count;
	ret = mmc_write_blocks(&card, buf, block_addr, 1);
	zassert_not_equal(ret, 0, "Out of bounds write should fail");

	block_addr = sector_count - 2;
	ret = mmc_write_blocks(&card, buf, block_addr, 2);
	zassert_equal(ret, 0, "Write to end of card failed");

	/* Verify that unaligned writes work */
	block_addr = 3;
	ret = mmc_write_blocks(&card, buf + MMC_UNALIGN_OFFSET, block_addr, SECTOR_COUNT - 1);
	zassert_equal(ret, 0, "Unaligned write failed");
}

/* Test reads and writes interleaved, to verify data is making it on disk */
ZTEST(sd_stack, test_wr)
{
	int ret;
	int block_addr = 0;

	/* Zero the write buffer */
	memset(buf, 0, BUF_SIZE);
	memset(check_buf, 0, BUF_SIZE);
	ret = mmc_write_blocks(&card, buf, block_addr, SECTOR_COUNT / 2);
	zassert_equal(ret, 0, "Write to card failed");
	/* Verify that a read from this area is empty */
	ret = mmc_read_blocks(&card, buf, block_addr, SECTOR_COUNT / 2);
	zassert_equal(ret, 0, "Read from card failed");
	zassert_mem_equal(buf, check_buf, BUF_SIZE, "Read of erased area was not zero");

	/* Now write nonzero data block */
	for (int i = 0; i < sizeof(buf); i++) {
		check_buf[i] = buf[i] = (uint8_t)i;
	}

	ret = mmc_write_blocks(&card, buf, block_addr, SECTOR_COUNT);
	zassert_equal(ret, 0, "Write to card failed");
	/* Clear the read buffer, then write to it again */
	memset(buf, 0, BUF_SIZE);
	ret = mmc_read_blocks(&card, buf, block_addr, SECTOR_COUNT);
	zassert_equal(ret, 0, "Read from card failed");
	zassert_mem_equal(buf, check_buf, BUF_SIZE, "Read of written area was not correct");

	block_addr = (sector_count / 3);
	for (int i = 0; i < 10; i++) {
		/* Verify that unaligned writes work */
		ret = mmc_write_blocks(&card, buf + MMC_UNALIGN_OFFSET, block_addr,
				       SECTOR_COUNT - 1);
		zassert_equal(ret, 0, "Write to card failed");
		/* Zero check buffer and read into it */
		memset(check_buf + MMC_UNALIGN_OFFSET, 0, (SECTOR_COUNT - 1) * sector_size);
		ret = mmc_read_blocks(&card, check_buf + MMC_UNALIGN_OFFSET, block_addr,
				      (SECTOR_COUNT - 1));
		zassert_equal(ret, 0, "Read from card failed");
		zassert_mem_equal(buf + MMC_UNALIGN_OFFSET, check_buf + MMC_UNALIGN_OFFSET,
				  (SECTOR_COUNT - 1) * sector_size,
				  "Unaligned read of written area was not correct");
	}
}

/* Simply dump the card configuration. */
ZTEST(sd_stack, test_card_config)
{
	switch (card.card_voltage) {
	case SD_VOL_1_2_V:
		TC_PRINT("Card voltage: 1.2V\n");
		break;
	case SD_VOL_1_8_V:
		TC_PRINT("Card voltage: 1.8V\n");
		break;
	case SD_VOL_3_0_V:
		TC_PRINT("Card voltage: 3.0V\n");
		break;
	case SD_VOL_3_3_V:
		TC_PRINT("Card voltage: 3.3V\n");
		break;
	default:
		zassert_unreachable("Card voltage is not known value");
	}
	zassert_equal(card.status, CARD_INITIALIZED, "Card status is not OK");
	switch (card.card_speed) {
	case MMC_LEGACY_TIMING:
		TC_PRINT("Card timing: Legacy MMC\n");
		break;
	case MMC_HS_TIMING:
		TC_PRINT("Card timing: High Speed MMC\n");
		break;
	case MMC_HS200_TIMING:
		TC_PRINT("Card timing: MMC HS200\n");
		break;
	case MMC_HS400_TIMING:
		TC_PRINT("Card timing: MMC HS400\n");
		break;
	default:
		zassert_unreachable("Card timing is not known value");
	}
	switch (card.type) {
	case CARD_SDIO:
		TC_PRINT("Card type: SDIO\n");
		break;
	case CARD_SDMMC:
		TC_PRINT("Card type: SDMMC\n");
		break;
	case CARD_COMBO:
		TC_PRINT("Card type: combo card\n");
		break;
	case CARD_MMC:
		TC_PRINT("Card type: MMC\n");
		break;
	default:
		zassert_unreachable("Card type is not known value");
	}
}

#ifdef CONFIG_MMC_DDR50
ZTEST(sd_stack, test_wr_multiple_mode_ddr)
{
	int ret;
	int block_addr = 0;
	int blk_cnt = 0;
	int loopcnt = 0;

	for (uint32_t bitwidth = 1; bitwidth < WIDTH_END_INDEX; bitwidth++) {
		for (uint32_t speed_index = 1; speed_index < SPEED_END_INDEX; speed_index++) {

			card.bus_width = sdio_test_widths[bitwidth].width;
			card.bus_io.clock = sdio_test_speeds[speed_index].speed;
			card.bus_io.timing = SDHC_TIMING_DDR50;
			card.host_props.host_caps.hs200_support = false;

			ret = sd_init(sdhc_dev, &card);
			zassert_equal(ret, 0, "mmc init failed");

			TC_PRINT("MMC DDR write read test width:%s speed:%s\n",
				 sdio_test_widths[bitwidth].string,
				 sdio_test_speeds[speed_index].string);

			for (blk_cnt = 1; blk_cnt <= SECTOR_COUNT; blk_cnt += SECTOR_COUNT / 4) {
				for (block_addr = 0; block_addr < EMMC_BLOCK_NUM;
				     block_addr += EMMC_BLOCK_NUM / 4) {
					TC_PRINT("MMC DDR write read write start block 0x%x, block "
						 "cnt = %d\n",
						 block_addr, blk_cnt);

					/* Now write nonzero data block */
					prepare_data_pattern(loopcnt % 5, check_buf, BUF_SIZE);

					ret = mmc_write_blocks(&card, check_buf, block_addr,
							       blk_cnt);
					zassert_equal(ret, 0, "Write to card failed");
					/* Clear the read buffer, then write to it again */
					memset(buf, 0, BUF_SIZE);
					ret = mmc_read_blocks(&card, buf, block_addr, blk_cnt);
					zassert_equal(ret, 0, "Read from card failed");
					zassert_mem_equal(buf, check_buf, SECTOR_SIZE * blk_cnt,
							  "Read of written area was not correct");
					loopcnt++;
				}
			}
		}
	}
}
#else
/* Test reads and writes in different mode settings */
ZTEST(sd_stack, test_wr_multiple_mode)
{
	int ret;
	int block_addr = 0;
	int blk_cnt = 0;
	int loopcnt = 0;

	/* Zero the write buffer */
	memset(buf, 0, BUF_SIZE);
	memset(check_buf, 0, BUF_SIZE);
	ret = mmc_write_blocks(&card, buf, block_addr, SECTOR_COUNT / 2);
	zassert_equal(ret, 0, "Write to card failed");
	/* Verify that a read from this area is empty */
	ret = mmc_read_blocks(&card, buf, block_addr, SECTOR_COUNT / 2);
	zassert_equal(ret, 0, "Read from card failed");
	zassert_mem_equal(buf, check_buf, BUF_SIZE, "Read of erased area was not zero");

	for (uint32_t bitwidth = WIDTH_START_INDEX; bitwidth < WIDTH_END_INDEX; bitwidth++) {
		for (uint32_t speed_index = SPEED_START_INDEX; speed_index < SPEED_END_INDEX;
		     speed_index++) {

			card.bus_width = sdio_test_widths[bitwidth].width;
			card.bus_io.clock = sdio_test_speeds[speed_index].speed;

			if ((sdio_test_speeds[speed_index].speed == 96000000 &&
			     card.host_props.host_caps.hs200_support == false) ||
			    (sdio_test_speeds[speed_index].speed == 48000000 &&
			     card.host_props.host_caps.hs200_support == true)) {
				continue;
			}

			if ((sdio_test_speeds[speed_index].speed == 96000000) &&
			    (sdio_test_widths[bitwidth].width == SDHC_BUS_WIDTH1BIT)) {
				continue;
			}

			ret = sd_init(sdhc_dev, &card);
			zassert_equal(ret, 0, "mmc init failed");

			TC_PRINT("MMC write read test width:%s speed:%s\n",
				 sdio_test_widths[bitwidth].string,
				 sdio_test_speeds[speed_index].string);

			for (blk_cnt = 1; blk_cnt <= SECTOR_COUNT; blk_cnt += SECTOR_COUNT / 4) {
				for (block_addr = 0; block_addr < EMMC_BLOCK_NUM;
				     block_addr += EMMC_BLOCK_NUM / 4) {
					TC_PRINT("MMC write read write start block 0x%x, block cnt "
						 "= %d\n",
						 block_addr, blk_cnt);

					/* Now write nonzero data block */
					prepare_data_pattern(loopcnt % 5, check_buf, BUF_SIZE);

					ret = mmc_write_blocks(&card, check_buf, block_addr,
							       blk_cnt);
					zassert_equal(ret, 0, "Write to card failed");
					/* Clear the read buffer, then write to it again */
					memset(buf, 0, BUF_SIZE);
					ret = mmc_read_blocks(&card, buf, block_addr, blk_cnt);
					zassert_equal(ret, 0, "Read from card failed");
					zassert_mem_equal(buf, check_buf, SECTOR_SIZE * blk_cnt,
							  "Read of written area was not correct");
					loopcnt++;
				}
			}
		}
	}
}
#endif

ZTEST_SUITE(sd_stack, NULL, NULL, NULL, NULL, NULL);
