/*
 * Copyright 2022 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sd/sdio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/disk.h>
#include <zephyr/drivers/sdhc.h>
#include <zephyr/ztest.h>

#define BLK_SIZE           (256)
#define BLK_NUM            (8)
#define BYTE_TEST_SIZE     (128)
#define MULTIPLE_BLK_SIZE  (256*BLK_NUM)
#define CARD_PING_ADDR     (0x18000)

#define SPEED_START_INDEX 0
#define SPEED_END_INDEX   6

static const struct device *sdhc_dev = DEVICE_DT_GET(DT_ALIAS(sdhc0));
static struct sd_card card;
static uint8_t buf[MULTIPLE_BLK_SIZE] __aligned(CONFIG_SDHC_BUFFER_ALIGNMENT);
static uint8_t check_buf[MULTIPLE_BLK_SIZE] __aligned(CONFIG_SDHC_BUFFER_ALIGNMENT);

typedef struct
{
  const uint32_t     speed;
  const char         *string;
} sdio_speed_t;

sdio_speed_t sdio_test_speeds[] =
{
  { 48000000,    "48MHz" },
  { 24000000,    "24MHz" },
  { 12000000,    "12MHz" },
  {  3000000,     "3MHz" },
  {   750000,    "750KHz"},
  {   375000,    "375KHz"},
};

void prepare_data_pattern(uint32_t pattern_index, uint8_t* buff, uint32_t len)
{
    uint32_t *pui32TxPtr = (uint32_t*)buff;
    uint8_t  *pui8TxPtr  = (uint8_t*)buff;

    switch ( pattern_index )
    {
        case 0:
            // 0x5555AAAA
            for (uint32_t i = 0; i < len / 4; i++)
            {
               pui32TxPtr[i] = (0x5555AAAA);
            }
            break;
        case 1:
            // 0xFFFF0000
            for (uint32_t i = 0; i < len / 4; i++)
            {
               pui32TxPtr[i] = (0xFFFF0000);
            }
            break;
        case 2:
            // walking
            for (uint32_t i = 0; i < len; i++)
            {
               pui8TxPtr[i] = 0x01 << (i % 8);
            }
            break;
        case 3:
            // incremental from 1
            for (uint32_t i = 0; i < len; i++)
            {
               pui8TxPtr[i] = ((i + 1) & 0xFF);
            }
            break;
        case 4:
            // decremental from 0xff
            for ( uint32_t i = 0; i < len; i++ )
            {
                // decrement starting from 0xff
                pui8TxPtr[i] = (0xff - i) & 0xFF;
            }
            break;
        default:
            // incremental from 1
            for (uint32_t i = 0; i < len; i++)
            {
               pui8TxPtr[i] = ((i ) & 0xFF);
            }
            break;
    }
}

/*
 * Enable SDIO card function's interrupt
 */
static int sdio_card_func_interrupt_enable(void)
{
	int ret;
	uint8_t reg;

	card.func0.num = 0;
	/* Enable the interrupt function */
	ret = sdio_read_byte(&card.func0, SDIO_CCCR_INT_EN, &reg);
	if (ret) {
		return ret;
	}

	reg |= (BIT(SDIO_FUNC_NUM_1) | 0x1);
	ret = sdio_write_byte(&card.func0, SDIO_CCCR_INT_EN, reg);
	if (ret) {
		return ret;
	}

	card.func0.num = 1;

	return ret;
}

/*
 * Verify that SD stack can initialize an SDIO card.
 * This test must run first, to ensure the card is initialized.
 */
ZTEST(sd_stack, test_0_init)
{
	int ret;

	zassert_true(device_is_ready(sdhc_dev), "SDHC device is not ready");

	ret = sd_is_card_present(sdhc_dev);
	zassert_equal(ret, 1, "SD card not present in slot");

	card.switch_caps.bus_speed = UHS_SDR50_BUS_SPEED | HIGH_SPEED_BUS_SPEED | UHS_SDR12_BUS_SPEED;
	ret = sd_init(sdhc_dev, &card);
	zassert_equal(ret, 0, "Card initialization failed");
}

/*
 * Verify that a register read works. Given the custom nature of SDIO devices,
 * we just read from the card common I/O area.
 */
ZTEST(sd_stack, test_read)
{
	int ret;
	uint8_t reg = 0xFF;

	/* Read from card common I/O area. */
	ret = sdio_read_byte(&card.func0, SDIO_CCCR_CCCR, &reg);
	zassert_equal(ret, 0, "SD card read failed");
	/* Check to make sure CCCR read actually returned valid data */
	zassert_not_equal(reg, 0xFF, "CCCR read returned invalid data");
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
	case SD_TIMING_SDR12:
		TC_PRINT("Card timing: SDR12\n");
		break;
	case SD_TIMING_SDR25:
		TC_PRINT("Card timing: SDR25\n");
		break;
	case SD_TIMING_SDR50:
		TC_PRINT("Card timing: SDR50\n");
		break;
	case SD_TIMING_SDR104:
		TC_PRINT("Card timing: SDR104\n");
		break;
	case SD_TIMING_DDR50:
		TC_PRINT("Card timing: DDR50\n");
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
	default:
		zassert_unreachable("Card type is not known value");
	}
	if (card.sd_version >= SD_SPEC_VER3_0) {
		TC_PRINT("Card spec: 3.0\n");
	} else if (card.sd_version >= SD_SPEC_VER2_0) {
		TC_PRINT("Card spec: 2.0\n");
	} else if (card.sd_version >= SD_SPEC_VER1_1) {
		TC_PRINT("Card spec: 1.1\n");
	} else if (card.sd_version >= SD_SPEC_VER1_0) {
		TC_PRINT("Card spec: 1.0\n");
	} else {
		zassert_unreachable("Card spec is unknown value");
	}
}

/* SDIO Card Write Read test with default bus io setting*/
ZTEST(sd_stack, test_wr_default_bus_io)
{
    int ret;

    /* Now write nonzero data block */
    for (int i = 0; i < sizeof(buf); i++) {
        check_buf[i] = (uint8_t)i;
    }

    ret = sdio_init_func(&card, &card.func0, SDIO_FUNC_NUM_1);
    zassert_equal(ret, 0, "Function1 init failed");
    card.func0.num = SDIO_FUNC_NUM_1;
    ret = sdio_enable_func(&card.func0);
    zassert_equal(ret, 0, "Function1 enable failed");

    ret = sdio_card_func_interrupt_enable();
    zassert_equal(ret, 0, "Function1 interrupt enable failed");

    ret = sdio_set_block_size(&card.func0, BLK_SIZE);
    zassert_equal(ret, 0, "Block size set failed");

    TC_PRINT("\nSDIO Card Start bytes write read test\n");

    /* Write 128 bytes to sdio card. */
    ret = sdio_write_addr(&card.func0, CARD_PING_ADDR, check_buf, BYTE_TEST_SIZE);
    zassert_equal(ret, 0, "Write to card failed");

    memset(buf, 0, BYTE_TEST_SIZE);

    ret = sdio_read_addr(&card.func0, CARD_PING_ADDR, buf, BYTE_TEST_SIZE);
    zassert_equal(ret, 0, "SD card read failed");

    zassert_mem_equal(buf, check_buf, BYTE_TEST_SIZE, "Read of written area was not correct");

    /* Write bytes to sdio card. */
    for (int test_len = 4; test_len <= BLK_SIZE; test_len += 4)
    {
        ret = sdio_write_addr(&card.func0, CARD_PING_ADDR, check_buf, test_len);
        zassert_equal(ret, 0, "Write to card failed");

        memset(buf, 0, BYTE_TEST_SIZE);

        ret = sdio_read_addr(&card.func0, CARD_PING_ADDR, buf, test_len);
        zassert_equal(ret, 0, "SD card read failed");

        zassert_mem_equal(buf, check_buf, test_len, "Read of written area was not correct");
    }

    TC_PRINT("\nSDIO Card Start blocks write read test\n");
    /* Write blocks to sdio card. */
    for (int blk_num = 1; blk_num <= BLK_NUM; blk_num ++)
    {
        uint32_t test_len = blk_num*BLK_SIZE;
        ret = sdio_write_addr(&card.func0, CARD_PING_ADDR, check_buf, test_len);
        zassert_equal(ret, 0, "Write to card failed");

        memset(buf, 0, test_len);

        ret = sdio_read_addr(&card.func0, CARD_PING_ADDR, buf, test_len);
        zassert_equal(ret, 0, "SD card read failed");

        zassert_mem_equal(buf, check_buf,test_len, "Read of written area was not correct");
    }
}

#ifdef CONFIG_SDIO_CARD_1BITWIDTH
/* SDIO Card different clock test in 1bitwidth*/
ZTEST(sd_stack, test_wr_clock_1bit_width)
{
    int ret;

    ret = sd_init(sdhc_dev, &card);
    zassert_equal(ret, 0, "Card initialization failed");

    for (uint32_t speed_index = SPEED_START_INDEX; speed_index < SPEED_END_INDEX; speed_index++)
    {
        TC_PRINT("\nSDIO Card transefer clock = %s, bit width =%d\n",sdio_test_speeds[speed_index].string, card.bus_io.bus_width);

        card.bus_io.clock = sdio_test_speeds[speed_index].speed;
        card.bus_io.bus_width = SDHC_BUS_WIDTH1BIT;
        ret = sdhc_set_io(card.sdhc, &card.bus_io);
        zassert_equal(ret, 0, "Set sdhc io clock failed");

        for (int i = 0; i < sizeof(buf); i++) {
            check_buf[i] = (uint8_t)i;
        }

        ret = sdio_init_func(&card, &card.func0, SDIO_FUNC_NUM_1);
        zassert_equal(ret, 0, "Function1 init failed");

        card.func0.num = SDIO_FUNC_NUM_1;
        ret = sdio_enable_func(&card.func0);
        zassert_equal(ret, 0, "Function1 enable failed");

        ret = sdio_card_func_interrupt_enable();
        zassert_equal(ret, 0, "Function1 interrupt enable failed");

        ret = sdio_set_block_size(&card.func0, BLK_SIZE);
        zassert_equal(ret, 0, "Block size set failed");

        TC_PRINT("\nSDIO Card Start bytes write read test\n");

        /* Write bytes to sdio card. */
        for (int test_len = 4; test_len <= BLK_SIZE; test_len += 4)
        {
            ret = sdio_write_addr(&card.func0, CARD_PING_ADDR, check_buf, test_len);
            zassert_equal(ret, 0, "Write to card failed");

            memset(buf, 0, BYTE_TEST_SIZE);

            ret = sdio_read_addr(&card.func0, CARD_PING_ADDR, buf, test_len);
            zassert_equal(ret, 0, "SD card read failed");

            zassert_mem_equal(buf, check_buf, test_len, "Read of written area was not correct");
        }

        /* Now write nonzero data block */
        prepare_data_pattern(speed_index, check_buf, MULTIPLE_BLK_SIZE);

        TC_PRINT("\nSDIO Card Start blocks write data\n");
        /* Write blocks to sdio card. */
        for (int blk_num = 1; blk_num <= BLK_NUM; blk_num ++)
        {
            uint32_t test_len = blk_num*BLK_SIZE;
            ret = sdio_write_addr(&card.func0, CARD_PING_ADDR, check_buf, test_len);
            zassert_equal(ret, 0, "Write to card failed");

            memset(buf, 0, test_len);

            ret = sdio_read_addr(&card.func0, CARD_PING_ADDR, buf, test_len);
            zassert_equal(ret, 0, "SD card read failed");

            zassert_mem_equal(buf, check_buf,test_len, "Read of written area was not correct");
        }
    }
}
#else
/* SDIO Card different clock test in 4 bitwidth*/
ZTEST(sd_stack, test_wr_clock_4bit_width)
{
    int ret;

    ret = sd_init(sdhc_dev, &card);
    zassert_equal(ret, 0, "Card initialization failed");

    for (uint32_t speed_index = SPEED_START_INDEX; speed_index < SPEED_END_INDEX; speed_index++)
    {
        TC_PRINT("\nSDIO Card transefer clock = %s, bit width =%d\n",sdio_test_speeds[speed_index].string, card.bus_io.bus_width);

        card.bus_io.clock = sdio_test_speeds[speed_index].speed;
        card.bus_io.bus_width = SDHC_BUS_WIDTH4BIT;
        ret = sdhc_set_io(card.sdhc, &card.bus_io);
        zassert_equal(ret, 0, "Set sdhc io clock failed");

        for (int i = 0; i < sizeof(buf); i++) {
            check_buf[i] = (uint8_t)i;
        }

        ret = sdio_init_func(&card, &card.func0, SDIO_FUNC_NUM_1);
        zassert_equal(ret, 0, "Function1 init failed");

        card.func0.num = SDIO_FUNC_NUM_1;
        ret = sdio_enable_func(&card.func0);
        zassert_equal(ret, 0, "Function1 enable failed");

        ret = sdio_card_func_interrupt_enable();
        zassert_equal(ret, 0, "Function1 interrupt enable failed");

        ret = sdio_set_block_size(&card.func0, BLK_SIZE);
        zassert_equal(ret, 0, "Block size set failed");

        TC_PRINT("\nSDIO Card Start bytes write read test\n");

        /* Write bytes to sdio card. */
        for (int test_len = 4; test_len <= BLK_SIZE; test_len += 4)
        {
            ret = sdio_write_addr(&card.func0, CARD_PING_ADDR, check_buf, test_len);
            zassert_equal(ret, 0, "Write to card failed");

            memset(buf, 0, BYTE_TEST_SIZE);

            ret = sdio_read_addr(&card.func0, CARD_PING_ADDR, buf, test_len);
            zassert_equal(ret, 0, "SD card read failed");

            zassert_mem_equal(buf, check_buf, test_len, "Read of written area was not correct");
        }

        /* Now write nonzero data block */
        prepare_data_pattern(speed_index, check_buf, MULTIPLE_BLK_SIZE);

        TC_PRINT("\nSDIO Card Start blocks write data\n");
        /* Write blocks to sdio card. */
        for (int blk_num = 1; blk_num <= BLK_NUM; blk_num ++)
        {
            uint32_t test_len = blk_num*BLK_SIZE;
            ret = sdio_write_addr(&card.func0, CARD_PING_ADDR, check_buf, test_len);
            zassert_equal(ret, 0, "Write to card failed");

            memset(buf, 0, test_len);

            ret = sdio_read_addr(&card.func0, CARD_PING_ADDR, buf, test_len);
            zassert_equal(ret, 0, "SD card read failed");

            zassert_mem_equal(buf, check_buf,test_len, "Read of written area was not correct");
        }
    }
}

#endif

ZTEST_SUITE(sd_stack, NULL, NULL, NULL, NULL, NULL);
