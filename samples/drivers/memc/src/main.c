/*
 * Copyright 2022 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/drivers/mspi.h>

#if DT_HAS_COMPAT_STATUS_OKAY(nxp_imx_flexspi)
/* Use memc API to get AHB base address for the device */
#include "memc_mcux_flexspi.h"
#define FLEXSPI_DEV DEVICE_DT_GET(DT_PARENT(DT_ALIAS(sram_ext)))
#define MEMC_PORT DT_REG_ADDR(DT_ALIAS(sram_ext))
#define MEMC_BASE memc_flexspi_get_ahb_address(FLEXSPI_DEV, MEMC_PORT, 0)
#define MEMC_SIZE (DT_PROP(DT_ALIAS(sram_ext), size) / 8)
#elif DT_HAS_COMPAT_STATUS_OKAY(ambiq_mspi_controller)
#define MSPI_ DT_BUS(DT_ALIAS(psram0))
#define mspi_get_xip_address(controller) DT_REG_ADDR_BY_IDX(controller, 1)
#define MEMC_BASE (void *)(mspi_get_xip_address(MSPI_))
#define MEMC_SIZE (DT_PROP(DT_ALIAS(psram0), size) / 8)
#endif

void dump_memory(uint8_t *p, uint32_t size)
{
	uint32_t i, j;

	for (i = 0, j = 0; j < (size / 16); i += 16, j++) {
		printk("%02x %02x %02x %02x %02x %02x %02x %02x ",
			p[i], p[i+1], p[i+2], p[i+3],
			p[i+4], p[i+5], p[i+6], p[i+7]);
		printk("%02x %02x %02x %02x %02x %02x %02x %02x\n",
			p[i+8], p[i+9], p[i+10], p[i+11],
			p[i+12], p[i+13], p[i+14], p[i+15]);
		/* Split dump at 256B boundaries */
		if (((i + 16) & 0xFF) == 0) {
			printk("\n");
		}
	}
	/* Dump any remaining data after 16 byte blocks */
	for (; i < size; i++) {
		printk("%02x ", p[i]);
	}
	printk("\n");
}

#define BUF_SIZE 1024

uint8_t memc_write_buffer[BUF_SIZE];
uint8_t memc_read_buffer[BUF_SIZE];

void async_cb(struct mspi_callback_context *mspi_cb_ctx, uint32_t status)
{
	mspi_cb_ctx->mspi_evt.evt_data.status = status;
	if (mspi_cb_ctx->mspi_evt.evt_data.payload_idx == mspi_cb_ctx->mspi_evt.evt_data.xfer->ui32NumPayload - 1) {
		*(volatile uint32_t *)mspi_cb_ctx->ctx = 0;
	}
}

struct mspi_buf buf1[] = {
	{
		.ui16DeviceInstr = 0x38,
		.ui32DeviceAddr = 0,
		.ui32NumBytes = 256,
		.pui32Buffer = memc_write_buffer,
		.eCBMask = MSPI_BUS_NO_CB,
	},
	{
		.ui16DeviceInstr = 0x38,
		.ui32DeviceAddr = 256,
		.ui32NumBytes = 256,
		.pui32Buffer = memc_write_buffer + 256,
		.eCBMask = MSPI_BUS_NO_CB,
	},
	{
		.ui16DeviceInstr = 0x38,
		.ui32DeviceAddr = 512,
		.ui32NumBytes = 256,
		.pui32Buffer = memc_write_buffer + 512,
		.eCBMask = MSPI_BUS_NO_CB,
	},
	{
		.ui16DeviceInstr = 0x38,
		.ui32DeviceAddr = 512 + 256,
		.ui32NumBytes = 256,
		.pui32Buffer = memc_write_buffer + 512 + 256,
		.eCBMask = MSPI_BUS_XFER_COMPLETE_CB,
	},
};

struct mspi_buf buf2[] = {
	{
		.ui16DeviceInstr = 0xEB,
		.ui32DeviceAddr = 0,
		.ui32NumBytes = 256,
		.pui32Buffer = memc_read_buffer,
		.eCBMask = MSPI_BUS_NO_CB,
	},
	{
		.ui16DeviceInstr = 0xEB,
		.ui32DeviceAddr = 256,
		.ui32NumBytes = 256,
		.pui32Buffer = memc_read_buffer + 256,
		.eCBMask = MSPI_BUS_NO_CB,
	},
	{
		.ui16DeviceInstr = 0xEB,
		.ui32DeviceAddr = 512,
		.ui32NumBytes = 256,
		.pui32Buffer = memc_read_buffer + 512,
		.eCBMask = MSPI_BUS_NO_CB,
	},
	{
		.ui16DeviceInstr = 0xEB,
		.ui32DeviceAddr = 512 + 256,
		.ui32NumBytes = 256,
		.pui32Buffer = memc_read_buffer + 512 + 256,
		.eCBMask = MSPI_BUS_XFER_COMPLETE_CB,
	},
};

struct mspi_xfer_packet pack1 = {
	.eMode = MSPI_DMA,
	.eDirection = MSPI_TX,
	.ui32TXDummy = 0,
	.ui16InstrLength = 1,
	.ui16AddrLength = 3,
	.ui8Priority = 1,
	.pPayload = &buf1,
	.ui32NumPayload = sizeof(buf2) / sizeof(struct mspi_buf),
};

struct mspi_xfer_packet pack2 = {
	.eMode = MSPI_DMA,
	.eDirection = MSPI_RX,
	.ui32RXDummy = 6,
	.ui16InstrLength = 1,
	.ui16AddrLength = 3,
	.ui8Priority = 1,
	.pPayload = &buf2,
	.ui32NumPayload = sizeof(buf2) / sizeof(struct mspi_buf),
};

int main(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_ALIAS(psram0));
	const struct device *controller = DEVICE_DT_GET(DT_BUS(DT_ALIAS(psram0)));
	struct mspi_dev_id *dev_id = 0x15638; /** need to make sure this is the same as dev->config->dev_id*/
	struct mspi_callback_context cb_ctx1,cb_ctx2;
	uint8_t *memc = (uint8_t *)MEMC_BASE;
	uint32_t i, j;

	volatile uint32_t writestatus, readstatus = ~0;

	/* Initialize write buffer */
	for (i = 0; i < BUF_SIZE; i++) {
		memc_write_buffer[i] = (uint8_t)i;
	}
	cb_ctx1.ctx = &writestatus;
	cb_ctx2.ctx = &readstatus;
	mspi_register_callback(controller, dev_id, MSPI_BUS_XFER_COMPLETE, (mspi_callback_handler_t)async_cb, &cb_ctx1);
	mspi_transceive_async(controller, dev_id, &pack1);

	mspi_register_callback(controller, dev_id, MSPI_BUS_XFER_COMPLETE, (mspi_callback_handler_t)async_cb, &cb_ctx2);
	mspi_transceive_async(controller, dev_id, &pack2);

	while(writestatus != 0 || readstatus != 0) {
		printk("w:%d,r:%d\n", cb_ctx1.mspi_evt.evt_data.payload_idx, cb_ctx2.mspi_evt.evt_data.payload_idx);
		k_busy_wait(10);
	}

	for (j = 0; j < BUF_SIZE; j++) {
		if (memc_write_buffer[j] != memc_read_buffer[j])
		{
			printk("Error: data differs at offset %d\n", j);
			break;
		}
	}
	if (j == BUF_SIZE) {
		printk("Read data matches written data\n");
	}

#if 0
	printk("Writing to memory region with base %p, size 0x%0x\n\n",
		MEMC_BASE, MEMC_SIZE);
	/* Copy write buffer into memc region */
	for (i = 0, j = 0; j < (MEMC_SIZE / BUF_SIZE); i += BUF_SIZE, j++) {
		memcpy(memc + i, memc_write_buffer, BUF_SIZE);
	}
	/* Copy any remaining space bytewise */
	for (; i < MEMC_SIZE; i++) {
		memc[i] = memc_write_buffer[i];
	}
	/* Read from memc region into buffer */
	for (i = 0, j = 0; j < (MEMC_SIZE / BUF_SIZE); i += BUF_SIZE, j++) {
		memcpy(memc_read_buffer, memc + i, BUF_SIZE);
		/* Compare memory */
		if (memcmp(memc_read_buffer, memc_write_buffer, BUF_SIZE)) {
			printk("Error: read data differs in range [0x%x- 0x%x]\n",
				i, i + (BUF_SIZE - 1));
			return 0;
		}
	}
	/* Copy any remaining space bytewise */
	for (; i < MEMC_SIZE; i++) {
		memc_read_buffer[i] = memc[i];
		if (memc_write_buffer[i] != memc_read_buffer[i]) {
			printk("Error: read data differs at offset 0x%x\n", i);
			return 0;
		}
	}
	printk("First 1KB of Data in memory:\n");
	printk("===========================\n");
	dump_memory(memc, MIN(MEMC_SIZE, KB(1)));

	printk("Read data matches written data\n");
#endif
	return 0;
}
