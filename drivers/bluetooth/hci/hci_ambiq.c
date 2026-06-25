/*
 * Copyright (c) 2023 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Ambiq SPI based Bluetooth HCI driver.
 */

#define DT_DRV_COMPAT ambiq_bt_hci_spi

#include <zephyr/init.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/bluetooth.h>
#include <zephyr/bluetooth/hci.h>

#define LOG_LEVEL CONFIG_BT_HCI_DRIVER_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bt_hci_driver);

#include "apollox_blue.h"
#if (CONFIG_SOC_APOLLO510B)
#include "am_devices_em9305.h"
#endif

/* Offset of special item */
#define PACKET_TYPE         0
#define PACKET_TYPE_SIZE    1
#define EVT_HEADER_TYPE     0
#define EVT_CMD_COMP_OP_LSB 3
#define EVT_CMD_COMP_OP_MSB 4
#define EVT_CMD_COMP_DATA   5

#define EVT_OK      0
#define EVT_DISCARD 1
#define EVT_NOP     2

#define BT_FEAT_SET_BIT(feat, octet, bit) (feat[octet] |= BIT(bit))
#define BT_FEAT_SET_NO_BREDR(feat)        BT_FEAT_SET_BIT(feat, 4, 5)
#define BT_FEAT_SET_LE(feat)              BT_FEAT_SET_BIT(feat, 4, 6)

/* Max SPI buffer length for transceive operations.
 */
#if (CONFIG_SOC_APOLLO510B)
#define SPI_MAX_TX_MSG_LEN 259
#define SPI_MAX_RX_MSG_LEN 258
#else
#define SPI_MAX_TX_MSG_LEN 524
#define SPI_MAX_RX_MSG_LEN 258
#endif

#define SPI_BUSY_WAIT_INTERVAL_MS 25
#define SPI_BUSY_TX_ATTEMPTS      200

#if (CONFIG_SOC_APOLLO510B)
#define SPI_BUSY_RX_DRAIN_WAIT_MS    10
#define SPI_BACKPRESSURE_TX_ATTEMPTS 200
#endif

static uint8_t __noinit rxmsg[SPI_MAX_RX_MSG_LEN];
static uint8_t __noinit rx_pending[SPI_MAX_RX_MSG_LEN * 2];
static size_t rx_pending_len;

#if (CONFIG_SOC_APOLLO510B)
static struct spi_dt_spec spi_bus =
	SPI_DT_SPEC_INST_GET(0, SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8));
#else
static struct spi_dt_spec spi_bus =
	SPI_DT_SPEC_INST_GET(0, SPI_OP_MODE_MASTER | SPI_HALF_DUPLEX | SPI_TRANSFER_MSB |
					SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8));
#endif
static K_KERNEL_STACK_DEFINE(spi_rx_stack, CONFIG_BT_DRV_RX_STACK_SIZE);
static struct k_thread spi_rx_thread_data;

static struct spi_buf spi_tx_buf;
static struct spi_buf spi_rx_buf;
static const struct spi_buf_set spi_tx = {.buffers = &spi_tx_buf, .count = 1};
static const struct spi_buf_set spi_rx = {.buffers = &spi_rx_buf, .count = 1};

static K_SEM_DEFINE(sem_irq, 0, 1);
static K_SEM_DEFINE(sem_spi_available, 1, 1);
#if (CONFIG_SOC_APOLLO510B)
static K_SEM_DEFINE(sem_rx_drained, 0, 1);
#endif

struct bt_apollo_data {
	bt_hci_recv_t recv;
};

void bt_packet_irq_isr(const struct device *unused1, struct gpio_callback *unused2,
		       uint32_t unused3)
{
	bt_apollo_rcv_isr_preprocess();
	k_sem_give(&sem_irq);
}

#if (CONFIG_SOC_APOLLO510B)
static bool spi_ret_retryable(int ret)
{
	return (ret == AM_DEVICES_EM9305_RX_FULL) || (ret == AM_DEVICES_EM9305_TX_BUSY) ||
	       (ret == AM_DEVICES_EM9305_NOT_READY);
}
#endif

#if (CONFIG_SOC_APOLLO510B)
static inline bool spi_ret_fatal(int ret)
{
	return (ret == AM_DEVICES_EM9305_DATA_LENGTH_ERROR) ||
	       (ret == AM_DEVICES_EM9305_TX_PARTIAL);
}
#else
static inline bool spi_ret_fatal(int ret)
{
	ARG_UNUSED(ret);
	return false;
}
#endif

static inline int bt_spi_transceive(void *tx, uint32_t tx_len, void *rx, uint32_t rx_len)
{
	spi_tx_buf.buf = tx;
	spi_tx_buf.len = (size_t)tx_len;
	spi_rx_buf.buf = rx;
	spi_rx_buf.len = (size_t)rx_len;

	if (tx_len && rx_len) {
		spi_bus.config.operation |= SPI_HOLD_ON_CS;
	} else {
		spi_bus.config.operation &= ~SPI_HOLD_ON_CS;
	}
	return spi_transceive_dt(&spi_bus, &spi_tx, &spi_rx);
}

static int spi_send_packet(uint8_t *data, uint16_t len)
{
	int ret = 0;
	uint16_t fail_count = 0;
#if (CONFIG_SOC_APOLLO510B)
	uint16_t backpressure_count = 0;
#endif

	while (true) {
		/* Wait for SPI bus to be available */
		k_sem_take(&sem_spi_available, K_FOREVER);

		/* Send the SPI packet to controller */
		ret = bt_apollo_spi_send(data, len, bt_spi_transceive);

		/* Free the SPI bus */
		k_sem_give(&sem_spi_available);

		if (ret) {
			if (spi_ret_fatal(ret)) {
				LOG_ERR("SPI TX fatal error %d (len=%u), aborting", ret, len);
#if (CONFIG_SOC_APOLLO510B)
				if (ret == AM_DEVICES_EM9305_TX_PARTIAL) {
					bt_apollo_schedule_radio_recovery();
				}
#endif
				break;
			}
#if (CONFIG_SOC_APOLLO510B)
			if (spi_ret_retryable(ret)) {
				if (backpressure_count++ >= SPI_BACKPRESSURE_TX_ATTEMPTS) {
					break;
				}
				k_sem_reset(&sem_rx_drained);
				k_sem_give(&sem_irq);
				k_sem_take(&sem_rx_drained, K_MSEC(SPI_BUSY_RX_DRAIN_WAIT_MS));
				continue;
			}
#endif
			if (fail_count++ >= SPI_BUSY_TX_ATTEMPTS) {
				break;
			}

			/* Give some chance to controller to complete the processing or
			 * packets sending.
			 */
			k_sleep(K_MSEC(SPI_BUSY_WAIT_INTERVAL_MS));
		} else {
			/* TX succeeded — restart heartbeat countdown so the
			 * ping is only sent after a full interval of silence.
			 */
#if (CONFIG_SOC_APOLLO510B)
			bt_apollo_heartbeat_restart();
#endif
			break;
		}
	}

	return ret;
}

static int spi_receive_packet(uint8_t *data, uint16_t *len)
{
	int ret;

	/* Wait for SPI bus to be available */
	k_sem_take(&sem_spi_available, K_FOREVER);

	/* Receive the SPI packet from controller */
	ret = bt_apollo_spi_rcv(data, len, bt_spi_transceive);

	/* Free the SPI bus */
	k_sem_give(&sem_spi_available);

	return ret;
}

static int hci_event_filter(const uint8_t *evt_data)
{
	uint8_t evt_type = evt_data[EVT_HEADER_TYPE];

	switch (evt_type) {
#if (CONFIG_SOC_APOLLO510B)
	case BT_HCI_EVT_HARDWARE_ERROR: {
		uint8_t hw_code = evt_data[sizeof(struct bt_hci_evt_hdr)];

		LOG_ERR("EM9305 hardware error 0x%02x; scheduling radio recovery", hw_code);
		bt_apollo_schedule_radio_recovery();
		return EVT_NOP;
	}
#endif /* CONFIG_SOC_APOLLO510B */
	case BT_HCI_EVT_LE_META_EVENT: {
		uint8_t subevt_type = evt_data[sizeof(struct bt_hci_evt_hdr)];

		switch (subevt_type) {
		/* Bluetooth 4.2+ */
		case BT_HCI_EVT_LE_DIRECT_ADV_REPORT:
		case BT_HCI_EVT_LE_SCAN_REQ_RECEIVED:
		/* Bluetooth 5.0+ */
		case BT_HCI_EVT_LE_EXT_ADVERTISING_REPORT:
		case BT_HCI_EVT_LE_PER_ADVERTISING_REPORT:
		/* Bluetooth 5.4+ */
		case BT_HCI_EVT_LE_PER_ADVERTISING_REPORT_V2:
			return EVT_DISCARD;
		default:
			return EVT_OK;
		}
	}
	case BT_HCI_EVT_CMD_COMPLETE: {
		uint16_t opcode = (uint16_t)(evt_data[EVT_CMD_COMP_OP_LSB] +
					     (evt_data[EVT_CMD_COMP_OP_MSB] << 8));
		bt_apollo_vsc_cc_observe(opcode, evt_data[EVT_CMD_COMP_DATA]);

		switch (opcode) {
		case BT_OP_NOP:
			return EVT_NOP;
		case BT_HCI_OP_READ_LOCAL_FEATURES: {
			struct bt_hci_rp_read_local_features *rp =
				(void *)&evt_data[EVT_CMD_COMP_DATA];
			if (rp->status == 0) {
				BT_FEAT_SET_NO_BREDR(rp->features);
				BT_FEAT_SET_LE(rp->features);
			}
			return EVT_OK;
		}
		default:
			return EVT_OK;
		}
	}
	default:
		return EVT_OK;
	}
}

static struct net_buf *bt_hci_evt_recv(uint8_t *data, size_t len)
{
	int evt_filter;
	bool discardable = false;
	struct bt_hci_evt_hdr hdr = {0};
	struct net_buf *buf;
	size_t buf_tailroom;

	if (len < sizeof(hdr)) {
		LOG_ERR("Not enough data for event header");
		return NULL;
	}

	evt_filter = hci_event_filter(data);
	if (evt_filter == EVT_NOP) {
		return NULL;
	} else if (evt_filter == EVT_DISCARD) {
		discardable = true;
	}

	memcpy((void *)&hdr, data, sizeof(hdr));
	data += sizeof(hdr);
	len -= sizeof(hdr);

	if (len != hdr.len) {
		LOG_ERR("Event payload length is not correct");
		return NULL;
	}

	buf = bt_buf_get_evt(hdr.evt, discardable, K_NO_WAIT);
	if (!buf) {
		if (discardable) {
			LOG_DBG("Discardable buffer pool full, ignoring event");
		} else {
			LOG_ERR("No available event buffers!");
		}
		return buf;
	}

	net_buf_add_mem(buf, &hdr, sizeof(hdr));

	buf_tailroom = net_buf_tailroom(buf);
	if (buf_tailroom < len) {
		LOG_ERR("Not enough space in buffer %zu/%zu", len, buf_tailroom);
		net_buf_unref(buf);
		return NULL;
	}

	net_buf_add_mem(buf, data, len);

	return buf;
}

static struct net_buf *bt_hci_acl_recv(uint8_t *data, size_t len)
{
	struct bt_hci_acl_hdr hdr = {0};
	struct net_buf *buf;
	size_t buf_tailroom;

	if (len < sizeof(hdr)) {
		LOG_ERR("Not enough data for ACL header");
		return NULL;
	}

	buf = bt_buf_get_rx(BT_BUF_ACL_IN, K_NO_WAIT);
	if (buf) {
		memcpy((void *)&hdr, data, sizeof(hdr));
		data += sizeof(hdr);
		len -= sizeof(hdr);
	} else {
		LOG_ERR("No available ACL buffers!");
		return NULL;
	}

	if (len != sys_le16_to_cpu(hdr.len)) {
		LOG_ERR("ACL payload length is not correct");
		net_buf_unref(buf);
		return NULL;
	}

	net_buf_add_mem(buf, &hdr, sizeof(hdr));
	buf_tailroom = net_buf_tailroom(buf);
	if (buf_tailroom < len) {
		LOG_ERR("Not enough space in buffer %zu/%zu", len, buf_tailroom);
		net_buf_unref(buf);
		return NULL;
	}

	net_buf_add_mem(buf, data, len);

	return buf;
}

static struct net_buf *bt_hci_iso_recv(uint8_t *data, size_t len)
{
	struct bt_hci_iso_hdr hdr = {0};
	struct net_buf *buf;
	size_t buf_tailroom;

	if (len < sizeof(hdr)) {
		LOG_ERR("Not enough data for ISO header");
		return NULL;
	}

	buf = bt_buf_get_rx(BT_BUF_ISO_IN, K_NO_WAIT);
	if (buf) {
		memcpy((void *)&hdr, data, sizeof(hdr));
		data += sizeof(hdr);
		len -= sizeof(hdr);
	} else {
		LOG_ERR("No available ISO buffers!");
		return NULL;
	}

	if (len != bt_iso_hdr_len(sys_le16_to_cpu(hdr.len))) {
		LOG_ERR("ISO payload length is not correct");
		net_buf_unref(buf);
		return NULL;
	}

	net_buf_add_mem(buf, &hdr, sizeof(hdr));
	buf_tailroom = net_buf_tailroom(buf);
	if (buf_tailroom < len) {
		LOG_ERR("Not enough space in buffer %zu/%zu", len, buf_tailroom);
		net_buf_unref(buf);
		return NULL;
	}

	net_buf_add_mem(buf, data, len);

	return buf;
}

static int bt_hci_get_frame_len(const uint8_t *data, size_t len, size_t *frame_len)
{
	if (len < PACKET_TYPE_SIZE) {
		return -EAGAIN;
	}

	switch (data[PACKET_TYPE]) {
	case BT_HCI_H4_EVT: {
		struct bt_hci_evt_hdr hdr;

		if (len < PACKET_TYPE_SIZE + sizeof(hdr)) {
			return -EAGAIN;
		}

		memcpy(&hdr, &data[PACKET_TYPE_SIZE], sizeof(hdr));
		*frame_len = PACKET_TYPE_SIZE + sizeof(hdr) + hdr.len;
		break;
	}
	case BT_HCI_H4_ACL: {
		struct bt_hci_acl_hdr hdr;

		if (len < PACKET_TYPE_SIZE + sizeof(hdr)) {
			return -EAGAIN;
		}

		memcpy(&hdr, &data[PACKET_TYPE_SIZE], sizeof(hdr));
		*frame_len = PACKET_TYPE_SIZE + sizeof(hdr) + sys_le16_to_cpu(hdr.len);
		break;
	}
	case BT_HCI_H4_ISO: {
		struct bt_hci_iso_hdr hdr;

		if (len < PACKET_TYPE_SIZE + sizeof(hdr)) {
			return -EAGAIN;
		}

		memcpy(&hdr, &data[PACKET_TYPE_SIZE], sizeof(hdr));
		*frame_len =
			PACKET_TYPE_SIZE + sizeof(hdr) + bt_iso_hdr_len(sys_le16_to_cpu(hdr.len));
		break;
	}
	default:
		LOG_WRN("Unknown BT buf type %d", data[PACKET_TYPE]);
		return -EINVAL;
	}

	if (*frame_len > len) {
		return -EAGAIN;
	}

	return 0;
}

static void bt_hci_recv_frames(const struct device *dev, uint8_t *data, size_t len)
{
	struct bt_apollo_data *hci = dev->data;
	size_t offset = 0;

	if ((rx_pending_len + len) > sizeof(rx_pending)) {
		LOG_ERR("H4 RX reassembly buffer overflow %zu/%zu", rx_pending_len + len,
			sizeof(rx_pending));
		rx_pending_len = 0;
	}

	memcpy(&rx_pending[rx_pending_len], data, len);
	rx_pending_len += len;

	while (offset < rx_pending_len) {
		struct net_buf *buf = NULL;
		size_t frame_len;
		int err;

		err = bt_hci_get_frame_len(&rx_pending[offset], rx_pending_len - offset,
					   &frame_len);
		if (err == -EAGAIN) {
			break;
		} else if (err) {
			offset++;
			continue;
		}

		switch (rx_pending[offset + PACKET_TYPE]) {
		case BT_HCI_H4_EVT:
			buf = bt_hci_evt_recv(&rx_pending[offset + PACKET_TYPE_SIZE],
					      frame_len - PACKET_TYPE_SIZE);
			break;
		case BT_HCI_H4_ACL:
			buf = bt_hci_acl_recv(&rx_pending[offset + PACKET_TYPE_SIZE],
					      frame_len - PACKET_TYPE_SIZE);
			break;
		case BT_HCI_H4_ISO:
			buf = bt_hci_iso_recv(&rx_pending[offset + PACKET_TYPE_SIZE],
					      frame_len - PACKET_TYPE_SIZE);
			break;
		default:
			/* Already checked by bt_hci_get_frame_len(). */
			break;
		}

		if (buf) {
			hci->recv(dev, buf);
		}

		offset += frame_len;
	}

	if (offset != 0) {
		rx_pending_len -= offset;
		if (rx_pending_len != 0) {
			memmove(rx_pending, &rx_pending[offset], rx_pending_len);
		}
	}
}

static void bt_spi_rx_thread(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	int ret;
	uint16_t len = 0;

	while (true) {
		/* Wait for controller interrupt */
		k_sem_take(&sem_irq, K_FOREVER);
		do {
			/* Receive the HCI packet via SPI */
			ret = spi_receive_packet(&rxmsg[0], &len);
			if (ret) {
#if (CONFIG_SOC_APOLLO510B)
				if (ret != AM_DEVICES_EM9305_NO_DATA_TX) {
					k_sleep(K_MSEC(SPI_BUSY_RX_DRAIN_WAIT_MS));
					k_sem_give(&sem_irq);
				}
#endif
				break;
			}
			if (len == 0) {
				break;
			}

			/* Check if needs to handle the vendor specific events which are
			 * incompatible with the standard Bluetooth HCI format.
			 */
			if (bt_apollo_vnd_rcv_ongoing(&rxmsg[0], len)) {
#if (CONFIG_SOC_APOLLO510B)
				k_sem_give(&sem_rx_drained);
#endif
				break;
			}

			bt_hci_recv_frames(dev, &rxmsg[0], len);

#if (CONFIG_SOC_APOLLO510B)
			k_sem_give(&sem_rx_drained);
			/* RX succeeded — restart heartbeat countdown. */
			bt_apollo_heartbeat_restart();
#endif
		} while (0);

#if (CONFIG_SOC_APOLLO510B)
		if (!ret && len > 0 && bt_apollo_irq_pending()) {
			k_sem_give(&sem_irq);
		}
#endif
	}
}

static int bt_apollo_send(const struct device *dev, struct net_buf *buf)
{
	int ret;

	if (buf->len > SPI_MAX_TX_MSG_LEN) {
		LOG_ERR("Message too long");
		net_buf_unref(buf);
		return -EINVAL;
	}

	/* Get the H4 type byte and determine buffer type */
	uint8_t h4_type = net_buf_pull_u8(buf);
	enum bt_buf_type buf_type = bt_buf_type_from_h4(h4_type, BT_BUF_OUT);

	switch (buf_type) {
	case BT_BUF_ACL_OUT:
		net_buf_push_u8(buf, BT_HCI_H4_ACL);
		break;
	case BT_BUF_CMD:
		net_buf_push_u8(buf, BT_HCI_H4_CMD);
		break;
	case BT_BUF_ISO_OUT:
		net_buf_push_u8(buf, BT_HCI_H4_ISO);
		break;
	default:
		LOG_ERR("Unsupported type: 0x%02x", h4_type);
		net_buf_unref(buf);
		return -EINVAL;
	}

	/* Send the SPI packet */
	ret = spi_send_packet(buf->data, buf->len);
	if (ret != 0) {
		LOG_ERR("SPI send failed: %d", ret);
		net_buf_unref(buf);
		return ret;
	}

	net_buf_unref(buf);

	return 0;
}

static int bt_apollo_open(const struct device *dev, bt_hci_recv_t recv)
{
	struct bt_apollo_data *hci = dev->data;
	int ret;

	/* Discard any H4 reassembly state left from a previous open/close
	 * cycle.
	 */
	rx_pending_len = 0;

#if (CONFIG_SOC_APOLLO510B)
	k_sem_reset(&sem_spi_available);
	k_sem_give(&sem_spi_available); /* restore count to 1 (bus free) */
	k_sem_reset(&sem_irq);          /* no pending IRQ from previous session */
	k_sem_reset(&sem_rx_drained);   /* no stale drain signal */
#endif

	ret = bt_hci_transport_setup(spi_bus.bus);
	if (ret) {
		return ret;
	}

	hci->recv = recv;

#if (CONFIG_SOC_APOLLO510B)
	ret = bt_apollo_controller_init(spi_send_packet, bt_spi_transceive);
#else
	/* Apollo3/Apollo4 controller init needs the RX thread to process
	 * firmware/SBL handshake packets from controller IRQs.
	 */
	k_thread_create(&spi_rx_thread_data, spi_rx_stack, K_KERNEL_STACK_SIZEOF(spi_rx_stack),
			(k_thread_entry_t)bt_spi_rx_thread, (void *)dev, NULL, NULL,
			K_PRIO_COOP(CONFIG_BT_DRIVER_RX_HIGH_PRIO), 0, K_NO_WAIT);

	ret = bt_apollo_controller_init(spi_send_packet, NULL);
#endif
	if (ret != 0) {
		LOG_ERR("BT controller initialization failed: %d", ret);
#if !(CONFIG_SOC_APOLLO510B)
		k_thread_abort(&spi_rx_thread_data);
#endif
		return ret;
	}

	/* Start RX thread */
#if (CONFIG_SOC_APOLLO510B)
	k_thread_create(&spi_rx_thread_data, spi_rx_stack, K_KERNEL_STACK_SIZEOF(spi_rx_stack),
			(k_thread_entry_t)bt_spi_rx_thread, (void *)dev, NULL, NULL,
			K_PRIO_COOP(CONFIG_BT_DRIVER_RX_HIGH_PRIO), 0, K_NO_WAIT);
#endif

	LOG_INF("BT controller initialized successfully");

	return 0;
}

static int bt_apollo_close(const struct device *dev)
{
	int ret;
	struct bt_apollo_data *hci = dev->data;

	ret = bt_apollo_controller_deinit();
	if (ret) {
		return ret;
	}

	/* Stop RX thread */
	k_thread_abort(&spi_rx_thread_data);

	hci->recv = NULL;

	return ret;
}

static int bt_apollo_setup(const struct device *dev, const struct bt_hci_setup_params *params)
{
	int ret;

#if (CONFIG_SOC_APOLLO510B)
	if ((params != NULL) && !bt_addr_eq(&params->public_addr, BT_ADDR_ANY)) {
		ret = bt_apollo_set_public_addr(params->public_addr.val);
		if (ret) {
			return ret;
		}
	}
#else
	ARG_UNUSED(params);
#endif /* CONFIG_SOC_APOLLO510B */

	ret = bt_apollo_vnd_setup();

	return ret;
}

static DEVICE_API(bt_hci, drv) = {
	.open = bt_apollo_open,
	.close = bt_apollo_close,
	.send = bt_apollo_send,
	.setup = bt_apollo_setup,
};

static int bt_apollo_init(const struct device *dev)
{
	int ret;

	ARG_UNUSED(dev);

	if (!device_is_ready(spi_bus.bus)) {
		LOG_ERR("SPI device not ready");
		return -ENODEV;
	}

	ret = bt_apollo_dev_init();
	if (ret) {
		return ret;
	}

	LOG_DBG("BT HCI initialized");

	return 0;
}

#define HCI_DEVICE_INIT(inst)                                                                      \
	static struct bt_apollo_data hci_data_##inst = {};                                         \
	DEVICE_DT_INST_DEFINE(inst, bt_apollo_init, NULL, &hci_data_##inst, NULL, POST_KERNEL,     \
			      CONFIG_BT_HCI_INIT_PRIORITY, &drv)

/* Only one instance supported right now */
HCI_DEVICE_INIT(0)
