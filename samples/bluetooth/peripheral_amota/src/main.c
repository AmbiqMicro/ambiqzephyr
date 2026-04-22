/* main.c - Application main entry point */

/*
 * Copyright (c) 2023 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/amota.h>

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static int start_advertising(void)
{
	int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), NULL, 0);

	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
	} else {
		printk("Advertising successfully started\n");
	}

	return err;
}

/*
 * The host calls the disconnected callback from conn deferred_work before it
 * releases the ACL connection object. With CONFIG_BT_MAX_CONN=1, starting
 * connectable advertising synchronously in disconnected() then needs a new
 * conn for BT_CONN_ADV_CONNECTABLE and hits -ENOMEM. Defer restart until after
 * that work item returns and bt_conn_unref() runs.
 */
static void adv_restart_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	int err = start_advertising();

	if (err == -ENOMEM) {
		(void)k_work_reschedule(dwork, K_MSEC(50));
	}
}

static K_WORK_DELAYABLE_DEFINE(adv_restart_work, adv_restart_work_handler);

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed, err 0x%02x %s\n", err, bt_hci_err_to_str(err));
	} else {
		printk("Connected\n");
		bt_amota_conn_init(conn);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	ARG_UNUSED(conn);

	printk("Disconnected, reason 0x%02x %s\n", reason, bt_hci_err_to_str(reason));

	bt_amota_conn_deinit();

	(void)k_work_reschedule(&adv_restart_work, K_NO_WAIT);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

static void bt_ready(void)
{
	printk("Bluetooth initialized\n");

	(void)start_advertising();
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing cancelled: %s\n", addr);
}

static struct bt_conn_auth_cb auth_cb_display = {
	.cancel = auth_cancel,
};

int main(void)
{
	int err;

	printk("BLE AMOTA Application\n");

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	bt_ready();

	bt_conn_auth_cb_register(&auth_cb_display);

	return 0;
}
