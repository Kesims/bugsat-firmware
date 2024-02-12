#include "bluetooth_core.h"
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/uuid.h>

LOG_MODULE_REGISTER(bluetooth_core, LOG_LEVEL_DBG);

// Define the advertising data
static const struct bt_data ad[] = {
        // Set the advertising flags
        BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
        // Set the advertising packet data
        BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static struct bt_le_adv_param *adv_param = BT_LE_ADV_PARAM((BT_LE_ADV_OPT_CONNECTABLE|BT_LE_ADV_OPT_USE_IDENTITY),
                                                           200, /*Min Advertising Interval 500ms (800*0.625ms) */
                                                           201, /*Max Advertising Interval 500.625ms (801*0.625ms)*/
                                                           NULL); /* Set to NULL for undirected advertising*/

// Define the scan response data
static const struct bt_data sd[] = {};

void bt_core_initialize() {
    int err;
    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)\n", err);
        return;
    }
    LOG_INF("Bluetooth initialized\n");

    err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad),
                          sd, ARRAY_SIZE(sd));
    if (err) {
        LOG_ERR("Advertising failed to start (err %d)\n", err);
        return;
    }
    LOG_INF("Advertising successfully started\n");
}