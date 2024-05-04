#include "misc_service.h"
#include "uart/uart_stm.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(misc_service, LOG_LEVEL_DBG);

static ssize_t restart_stm_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags) {
    uart_reboot_stm();
    LOG_INF("Rebooting STM32...\n");
    return len;
}

BT_GATT_SERVICE_DEFINE(
        misc_service,
        BT_GATT_PRIMARY_SERVICE(BT_UUID_MISC_SERVICE),
        BT_GATT_CHARACTERISTIC(
                BT_UUID_STM_REBOOT,
                BT_GATT_CHRC_WRITE,
                BT_GATT_PERM_WRITE,
                NULL,
                restart_stm_cb,
                NULL
        ),
);

