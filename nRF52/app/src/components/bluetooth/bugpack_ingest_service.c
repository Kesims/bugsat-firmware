#include "bugpack_ingest_service.h"
#include "uart/uart_stm.h"

#include <zephyr/logging/log.h>

static BugPackData bugpack_data = {0};

static ssize_t bugpack_data_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags) {
    if (len != sizeof(BugPackData)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
    memcpy(&bugpack_data, buf, sizeof(BugPackData));
    uart_send_bugpack_data((uint8_t *) &bugpack_data, sizeof(BugPackData));
    return len;
}

BT_GATT_SERVICE_DEFINE(
        bugpack_ingest_service,
        BT_GATT_PRIMARY_SERVICE(BT_UUID_BUGPACK_INGEST_SERVICE),
        BT_GATT_CHARACTERISTIC(
                BT_UUID_BUGPACK_INGEST,
                BT_GATT_CHRC_WRITE,
                BT_GATT_PERM_WRITE,
                NULL,
                bugpack_data_cb,
                &bugpack_data
        ),
);