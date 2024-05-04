#include "lora_configuration_service.h"
#include "uart/uart_stm.h"

#include <zephyr/logging/log.h>
#include <zephyr/sys/reboot.h>

LOG_MODULE_REGISTER(lora_configuration_service, LOG_LEVEL_DBG);


uint32_t lora_frequency = 0;
uint32_t lora_bandwidth = 0;
uint16_t lora_sync_word = 0;
uint8_t lora_spreading_factor = 0;
uint8_t lora_tx_power = 0;


static ssize_t byte_read_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    const uint8_t *value = attr->user_data;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(*value));
}

static ssize_t uint16_read_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    const uint16_t *value = attr->user_data;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(*value));
}

static ssize_t uint32_read_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    const uint32_t *value = attr->user_data;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(*value));
}

static ssize_t write_lora_frequency_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    uint32_t *value = attr->user_data;

    // Check for data validity
    if (offset + len > sizeof(*value)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }
    uint32_t *new_frequency = (uint32_t *) buf;
    if (*new_frequency < 410000000 || *new_frequency > 525000000) {
        return BT_GATT_ERR(BT_ATT_ERR_NOT_SUPPORTED);
    }

    // Copy the new frequency to the user data
    memcpy(value + offset, buf, len);
    lora_frequency = *value;
    uart_set_lora_frequency();
    return len;
}

static ssize_t write_lora_bandwidth_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    uint32_t *value = attr->user_data;

    // Check for data validity
    if (offset + len > sizeof(*value)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }
    uint32_t *new_bandwidth = (uint32_t *) buf;
    if (*new_bandwidth < 7800 || *new_bandwidth > 500000) {
        return BT_GATT_ERR(BT_ATT_ERR_NOT_SUPPORTED);
    }

    // Copy the new bandwidth to the user data
    memcpy(value + offset, buf, len);
    lora_bandwidth = *value;
    uart_set_lora_bandwidth();
    return len;
}

static ssize_t write_lora_sync_word_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    uint16_t *value = attr->user_data;

    // Check for data validity
    if (offset + len > sizeof(*value)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    // Copy the new sync word to the user data
    memcpy(value + offset, buf, len);
    lora_sync_word = *value;
    uart_set_lora_sync_word();
    return len;
}

static ssize_t write_lora_spreading_factor_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    uint8_t *value = attr->user_data;

    // Check for data validity
    if (offset + len > sizeof(*value)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }
    uint8_t *new_spreading_factor = (uint8_t *) buf;
    if (*new_spreading_factor < 6 || *new_spreading_factor > 12) {
        return BT_GATT_ERR(BT_ATT_ERR_NOT_SUPPORTED);
    }

    // Copy the new spreading factor to the user data
    memcpy(value + offset, buf, len);
    lora_spreading_factor = *value;
    uart_set_lora_spreading_factor();
    return len;
}

static ssize_t write_lora_tx_power_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    uint8_t *value = attr->user_data;

    // Check for data validity
    if (offset + len > sizeof(*value)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }
    uint8_t *new_tx_power = (uint8_t *) buf;
    if (*new_tx_power < 1 || *new_tx_power > 20) {
        return BT_GATT_ERR(BT_ATT_ERR_NOT_SUPPORTED);
    }

    // Copy the new tx power to the user data
    memcpy(value + offset, buf, len);
    lora_tx_power = *value;
    uart_set_lora_tx_power();
    return len;
}

// Declare the configuration service
BT_GATT_SERVICE_DEFINE(
        lora_configuration_service,
        BT_GATT_PRIMARY_SERVICE(BT_UUID_LORA_CONF_SERVICE),
        BT_GATT_CHARACTERISTIC(
                BT_UUID_LORA_FREQUENCY, // UUID
                BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE, // Properties
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, // Permissions
                uint32_read_cb, // Read callback
                write_lora_frequency_cb, // Write callback
                &lora_frequency // User data
        ),
        BT_GATT_CHARACTERISTIC(
                BT_UUID_LORA_TX_POWER, // UUID
                BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE, // Properties
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, // Permissions
                byte_read_cb, // Read callback
                write_lora_tx_power_cb, // Write callback
                &lora_tx_power // User data
        ),
        BT_GATT_CHARACTERISTIC(
                BT_UUID_LORA_BANDWIDTH, // UUID
                BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE, // Properties
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, // Permissions
                uint32_read_cb, // Read callback
                write_lora_bandwidth_cb, // Write callback
                &lora_bandwidth // User data
        ),
        BT_GATT_CHARACTERISTIC(
                BT_UUID_LORA_SYNC_WORD, // UUID
                BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE, // Properties
                BT_GATT_PERM_READ |BT_GATT_PERM_WRITE, // Permissions
                uint16_read_cb, // Read callback
                write_lora_sync_word_cb, // Write callback
                &lora_sync_word // User data
        ),
        BT_GATT_CHARACTERISTIC(
                BT_UUID_LORA_SPREADING_FACTOR, // UUID
                BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE, // Properties
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, // Permissions
                byte_read_cb, // Read callback
                write_lora_spreading_factor_cb, // Write callback
                &lora_spreading_factor // User data
        ),
);