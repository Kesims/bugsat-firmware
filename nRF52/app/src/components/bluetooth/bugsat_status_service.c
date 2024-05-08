#include "bugsat_status_service.h"


bool gps_ready = false;
bool launch_detected = false;
bool bugs_deployed = false;

static ssize_t byte_read_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    const uint8_t *value = attr->user_data;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(*value));
}

void cccd_changed_gps_cb(const struct bt_gatt_attr *attr, uint16_t value)
{
    gps_ready = (value == BT_GATT_CCC_NOTIFY);
}

void cccd_changed_launch_cb(const struct bt_gatt_attr *attr, uint16_t value)
{
    launch_detected = (value == BT_GATT_CCC_NOTIFY);
}

void cccd_changed_bugs_cb(const struct bt_gatt_attr *attr, uint16_t value)
{
    bugs_deployed = (value == BT_GATT_CCC_NOTIFY);
}

// Declare the configuration service
BT_GATT_SERVICE_DEFINE(
        bugsat_status_service,
        BT_GATT_PRIMARY_SERVICE(BT_UUID_STATUS_SERVICE),
        BT_GATT_CHARACTERISTIC(
                BT_UUID_GPS_READY,
                BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                BT_GATT_PERM_READ,
                byte_read_cb,
                NULL,
                &gps_ready
        ),
        BT_GATT_CCC(
                cccd_changed_gps_cb,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE
        ),
        BT_GATT_CHARACTERISTIC(
                BT_UUID_LAUNCH_DETECTED,
                BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                BT_GATT_PERM_READ,
                byte_read_cb,
                NULL,
                &launch_detected
        ),
        BT_GATT_CCC(
                cccd_changed_launch_cb,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE
        ),
        BT_GATT_CHARACTERISTIC(
                BT_UUID_BUGS_DEPLOYED,
                BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                BT_GATT_PERM_READ,
                byte_read_cb,
                NULL,
                &bugs_deployed
        ),
        BT_GATT_CCC(
                cccd_changed_bugs_cb,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE
        )
);

void gps_notify()
{
    bt_gatt_notify(NULL, &bugsat_status_service.attrs[1], &gps_ready, sizeof(gps_ready));
}

void launch_notify()
{
    bt_gatt_notify(NULL, &bugsat_status_service.attrs[4], &launch_detected, sizeof(launch_detected));
}

void bugs_notify()
{
    bt_gatt_notify(NULL, &bugsat_status_service.attrs[7], &bugs_deployed, sizeof(bugs_deployed));
}
