#ifndef CANSAT_BLE_BUGSAT_STATUS_SERVICE_H
#define CANSAT_BLE_BUGSAT_STATUS_SERVICE_H


#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#define BT_UUID_STATUS_SERVICE_VAL \
	BT_UUID_128_ENCODE(0xe7e25000, 0xfee9, 0x4d67, 0xa4f5, 0xb5144cea14bd)

#define BT_UUID_GPS_READY_VAL \
	BT_UUID_128_ENCODE(0xe7e25001, 0xfee9, 0x4d67, 0xa4f5, 0xb5144cea14bd)

#define BT_UUID_LAUNCH_DETECTED_VAL \
    BT_UUID_128_ENCODE(0xe7e25002, 0xfee9, 0x4d67, 0xa4f5, 0xb5144cea14bd)

#define BT_UUID_BUGS_DEPLOYED_VAL \
    BT_UUID_128_ENCODE(0xe7e25003, 0xfee9, 0x4d67, 0xa4f5, 0xb5144cea14bd)

#define BT_UUID_STATUS_SERVICE BT_UUID_DECLARE_128(BT_UUID_STATUS_SERVICE_VAL)
#define BT_UUID_GPS_READY BT_UUID_DECLARE_128(BT_UUID_GPS_READY_VAL)
#define BT_UUID_LAUNCH_DETECTED BT_UUID_DECLARE_128(BT_UUID_LAUNCH_DETECTED_VAL)
#define BT_UUID_BUGS_DEPLOYED BT_UUID_DECLARE_128(BT_UUID_BUGS_DEPLOYED_VAL)

void gps_notify();
void launch_notify();
void bugs_notify();

#endif //CANSAT_BLE_BUGSAT_STATUS_SERVICE_H
