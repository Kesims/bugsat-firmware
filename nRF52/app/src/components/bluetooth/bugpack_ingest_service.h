#ifndef CANSAT_BLE_BUGPACK_INGEST_SERVICE_H
#define CANSAT_BLE_BUGPACK_INGEST_SERVICE_H


#include <stdint.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>


typedef struct BugPackData {
    uint8_t bugpack_id;
    uint16_t battery_voltage;
    float acceleration_x;
    float acceleration_y;
    float acceleration_z;
} BugPackData;

#define BT_UUID_BUGPACK_INGEST_SERVICE_VAL \
	BT_UUID_128_ENCODE(0xe7e24000, 0xfee9, 0x4d67, 0xa4f5, 0xb5144cea14bd)

#define BT_UUID_BUGPACK_INGEST_VAL \
	BT_UUID_128_ENCODE(0xe7e24001, 0xfee9, 0x4d67, 0xa4f5, 0xb5144cea14bd)

#define BT_UUID_BUGPACK_INGEST_SERVICE BT_UUID_DECLARE_128(BT_UUID_BUGPACK_INGEST_SERVICE_VAL)
#define BT_UUID_BUGPACK_INGEST BT_UUID_DECLARE_128(BT_UUID_BUGPACK_INGEST_VAL)

#endif //CANSAT_BLE_BUGPACK_INGEST_SERVICE_H
