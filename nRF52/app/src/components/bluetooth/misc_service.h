#ifndef CANSAT_BLE_MISC_SERVICE_H
#define CANSAT_BLE_MISC_SERVICE_H

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#define BT_UUID_MISC_SERVICE_VAL \
	BT_UUID_128_ENCODE(0xe7e23000, 0xfee9, 0x4d67, 0xa4f5, 0xb5144cea14bd)

#define BT_UUID_STM_REBOOT_VAL \
	BT_UUID_128_ENCODE(0xe7e23001, 0xfee9, 0x4d67, 0xa4f5, 0xb5144cea14bd)

#define BT_UUID_MISC_SERVICE BT_UUID_DECLARE_128(BT_UUID_MISC_SERVICE_VAL)
#define BT_UUID_STM_REBOOT BT_UUID_DECLARE_128(BT_UUID_STM_REBOOT_VAL)

#endif //CANSAT_BLE_MISC_SERVICE_H
