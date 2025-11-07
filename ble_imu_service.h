
/* Declares the BLE IMU custom service: UUID assignments, context structure,
 * and public API used by the application to initialize the service, send data,
 * and route BLE events.
 */
#pragma once
#include <stdint.h>
#include "ble.h"
#include "ble_srv_common.h"

#define BLE_IMU_UUID_BASE      {0xA6,0x00,0x00,0x00,0xB5,0xA3,0xF3,0x93,0xE0,0x11,0x49,0xA1,0x00,0x00,0x40,0x6E}
#define BLE_IMU_UUID_SERVICE   0xA600
#define BLE_IMU_UUID_CTRL_CHAR 0xA601
#define BLE_IMU_UUID_DATA_CHAR 0xA602

typedef struct {
    uint16_t                 service_handle;
    ble_gatts_char_handles_t ctrl_handles;
    ble_gatts_char_handles_t data_handles;
    uint16_t                 conn_handle;
    uint8_t                  uuid_type;
    volatile uint8_t         streaming; // 1=start, 0=stop
} ble_imu_t;

uint32_t ble_imu_service_init(ble_imu_t * p_imu);
uint32_t ble_imu_notify(ble_imu_t * p_imu, const uint8_t * data, uint16_t len);
void     ble_imu_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);
