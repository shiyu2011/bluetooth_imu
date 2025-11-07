
/* Implements a Nordic BLE custom IMU service that exposes control and data
 * characteristics and processes GAP/GATTS events so applications can toggle
 * streaming and send IMU notifications safely.
 */
#include "ble_imu_service.h"
#include "nrf_log.h"
#include <string.h>

static void on_write(ble_imu_t * p_imu, ble_evt_t const * p_ble_evt)
{
    const ble_gatts_evt_write_t * evt = &p_ble_evt->evt.gatts_evt.params.write;
    if (evt->handle == p_imu->ctrl_handles.value_handle && evt->len >= 1) {
        p_imu->streaming = evt->data[0] ? 1 : 0;
        NRF_LOG_INFO("Streaming %s", p_imu->streaming ? "ON" : "OFF");
    }
}

void ble_imu_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_imu_t * p_imu = (ble_imu_t *)p_context;
    if (p_imu == NULL || p_ble_evt == NULL) return;

    switch (p_ble_evt->header.evt_id) {
        case BLE_GATTS_EVT_WRITE:
            on_write(p_imu, p_ble_evt);
            break;
        case BLE_GAP_EVT_CONNECTED:
            p_imu->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
        case BLE_GAP_EVT_DISCONNECTED:
            p_imu->conn_handle = BLE_CONN_HANDLE_INVALID;
            p_imu->streaming = 0;
            break;
        default:
            break;
    }
}

static uint32_t char_add_notify(ble_imu_t * p_imu, ble_gatts_char_handles_t * handles, uint16_t uuid)
{
    ble_gatts_char_md_t char_md = {0};
    ble_gatts_attr_md_t cccd_md = {0};
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    char_md.char_props.notify = 1;
    char_md.p_cccd_md = &cccd_md;

    ble_uuid_t ble_uuid;
    ble_uuid.type = p_imu->uuid_type;
    ble_uuid.uuid = uuid;

    ble_gatts_attr_md_t attr_md = {0};
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    attr_md.vloc = BLE_GATTS_VLOC_STACK;

    ble_gatts_attr_t    attr_char_value = {0};
    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 1;
    attr_char_value.max_len   = 244;
    attr_char_value.p_value   = NULL;

    return sd_ble_gatts_characteristic_add(p_imu->service_handle, &char_md, &attr_char_value, handles);
}

static uint32_t char_add_write(ble_imu_t * p_imu, ble_gatts_char_handles_t * handles, uint16_t uuid)
{
    ble_gatts_char_md_t char_md = {0};
    char_md.char_props.write = 1;
    char_md.char_props.write_wo_resp = 1;

    ble_uuid_t ble_uuid;
    ble_uuid.type = p_imu->uuid_type;
    ble_uuid.uuid = uuid;

    ble_gatts_attr_md_t attr_md = {0};
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc = BLE_GATTS_VLOC_STACK;

    ble_gatts_attr_t attr_char_value = {0};
    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 1;
    attr_char_value.max_len   = 20;
    attr_char_value.p_value   = NULL;

    return sd_ble_gatts_characteristic_add(p_imu->service_handle, &char_md, &attr_char_value, handles);
}

uint32_t ble_imu_service_init(ble_imu_t * p_imu)
{
    p_imu->conn_handle = BLE_CONN_HANDLE_INVALID;
    p_imu->streaming = 0;

    ble_uuid128_t base_uuid = { BLE_IMU_UUID_BASE };
    sd_ble_uuid_vs_add(&base_uuid, &p_imu->uuid_type);

    ble_uuid_t ble_uuid;
    ble_uuid.type = p_imu->uuid_type;
    ble_uuid.uuid = BLE_IMU_UUID_SERVICE;

    uint32_t err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_imu->service_handle);
    if (err_code != NRF_SUCCESS) return err_code;

    err_code = char_add_write(p_imu, &p_imu->ctrl_handles, BLE_IMU_UUID_CTRL_CHAR);
    if (err_code != NRF_SUCCESS) return err_code;

    err_code = char_add_notify(p_imu, &p_imu->data_handles, BLE_IMU_UUID_DATA_CHAR);
    return err_code;
}

uint32_t ble_imu_notify(ble_imu_t * p_imu, const uint8_t * data, uint16_t len)
{
    if (p_imu->conn_handle == BLE_CONN_HANDLE_INVALID) return NRF_ERROR_INVALID_STATE;

    ble_gatts_hvx_params_t hvx = {0};
    hvx.handle = p_imu->data_handles.value_handle;
    hvx.type   = BLE_GATT_HVX_NOTIFICATION;
    hvx.p_data = (uint8_t*)data;
    hvx.p_len  = &len;
    return sd_ble_gatts_hvx(p_imu->conn_handle, &hvx);
}
