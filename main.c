
#include <string.h>
#include "nrf.h"
#include "nrf_gpio.h"
#include "app_error.h"
#include "app_timer.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "ble_gap.h"
#include "nrf_ble_gatt.h"
#include "nrfx_spim.h"
#include "nrfx_timer.h"
#include "nrfx_gpiote.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "board_config.h"
#include "bmi270_driver.h"
#include "ble_imu_service.h"

NRF_BLE_GATT_DEF(m_gatt);
BLE_IMU_T m_imu_service;
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;

static const nrfx_spim_t spi0 = NRFX_SPIM_INSTANCE(0);
static const nrfx_timer_t ts_timer = NRFX_TIMER_INSTANCE(1); // 1 MHz timestamp timer

// ---- IMU record ----
typedef struct __attribute__((packed)) {
    uint32_t ts_us;
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    uint16_t seq;
} imu_record_t;

#define FIFO_CAP 128
static imu_record_t fifo[FIFO_CAP];
static volatile uint16_t widx = 0, ridx = 0;
static volatile uint16_t seq  = 0;

static inline bool fifo_push(const imu_record_t *r){
    uint16_t next = (widx + 1) % FIFO_CAP;
    if (next == ridx) return false;
    fifo[widx] = *r;
    widx = next;
    return true;
}

static inline bool fifo_pop(imu_record_t *r){
    if (ridx == widx) return false;
    *r = fifo[ridx];
    ridx = (ridx + 1) % FIFO_CAP;
    return true;
}

// ---- Timestamp timer ----
static void timestamp_timer_init(void)
{
    nrfx_timer_config_t cfg = NRFX_TIMER_DEFAULT_CONFIG;
    cfg.frequency = NRF_TIMER_FREQ_1MHz;
    cfg.mode      = NRF_TIMER_MODE_TIMER;
    cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    APP_ERROR_CHECK(nrfx_timer_init(&ts_timer, &cfg, NULL));
    nrfx_timer_enable(&ts_timer);
}

static inline uint32_t ts_now_us(void)
{
    return nrfx_timer_capture(&ts_timer, NRF_TIMER_CC_CHANNEL0);
}

// ---- BLE Stack ----
#define APP_BLE_CONN_CFG_TAG 1
#define APP_ADV_INTERVAL     64
#define APP_ADV_DURATION     0

static void gap_params_init(void)
{
    ret_code_t err_code;
    ble_gap_conn_params_t gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));
    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_code_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;
    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

static void gatt_init(void)
{
    APP_ERROR_CHECK(nrf_ble_gatt_init(&m_gatt, NULL));
    APP_ERROR_CHECK(nrf_ble_gatt_att_mtu_periph_set(&m_gatt, 247));
}

static void services_init(void)
{
    APP_ERROR_CHECK(ble_imu_service_init(&m_imu_service));
}

static void advertising_start(void)
{
    ble_advdata_t advdata = {0};
    ble_advdata_t srdata  = {0};
    uint8_t flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    ble_uuid_t adv_uuids[] = {
        { .uuid = BLE_IMU_UUID_SERVICE, .type = m_imu_service.uuid_type }
    };

    advdata.name_type = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
    advdata.flags = flags;

    srdata.uuids_complete.uuid_cnt = 1;
    srdata.uuids_complete.p_uuids  = adv_uuids;

    ble_gap_adv_params_t adv_params = {0};
    adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
    adv_params.interval        = APP_ADV_INTERVAL;
    adv_params.duration        = APP_ADV_DURATION;

    uint8_t advbuf[64];
    ble_advdata_encode(&advdata, advbuf, (uint16_t[]){sizeof(advbuf)});
    uint8_t srbuf[64];
    ble_advdata_encode(&srdata, srbuf, (uint16_t[]){sizeof(srbuf)});

    ble_gap_adv_data_t advdata_raw = {
        .adv_data = {.p_data = advbuf, .len = sizeof(advbuf)},
        .scan_rsp_data = {.p_data = srbuf, .len = sizeof(srbuf)},
    };

    APP_ERROR_CHECK(sd_ble_gap_adv_set_configure(NULL, &advdata_raw, &adv_params));
    APP_ERROR_CHECK(sd_ble_gap_adv_start(NULL, APP_BLE_CONN_CFG_TAG));
}

// ---- BLE Evt ----
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_imu_on_ble_evt(p_ble_evt, &m_imu_service);
    switch (p_ble_evt->header.evt_id) {
        case BLE_GAP_EVT_CONNECTED:
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
        case BLE_GAP_EVT_DISCONNECTED:
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;
        default:
            break;
    }
}

NRF_SDH_BLE_OBSERVER(m_ble_observer, 3, ble_evt_handler, NULL);

// ---- IMU -> ISR ----
static void imu_drdy_callback(void)
{
    bmi270_sample_t s;
    if (!bmi270_read_sample(&s)) return;

    imu_record_t rec;
    rec.ts_us = ts_now_us();
    rec.ax = s.ax; rec.ay = s.ay; rec.az = s.az;
    rec.gx = s.gx; rec.gy = s.gy; rec.gz = s.gz;
    rec.seq = seq++;

    fifo_push(&rec);
}

// ---- Process & Notify ----
static void process_and_notify(void)
{
    if (!m_imu_service.streaming || m_conn_handle == BLE_CONN_HANDLE_INVALID) return;

    // Pack as many records as fit (<=244 bytes)
    uint8_t payload[244];
    uint16_t used = 0;
    imu_record_t r;
    while ((used + sizeof(r)) <= sizeof(payload) && fifo_pop(&r)) {
        memcpy(&payload[used], &r, sizeof(r));
        used += sizeof(r);
    }
    if (used) {
        ble_imu_notify(&m_imu_service, payload, used);
    }
}

// ---- Init ----
static void spi_init(void)
{
    nrfx_spim_config_t cfg = NRFX_SPIM_DEFAULT_CONFIG;
    cfg.sck_pin  = SPI_SCK_PIN;
    cfg.mosi_pin = SPI_MOSI_PIN;
    cfg.miso_pin = SPI_MISO_PIN;
    cfg.ss_pin   = NRFX_SPIM_PIN_NOT_USED; // manual CS
    cfg.frequency = BMI270_SPI_FREQ;
    cfg.mode      = BMI270_SPI_MODE;
    cfg.bit_order = NRF_SPIM_BIT_ORDER_MSB_FIRST;
    APP_ERROR_CHECK(nrfx_spim_init(&spi0, &cfg, NULL, NULL));
    nrf_gpio_cfg_output(SPI_CS_PIN);
    nrf_gpio_pin_set(SPI_CS_PIN);
}

static void log_init(void)
{
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

int main(void)
{
    log_init();
    APP_ERROR_CHECK(nrf_pwr_mgmt_init());
    APP_ERROR_CHECK(nrf_sdh_enable_request());
    uint32_t ram_start = 0;
    APP_ERROR_CHECK(nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start));
    APP_ERROR_CHECK(nrf_sdh_ble_enable(&ram_start));

    gap_params_init();
    gatt_init();
    services_init();
    advertising_start();

    timestamp_timer_init();
    spi_init();
    bmi270_init(&spi0, imu_drdy_callback);

    for (;;) {
        if (NRF_LOG_PROCESS() == false) {
            __WFE();
        }
        process_and_notify();
    }
}
