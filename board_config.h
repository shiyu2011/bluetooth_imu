
#pragma once

// ---------- Board Pins (EDIT THESE FOR YOUR PCB) ----------

// SPI0 (nrfx_spim0)
#define SPI_SCK_PIN     13
#define SPI_MOSI_PIN    15
#define SPI_MISO_PIN    14
#define SPI_CS_PIN      12   // BMI270 CS (active low)

// BMI270 interrupt (data ready)
#define IMU_INT_PIN     11

// LEDs / Button (optional)
#define LED1_PIN        17
#define BUTTON1_PIN     13  // change if conflicts

// BLE device name
#define DEVICE_NAME     "Ativafit-IMU"

// BLE connection parameters
#define MIN_CONN_INTERVAL   MSEC_TO_UNITS(15, UNIT_1_25_MS)
#define MAX_CONN_INTERVAL   MSEC_TO_UNITS(30, UNIT_1_25_MS)
#define SLAVE_LATENCY       0
#define CONN_SUP_TIMEOUT    MSEC_TO_UNITS(4000, UNIT_10_MS)
