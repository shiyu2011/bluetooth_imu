
/* Minimal SPI-based driver for the Bosch BMI270 IMU: configures the sensor,
 * sets up the interrupt line via GPIOTE, and exposes helpers to read raw
 * accelerometer/gyroscope samples.
 */
#include "bmi270_driver.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrfx_gpiote.h"
#include "board_config.h"

// BMI270 SPI specifics
// SPI mode 3, max 10 MHz (we use 8 MHz)
#define BMI270_SPI_MODE NRF_SPIM_MODE_3
#define BMI270_SPI_FREQ NRF_SPIM_FREQ_8M
#define BMI270_CS_LOW()  nrf_gpio_pin_clear(SPI_CS_PIN)
#define BMI270_CS_HIGH() nrf_gpio_pin_set(SPI_CS_PIN)

// BMI270 registers (subset)
#define BMI270_REG_CHIP_ID      0x00
#define BMI270_REG_STATUS       0x03
#define BMI270_REG_ACC_CONF     0x40
#define BMI270_REG_ACC_RANGE    0x41
#define BMI270_REG_GYR_CONF     0x42
#define BMI270_REG_GYR_RANGE    0x43
#define BMI270_REG_INT1_IO_CTRL 0x53
#define BMI270_REG_INT1_MAP_FEAT 0x56
#define BMI270_REG_DATA_START   0x0C  // Gyr/Acc data start (little-endian)

// Read mask: set MSB=1 for read auto-increment
static inline uint8_t bmi270_read_cmd(uint8_t reg) { return reg | 0x80; }
static inline uint8_t bmi270_write_cmd(uint8_t reg){ return reg & 0x7F; }

static const nrfx_spim_t * m_spim;
static bmi270_drdy_cb_t m_cb;

static void spi_txrx(uint8_t *tx, size_t txlen, uint8_t *rx, size_t rxlen)
{
    nrfx_spim_xfer_desc_t xfer = NRFX_SPIM_XFER_TRX(tx, txlen, rx, rxlen);
    nrfx_spim_xfer(m_spim, &xfer, 0);
}

static void write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { bmi270_write_cmd(reg), val };
    uint8_t rx[2] = {0};
    BMI270_CS_LOW();
    spi_txrx(tx, 2, rx, 2);
    BMI270_CS_HIGH();
    nrf_delay_us(20);
}

static uint8_t read_reg(uint8_t reg)
{
    uint8_t tx[2] = { bmi270_read_cmd(reg), 0x00 };
    uint8_t rx[2] = {0};
    BMI270_CS_LOW();
    spi_txrx(tx, 2, rx, 2);
    BMI270_CS_HIGH();
    return rx[1];
}

static void read_burst(uint8_t start_reg, uint8_t *buf, size_t len)
{
    uint8_t tx0 = bmi270_read_cmd(start_reg);
    nrfx_spim_xfer_desc_t xfer = NRFX_SPIM_XFER_TRX(&tx0, 1, buf, len);
    BMI270_CS_LOW();
    nrfx_spim_xfer(m_spim, &xfer, 0);
    BMI270_CS_HIGH();
}

void bmi270_gpiote_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    if (pin == IMU_INT_PIN && m_cb) m_cb();
}

void bmi270_init(const nrfx_spim_t * spim_inst, bmi270_drdy_cb_t cb)
{
    m_spim = spim_inst;
    m_cb = cb;

    nrf_gpio_cfg_output(SPI_CS_PIN);
    BMI270_CS_HIGH();

    // Init SPI instance done in main.c

    // Basic BMI270 setup (ODR ~100 Hz, ranges +-2g, +-250 dps)
    // Actual BMI270 requires feature config loading in production;
    // for MVP streaming, we rely on base reg config.
    // Accel config: ODR=100Hz (0x08), OSR=2, perf mode normal (simplified)
    write_reg(BMI270_REG_ACC_CONF,  0xA8); // example config
    write_reg(BMI270_REG_ACC_RANGE, 0x00); // ±2g
    // Gyro config: ODR=100Hz, normal mode
    write_reg(BMI270_REG_GYR_CONF,  0xA9);
    write_reg(BMI270_REG_GYR_RANGE, 0x00); // ±2000 dps -> set to 0x00 for ±2000? adjust as needed

    // Map data-ready to INT1 (simplified)
    write_reg(BMI270_REG_INT1_IO_CTRL, 0x0A); // push-pull, active high
    write_reg(BMI270_REG_INT1_MAP_FEAT, 0x01); // map DRDY to INT1 (example)

    // Setup GPIOTE for IMU INT
    if (!nrfx_gpiote_is_init()) nrfx_gpiote_init(0);
    nrfx_gpiote_in_config_t in_cfg = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
    in_cfg.pull = NRF_GPIO_PIN_PULLDOWN;
    nrfx_gpiote_in_init(IMU_INT_PIN, &in_cfg, bmi270_gpiote_handler);
    nrfx_gpiote_in_event_enable(IMU_INT_PIN, true);
}

bool bmi270_read_sample(bmi270_sample_t *out)
{
    uint8_t raw[12] = {0};
    read_burst(BMI270_REG_DATA_START, raw, sizeof(raw));

    // Order: GxL,GxH,GyL,GyH,GzL,GzH,AxL,AxH,AyL,AyH,AzL,AzH
    out->gx = (int16_t)((raw[1] << 8) | raw[0]);
    out->gy = (int16_t)((raw[3] << 8) | raw[2]);
    out->gz = (int16_t)((raw[5] << 8) | raw[4]);
    out->ax = (int16_t)((raw[7] << 8) | raw[6]);
    out->ay = (int16_t)((raw[9] << 8) | raw[8]);
    out->az = (int16_t)((raw[11]<< 8) | raw[10]);
    return true;
}

void bmi270_int_handler(void)
{
    // Placeholder if polled instead of GPIOTE callback
}
