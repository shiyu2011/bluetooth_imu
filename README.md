
# Ativafit BMI270 BLE Stream (nRF52832, SPI 8 MHz)

This is a minimal, **nRF5 SDK 17.1**-style project that reads **BMI270** IMU data over **SPI (8 MHz)**,
timestamps each sample with a **1 MHz hardware timer**, and streams records over **BLE notifications** to a mobile app.

> Target: **nRF52832** (S132 SoftDevice).  
> IMU: **Bosch BMI270** via SPI.  
> Purpose: **Real-time BLE streaming** (Notify).

---

## Project layout

```
firmware/
 ├── README.md
 ├── board_config.h
 ├── sdk_config.h
 ├── main.c
 ├── bmi270_driver.c
 ├── bmi270_driver.h
 ├── ble_imu_service.c
 ├── ble_imu_service.h
 └── pca10040/s132/armgcc/Makefile
```

- `board_config.h` — pin mappings (edit for your board).
- `bmi270_driver.*` — SPI init, data-ready INT, register reads for BMI270.
- `ble_imu_service.*` — custom GATT service (Write control + Notify data).
- `main.c` — app init, timestamp timer init (1 MHz), ISR, FIFO, BLE notifications.
- `sdk_config.h` — minimal set of NRFX + drivers enabled for this build.
- `Makefile` — GCC build. Set `SDK_ROOT` to your nRF5 SDK 17.1 path.

---

## Packet format (Notify)

Each notification carries **one or more records** of this packed struct:

```
struct __attribute__((packed)) imu_record_t {
  uint32_t ts_us;   // timestamp in microseconds (1 MHz TIMER)
  int16_t  ax, ay, az;
  int16_t  gx, gy, gz;
  uint16_t seq;     // sequence id (increments per record)
};
```

**Size:** 4 + 12 + 12 + 2 = 30 bytes/record.  
With an MTU of 247 (payload ~244), you can pack up to 8 records per notification (240 bytes).

---

## Build & Flash

1. Install **nRF5 SDK 17.1.0** and **GNU Arm Embedded Toolchain**.
2. Set environment variables:
   ```bash
   export SDK_ROOT=/path/to/nRF5_SDK_17.1.0_ddde560
   export TOOLCHAIN_PATH=/path/to/gcc-arm-none-eabi
   ```
3. Build:
   ```bash
   make -C pca10040/s132/armgcc
   ```
4. Flash SoftDevice (once):
   ```bash
   nrfjprog --eraseall
   nrfjprog --program ${SDK_ROOT}/components/softdevice/s132/hex/s132_nrf52_7.2.0_softdevice.hex
   nrfjprog --reset
   ```
5. Flash app:
   ```bash
   nrfjprog --program _build/firmware.hex --verify
   nrfjprog --reset
   ```

---

## BLE Service

- **Service UUID**: 0xA600
- **Control Char (Write)**: 0xA601  
  - 0x01 = start streaming  
  - 0x00 = stop streaming
- **Data Char (Notify)**: 0xA602 — streams packed `imu_record_t` frames.

---

## Notes

- BMI270 register init configures **ODR 100 Hz** for accel+gyro; change in `bmi270_driver.c` if needed.
- Timestamp uses **NRFX TIMER1 @ 1 MHz** for stable microsecond ticks.
- INT1 of BMI270 is routed to `IMU_INT_PIN`; set in `board_config.h`.
- SPI runs at **8 MHz**, mode 3. Pin assignment in `board_config.h`.

