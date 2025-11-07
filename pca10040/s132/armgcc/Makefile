
PROJECT_NAME := firmware

# Set these paths for your environment
SDK_ROOT ?= $(HOME)/nRF5_SDK_17.1.0_ddde560
PROJ_DIR := $(CURDIR)/../../..

TARGETS := nrf52832_xxAA
OUTPUT_DIRECTORY := _build

LINKER_SCRIPT := $(SDK_ROOT)/modules/nrfx/mdk/nrf52832_xxAA.ld

C_SOURCE_FILES += \
  $(PROJ_DIR)/main.c \
  $(PROJ_DIR)/bmi270_driver.c \
  $(PROJ_DIR)/ble_imu_service.c \
  $(SDK_ROOT)/components/libraries/util/app_error.c \
  $(SDK_ROOT)/components/libraries/util/app_error_handler_gcc.c \
  $(SDK_ROOT)/components/libraries/util/app_error_weak.c \
  $(SDK_ROOT)/components/libraries/strerror/nrf_strerror.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_backend_rtt.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_backend_interface.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_default_backends.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_frontend.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_str_formatter.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_buf.c \
  $(SDK_ROOT)/components/softdevice/common/nrf_sdh.c \
  $(SDK_ROOT)/components/softdevice/common/nrf_sdh_ble.c \
  $(SDK_ROOT)/components/ble/common/ble_advdata.c \
  $(SDK_ROOT)/components/ble/common/ble_conn_params.c \
  $(SDK_ROOT)/components/ble/ble_services/ble_bas/ble_bas.c \
  $(SDK_ROOT)/components/ble/common/ble_srv_common.c \
  $(SDK_ROOT)/components/ble/nrf_ble_gatt/nrf_ble_gatt.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_spim.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_timer.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_gpiote.c \
  $(SDK_ROOT)/integration/nrfx/legacy/nrf_drv_clock.c \
  $(SDK_ROOT)/modules/nrfx/mdk/system_nrf52.c

INC_FOLDERS += \
  $(PROJ_DIR) \
  $(PROJ_DIR)/../.. \
  $(SDK_ROOT)/components \
  $(SDK_ROOT)/components/ble/common \
  $(SDK_ROOT)/components/ble/nrf_ble_gatt \
  $(SDK_ROOT)/components/ble/ble_services/ble_bas \
  $(SDK_ROOT)/components/libraries/strerror \
  $(SDK_ROOT)/components/libraries/util \
  $(SDK_ROOT)/components/libraries/log \
  $(SDK_ROOT)/components/libraries/log/src \
  $(SDK_ROOT)/components/softdevice/s132/headers \
  $(SDK_ROOT)/components/softdevice/s132/headers/nrf52 \
  $(SDK_ROOT)/components/softdevice/common \
  $(SDK_ROOT)/modules/nrfx \
  $(SDK_ROOT)/modules/nrfx/drivers/include \
  $(SDK_ROOT)/modules/nrfx/mdk \
  $(SDK_ROOT)/integration/nrfx \
  $(SDK_ROOT)/integration/nrfx/legacy

CFLAGS += -DBOARD_PCA10040 -DS132 -DNRF52 -DNRF52832_XXAA -DBLE_STACK_SUPPORT_REQD
CFLAGS += -O3 -g3 -mcpu=cortex-m4 -mthumb -mfloat-abi=softfp -mfpu=fpv4-sp-d16
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing -fno-builtin -fshort-enums
CFLAGS += -DNRF_SD_BLE_API_VERSION=7

ASM_SOURCE_FILES += \
  $(SDK_ROOT)/modules/nrfx/mdk/gcc_startup_nrf52.S

LDFLAGS += -L$(SDK_ROOT)/modules/nrfx/mdk -T$(LINKER_SCRIPT) -Wl,--gc-sections

$(OUTPUT_DIRECTORY)/$(PROJECT_NAME).out: $(C_SOURCE_FILES) $(ASM_SOURCE_FILES)
	@mkdir -p $(OUTPUT_DIRECTORY)
	$(CC) $(CFLAGS) $(INC_FOLDERS:%=-I%) $(ASM_SOURCE_FILES) $(C_SOURCE_FILES) $(LDFLAGS) -o $@
	$(OBJCOPY) -O ihex $@ $(OUTPUT_DIRECTORY)/$(PROJECT_NAME).hex

.PHONY: clean
clean:
	rm -rf $(OUTPUT_DIRECTORY)
