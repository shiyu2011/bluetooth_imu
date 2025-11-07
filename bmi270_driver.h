
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "nrfx_spim.h"

typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
} bmi270_sample_t;

typedef void (*bmi270_drdy_cb_t)(void);

void bmi270_init(const nrfx_spim_t * spim_inst, bmi270_drdy_cb_t cb);
bool bmi270_read_sample(bmi270_sample_t *out);
void bmi270_int_handler(void);
