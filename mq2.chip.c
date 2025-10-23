// Wokwi Custom Chip – MQ-2   (versión estable)
// SPDX-License-Identifier: MIT

#include "wokwi-api.h"
#include <stdlib.h>

typedef struct {
  pin_t    pin_out;           // OUT0  → ESP-GPIO34
  pin_t    pin_d0;            // D0    → sin uso (alarma)
  uint32_t gas_attr;          // 0-100  %
  uint32_t thr_attr;          // 0-100  %
} state_t;

static void on_timer(void *user_data);

/* -------- init -------- */
void chip_init(void) {

  state_t *s = malloc(sizeof(state_t));

  s->pin_out  = pin_init("OUT0", ANALOG);
  s->pin_d0   = pin_init("D0",   OUTPUT_LOW);

  s->gas_attr = attr_init("gas",       10);
  s->thr_attr = attr_init("threshold", 50);

  const timer_config_t cfg = { .callback = on_timer, .user_data = s };
  timer_start(timer_init(&cfg), 100 /* ms */, true);
}

/* ----- timer ISR ------ */
static void on_timer(void *user_data) {

  state_t *s = (state_t*)user_data;

  float v_gas = attr_read_float(s->gas_attr) * 3.3f / 100.0f;
  float v_thr = attr_read_float(s->thr_attr) * 3.3f / 100.0f;

  pin_dac_write(s->pin_out, v_gas);             // ¡siempre!
  pin_write(s->pin_d0,     v_gas > v_thr);      // HIGH/LOW
}
