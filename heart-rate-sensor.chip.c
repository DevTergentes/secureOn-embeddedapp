// Wokwi Custom Chip – Heart-Rate
// SPDX-License-Identifier: MIT

#include "wokwi-api.h"
#include <stdlib.h>

typedef struct {
  pin_t    pin_out;           // “S” → ESP-GPIO33
  uint32_t hr_attr;           // 0-200 BPM
} state_t;

static void on_timer(void *user_data);

/* -------- init -------- */
void chip_init(void) {

  state_t *s  = malloc(sizeof(state_t));

  s->pin_out  = pin_init("S", ANALOG);          //   <-- el pin del JSON
  s->hr_attr  = attr_init("heartRate", 60);

  const timer_config_t cfg = { .callback = on_timer, .user_data = s };
  timer_start(timer_init(&cfg), 100 /* ms */, true);
}

/* ----- timer ISR ------ */
static void on_timer(void *user_data) {

  state_t *s = (state_t*)user_data;
  float v = attr_read_float(s->hr_attr) * 3.3f / 200.0f;

  pin_dac_write(s->pin_out, v);                 // ¡sin condicionales!
}
