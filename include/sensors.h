/* sensors.h: the PCF8575 expander, the mux, the open-channel canary, the home hall and I2C
   health. EVERY failure returns false and never a value. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

bool     sensors_begin(void);                   /* probe, one recovery, reset the counters */
bool     sensors_select(uint8_t ch);            /* P0..P3 = ch, P4..P15 HIGH, always */
bool     sensors_read_raw(uint8_t ch, uint16_t *raw);  /* select, >= 1 ms, read twice, keep 2nd */

/* ONE caller: netfsm.cpp's NET_IDLE pass, once per report cycle, immediately before
   report_build() reads the channels. That pass issues zero AT commands, which is how the
   sweep is skipped in any pass where a modem command ran. main.cpp's loop() does NOT call
   this. Reads ch0..ch5 AND the unwired canary channel, feeding between channels. Every
   failure returns false; the canary matching every wired channel is one. */
bool     sensors_sweep(void);

uint16_t sensors_value(uint8_t ch);
bool     sensors_valid(uint8_t ch);
bool     sensors_stuck(void);                   /* the canary equalled every wired channel */

/* false == bus error, NEVER "not home". A failed read must refuse to move or pump. */
bool     sensors_home_hall(bool *home);

bool     sensors_i2c_healthy(void);
uint32_t sensors_i2c_errors(void);
uint32_t sensors_i2c_txn_per_min(void);         /* `status` prints it so the cost stays visible */

/* Seconds since D5 last changed state: a bare non-negative integer, ALWAYS. 0 before D5
   has ever moved, never a sentinel — butler rejects a leading '-' and any non-digit, so a
   "-1" or "never" in a chN would 400 the whole report. Staleness is REPORTED, never
   enforced. */
uint32_t sensors_float_change_age_s(void);

void     sensors_scan(char *out, size_t cap);   /* the `i2c` console command */

#ifdef PB_NATIVE
/* Host-suite seam: g_healthy, g_fails and g_backoff_until are process-lifetime statics
   that only sensors_begin() clears. A test that injects an I2C fault and then fails its
   OWN assertion never reaches its own cleanup line -- Unity longjmps past it -- and leaves
   the bus unhealthy for every later test in the binary. pb_test_teardown() is the only
   caller. */
void sensors_test_reset_health_(void);
#endif
