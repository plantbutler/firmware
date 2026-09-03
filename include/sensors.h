/* include/sensors.h — PCF8575 + mux + the open-channel canary + the home hall + I2C
   health. Spec §1's module table, §2.10, §2.13, §5, §7.
   EVERY failure returns false and never a value. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

bool     sensors_begin(void);                   /* probe, one recovery, reset the counters */
bool     sensors_select(uint8_t ch);            /* P0..P3 = ch, P4..P15 HIGH, always */
bool     sensors_read_raw(uint8_t ch, uint16_t *raw);  /* select, >= 1 ms, read twice, keep 2nd */

/* ONE caller: netfsm.cpp's NET_IDLE pass (task 24), once per report cycle, immediately
   before report_build() reads the channels. That pass issues zero AT commands, which is how
   §3's "skipped in any pass where a modem command ran" holds by construction. main.cpp's
   loop() does NOT call this. Reads ch0..ch5 AND the unwired canary channel, feeding between
   channels. Every failure returns false; the canary matching every wired channel is one. */
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
   has ever moved, never a sentinel — _int_in rejects a leading '-' and any non-digit, so
   a "-1" or "never" in a chN would 400 the whole report (§2.10). Staleness is REPORTED,
   never enforced. */
uint32_t sensors_float_change_age_s(void);

void     sensors_scan(char *out, size_t cap);   /* the `i2c` console command */
