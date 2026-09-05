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

#ifdef PB_NATIVE
/* Host-suite seam, same shape as safety.cpp's g_dosing/g_float_refusals/g_last_end_ms trio
   (task 18 fix round 1, finding 2): g_healthy, g_fails and g_backoff_until are process-
   lifetime statics in sensors.cpp with no ordinary reset path of their own -- only
   sensors_begin() clears them, and no test is obliged to call that if it never touches the
   bus. A test that DOES call sim_set_i2c_fail(true) and then fails its OWN assertion before
   its OWN cleanup line runs never gets there: Unity's longjmp on a failing TEST_ASSERT_*
   skips every line after it, including a `sim_set_i2c_fail(false); sensors_begin();` written
   as the test's own last lines. That leaves g_healthy false (or g_fails/g_backoff_until
   mid-count) for every later test in the binary that never happens to call sensors_begin()
   itself -- one real mutation's diagnostic was seen buried under two misleading downstream
   DOSE_REFUSED_I2C failures this way. pb_test_teardown() is the only caller. */
void sensors_test_reset_health_(void);
#endif
