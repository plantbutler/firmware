/* include/safety.h — D6's whole story. READ THIS FILE FIRST.
   Includes neither link.h, Network.h nor WiFiS3.h: this layer *cannot* make a network
   call, and tools/check.sh greps for it (§3, §9). */
#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Pump idle re-asserted, then the dog fed — in that order, in one function, with nothing
   between them (§2.4). Called at the top of loop(), inside safety_wait_ms()'s loop, and
   inside dose_run()'s loop. Nowhere else. */
void safety_tick(void);

/* A bounded wait that calls safety_tick() on every iteration. Every loop in the program
   that can iterate over an I2C transfer, a modem call or a millisecond of wall clock
   uses this or its own safety_tick() (§3). */
void safety_wait_ms(uint32_t ms);

bool safety_dosing(void);

/* The only production writer is dose_run() (task 17), which sets g_dosing directly; this
   exists so the host suites can reproduce a dose in flight — sensors.cpp's recovery guard
   (§2.13) has to be testable before dose_run() is written. */
void safety_set_dosing(bool on);

/* §2.10. PB_FLOAT_OK_SAMPLES (3) consecutive OK readings to GRANT; ONE bad sample refuses
   IMMEDIATELY. The asymmetry is the whole design: refusing on one bad sample is safe,
   granting on one is not, because D5 runs up to a metre to the reservoir alongside a 12 V
   pump lead. The wait between samples is safety_wait_ms(), which calls safety_tick() on
   every iteration, so the dog is fed and the pump idle-re-asserted throughout. */
bool safety_float_ok_debounced(void);

/* §2.11. The operator's `dry on|off`. Writes g_nv.dry_latched and recomputes the .noinit
   checksum on every write (§2.3), so the latch survives a WARM reset (watchdog, RESET
   button) — the case that mattered, because a brown-out at pump start used to silently
   clear it while the operator's hands were in the plumbing. It does not survive a cold
   boot: nothing in .noinit does, and PB_BOOT_GAP_MS refuses for the first 10 s after one
   regardless. */
void safety_dry_set(bool on);
bool safety_dry(void);

/* §2.10's second consequence. >= PB_FLOAT_FLAP_LIMIT (3) CONSECUTIVE DOSE_REFUSED_FLOAT
   results trips it; task 22 forces float=0 and err=float on the wire while it holds,
   regardless of the report-time debounce. Cleared by any GRANTED dose. (Spec §2.10's prose
   says "above" PB_FLOAT_FLAP_LIMIT while §9's own test name says "after three" — the two
   readings are reconciled in the test name's favour: the predicate below is >=, so the
   THIRD consecutive refusal trips it.) */
bool safety_float_flap(void);

/* Exactly two call sites, both in dose_run()'s exit helpers (task 17): dose_end_() calls
   this with `true` ONLY on the DOSE_REFUSED_FLOAT arm; dose_end_ml_() calls it with `false`
   UNCONDITIONALLY, because §2.10 says the counter is cleared by any GRANTED dose and
   dose_end_ml_() is the function only a granted dose reaches. A refusal for cooldown, i2c,
   position or any other reason must leave the counter ALONE: a rig refusing for a stalled
   cart must not quietly forget that the float has been flapping for an hour. */
void safety_float_refusal_count(bool refused_for_float);
