/* safety.h: D6's whole story. READ THIS FILE FIRST.
   Includes neither link.h, Network.h, netfsm.h nor WiFiS3.h: this layer cannot make a
   network call, and tools/check.sh greps for all four. */
#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Pump idle re-asserted, then the dog fed — in that order, in one function, with nothing
   between them. Called at the top of loop(), inside safety_wait_ms()'s loop, and inside
   the dose loop. Nowhere else. */
void safety_tick(void);

/* A bounded wait that calls safety_tick() on every iteration. Every loop in the program
   that can iterate over an I2C transfer, a modem call or a millisecond of wall clock
   uses this or its own safety_tick(). */
void safety_wait_ms(uint32_t ms);

bool safety_dosing(void);

/* The only production writer is the dose loop, which sets g_dosing directly; this exists
   so the host suites can reproduce a dose in flight. */
void safety_set_dosing(bool on);

/* PB_FLOAT_OK_SAMPLES (3) consecutive OK readings to GRANT; ONE bad sample refuses
   IMMEDIATELY. The asymmetry is the whole design: refusing on one bad sample is safe,
   granting on one is not, because D5 runs up to a metre to the reservoir alongside a 12 V
   pump lead. The wait between samples is safety_wait_ms(), so the dog is fed and the pump
   idle-re-asserted throughout. */
bool safety_float_ok_debounced(void);

/* The operator's `dry on|off`. Writes g_nv.dry_latched and recomputes the .noinit
   checksum on every write, so the latch survives a WARM reset (watchdog, RESET button) —
   the case that matters is a reset at pump start, with the operator's hands in the
   plumbing. It does not survive a cold boot: nothing in .noinit does, and PB_BOOT_GAP_MS
   refuses for the first 10 s after one regardless. */
void safety_dry_set(bool on);
bool safety_dry(void);

/* >= PB_FLOAT_FLAP_LIMIT (3) CONSECUTIVE DOSE_REFUSED_FLOAT results trips it; while it
   holds the wire carries float=0 and err=float regardless of the report-time debounce.
   Cleared by any GRANTED dose. The predicate is >=, so the THIRD consecutive refusal
   trips it. */
bool safety_float_flap(void);

/* Exactly two call sites, both in the dose loop's exit helpers: dose_end_() passes `true`
   ONLY on the DOSE_REFUSED_FLOAT arm; dose_end_ml_() passes `false` UNCONDITIONALLY,
   because only a granted dose reaches it. A refusal for cooldown, i2c, position or any
   other reason must leave the counter ALONE: a rig refusing for a stalled cart must not
   quietly forget that the float has been flapping for an hour. */
void safety_float_refusal_count(bool refused_for_float);

/* g_nv.contra_latched. SET in exactly one place -- dose_end_ml_(), under five conditions
   each doing one job (safety.cpp). There is deliberately no test setter: a hook that set
   the latch directly would be a second setter, the very thing this design prevents. The
   dose ladder checks this above the dry latch, so the more specific reason is reported. */
bool safety_contra(void);

/* THE ONLY CLEAR. The console command `clear contra` -- two literal tokens, no
   abbreviation, present in BOTH the bench and bring-up binaries because the unattended
   one can latch and a rig releasable only by a reflash is worse. Returns true if it WAS
   latched, so the console can tell "cleared" from "contra=0 already" without a second
   read of safety_contra(). Nothing else in the tree may call this: no timer, no
   successful anything, no backend command, no `dry off`. */
bool safety_contra_clear(void);

/* DOSE_RESULT_COUNT is what lets test_pump_is_off_on_every_exit_path loop over the enum,
   so a result added later without a way to reach it fails a test instead of going quietly
   unreachable. err_of() must never map it. */
typedef enum { DOSE_OK = 0, DOSE_REFUSED_WDT, DOSE_REFUSED_DRY, DOSE_REFUSED_CONTRA,
               DOSE_REFUSED_BOOT, DOSE_REFUSED_RANGE, DOSE_REFUSED_CAL, DOSE_REFUSED_FLOAT,
               DOSE_REFUSED_POS, DOSE_REFUSED_I2C, DOSE_REFUSED_BUSY, DOSE_REFUSED_COOLDOWN,
               DOSE_REFUSED_NOISE, DOSE_ABORT_CAP, DOSE_ABORT_NOFLOW, DOSE_ABORT_NOISE,
               DOSE_ABORT_FLOAT, DOSE_ABORT_POS, DOSE_ABORT_STOP,
               DOSE_RESULT_COUNT } dose_result_t;

typedef struct { uint8_t outlet; uint16_t ml; bool by_time; uint32_t cap_ms;
                 bool need_pos; bool long_prime;
                 bool hang;   /* bring-up only: run the dose PB_HANG_MS, then STOP FEEDING. The
                     field is UNCONDITIONAL: guarding it would break the safety.o hash equality
                     between the bench and bring-up binaries, which is what lets the watchdog
                     be proven on one binary and meant about the other. Only the bring-up
                     console ever sets it true. */
               } dose_req_t;
/* dose_req_t.outlet is NEVER a sentinel: water=0 is a legal backend command (butler's
   `outlet is None` guard does not catch 0), so 0 arrives from the wire and is refused here
   as well as by exec's range check. There is NO `return` between the ON write and the OFF
   write in the dose loop: its only exit is a `break`. */

/* THE ONLY CALLER OF hal_pump_write(true) IN THE PROGRAM. */
dose_result_t dose_run(const dose_req_t *q);
uint16_t      dose_flow_ml(void);
dose_result_t dose_last_result(void);
uint32_t      dose_last_ms(void);
uint32_t      dose_last_pulses(void);
uint8_t       dose_last_outlet(void);
const char   *err_of(dose_result_t r);
const char   *safety_last_err(void);
void          safety_set_err(const char *tok);
uint16_t      cfg_pulses_per_l_get(void);
bool          cfg_pulses_per_l_set(uint16_t v);

#ifdef PB_NATIVE
/* Host-suite seam: cfg_pulses_per_l_set() refuses an out-of-range value by contract, so
   this is the ONLY way a host case can put an out-of-range calibration behind
   DOSE_REFUSED_CAL. The rung exists to catch a value that got in some OTHER way — a
   corrupted .noinit, or a bug — and a test that cannot produce one is not testing it. */
void safety_force_bad_cal_(void);

/* Host-suite seam: g_last_end_ms is a process-lifetime static with no production reset
   path — a real boot starts the process fresh; a host suite reruns hundreds of "boots"
   inside one binary and needs to say "no dose has ended yet" between them, which no dose
   can express (every real end is a fresh non-zero stamp, never a clear).
   pb_test_teardown() is the only caller. */
void safety_reset_dose_cooldown_(void);
#endif
