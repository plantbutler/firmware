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

/* §2.7. g_nv.contra_latched, read-only from here — task 19 owns the setter and the
   `clear contra` release. Declared now, ahead of its own setter, because dose_run()'s
   ladder (below) checks it above the dry latch and task 19 step 4's own code assumes the
   declaration already exists: writing the ladder without this arm and inserting it later
   is how an ordering gets lost. */
bool safety_contra(void);

/* THIS TASK DECLARES dose_result_t, and it is the first declaration in the tree: task 5's
   cut of safety.h carried only safety_tick/safety_wait_ms/safety_dosing/safety_set_dosing,
   and tasks 15 and 16 named DOSE_REFUSED_FLOAT only in prose and comments. DOSE_RESULT_COUNT
   is an ADDITION to spec §2.8's printed enum, and the only one: it is what lets
   test_pump_is_off_on_every_exit_path loop over the enum, so a result added later without a
   way to reach it fails a test instead of going quietly unreachable. err_of() must never
   map it. */
typedef enum { DOSE_OK = 0, DOSE_REFUSED_WDT, DOSE_REFUSED_DRY, DOSE_REFUSED_CONTRA,
               DOSE_REFUSED_BOOT, DOSE_REFUSED_RANGE, DOSE_REFUSED_CAL, DOSE_REFUSED_FLOAT,
               DOSE_REFUSED_POS, DOSE_REFUSED_I2C, DOSE_REFUSED_BUSY, DOSE_REFUSED_COOLDOWN,
               DOSE_REFUSED_NOISE, DOSE_ABORT_CAP, DOSE_ABORT_NOFLOW, DOSE_ABORT_NOISE,
               DOSE_ABORT_FLOAT, DOSE_ABORT_POS, DOSE_ABORT_STOP,
               DOSE_RESULT_COUNT } dose_result_t;

typedef struct { uint8_t outlet; uint16_t ml; bool by_time; uint32_t cap_ms;
                 bool need_pos; bool long_prime; } dose_req_t;
/* dose_req_t.outlet is NEVER a sentinel: water=0 is a legal backend command
   (_int_in(v,"water",0,256), and butler's `outlet is None` guard does not catch 0), so 0
   arrives from the wire and is refused here as well as by task 26's range check. There is
   NO `return` between the ON write and the OFF write in dose_run(): the loop's only exit
   is a `break`. */

/* THE ONLY CALLER OF hal_pump_write(true) IN THE PROGRAM. Spec §2.8. */
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
/* Host-suite seam, exactly like safety_set_dosing() above: cfg_pulses_per_l_set() refuses
   an out-of-range value by contract, so this is the ONLY way a host case can put an
   out-of-range calibration behind DOSE_REFUSED_CAL. The rung exists to catch a value that
   got in some OTHER way — a corrupted .noinit, a future backend cal=, or a bug — and a
   test that cannot produce one is not testing it. */
void safety_force_bad_cal_(void);
#endif
