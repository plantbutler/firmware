/* src/safety.cpp — safety_tick, safety_wait_ms, the float debounce, both latches and
   dose_run(). Spec §2. The include list is deliberately complete and deliberately SHORT:
   §9 greps this file for zero hits of the network seam, the library wrapper and the modem
   driver, and none of the eight below is any of those. */
#include "safety.h"
#include "cart.h"        /* cart_pos_known(), cart_pos() -- task 18's rule 9 */
#include "cli.h"         /* cli_stop_requested(), cli_stop_clear() -- task 16 */
#include "config.h"      /* every PB_* the ladder, the caps and the loop read */
#include "hal.h"
#include "noinit.h"      /* g_nv, noinit_commit(), noinit_reset_mid() -- the two latches */
#include "pins.h"        /* PIN_HALL_FLOAT -- the float debounce reads it directly */
#include "pulses.h"      /* pulses_flow(), pulses_flow_rate(), pulses_to_ml(), the leak rearm */
#include "sensors.h"     /* sensors_i2c_healthy() -- the ladder's i2c refusal */

static bool g_dosing;                    /* true only between the ON and OFF writes */

bool safety_dosing(void) { return g_dosing; }
void safety_set_dosing(bool on) { g_dosing = on; }

void safety_tick(void) {
  if (!g_dosing) hal_pump_write(false);  /* idle ACTIVELY re-asserts OFF, every pass */
  hal_wdt_feed();                        /* the ONE feeder in the program */
}

void safety_wait_ms(uint32_t ms) {
  uint32_t t0 = hal_millis();
  while (hal_millis() - t0 < ms) safety_tick();
}

/* §2.10. N consecutive OK to GRANT; ONE bad sample refuses immediately. The wait between
   samples is safety_wait_ms(), which calls safety_tick() on every iteration -- so the pump
   is idle-re-asserted and the dog fed throughout. */
bool safety_float_ok_debounced(void) {
  for (uint8_t i = 0; i < PB_FLOAT_OK_SAMPLES; ++i) {
    if (hal_pin_read(PIN_HALL_FLOAT) != PB_LOW) return false;
    if (i + 1u < PB_FLOAT_OK_SAMPLES) safety_wait_ms(PB_FLOAT_SAMPLE_MS);
  }
  return true;
}

/* §2.11. The checksum is recomputed on EVERY write (§2.3), so a partial clobber cannot
   read back as a valid latch. */
void safety_dry_set(bool on) {
  g_nv.dry_latched = on;
  noinit_commit();
}
bool safety_dry(void) { return g_nv.dry_latched; }

/* §2.10's second consequence: the flap counter. Not persisted -- process-lifetime state,
   exactly like g_dosing above. TWO call sites, both in task 17's dose_run() exit helpers;
   see the header for the exact contract. */
static uint8_t g_float_refusals;

void safety_float_refusal_count(bool refused_for_float) {
  if (refused_for_float) { if (g_float_refusals < 255u) g_float_refusals++; }
  else                     g_float_refusals = 0u;
}
bool safety_float_flap(void) { return g_float_refusals >= PB_FLOAT_FLAP_LIMIT; }

/* §2.7. The reader dose_run()'s ladder checks, above the dry latch, so the more specific
   reason is the one reported. The setter lives in dose_end_ml_() below, and nowhere else. */
bool safety_contra(void) { return g_nv.contra_latched; }

/* THE ONLY CLEAR. No timer, no successful anything, no backend command, no `dry off`. */
bool safety_contra_clear(void) {
  bool was = g_nv.contra_latched;
  g_nv.contra_latched = false;
  noinit_commit();
  return was;
}

/* ---- the two exits. Both ALWAYS set every field, so a refusal can never ack the
   previous dose's millilitres (§2.8's second eye-checkable property). ---- */
static dose_result_t g_last_result = DOSE_OK;
static uint16_t      g_last_flow_ml;
static uint32_t      g_last_ms, g_last_pulses, g_last_end_ms;
static uint8_t       g_last_outlet;
static const char   *g_last_err;
static bool          g_float_granted;
static uint16_t      g_pulses_per_l = PB_PULSES_PER_L_DEFAULT;
static uint32_t      g_prime_ms     = PB_PRIME_MS_DEFAULT;
static uint32_t      g_stall_ms     = PB_STALL_MS_DEFAULT;

static dose_result_t dose_end_(dose_result_t r, const dose_req_t *q) {
  g_last_result = r;
  g_last_flow_ml = 0u;              /* a refusal delivered nothing. Never the last figure. */
  g_last_ms = 0u; g_last_pulses = 0u;
  g_last_outlet = q ? q->outlet : 0u;
  g_last_err = err_of(r);
  /* §2.10: the counter is incremented ONLY by a float refusal. Every other refusal leaves
     it alone -- a rig refusing for a stalled cart must not forget that the float has been
     flapping for an hour. dose_end_ml_() below is what clears it. */
  if (r == DOSE_REFUSED_FLOAT) safety_float_refusal_count(true);
  return r;
}

static dose_result_t dose_end_ml_(dose_result_t r, uint32_t got_pulses, uint32_t elapsed_ms,
                                  uint8_t outlet, uint32_t prime_ms, bool long_prime) {
  g_last_result  = r;
  g_last_pulses  = got_pulses;
  g_last_flow_ml = (uint16_t)pulses_to_ml(got_pulses, g_pulses_per_l);
  g_last_ms      = elapsed_ms;
  g_last_outlet  = outlet;
  g_last_err     = err_of(r);
  safety_float_refusal_count(false);     /* only a GRANTED dose reaches here (§2.10) */

  /* §2.7. THE ONE PLACE THE LATCH IS SET. Five conditions, each doing one job:
       g_float_granted      this was a PERMITTED dose -- a refusal proves nothing about flow;
       !long_prime          a console `pump <ms> prime` of a dry line is where "float OK, no
                            flow" is the EXPECTED result: priming a dry line IS running the
                            pump into air on purpose, and without this exemption bring-up 7a
                            would latch on its very first attempt and §13's "run it again"
                            would be wrong, because the second run would return
                            DOSE_REFUSED_CONTRA;
       float still OK       if it dropped, the two sensors AGREE (the tank ran out) and this
                            is an ordinary DOSE_ABORT_FLOAT;
       got_pulses == 0      NOTHING at all, ever -- a dose that flowed and then stalled has
                            got > 0, and that is an ordinary DOSE_ABORT_NOFLOW;
       elapsed >= prime_ms  THIS dose's effective prime window, not the configured default:
                            a `stop` typed 200 ms in is not evidence of anything. */
  if (g_float_granted && !long_prime &&
      hal_pin_read(PIN_HALL_FLOAT) == PB_LOW &&
      got_pulses == 0u && elapsed_ms >= prime_ms) {
    g_nv.contra_latched = true;
    noinit_commit();
    safety_set_err("contra");      /* AFTER err_of(r): the latch is the louder fact */
  }
  return r;
}

uint16_t      dose_flow_ml(void)     { return g_last_flow_ml; }
dose_result_t dose_last_result(void) { return g_last_result; }
uint32_t      dose_last_ms(void)     { return g_last_ms; }
uint32_t      dose_last_pulses(void) { return g_last_pulses; }
uint8_t       dose_last_outlet(void) { return g_last_outlet; }

/* §4.1's fixed enum: bare lowercase tokens, no whitespace, ever. A space here splits into
   a non-k=v token and 400s the whole report at exactly the moment it matters. */
const char *err_of(dose_result_t r) {
  switch (r) {
    case DOSE_OK:               return "none";
    case DOSE_REFUSED_WDT:      return "wdt";
    case DOSE_REFUSED_DRY:      return "dry";
    case DOSE_REFUSED_CONTRA:   return "contra";
    case DOSE_REFUSED_BOOT:     return "boot";
    case DOSE_REFUSED_RANGE:    return "range";
    case DOSE_REFUSED_CAL:      return "cal";
    case DOSE_REFUSED_FLOAT:    return "float";
    case DOSE_REFUSED_POS:      return "pos";
    case DOSE_REFUSED_I2C:      return "i2c";
    case DOSE_REFUSED_BUSY:     return "busy";
    case DOSE_REFUSED_COOLDOWN: return "cooldown";
    case DOSE_REFUSED_NOISE:    return "noise";
    case DOSE_ABORT_CAP:        return "cap";
    case DOSE_ABORT_NOFLOW:     return "noflow";
    case DOSE_ABORT_NOISE:      return "noise";
    case DOSE_ABORT_FLOAT:      return "float";
    case DOSE_ABORT_POS:        return "pos";
    case DOSE_ABORT_STOP:       return "stop";
    case DOSE_RESULT_COUNT:     break;      /* the sentinel is not a result */
  }
  return "none";
}

const char *safety_last_err(void) {
  /* The `resetmid` token's ONLY producer, and it is lazy on purpose: main.cpp does not
     have to remember to call anything for bring-up 7c's `last=resetmid` to be reachable
     after a hang-forced reset (§2.3). */
  if (g_last_err == 0 && noinit_reset_mid()) g_last_err = "resetmid";
  return g_last_err ? g_last_err : "none";
}
void safety_set_err(const char *tok) { g_last_err = tok; }

uint16_t cfg_pulses_per_l_get(void) { return g_pulses_per_l; }
bool cfg_pulses_per_l_set(uint16_t v) {
  if (v < PB_PULSES_PER_L_MIN || v > PB_PULSES_PER_L_MAX) return false;
  g_pulses_per_l = v;
  return true;
}

/* ---- dose_run(). THE ONLY CALLER THAT EVER ASSERTS D6 IN THE PROGRAM.
   Three properties are meant to be checkable by eye (§2.8) -- and by a plain grep of this
   file for the two calls themselves, which is why this comment names neither literally:
     (a) exactly one call that turns the pump ON and exactly one that turns it OFF below
         it, with NO `return` between them -- the loop's only exit is a `break`;
     (b) every refusal is ABOVE the ON write, so a refused dose never asserts D6;
     (c) the loop body's first statement is safety_tick(), so the dog is fed on every
         iteration and a 60 s dose is legal under a 5592 ms grant (§3). ---- */
dose_result_t dose_run(const dose_req_t *q) {
  cli_stop_clear();          /* a stop typed and answered BEFORE this dose is not its abort */
  g_float_granted = false;

  /* --- the ladder, in §2.8's printed order. The ORDER is the contract: the more specific
     reason must be the one reported, so contra sits above dry and cal above range. --- */
  if (safety_dosing())                       return dose_end_(DOSE_REFUSED_BUSY, q);
  if (!hal_wdt_alive())                      return dose_end_(DOSE_REFUSED_WDT, q);
  if (safety_contra())                       return dose_end_(DOSE_REFUSED_CONTRA, q);
  if (safety_dry())                          return dose_end_(DOSE_REFUSED_DRY, q);
  if (hal_millis() < PB_BOOT_GAP_MS)         return dose_end_(DOSE_REFUSED_BOOT, q);
  if (g_last_end_ms != 0u &&
      hal_millis() - g_last_end_ms < PB_DOSE_MIN_GAP_MS)
                                             return dose_end_(DOSE_REFUSED_COOLDOWN, q);
  if (g_pulses_per_l < PB_PULSES_PER_L_MIN ||
      g_pulses_per_l > PB_PULSES_PER_L_MAX)  return dose_end_(DOSE_REFUSED_CAL, q);
  if (!q->by_time && (q->ml == 0u || q->ml > PB_DOSE_RIG_MAX_ML))
                                             return dose_end_(DOSE_REFUSED_RANGE, q);
  if (q->cap_ms == 0u)                       return dose_end_(DOSE_REFUSED_RANGE, q);
  /* outlet 0 is a LEGAL backend command, so it arrives here from the wire and is refused
     here as well as by exec.cpp -- never a sentinel, never assumed non-zero. */
  if (q->need_pos && (q->outlet < 1u || q->outlet > PB_OUTLETS))
                                             return dose_end_(DOSE_REFUSED_RANGE, q);
  if (pulses_flow_rate() > PB_FLOW_IDLE_MAX_HZ)
                                             return dose_end_(DOSE_REFUSED_NOISE, q);
  if (!sensors_i2c_healthy())                return dose_end_(DOSE_REFUSED_I2C, q);
  if (!safety_float_ok_debounced())          return dose_end_(DOSE_REFUSED_FLOAT, q);
  if (q->need_pos && !cart_pos_known())      return dose_end_(DOSE_REFUSED_POS, q);
  if (q->need_pos && cart_pos() != q->outlet) return dose_end_(DOSE_REFUSED_POS, q);
  g_float_granted = true;                    /* consumed by dose_end_ml_() -- §2.7 */

  /* --- the two cap clamps, in §2.8's own arithmetic and its own order --- */
  uint32_t cap_ms = q->cap_ms;
  if (cap_ms > PB_DOSE_CAP_MS_MAX) cap_ms = PB_DOSE_CAP_MS_MAX;   /* == butler MAX_CAP_S */
#if PB_ML_PER_S_MEASURED > 0
  if (!q->by_time) {           /* the cap may never authorise more than 2x the water asked for */
    uint32_t bound = (uint32_t)q->ml * 1000u / PB_ML_PER_S_MEASURED
                     * PB_CAP_SLACK_NUM / PB_CAP_SLACK_DEN;
    if (bound && cap_ms > bound) cap_ms = bound;
  }
#endif
  uint32_t target = 0u;
  if (!q->by_time) {
    /* MULTIPLY FIRST, DIVIDE SECOND. `ml * (cfg/1000)` truncates the calibration to whole
       pulses per millilitre: at the nominal cfg = 5880 that is 5 instead of 5.88, so every
       metered dose stops 15% short forever, and at the legal cfg = 1999 it is a 2x error.
       Neither is visible to butler's 2*flow_ml < ml alert. Overflow is impossible and the
       range checks above are the proof: 250 x 20000 = 5e6, three orders below UINT32_MAX. */
    target = (uint32_t)q->ml * (uint32_t)g_pulses_per_l / 1000u;
    if (target == 0u) return dose_end_(DOSE_REFUSED_RANGE, q);  /* never "run to cap" */
  }
  uint32_t prime_ms = q->long_prime ? PB_PRIME_LONG_MS : g_prime_ms;
  if (q->long_prime && cap_ms > PB_PRIME_CAP_MS) cap_ms = PB_PRIME_CAP_MS;

  uint32_t flow0 = pulses_flow(), got = 0u, last_got = 0u;
  uint32_t t0 = hal_millis(), last_edge = t0, last_bus = t0, el = 0u;
  dose_result_t r = DOSE_ABORT_CAP;

  g_nv.dose_in_flight = true; noinit_commit();  /* a reset from here on latches dry (§2.3) */
  safety_set_dosing(true);                     /* safety_tick() now KEEPS the ON write */
  hal_pump_write(true);                         /* <-- THE ONLY ASSERTION OF D6 */
  for (;;) {
    safety_tick();                              /* fed on EVERY iteration */
    uint32_t now = hal_millis();
    el  = now - t0;                             /* unsigned diff: rollover-safe */
    got = pulses_flow() - flow0;
    if (got != last_got) { last_got = got; last_edge = now; }

    /* 1. BOTH NOISE RULES COME FIRST, ABOVE THE TARGET RULE. A D2 storming at the ISR's
          own 2 kHz ceiling reaches a 250 ml target (1250 pulses at cfg = 5000) in ~625 ms;
          testing `got >= target` first would ack DOSE_OK and flow_ml=250 for water that
          never moved. The estimator's window is 100 ms, so the storm is visible in ~0.1 s
          against a >= 0.6 s target. The pre-dose PB_FLOW_IDLE_MAX_HZ guard only catches a
          storm that was ALREADY running; this catches one that starts with the pump. */
    if (pulses_flow_rate() > PB_FLOW_MAX_HZ)           { r = DOSE_ABORT_NOISE;  break; }
#if PB_ML_PER_S_MEASURED > 0
    /* 2. Delivered-vs-elapsed plausibility on the DOSE_OK path: reaching the target in far
          less time than the rig can physically deliver it is noise, not a fast pump. Armed
          only once bring-up 7b commits the rate. */
    if (target && got >= target &&
        el * PB_ML_PER_S_MEASURED * PB_PLAUS_NUM < (uint32_t)q->ml * 1000u * PB_PLAUS_DEN)
                                                       { r = DOSE_ABORT_NOISE;  break; }
#endif
    if (target && got >= target)                       { r = DOSE_OK;           break; }
    if (el >= cap_ms)                                  { r = DOSE_ABORT_CAP;    break; }

    /* 5. PRIME: nothing at all came out in the prime window. `prime` EXTENDS it (task 17
          sets prime_ms to PB_PRIME_LONG_MS); it never removes it.
       6. STALL: armed on TIME, not on `got`, so zero flow can never disarm it. Arming it
          on got >= PB_PRIME_MIN_PULSES -- the design's own form -- meant that a dose with
          no flow at all never armed it, and `prime` suppressed rule 5, which made
          `pump 60000 prime` an unconditional sixty-second dry run. */
    if (el >= prime_ms && got < PB_PRIME_MIN_PULSES)    { r = DOSE_ABORT_NOFLOW; break; }
    if (el >= prime_ms && (now - last_edge) >= g_stall_ms)
                                                       { r = DOSE_ABORT_NOFLOW; break; }

    /* 7. the float: ONE bad sample aborts. That direction is dry (§2.10). */
    if (hal_pin_read(PIN_HALL_FLOAT) != PB_LOW)        { r = DOSE_ABORT_FLOAT;  break; }
    /* 8. the console's last-resort abort (§2.12). */
    if (cli_stop_requested())                          { r = DOSE_ABORT_STOP;   break; }
    /* 9. the README's "I2C hung, home hall unreadable" row: a LIVE expander read, at most
          once per PB_POS_RECHECK_MS, inside the dose. */
    if ((now - last_bus) >= PB_POS_RECHECK_MS) {
      last_bus = now;
      if (!cart_bus_check())                           { r = DOSE_ABORT_POS;    break; }
    }
    if (q->hang && el >= PB_HANG_MS) {
      /* bring-up 7c. D6 STAYS ASSERTED and the dog is NOT fed: the watchdog must bite, the
         reset must drop the pump, and the honest spill is 5592 ms x the flow rate 7b
         measured (~170 ml at 30 ml/s). Writing the pump off here would make the spill zero
         and 7c would prove nothing. This is the only loop in the program that is meant not
         to terminate, and it is in the one function §9's for(;;) grep exempts.
         hal_millis() is called so the loop is not an empty body with no forward progress. */
      for (;;) { (void)hal_millis(); }
    }
  }
  hal_pump_write(false);          /* unconditional, ONE exit, before any bookkeeping */

  safety_set_dosing(false);
  g_nv.dose_in_flight = false; noinit_commit();
  g_last_end_ms = hal_millis();
  pulses_leak_rearm_at(g_last_end_ms + PB_COAST_MS);  /* impeller coast-down is not a leak */
  hal_serial_drain();             /* the UART ring: impatience typed during the dose */
  cli_stop_clear();               /* and cli.cpp's pushback buffer, for the same reason
                                     (§2.8, §15.3): three impatient `pump 60000` lines must
                                     not become 180 seconds of pumping the moment this ends */
  return dose_end_ml_(r, got, g_last_end_ms - t0, q->outlet, prime_ms, q->long_prime);
}

#ifdef PB_NATIVE
/* Host-suite seam. cfg_pulses_per_l_set() refuses an out-of-range value by contract, so
   this is the only way a test can put a corrupted calibration behind DOSE_REFUSED_CAL --
   the rung exists to catch a value that got in some OTHER way (a corrupted .noinit, a
   future backend cal=, or a bug), and a test that cannot produce one is not testing it. */
void safety_force_bad_cal_(void) { g_pulses_per_l = 0u; }

/* Host-suite seam (task 17 fix round 2). g_last_end_ms is the third process-lifetime
   static of this shape in this file, after g_dosing and g_float_refusals -- no ordinary
   dose_run() call can express "no dose has ever ended", only a fresh non-zero stamp, so
   the host suite needs its own way to say it between the hundreds of "boots" one binary
   runs. pb_test_teardown() is the only caller. */
void safety_reset_dose_cooldown_(void) { g_last_end_ms = 0u; }
#endif
