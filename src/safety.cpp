/* safety.cpp: the pump's only writer, the watchdog's only feeder, the float debounce, the latches, dose_run.
   Includes no network seam, no board library and no modem driver: the build check greps this file for them. */
#include "safety.h"
#include "cart.h"
#include "cli.h"
#include "config.h"
#include "hal.h"
#include "noinit.h"
#include "pins.h"
#include "pulses.h"
#include "sensors.h"

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

/* N consecutive OK samples to grant; one bad sample refuses at once. The wait between
   samples feeds the dog and re-asserts pump OFF. */
bool safety_float_ok_debounced(void) {
  for (uint8_t i = 0; i < PB_FLOAT_OK_SAMPLES; ++i) {
    if (hal_pin_read(PIN_HALL_FLOAT) != PB_LOW) return false;
    if (i + 1u < PB_FLOAT_OK_SAMPLES) safety_wait_ms(PB_FLOAT_SAMPLE_MS);
  }
  return true;
}

/* The checksum is recomputed on every write, so a partial clobber cannot read back as a
   valid latch. */
void safety_dry_set(bool on) {
  g_nv.dry_latched = on;
  noinit_commit();
}
bool safety_dry(void) { return g_nv.dry_latched; }

/* The float flap counter. Process-lifetime like g_dosing, not persisted; the two dose
   exits below are its only callers. */
static uint8_t g_float_refusals;

void safety_float_refusal_count(bool refused_for_float) {
  if (refused_for_float) { if (g_float_refusals < 255u) g_float_refusals++; }
  else                     g_float_refusals = 0u;
}
bool safety_float_flap(void) { return g_float_refusals >= PB_FLOAT_FLAP_LIMIT; }

/* Read by the ladder above the dry latch, so the more specific reason is reported. Set in
   dose_end_ml_() below and nowhere else. */
bool safety_contra(void) { return g_nv.contra_latched; }

/* THE ONLY CLEAR: no timer, no successful dose, no backend command, no `dry off`. */
bool safety_contra_clear(void) {
  bool was = g_nv.contra_latched;
  g_nv.contra_latched = false;
  noinit_commit();
  return was;
}

/* ---- the two exits. Both set every field, so a refusal can never ack the previous
   dose's millilitres. ---- */
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
  /* Only a float refusal counts. Any other refusal leaves the counter alone: a stalled
     cart must not forget an hour of float flapping. dose_end_ml_() below clears it. */
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
  safety_float_refusal_count(false);     /* only a GRANTED dose reaches here */

  /* THE ONE PLACE THE LATCH IS SET. Five conditions, each doing one job:
       g_float_granted      a PERMITTED dose -- a refusal proves nothing about flow;
       !long_prime          priming a dry line runs the pump into air on purpose, so
                            "float OK, no flow" is the expected result there;
       float still OK       if it dropped, both sensors agree the tank ran out: an
                            ordinary DOSE_ABORT_FLOAT;
       got_pulses == 0      nothing at all -- flow that then stalled is DOSE_ABORT_NOFLOW;
       elapsed >= prime_ms  THIS dose's prime window, not the default: a `stop` typed
                            200 ms in is not evidence of anything. */
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

/* Wire tokens: bare lowercase, no whitespace, ever. A space splits into a non-k=v token
   and 400s the whole report at exactly the moment it matters. */
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
  /* The `resetmid` token's ONLY producer, lazy on purpose: main.cpp need not call anything
     for `last=resetmid` to be reachable after a mid-dose reset. */
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

/* ---- THE ONLY CALLER THAT EVER ASSERTS D6. Three properties checkable by eye, and by a
   grep of this file for the pump write, which is why this comment does not name it:
     (a) one call turns the pump ON and one turns it OFF below it, with NO `return`
         between them -- the loop's only exit is a `break`;
     (b) every refusal is ABOVE the ON write, so a refused dose never asserts D6;
     (c) the loop body's first statement feeds the dog, so a 60 s dose is legal under a
         5592 ms grant. ---- */
dose_result_t dose_run(const dose_req_t *q) {
  cli_stop_clear();          /* a stop typed and answered BEFORE this dose is not its abort */
  g_float_granted = false;

  /* --- the ladder. The ORDER is the contract: the more specific reason is the one
     reported, so contra sits above dry and cal above range. --- */
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
  /* outlet 0 is a LEGAL backend command, so it is refused here as well as in exec.cpp --
     never a sentinel, never assumed non-zero. */
  if (q->need_pos && (q->outlet < 1u || q->outlet > PB_OUTLETS))
                                             return dose_end_(DOSE_REFUSED_RANGE, q);
  if (pulses_flow_rate() > PB_FLOW_IDLE_MAX_HZ)
                                             return dose_end_(DOSE_REFUSED_NOISE, q);
  if (!sensors_i2c_healthy())                return dose_end_(DOSE_REFUSED_I2C, q);
  if (!safety_float_ok_debounced())          return dose_end_(DOSE_REFUSED_FLOAT, q);
  if (q->need_pos && !cart_pos_known())      return dose_end_(DOSE_REFUSED_POS, q);
  if (q->need_pos && cart_pos() != q->outlet) return dose_end_(DOSE_REFUSED_POS, q);
  g_float_granted = true;                    /* consumed by dose_end_ml_() */

  /* --- the two cap clamps --- */
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
       pulses per millilitre: at cfg = 5880 that is 5 for 5.88, so every metered dose stops
       15% short forever, and at the legal cfg = 1999 it is a 2x error, neither visible to
       butler's 2*flow_ml < ml alert. Overflow is impossible: the range checks above bound
       it at 250 x 20000 = 5e6, three orders below UINT32_MAX. */
    target = (uint32_t)q->ml * (uint32_t)g_pulses_per_l / 1000u;
    if (target == 0u) return dose_end_(DOSE_REFUSED_RANGE, q);  /* never "run to cap" */
  }
  uint32_t prime_ms = q->long_prime ? PB_PRIME_LONG_MS : g_prime_ms;
  if (q->long_prime && cap_ms > PB_PRIME_CAP_MS) cap_ms = PB_PRIME_CAP_MS;

  uint32_t flow0 = pulses_flow(), got = 0u, last_got = 0u;
  uint32_t t0 = hal_millis(), last_edge = t0, last_bus = t0, el = 0u;
  dose_result_t r = DOSE_ABORT_CAP;

  g_nv.dose_in_flight = true; noinit_commit();  /* a reset from here on latches dry */
  safety_set_dosing(true);                     /* safety_tick() now KEEPS the ON write */
  hal_pump_write(true);                         /* <-- THE ONLY ASSERTION OF D6 */
  for (;;) {
    safety_tick();                              /* fed on EVERY iteration */
    uint32_t now = hal_millis();
    el  = now - t0;                             /* unsigned diff: rollover-safe */
    got = pulses_flow() - flow0;
    if (got != last_got) { last_got = got; last_edge = now; }

    /* 1. BOTH NOISE RULES COME FIRST, ABOVE THE TARGET RULE. A D2 storming at the ISR's
          2 kHz ceiling reaches a 250 ml target (1250 pulses at cfg = 5000) in ~625 ms;
          testing `got >= target` first would ack DOSE_OK for water that never moved. The
          rate window is 100 ms, so the storm shows in ~0.1 s. The pre-dose idle guard only
          catches a storm ALREADY running; this catches one that starts with the pump. */
    if (pulses_flow_rate() > PB_FLOW_MAX_HZ)           { r = DOSE_ABORT_NOISE;  break; }
#if PB_ML_PER_S_MEASURED > 0
    /* 2. Reaching the target in far less time than the rig can physically deliver it is
          noise, not a fast pump. Armed only once the rate has been measured. */
    if (target && got >= target &&
        el * PB_ML_PER_S_MEASURED * PB_PLAUS_NUM < (uint32_t)q->ml * 1000u * PB_PLAUS_DEN)
                                                       { r = DOSE_ABORT_NOISE;  break; }
#endif
    if (target && got >= target)                       { r = DOSE_OK;           break; }
    if (el >= cap_ms)                                  { r = DOSE_ABORT_CAP;    break; }

    /* 5. PRIME: nothing came out in the prime window. `prime` EXTENDS the window; it never
          removes it.
       6. STALL: armed on TIME, not on `got`, so zero flow can never disarm it. Armed on
          got >= PB_PRIME_MIN_PULSES instead, a dose with no flow at all would never arm
          it, and `pump 60000 prime` would be a sixty-second dry run. */
    if (el >= prime_ms && got < PB_PRIME_MIN_PULSES)    { r = DOSE_ABORT_NOFLOW; break; }
    if (el >= prime_ms && (now - last_edge) >= g_stall_ms)
                                                       { r = DOSE_ABORT_NOFLOW; break; }

    /* 7. the float: ONE bad sample aborts. That direction is dry. */
    if (hal_pin_read(PIN_HALL_FLOAT) != PB_LOW)        { r = DOSE_ABORT_FLOAT;  break; }
    /* 8. the console's last-resort abort. */
    if (cli_stop_requested())                          { r = DOSE_ABORT_STOP;   break; }
    /* 9. I2C hung, home hall unreadable: a LIVE expander read, at most once per
          PB_POS_RECHECK_MS, inside the dose. */
    if ((now - last_bus) >= PB_POS_RECHECK_MS) {
      last_bus = now;
      if (!cart_bus_check())                           { r = DOSE_ABORT_POS;    break; }
    }
    if (q->hang && el >= PB_HANG_MS) {
      /* The deliberate hang. D6 STAYS ASSERTED and the dog is NOT fed: the watchdog must
         bite and the reset must drop the pump, spilling 5592 ms x the flow rate (~170 ml
         at 30 ml/s). Writing the pump off here would prove nothing. The only loop in the
         program meant not to terminate, in the one function the build check exempts.
         hal_millis() keeps the body from being empty. */
      for (;;) { (void)hal_millis(); }
    }
  }
  hal_pump_write(false);          /* unconditional, ONE exit, before any bookkeeping */

  safety_set_dosing(false);
  g_nv.dose_in_flight = false; noinit_commit();
  g_last_end_ms = hal_millis();
  pulses_leak_rearm_at(g_last_end_ms + PB_COAST_MS);  /* impeller coast-down is not a leak */
  hal_serial_drain();             /* the UART ring: impatience typed during the dose */
  cli_stop_clear();               /* and cli.cpp's pushback buffer: three impatient
                                     `pump 60000` lines must not become 180 s of pumping
                                     the moment this ends */
  return dose_end_ml_(r, got, g_last_end_ms - t0, q->outlet, prime_ms, q->long_prime);
}

#ifdef PB_NATIVE
/* Host-suite seam. cfg_pulses_per_l_set() refuses an out-of-range value, so this is the
   only way a test can put a corrupted calibration behind DOSE_REFUSED_CAL. */
void safety_force_bad_cal_(void) { g_pulses_per_l = 0u; }

/* Host-suite seam. No dose can express "no dose has ever ended", only a fresh stamp, and
   one host binary runs hundreds of boots. pb_test_teardown() is the only caller. */
void safety_reset_dose_cooldown_(void) { g_last_end_ms = 0u; }
#endif
