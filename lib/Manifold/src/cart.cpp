/* cart.cpp: moves the magnet cart along the lead screw by counted pulses; position 0 is home. Includes no network header of any kind, so this file cannot make a network call. */
#include "cart.h"
#include "config.h"
#include "hal.h"
#include "pulses.h"
#include "safety.h"
#include "sensors.h"

static uint32_t    g_pulses;        /* screw pulses since the last successful home */
static bool        g_home_seen;     /* a home hall assertion has been observed since boot */
static uint8_t     g_pos;           /* 0 == home; 1..PB_OUTLETS == over that gate */
static bool        g_pos_valid;
static bool        g_busy;
static const char *g_err = "none";

bool        cart_busy(void)   { return g_busy; }
uint32_t    cart_pulses(void) { return g_pulses; }
const char *cart_err(void)    { return g_err; }
uint8_t     cart_pos(void)    { return g_pos; }
bool        cart_parked(void) { return g_home_seen && g_pos_valid && g_pos == 0u; }

bool cart_begin(void) {
  hal_servo_us(PB_SERVO_STOP_US);   /* the servo is stopped before anything else is true */
  g_pulses = 0u; g_home_seen = false; g_pos = 0u; g_pos_valid = false;
  g_busy = false; g_err = "none";
  return true;                      /* no movement at boot: the self-home is exec.cpp's,
                                       PB_BOOT_HOME_MS after reset */
}

bool cart_bus_check(void) {         /* one live expander read; false is a bus error */
  bool home = false;
  return sensors_home_hall(&home);
}

/* The one move primitive: a deadline and a stall window, never a blocking wait, with
   safety_tick() on every pass -- what keeps a 45 s traverse legal under a 5592 ms watchdog
   grant. `stop_at_delta` is a pulse count on pulses_screw(); `home_stops` makes the home
   hall a second terminating condition. Returns true only if a terminating condition was
   met; every exit stops the servo. */
static bool move_(uint16_t us, uint32_t stop_at_delta, bool home_stops) {
  uint32_t t0        = hal_millis();
  uint32_t s0        = pulses_screw();
  uint32_t last_edge = t0;
  uint32_t last_seen = s0;
  uint32_t now       = t0;
  bool     ok        = false;

  /* The move cap is the loop condition, so the loop is bounded in its own head. "timeout"
     is armed before the loop so that falling out of the head carries that reason; a
     terminating condition clears it below. */
  g_err  = "timeout";
  g_busy = true;
  hal_servo_us(us);                      /* the one servo start */
  while ((now - t0) < PB_MOVE_CAP_MS) {  /* unsigned: holds across a millis() rollover */
    safety_tick();                       /* pump idle re-asserted, then the dog fed */
    now           = hal_millis();
    uint32_t seen = pulses_screw();
    if (seen != last_seen) { last_seen = seen; last_edge = now; }

    if (home_stops) {
      bool home = false;
      if (!sensors_home_hall(&home)) {   /* a bus error is unknown, never "not home": */
        g_err = "i2c";                   /* driving on blind is what "not home" would do */
        break;
      }
      if (home) { ok = true; break; }
    }
    if (stop_at_delta != 0u && (seen - s0) >= stop_at_delta) { ok = true; break; }
    if ((now - last_edge) >= PB_STALL_WINDOW_MS) { g_err = "stall"; break; }
  }
  hal_servo_us(PB_SERVO_STOP_US);        /* the one servo stop, on every exit path */
  g_busy = false;
  if (ok) g_err = "none";
  return ok;
}

/* Homing is not watering: it runs under the dry latch and the contradiction latch alike,
   so there is no latch check here. A reader who adds one leaves the cart holding gate N
   open under the reservoir head for as long as the latch stands -- after a mid-dose
   watchdog reset, which latches dry, that is until a human types `dry off`. */
bool cart_home(void) {
  g_err = "none";
  g_pos_valid = false;                 /* unknown while moving, so a failure leaves it so */
  if (!move_(PB_SERVO_REV_US, 0u, true)) return false;   /* g_err set by move_ */
  g_pulses = 0u;                       /* zeroed only because the hall actually asserted */
  g_home_seen = true;
  g_pos = 0u;
  g_pos_valid = true;
  return true;
}

void cart_jog(int16_t us, uint32_t ms) {          /* console only; bounded */
  if (ms > PB_SERVO_CAP_MS) ms = PB_SERVO_CAP_MS; /* a typo may not run the screw forever */
  g_pos_valid = false;                            /* an untracked move loses the position */
  g_busy = true;
  hal_servo_us((uint16_t)us);
  safety_wait_ms(ms);                             /* fed on every iteration */
  hal_servo_us(PB_SERVO_STOP_US);
  g_busy = false;
}

#if PB_PULSES_PER_GATE == 0
/* The refusal is structural: with the pitch unknown, the arithmetic below would find the
   target already satisfied at home and report pos=ok while the pump dead-heads against a
   closed manifold. Measuring the pitch on the bench is what retires this arm. */
bool cart_goto(uint8_t outlet) { (void)outlet; g_err = "uncal"; return false; }
bool cart_pos_known(void)      { return false; }
#else
bool cart_goto(uint8_t outlet) {
  g_err = "none";
  if (outlet < 1u || outlet > PB_OUTLETS) { g_err = "range"; return false; }
  if (!g_home_seen && !cart_home()) return false;      /* position must start from home */
  uint32_t want = (uint32_t)PB_PULSES_HOME_TO_1 +
                  (uint32_t)(outlet - 1u) * (uint32_t)PB_PULSES_PER_GATE;
  uint32_t have = g_pulses;
  if (want == have) { g_pos = outlet; g_pos_valid = true; return true; }
  bool     fwd   = want > have;
  uint32_t delta = fwd ? (want - have) : (have - want);
  g_pos_valid = false;                                  /* unknown until the move lands */
  uint32_t s0 = pulses_screw();
  bool ok = move_(fwd ? PB_SERVO_FWD_US : PB_SERVO_REV_US, delta, false);
  uint32_t moved = pulses_screw() - s0;                 /* what actually happened, not what
                                                            was asked for: a stall stops here */
  g_pulses = fwd ? (have + moved) : (have >= moved ? have - moved : 0u);
  if (!ok) { g_home_seen = false; return false; }       /* a stall loses the position */
  g_pos = outlet;
  g_pos_valid = true;
  return true;
}
bool cart_pos_known(void) { return g_home_seen && g_pos_valid; }
#endif
