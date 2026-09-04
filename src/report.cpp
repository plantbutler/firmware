/* src/report.cpp — the wire protocol, pure. Where the traps live (spec §4).
   NO signed integer conversion anywhere: t = hal_boot_salt() + hal_millis() is above 2^31 on
   ordinary boots, so a single percent-d against a uint32_t prints a leading '-', _int_in
   rejects it, and EVERY report 400s from the first one (spec §15.2, §9's grep). Every numeric
   site is %lu with an explicit cast. */
#include "report.h"
#include "config.h"
#include "hal.h"
#include "link.h"
#include "sensors.h"
#include "pulses.h"
#include "safety.h"
#include "cart.h"
#include "secrets.h"   /* PB_CONTROLLER: the two static_asserts below and the c= field.
                          [env:native] and [env:uno_r4_wifi_sim] pass it in build_flags;
                          the two device envs do not, and this header is its only other
                          definition. Its #ifndef guard makes both routes agree. */
#include <stdio.h>
#include <string.h>

static_assert(sizeof(PB_CONTROLLER) + 2 + PB_BODY_WORST_FIXED <= PB_BODY_CAP,
              "the body's own worst case does not fit PB_BODY_CAP (spec §7's term-by-term sum)");
static_assert(sizeof(PB_CONTROLLER) > 1,
              "an empty c= is 'no c= in the report': a permanent 400 (butler.py:252-253)");

static uint32_t g_t_wire, g_t_ms;
static report_ack_t g_ack;
static bool     g_ack_set;
static uint16_t g_last_len;
static uint32_t g_txcap_drops;

void     report_stamp(void)  { g_t_ms = hal_millis(); g_t_wire = hal_boot_salt() + g_t_ms; }
uint32_t report_t_wire(void) { return g_t_wire; }
uint32_t report_t_ms(void)   { return g_t_ms; }
uint16_t report_last_len(void)    { return g_last_len; }
uint32_t report_txcap_drops(void) { return g_txcap_drops; }

static bool put_s(char *b, uint16_t cap, uint16_t *n, const char *fmt, const char *v) {
  if (*n >= cap) return false;
  int w = snprintf(b + *n, (size_t)(cap - *n), fmt, v);
  if (w < 0 || (uint16_t)w >= (uint16_t)(cap - *n)) return false;
  *n = (uint16_t)(*n + w);
  return true;
}
static bool put_u(char *b, uint16_t cap, uint16_t *n, const char *fmt, uint32_t v) {
  if (*n >= cap) return false;
  int w = snprintf(b + *n, (size_t)(cap - *n), fmt, (unsigned long)v);
  if (w < 0 || (uint16_t)w >= (uint16_t)(cap - *n)) return false;
  *n = (uint16_t)(*n + w);
  return true;
}
static bool put_ch(char *b, uint16_t cap, uint16_t *n, uint32_t ch, uint32_t v) {
  if (*n >= cap) return false;
  int w = snprintf(b + *n, (size_t)(cap - *n), " ch%lu=%lu",
                   (unsigned long)ch, (unsigned long)v);
  if (w < 0 || (uint16_t)w >= (uint16_t)(cap - *n)) return false;
  *n = (uint16_t)(*n + w);
  return true;
}

/* Spec §12 item 0: "hal_begin() and EVERY REPORT check the break against the stack, because
   nothing else will." _sbrk is the unchecked libnosys version and __HeapLimit is referenced by
   nothing in the image, so this is the only heap bound that exists — and the 48-hour run is
   exactly when the largest allocator in the program, the network stack, is active. Returns
   false once the margin is crossed, and latches err=heap so the fact reaches the wire.

   It does NOT call net_disable() itself: that flag lives in netfsm.cpp, this file must not
   grow a dependency on a translation unit that does not exist until task 24, and report.cpp
   is compiled into every host suite. Task 24's NET_IDLE pass is the one caller that turns a
   false into `net_disable("heap")`; this function's job is the measurement and the token. */
bool report_heap_ok(void) {
  if (hal_heap_break() < hal_stack_limit() - (uint32_t)PB_STACK_MARGIN) return true;
  safety_set_err("heap");
  return false;
}

uint16_t report_build(char *buf, uint16_t cap) {
  if (!report_may_build()) return 0;    /* err=recv must never reach the wire (§4.3) */
  uint16_t n = 0;
  bool ok = true;
  (void)report_heap_ok();      /* §12 item 0's per-report half. It does not abort the body:
                                  a report that says err=heap is worth more than no report. */
  const bool stuck = sensors_stuck();

  ok = ok && put_s(buf, cap, &n, "c=%s", PB_CONTROLLER);
  ok = ok && put_u(buf, cap, &n, " t=%lu", g_t_wire);

  if (!stuck) {                       /* §5: a stuck mux omits the WIRED channels, not the body */
    for (uint8_t ch = 0; ch < PB_CHANNELS; ++ch)
      if (sensors_valid(ch))
        ok = ok && put_ch(buf, cap, &n, ch, sensors_value(ch));
  }

  /* ch200..ch209, every one clamped: chN must be < MAX_RAW = 2^31 (butler.py:88,251), and a
     storming D2 pushes ch205 past 2^31 in ~12.4 days. At least one is ALWAYS present, so a
     wedged bus produces an alarm instead of silence (§4.1). */
  const uint32_t diag[10] = {
    hal_heap_arena(), hal_heap_ordblks(), hal_stack_hwm(),
    sensors_i2c_errors(), sensors_float_change_age_s(), pulses_leak_count(),
    (uint32_t)link_desyncs(), safety_contra() ? 1u : 0u,
    cart_parked() ? 1u : 0u, hal_wdt_last_delta()
  };
  for (uint32_t i = 0; i < 10; ++i) {
    uint32_t v = diag[i] > (uint32_t)PB_DIAG_CLAMP ? (uint32_t)PB_DIAG_CLAMP : diag[i];
    ok = ok && put_ch(buf, cap, &n, 200u + i, v);
  }

  /* §2.10, §4.1: the DEBOUNCED tank verdict, ANDed with !contra, forced to 0 above
     PB_FLOAT_FLAP_LIMIT consecutive DOSE_REFUSED_FLOAT results. Never 2, never negative:
     _int_in(v,"float",0,2) is half-open and ASCII-digits-only. */
  const bool fl = safety_float_ok_debounced() && !safety_contra() && !safety_float_flap();
  ok = ok && put_u(buf, cap, &n, " float=%lu", fl ? 1u : 0u);

  /* §4.1, §2.10, §4.6: unknown UNCONDITIONALLY while the going-live flag is defined AND
     unconditionally while the dry latch is set (otherwise water_rules queues doses the board
     will refuse and ack, paging HIGH once per cooldown, forever). Otherwise ok only when the
     gate pitch is calibrated, a home has been seen since boot, and the last expander read
     succeeded. */
#if PB_REPORT_POS_UNKNOWN
  const bool pos_ok = false;
#else
  const bool pos_ok = !safety_dry() && cart_pos_known() && sensors_i2c_healthy();
#endif
  ok = ok && put_s(buf, cap, &n, " pos=%s", pos_ok ? "ok" : "unknown");

  const char *err;
  if (g_ack_set) {
    ok = ok && put_u(buf, cap, &n, " ack=%lu", g_ack.id);
    ok = ok && put_u(buf, cap, &n, " flow_ml=%lu", (uint32_t)g_ack.flow_ml);
    err = g_ack.err;
  } else {
    /* §1: pulses with the pump off raise ch205 and err=leak, and they never block a dose.
       This is that token's ONLY producer in the program: pulses_leak_poll() (driven from
       loop(), task 12 step 4) advances the count, and this line puts it on the wire. Below
       `stuck`, because a mux that is lying about every channel is the larger fact. */
    err = stuck ? "stuck" : (pulses_leak_seen() ? "leak" : safety_last_err());
  }
  if (!err || !*err) err = "none";
  ok = ok && put_s(buf, cap, &n, " err=%s", err);

  ok = ok && put_s(buf, cap, &n, "%s", "\n");
  if (!ok) { ++g_txcap_drops; safety_set_err("txcap"); return 0; }
  /* A truncation set err=txcap, and nothing else in the program ever clears it. One 384-byte
     body would otherwise put txcap on EVERY later report forever, long after the condition
     that caused it. Only this function's own token is cleared, and only by a body that fit —
     the boot tokens (wdt, adc, heap) and the dose tokens are none of this function's business. */
  if (strcmp(safety_last_err(), "txcap") == 0) safety_set_err("none");
  g_last_len = n;
  return n;
}

/* butler's ack UPDATE writes flow_ml = ? unconditionally (butler.py:830), so an ack= without a
   flow_ml= stores NULL, charges the pot the FULL ml against its daily cap (COALESCE(flow_ml, ml),
   :745-751) and skips the 2*flow_ml < ml branch entirely. Structurally one pair, both ways. */
void report_set_ack(uint32_t id, uint16_t flow_ml, const char *err) {
  g_ack.id = id; g_ack.flow_ml = flow_ml; g_ack.err = err;
  g_ack_set = (id != 0);                 /* ack=0 400s the whole report; never emit it */
}
void report_clear_ack(void) { g_ack_set = false; g_ack.id = 0; g_ack.flow_ml = 0; g_ack.err = 0; }
bool report_ack_is_recv(void) {
  return g_ack_set && g_ack.err && strcmp(g_ack.err, "recv") == 0;
}
/* §4.3: no report may be built while a command is pending and the ack slot still reads "recv".
   Without this the placeholder reaches the wire, butler marks the command acked with flow_ml=0,
   pages HIGH, sets the pot's cooldown and charges 0 ml — and THEN the board runs the dose. */
bool report_may_build(void) { return !report_ack_is_recv(); }
