/* src/report.cpp — the wire protocol, pure. Where the traps live (spec §4).
   NO signed integer conversion anywhere: t = hal_boot_salt() + hal_millis() is above 2^31 on
   ordinary boots, so a single percent-d against a uint32_t prints a leading '-', _int_in
   rejects it, and EVERY report 400s from the first one (spec §15.2, §9's grep). Every numeric
   site is %lu with an explicit cast. */
#include "report.h"
#include "config.h"
#include "hal.h"
#include "netfsm.h"    /* net_desyncs(): fix round, task 27 -- the seam's own desync counter
                          is read through netfsm.cpp's cache now, same as every other module
                          outside netfsm.cpp and its driver. */
#include "sensors.h"
#include "pulses.h"
#include "safety.h"
#include "cart.h"
#include "noinit.h"    /* g_nv.cmd_high_water: the replay guard, §4.3 */
#include "secrets.h"   /* PB_CONTROLLER: the two static_asserts below and the c= field.
                          An INTEGER since 2026-09-05, so `c=` is the same shape as every
                          other identifier on the wire and a typo cannot open a second
                          garden. Board 0 is a real board, which is why the assert below
                          is a range and not a "not empty".
                          [env:native] and [env:uno_r4_wifi_sim] pass it in build_flags;
                          the two device envs do not, and this header is its only other
                          definition. Its #ifndef guard makes both routes agree. */
#include <stdio.h>
#include <string.h>

static_assert(PB_CONTROLLER_WIRE + 2 + PB_BODY_WORST_FIXED <= PB_BODY_CAP,
              "the body's own worst case does not fit PB_BODY_CAP (spec §7's term-by-term sum)");
static_assert(PB_CONTROLLER >= 0 && PB_CONTROLLER <= 255,
              "c= is 0..255 on the wire: butler.py refuses anything else (MAX_CONTROLLER)");

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

  ok = ok && put_u(buf, cap, &n, "c=%lu", (uint32_t)PB_CONTROLLER);
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
    (uint32_t)net_desyncs(), safety_contra() ? 1u : 0u,
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

/* ---- response parsing — where a fault becomes water (spec §4.5) ---- */

/* One unsigned field out of a k=v token. false == absent, non-numeric, or overlong.
   ASCII digits only, like butler's own _int_in (butler.py:192-201).

   The per-digit overflow check is EXACT, not a rounded-down constant: v > (UINT32_MAX - d) / 10
   is the precise "would v*10+d overflow" test for the digit about to be consumed, so it accepts
   every representable uint32_t (up to and including 4294967295) and rejects every string that
   would not fit — never silently wraps. A cruder single-threshold guard (reject once v exceeds
   some fixed constant, then multiply unconditionally) is one edge case short of that: at the
   boundary itself the very next digit can still carry v past UINT32_MAX and wrap modulo 2^32,
   turning e.g. "water=4294967297" into a silently-accepted outlet=1 instead of a rejected field
   — exactly the "partial" acceptance rule 1 above exists to forbid. */
static bool field_u32(const char *line, uint16_t len, const char *key, uint32_t *out) {
  const size_t kl = strlen(key);
  for (uint16_t i = 0; i < len; ++i) {
    if (i && line[i - 1] != ' ') continue;
    if (len - i < kl || memcmp(line + i, key, kl) != 0) continue;
    uint16_t j = (uint16_t)(i + kl);
    if (j >= len || line[j] < '0' || line[j] > '9') return false;
    uint32_t v = 0;
    for (; j < len && line[j] >= '0' && line[j] <= '9'; ++j) {
      const uint32_t d = (uint32_t)(line[j] - '0');
      if (v > (0xFFFFFFFFu - d) / 10u) return false;   /* would overflow: reject, never wrap */
      v = v * 10u + d;
    }
    if (j < len && line[j] != ' ') return false;     /* a trailing non-digit is not a number */
    *out = v;
    return true;
  }
  return false;
}

bool response_parse(const char *body, uint16_t len, response_t *out) {
  out->next_s = 0;                        /* 0 == keep the previous interval */
  out->cmd.id = 0; out->cmd.kind = CMD_NONE;
  out->cmd.outlet = 0; out->cmd.ml = 0; out->cmd.cap_s = 0;
  if (!body || len == 0) return false;

  /* Every line must be newline-TERMINATED inside len. A body truncated mid-token must never
     water: a half-read reply is not a command (§4.5). */
  uint16_t pos = 0;
  while (pos < len) {
    const char *nl = (const char *)memchr(body + pos, '\n', (size_t)(len - pos));
    if (!nl) break;                       /* trailing partial line: ignored, never parsed */
    const char *line = body + pos;
    const uint16_t llen = (uint16_t)(nl - line);
    pos = (uint16_t)(nl - body + 1);

    uint32_t v;
    if (field_u32(line, llen, "next=", &v) && v >= 5u && v <= 3600u)
      out->next_s = (uint16_t)v;          /* outside [5,3600]: keep the previous interval */

    if (!field_u32(line, llen, "cmd=", &v)) continue;
    if (v == 0) continue;                 /* ack must be >= 1 or the whole report 400s */
    /* Replay guard (§4.3): a response body left over from an earlier round trip — exactly what a
       poisoned AT session produces — would otherwise run cmd=17 a SECOND time; the second ack
       lands on a row no longer state='sent', so the UPDATE is a silent no-op, the cooldown and
       daily cap never see it, and the plant gets double the water with no alert. */
    if (v <= g_nv.cmd_high_water) continue;
    const uint32_t id = v;

    uint32_t stop = 0;
    if (field_u32(line, llen, "stop=", &stop) && stop != 0) {
      out->cmd.id = id; out->cmd.kind = CMD_STOP;
    } else {
      /* Sentinel-initialised, not left indeterminate: field_u32 only writes *out on success, so
         an absent field must fail the width check three lines down on its own, EVEN IF a future
         edit drops the `continue` that is supposed to catch it here. An uninitialised local
         would make that failure mode depend on whatever the stack happened to hold — caught in
         this task's own mutation sweep, where deleting the ml= guard alone slipped through
         because the leftover stack value happened to still land outside [1,65535]. A sentinel
         above every field's own maximum removes the coin flip: the guards below stay the fast,
         readable path, and the width check is the backstop that holds even if they are gone. */
      uint32_t outlet = 0xFFFFFFFFu, ml = 0xFFFFFFFFu, cap_s = 0xFFFFFFFFu;
      if (!field_u32(line, llen, "water=", &outlet)) continue;
      if (!field_u32(line, llen, "ml=", &ml))       continue;   /* no ml= is no command */
      if (!field_u32(line, llen, "cap_s=", &cap_s)) continue;   /* an absent cap is unbounded */
      if (ml == 0) continue;
      if (outlet > 255u || ml > 65535u || cap_s > 65535u) continue;
      /* An outlet outside 1..PB_OUTLETS — water=0 included — IS accepted here and refused with
         err=range by exec_pending(), above cart_goto(), so the backend learns the real reason.
         cap_s rides through UNCLAMPED: the firmware's own ceiling (PB_DOSE_CAP_MS_MAX) is
         dose_run()'s job (safety.cpp:222), not this parser's — a hostile or buggy cap_s widens
         nothing, because nothing here ever narrows it either. Parse into a struct, decide
         nothing (spec §4.5, this task's requirement 4). */
      out->cmd.id = id; out->cmd.kind = CMD_WATER;
      out->cmd.outlet = (uint8_t)outlet;
      out->cmd.ml = (uint16_t)ml;
      out->cmd.cap_s = (uint16_t)cap_s;
    }
    g_nv.cmd_high_water = id;             /* bumped the moment a command is ACCEPTED */
    noinit_commit();                      /* .noinit, so a warm reset does not reopen the window */
    return true;
  }
  return false;
}
