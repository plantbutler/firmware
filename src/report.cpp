/* report.cpp: the wire protocol, pure -- the k=v body and the response parser.
   No signed conversion anywhere: t = hal_boot_salt() + hal_millis() is above 2^31 on
   ordinary boots, so a signed print gives a leading '-' and every report 400s. */
#include "report.h"
#include "config.h"
#include "hal.h"
#include "netfsm.h"
#include "sensors.h"
#include "pulses.h"
#include "safety.h"
#include "cart.h"
#include "noinit.h"
#include "secrets.h"   /* PB_CONTROLLER: build_flags in the host and sim environments, this header on the device ones; its #ifndef guard makes both agree */
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

/* ch200..ch211, every one clamped: chN must be below 2^31 on the wire, and a storming D2
   pushes ch205 past it in ~12.4 days. At least one is always present, so a wedged bus
   produces an alarm instead of silence. A function of the array, not the producers, so
   the host can prove the clamp for every index. */
bool report_put_diags(char *b, uint16_t cap, uint16_t *n, const uint32_t *diag) {
  for (uint32_t i = 0; i < (uint32_t)PB_DIAG_CHANNELS; ++i) {
    const uint32_t v = diag[i] > (uint32_t)PB_DIAG_CLAMP ? (uint32_t)PB_DIAG_CLAMP : diag[i];
    if (!put_ch(b, cap, n, 200u + i, v)) return false;
  }
  return true;
}

/* The only heap bound that exists: _sbrk is the unchecked libnosys version and nothing
   references __HeapLimit. Checked at boot and on every report, when the network stack --
   the largest allocator -- is active. Latches err=heap so the fact reaches the wire; the
   net-disable half lives in netfsm.cpp, which not every host suite links. */
bool report_heap_ok(void) {
  if (hal_heap_break() < hal_stack_limit() - (uint32_t)PB_STACK_MARGIN) return true;
  safety_set_err("heap");
  return false;
}

uint16_t report_build(char *buf, uint16_t cap) {
  if (!report_may_build()) return 0;    /* err=recv must never reach the wire */
  uint16_t n = 0;
  bool ok = true;
  (void)report_heap_ok();      /* does not abort the body: a report that says err=heap is
                                  worth more than no report */
  const bool stuck = sensors_stuck();

  ok = ok && put_u(buf, cap, &n, "c=%lu", (uint32_t)PB_CONTROLLER);
  ok = ok && put_u(buf, cap, &n, " t=%lu", g_t_wire);

  if (!stuck) {                       /* a stuck mux omits the wired channels, not the body */
    for (uint8_t ch = 0; ch < PB_CHANNELS; ++ch)
      if (sensors_valid(ch))
        ok = ok && put_ch(buf, cap, &n, ch, sensors_value(ch));
  }

  /* The twelve producers, in channel order. ch207 (contra) and ch210 (the float flap) say
     WHY float= is 0; ch211 (the dry latch) is no float= term at all: it forces pos=unknown
     below. The count is pinned because the body's worst-case sum is done at it. */
  const uint32_t diag[] = {
    hal_heap_arena(), hal_heap_ordblks(), hal_stack_hwm(),
    sensors_i2c_errors(), sensors_float_change_age_s(), pulses_leak_count(),
    (uint32_t)net_desyncs(), safety_contra() ? 1u : 0u,
    cart_parked() ? 1u : 0u, hal_wdt_last_delta(),
    safety_float_flap() ? 1u : 0u, safety_dry() ? 1u : 0u
  };
  static_assert(sizeof diag / sizeof diag[0] == PB_DIAG_CHANNELS,
                "the diag array and PB_DIAG_CHANNELS disagree: re-do PB_BODY_WORST_SUM");
  ok = ok && report_put_diags(buf, cap, &n, diag);

  /* The debounced tank verdict, ANDed with !contra, forced to 0 after PB_FLOAT_FLAP_LIMIT
     consecutive float refusals. Never 2, never negative: butler accepts 0 or 1 only. */
  const bool fl = safety_float_ok_debounced() && !safety_contra() && !safety_float_flap();
  ok = ok && put_u(buf, cap, &n, " float=%lu", fl ? 1u : 0u);

  /* Unknown unconditionally while the going-live flag is defined and while the dry latch
     stands (otherwise the backend queues doses the board refuses and acks, paging once per
     cooldown, forever). Otherwise ok only with a calibrated cart, a home seen since boot
     and a healthy expander. */
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
    /* Pulses with the pump off raise ch205 and err=leak and never block a dose; this is
       the token's only producer. Below `stuck`: a mux lying about every channel is the
       larger fact. */
    err = stuck ? "stuck" : (pulses_leak_seen() ? "leak" : safety_last_err());
  }
  if (!err || !*err) err = "none";
  ok = ok && put_s(buf, cap, &n, " err=%s", err);

  ok = ok && put_s(buf, cap, &n, "%s", "\n");
  if (!ok) { ++g_txcap_drops; safety_set_err("txcap"); return 0; }
  /* Nothing else clears err=txcap, so one oversized body would otherwise mark every later
     report forever. Only this function's own token, and only once a body fit: the boot
     and dose tokens are not its business. */
  if (strcmp(safety_last_err(), "txcap") == 0) safety_set_err("none");
  g_last_len = n;
  return n;
}

/* butler's ack UPDATE writes flow_ml unconditionally: an ack= without flow_ml= stores NULL,
   charges the pot the full ml against its daily cap and skips the short-dose branch. */
void report_set_ack(uint32_t id, uint16_t flow_ml, const char *err) {
  g_ack.id = id; g_ack.flow_ml = flow_ml; g_ack.err = err;
  g_ack_set = (id != 0);                 /* ack=0 400s the whole report; never emit it */
}
void report_clear_ack(void) { g_ack_set = false; g_ack.id = 0; g_ack.flow_ml = 0; g_ack.err = 0; }
bool report_ack_is_recv(void) {
  return g_ack_set && g_ack.err && strcmp(g_ack.err, "recv") == 0;
}
/* No report while a command is pending and the ack slot reads "recv": the placeholder on
   the wire would mark the command acked with flow_ml=0, page, set the cooldown, charge
   0 ml -- and THEN the board would run the dose. */
bool report_may_build(void) { return !report_ack_is_recv(); }

/* ---- response parsing: where a fault becomes water ---- */

/* One unsigned field out of a k=v token. false == absent, non-numeric, or overlong.
   ASCII digits only, like butler's own parser. The overflow check is exact per digit:
   v > (UINT32_MAX - d) / 10 is precisely "would v*10+d overflow", so every uint32_t is
   accepted and nothing wraps -- a fixed-threshold guard is one digit short and would turn
   "water=4294967297" into an accepted outlet=1. */
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

  /* Every line must be newline-terminated inside len: a body truncated mid-token must
     never water. */
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
    /* Replay guard: a response body left over from an earlier round trip -- what a poisoned
       AT session produces -- would run the same command a second time; the second ack lands
       on a row no longer 'sent', so the backend's UPDATE is a silent no-op and the plant
       gets double the water with no alert. */
    if (v <= g_nv.cmd_high_water) continue;
    const uint32_t id = v;

    uint32_t stop = 0;
    if (field_u32(line, llen, "stop=", &stop) && stop != 0) {
      out->cmd.id = id; out->cmd.kind = CMD_STOP;
    } else {
      /* Sentinels above every field's maximum, not indeterminate locals: field_u32 writes
         *out only on success, so an absent field fails the width check on its own even if
         a `continue` above it is ever dropped. */
      uint32_t outlet = 0xFFFFFFFFu, ml = 0xFFFFFFFFu, cap_s = 0xFFFFFFFFu;
      if (!field_u32(line, llen, "water=", &outlet)) continue;
      if (!field_u32(line, llen, "ml=", &ml))       continue;   /* no ml= is no command */
      if (!field_u32(line, llen, "cap_s=", &cap_s)) continue;   /* an absent cap is unbounded */
      if (ml == 0) continue;
      if (outlet > 255u || ml > 65535u || cap_s > 65535u) continue;
      /* An outlet outside 1..PB_OUTLETS, water=0 included, is accepted here and refused
         with err=range by exec_pending(), so the backend learns the real reason. cap_s
         rides through unclamped: the firmware's own ceiling is the dosing entry point's
         job. Parse into a struct, decide nothing. */
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
