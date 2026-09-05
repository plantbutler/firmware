/* src/cli.cpp -- the console (spec §6). Bench commands here; task 20 adds the
   #if PB_BRINGUP block, and tasks 15/16/19 add dry, stop and the latch release.
   secrets.h is here for PB_CONTROLLER, which `status` prints: neither [env:uno_r4_wifi]
   nor [env:uno_r4_wifi_bringup] passes it in build_flags, and secrets.h is the only
   header that defines it. Task 14 adds cart.h, task 15/16 safety.h, task 24 netfsm.h --
   each at TOP LEVEL, never inside the #if PB_BRINGUP block, because cli_print_status()
   calls into all of them unconditionally. Fix round, task 27: this file used to reach
   past netfsm.h straight into the seam for link/rssi/ip/desync info; it now reads
   netfsm.h's own cached accessors instead and has no seam header of its own to include. */
#include "cart.h"
#include "cli.h"
#include "config.h"
#include "hal.h"
#include "netfsm.h"
#include "noinit.h"
#include "pins.h"
#include "pulses.h"
#include "report.h"
#include "safety.h"
#include "secrets.h"
#include "sensors.h"
#include <stdio.h>
#include <string.h>

#if defined(PB_SIM)
#  define PB_BUILD_NAME "sim"
#elif defined(PB_BRINGUP)
#  define PB_BUILD_NAME "bringup"
#else
#  define PB_BUILD_NAME "bench"
#endif

static char     g_line[PB_LINE_CAP];
static uint16_t g_len;
static bool     g_overlong;
static bool     g_hall_stream;
static uint32_t g_hall_next_ms;
static uint32_t g_arena_min, g_arena_max, g_hwm_max;

void cli_printf_u32(const char *fmt, uint32_t v) {
  char b[PB_LINE_CAP];
  snprintf(b, sizeof b, fmt, (unsigned long)v);
  hal_serial_write(b);
}

void cli_printf_i32(const char *fmt, int32_t v) {
  char b[PB_LINE_CAP];
  snprintf(b, sizeof b, fmt, (long)v);
  hal_serial_write(b);
}

static bool parse_u32_(const char *s, uint32_t *out) {
  uint32_t v = 0;
  if (*s < '0' || *s > '9') return false;
  for (; *s != '\0'; ++s) {
    if (*s < '0' || *s > '9') return false;
    v = v * 10u + (uint32_t)(*s - '0');
  }
  *out = v;
  return true;
}

/* parse_u32_()'s bounded sibling: parse_u32_() requires a NUL terminator, and a
   space-separated argument on a console line ends at a space instead. Digit-only, exactly
   like parse_u32_() -- a strtol here would silently accept `servo 0x600 200`. */
static bool parse_u32_range_(const char *begin, const char *end, uint32_t *out) {
  uint32_t v = 0;
  if (begin >= end || *begin < '0' || *begin > '9') return false;
  for (const char *s = begin; s < end; ++s) {
    if (*s < '0' || *s > '9') return false;
    v = v * 10u + (uint32_t)(*s - '0');
  }
  *out = v;
  return true;
}

/* Pointer to the first space or NUL in s -- the end of one console token. */
static const char *token_end_(const char *s) {
  while (*s != '\0' && *s != ' ') ++s;
  return s;
}

static void note_memory_(void) {
  uint32_t a = hal_heap_arena(), h = hal_stack_hwm();
  if (a < g_arena_min) g_arena_min = a;
  if (a > g_arena_max) g_arena_max = a;
  if (h > g_hwm_max)   g_hwm_max = h;
}

static void cmd_i2c_(void) {                       /* bring-up 1: expect 0x20, 0x27, 0x3C */
  char scan[96];
  sensors_scan(scan, sizeof scan);
  hal_serial_write("i2c: ");
  hal_serial_write(scan);
  hal_serial_write("\n");
}

static void print_mux_(uint8_t ch) {                /* select, >=1 ms, read twice, keep 2nd */
  uint16_t raw = 0;
  char b[48];
  if (sensors_read_raw(ch, &raw))
    snprintf(b, sizeof b, "mux %u = %lu\n", (unsigned)ch, (unsigned long)raw);
  else
    snprintf(b, sizeof b, "mux %u = error\n", (unsigned)ch);   /* never a value on failure */
  hal_serial_write(b);
}

static bool cmd_mux_(const char *arg) {
  if (strcmp(arg, "all") == 0) {
    for (uint8_t ch = 0; ch < 16; ++ch) print_mux_(ch);
    return true;
  }
  uint32_t ch;
  if (!parse_u32_(arg, &ch) || ch > 15u) { hal_serial_write("mux <0-15>|all\n"); return false; }
  print_mux_((uint8_t)ch);
  return true;
}

/* Streams screw / home / float. An I2C error prints `home unknown` -- never `home 0`,
   because a bus error is not the same fact as "not home" (spec §6, §13 step 3). */
static void cmd_hall_line_(void) {
  bool home = false;
  char b[64];
  if (sensors_home_hall(&home))
    snprintf(b, sizeof b, "hall screw=%lu home=%u float=%u\n",
             (unsigned long)pulses_screw(), (unsigned)home,
             (unsigned)(hal_pin_read(PIN_HALL_FLOAT) == PB_LOW));
  else
    snprintf(b, sizeof b, "hall screw=%lu home=unknown float=%u\n",
             (unsigned long)pulses_screw(),
             (unsigned)(hal_pin_read(PIN_HALL_FLOAT) == PB_LOW));
  hal_serial_write(b);
}

static void cmd_flow_(void) {
  char b[64];
  snprintf(b, sizeof b, "flow hz=%lu total=%lu leak=%lu\n",
           (unsigned long)pulses_flow_rate(), (unsigned long)pulses_flow(),
           (unsigned long)pulses_leak_count());
  hal_serial_write(b);
}

static void cmd_help_(void) {
  hal_serial_write(
    "i2c              scan the bus (expect 0x20 0x27 0x3C)\n"
    "mux <0-15>|all   select, settle, read twice, print the second (14-bit raw)\n"
    "hall             stream screw/home/float at 5 Hz; any key stops it\n"
    "flow             pulses/second and total since reset\n"
    "status           everything this board knows about itself\n"
    "help             this\n");
}

void cli_begin(void) {
  g_len = 0; g_overlong = false; g_hall_stream = false;
  g_arena_min = 0xFFFFFFFFu; g_arena_max = 0; g_hwm_max = 0;
  hal_serial_write("type help\n");
}

/* Bytes cli_stop_requested() read from the UART and did NOT consume. cli_poll() drains
   this before it touches the UART, so the two readers cannot lose or reorder a byte
   between them. It is not a queue of commands: the dosing loop clears it at the end of
   every dose (§2.8, §15.3), so impatience typed during a dose is discarded, not
   executed. */
static char     g_push[PB_LINE_CAP];
static uint16_t g_push_len;

static void push_back_(const char *b, size_t n) {
  for (size_t i = 0; i < n && g_push_len < sizeof g_push; ++i) g_push[g_push_len++] = b[i];
}

static size_t read_console_(char *buf, size_t cap) {
  if (g_push_len > 0u) {                       /* pushback FIRST, in arrival order */
    size_t n = g_push_len < cap ? g_push_len : cap;
    for (size_t i = 0; i < n; ++i) buf[i] = g_push[i];
    for (size_t i = n; i < g_push_len; ++i) g_push[i - n] = g_push[i];
    g_push_len = (uint16_t)(g_push_len - n);
    return n;
  }
  return hal_serial_read(buf, cap);
}

/* §2.12. Consumes ONLY an exact `stop\n` or `dry on\n`. Every other byte is pushed back
   unread, so `status` typed at the console is still `status` when cli_poll() reads it.

   §2.12 calls this "a four-byte state"; `dry on` is six bytes, so the partial buffer is
   eight. Four would truncate `dry on` into a permanent non-match: the match state has to
   hold the whole word before it can be recognised, and "stop" plus "dry on" both have to
   fit without a plan review needing to revisit the size a second time. Do not trim this
   back to four for tidiness -- it is settled at eight on purpose. */
#define PB_STOP_MATCH_CAP 8

static char     g_pfx[PB_STOP_MATCH_CAP];
static uint8_t  g_pfx_len;
static bool     g_line_dirty;    /* this line already broke the match: push everything back */
static bool     g_stop_req;

static bool pfx_is_prefix_of_(const char *word) {
  for (uint8_t i = 0; i < g_pfx_len; ++i) if (word[i] == '\0' || word[i] != g_pfx[i]) return false;
  return true;
}

void cli_stop_clear(void) {
  g_stop_req = false;
  g_pfx_len = 0u;
  g_line_dirty = false;
  g_push_len = 0u;      /* forget every byte the console has seen and not acted on */
}

bool cli_stop_requested(void) {
  char b[16];
  size_t n = hal_serial_read(b, sizeof b);
  for (size_t i = 0; i < n; ++i) {
    char c = b[i];
    if (c == '\r') continue;
    if (c == '\n') {
      if (!g_line_dirty && g_pfx_len == 4u && pfx_is_prefix_of_("stop")) {
        g_stop_req = true;
      } else if (!g_line_dirty && g_pfx_len == 6u && pfx_is_prefix_of_("dry on")) {
        safety_dry_set(true);       /* the word means the same during a dose as before one */
        g_stop_req = true;
      } else {
        push_back_(g_pfx, g_pfx_len);
        push_back_("\n", 1u);       /* the line is handed on WHOLE, newline included */
      }
      g_pfx_len = 0u; g_line_dirty = false;
      continue;
    }
    if (g_line_dirty) { push_back_(&c, 1u); continue; }
    if (g_pfx_len < PB_STOP_MATCH_CAP) g_pfx[g_pfx_len++] = c;
    if (!pfx_is_prefix_of_("stop") && !pfx_is_prefix_of_("dry on")) {
      push_back_(g_pfx, g_pfx_len);   /* the whole prefix, in order: `sta` is three bytes */
      g_pfx_len = 0u;
      g_line_dirty = true;            /* and the rest of THIS line is not a match either */
    }
  }
  return g_stop_req;
}

void cli_poll(void) {
  note_memory_();
  if (g_hall_stream && hal_millis() >= g_hall_next_ms) {
    g_hall_next_ms = hal_millis() + 200u;         /* 5 Hz, spec §13 step 3 */
    cmd_hall_line_();
  }
  char buf[32];
  size_t n = read_console_(buf, sizeof buf);
  if (n > 0) g_hall_stream = false;               /* any byte stops the stream */
  for (size_t i = 0; i < n; ++i) {
    char c = buf[i];
    if (c == '\r') continue;
    if (c == '\n') {
      if (g_overlong)   hal_serial_write("line too long, dropped whole\n");
      else if (g_len)   { g_line[g_len] = '\0'; cli_dispatch(g_line); }
      g_len = 0; g_overlong = false;
      continue;
    }
    if (g_overlong) continue;                     /* keep discarding until the newline */
    if (g_len + 1u >= PB_LINE_CAP) { g_overlong = true; g_len = 0; continue; }
    g_line[g_len++] = c;
  }
}

#if PB_BRINGUP
/* cart.h is ALREADY included at the top of this file (task 14 step 10) -- cli_print_status()
   calls the cart in both binaries, so the include cannot live in here. Do not add a second
   one: this block is bringup-only and the cart is not. */

/* WHOLE-TOKEN parse of `pump`'s [prime] [hang] flags, over the text AFTER the ms argument:
   `hanging` is not `hang`, and a substring match would starve the watchdog on a word
   nobody typed. Pure string inspection with no side effect and no call into the dosing
   entry point -- so the literal-token requirement can be proven directly by a host case,
   without ever letting hang=true reach the loop that deliberately starves the dog (a host
   case that did THAT would hang the suite by construction: see safety.cpp's hang hook).
   (Not spelled literally: §9's count of that call in this file must stay exactly one.) */
static void cli_pump_flags_(const char *args, bool *prime, bool *hang) {
  *prime = false; *hang = false;
  for (const char *t = args; *t != '\0'; ) {
    while (*t == ' ') ++t;
    const char *e = t; while (*e != '\0' && *e != ' ') ++e;
    size_t n = (size_t)(e - t);
    if (n == 5u && strncmp(t, "prime", 5) == 0) *prime = true;
    if (n == 4u && strncmp(t, "hang",  4) == 0) *hang  = true;
    t = e;
  }
}

#ifdef PB_NATIVE
/* Host-suite seam. cli_pump_flags_() is static and would otherwise be unreachable from
   test_cli.cpp -- this is a thin, side-effect-free forward so the literal-token case can
   call the REAL parser rather than a second, hand-copied one that could drift from it. */
void cli_pump_flags_for_test_(const char *args, bool *prime, bool *hang) {
  cli_pump_flags_(args, prime, hang);
}
#endif

/* THE ONE CALL SITE OF THE DOSING ENTRY POINT IN THIS FILE -- §9 counts exactly one in
   cli.cpp, and this comment may not spell the token it counts.
   `pump` and `calib` both come through here, and so does the summary line. */
static void cli_run_dose_(uint32_t ms, bool long_prime, bool hang) {
  dose_req_t q = {0};
  q.by_time    = true;
  q.need_pos   = false;                    /* bring-up 4a/5a/5b run before the cart is
                                              calibrated; a pump that demanded a position
                                              would make them unrunnable */
  q.cap_ms     = ms > PB_DOSE_CAP_MS_MAX ? PB_DOSE_CAP_MS_MAX : ms;
  q.long_prime = long_prime;
  q.hang       = hang;
  (void)dose_run(&q);
  cli_print_dose_summary();
}

/* spec §6. Every command here is compiled out of the binary that runs unattended, and
   make check proves it on the PREPROCESSED source of this file (task 30), not on this #if. */
static bool cli_dispatch_bringup_(const char *line) {
  if (strncmp(line, "servo ", 6) == 0) {
    const char *sp = strchr(line + 6, ' ');
    uint32_t us = 0u, ms = 0u;
    if (!sp || !parse_u32_range_(line + 6, sp, &us) || !parse_u32_(sp + 1, &ms) ||
        us < 1000u || us > 2000u || ms == 0u) {
      hal_serial_write("usage: servo <1000-2000> <ms>\n");
      return true;
    }
    if (ms > PB_SERVO_CAP_MS) ms = PB_SERVO_CAP_MS;  /* a typo may not run the screw forever */
    cart_jog((int16_t)us, ms);
    hal_serial_write("servo done\n");
    return true;
  }
  if (strcmp(line, "home") == 0) {
    if (cart_home()) hal_serial_write("home ok\n");                 /* ONE traverse */
    else { hal_serial_write("home FAILED: ");
           hal_serial_write(cart_err()); hal_serial_write("\n"); }
    return true;
  }
  if (strncmp(line, "goto ", 5) == 0) {
    uint32_t o = 0u;
    if (!parse_u32_(line + 5, &o) || o < 1u || o > PB_OUTLETS) {
      hal_serial_write("goto: outlet must be 1..5\n");              /* the range, by name */
      return true;
    }
    if (cart_goto((uint8_t)o)) hal_serial_write("goto ok\n");
    else { hal_serial_write("goto FAILED: ");
           hal_serial_write(cart_err()); hal_serial_write("\n"); }
    return true;
  }

  if (strncmp(line, "pump", 4) == 0) {
    uint32_t ms = 0u;
    const char *arg = line + 4;
    if (*arg != ' ' || !parse_u32_range_(arg + 1, token_end_(arg + 1), &ms) || ms == 0u) {
      hal_serial_write("usage: pump <ms> [prime] [hang]\n");
      return true;
    }
    bool prime = false, hang = false;
    cli_pump_flags_(arg + 1, &prime, &hang);
    cli_run_dose_(ms, prime, hang);
    return true;
  }
  if (strcmp(line, "calib") == 0) { cli_run_dose_(10000u, true, false); return true; }  /* 7b */
  if (strncmp(line, "cal ", 4) == 0) {
    /* `cal 0` - one token on the serial line, or a stray byte parsed as one - used to make
       target = 0 for EVERY subsequent command, so each dose ignored its millilitre target
       and ran the full cap_ms; pulses_to_ml then divided by zero, and the Cortex-M4's UDIV
       returns 0 without DIV_0_TRP - so the flood happened and the report said nothing came
       out. The dosing entry point re-checks the same range as DOSE_REFUSED_CAL (§6). */
    uint32_t v = 0u;
    if (!parse_u32_(line + 4, &v) || v < PB_PULSES_PER_L_MIN || v > PB_PULSES_PER_L_MAX ||
        !cfg_pulses_per_l_set((uint16_t)v)) {
      hal_serial_write("cal: pulses_per_l must be 1000..20000\n");
      return true;
    }
    cli_printf_u32("pulses_per_l=%lu\n", (uint32_t)cfg_pulses_per_l_get());
    return true;
  }
  if (strcmp(line, "noinit pattern") == 0) {        /* bring-up 7c' */
    g_nv.pattern = 0xC0FFEE01u;
    noinit_commit();
    hal_serial_write("noinit pattern written. Now: `pump 3000 hang`, wait for the reset, "
                     "then `status` - the pattern AND the checksum must both survive.\n");
    return true;
  }
  return false;
}
#endif /* PB_BRINGUP */

#if PB_SIM_CLI
#include "sim.h"           /* the injectors AND link_fake_queue_response(): task 21 put the
                              link_fake_* control surface at the end of this header, and
                              there is no include/link_fake.h in this tree */

/* spec §8. Each of these maps to a bring-up step or a finding:
   float 0|1        -> 5a, and dropping it mid-dose is 5b
   flow <ml_s>      -> the prime abort at 0; a mid-dose stop is the stall abort (7b)
   flow storm       -> DOSE_REFUSED_NOISE / DOSE_ABORT_NOISE
   i2c fail|ok      -> position unknown, pump refused, pos=unknown
   mux stuck        -> the canary, err=stuck
   stall on|off     -> goto aborts, position lost
   leak on          -> ch205 rises and err=leak
   wdt stop         -> hal_wdt_alive() false, every dose refused
   wdt slow <hz>    -> delta below PB_WDT_PROBE_MIN_COUNTS
   noinit clobber   -> the checksum fails and it reads as a cold boot
   ch <0-5> <raw>   -> plant a raw count
   resp "<body>"    -> the ack offbeat, on a desk
   reset warm|cold  -> re-enter setup() with .noinit kept or cleared */
static bool cmd_sim_(const char *a) {
  uint32_t v = 0, w = 0;
  if (strcmp(a, "float 0") == 0)  { sim_set_float(false);   return true; }
  if (strcmp(a, "float 1") == 0)  { sim_set_float(true);    return true; }
  if (strcmp(a, "flow storm") == 0) { sim_flow_storm(10000u); return true; }
  if (strncmp(a, "flow ", 5) == 0 && parse_u32_(a + 5, &v) && v <= 65535u) {
    sim_set_flow_ml_s((uint16_t)v); return true;
  }
  if (strcmp(a, "i2c fail") == 0) { sim_set_i2c_fail(true);  return true; }
  if (strcmp(a, "i2c ok") == 0)   { sim_set_i2c_fail(false); return true; }
  if (strcmp(a, "mux stuck") == 0){ sim_set_mux_stuck(true); return true; }
  if (strcmp(a, "stall on") == 0) { sim_set_stall(true);     return true; }
  if (strcmp(a, "stall off") == 0){ sim_set_stall(false);    return true; }
  if (strcmp(a, "leak on") == 0)  { sim_set_leak(true);      return true; }
  if (strcmp(a, "wdt stop") == 0) { sim_wdt_stop();          return true; }
  if (strncmp(a, "wdt slow ", 9) == 0 && parse_u32_(a + 9, &v)) {
    sim_wdt_rate_hz(v); return true;
  }
  if (strcmp(a, "noinit clobber") == 0) { sim_noinit_clobber(); return true; }
  if (strncmp(a, "ch ", 3) == 0) {
    const char *sp = strchr(a + 3, ' ');
    char chbuf[4];
    if (!sp || (size_t)(sp - (a + 3)) >= sizeof chbuf) return false;
    memcpy(chbuf, a + 3, (size_t)(sp - (a + 3)));
    chbuf[sp - (a + 3)] = '\0';
    if (!parse_u32_(chbuf, &v) || v >= PB_CHANNELS) return false;
    if (!parse_u32_(sp + 1, &w) || w > 16383u) return false;
    sim_set_channel((uint8_t)v, (uint16_t)w);
    return true;
  }
  if (strncmp(a, "resp \"", 6) == 0) {
    const char *body = a + 6;
    size_t n = strlen(body);
    if (n == 0 || body[n - 1] != '"') return false;
    link_fake_queue_response(body, n - 1);
    return true;
  }
  if (strcmp(a, "reset warm") == 0) { sim_reset(true);  return true; }
  if (strcmp(a, "reset cold") == 0) { sim_reset(false); return true; }
  hal_serial_write("sim: unknown injector; see spec section 8\n");
  return false;
}
#endif /* PB_SIM_CLI */

bool cli_dispatch(const char *line) {
  if (strcmp(line, "i2c")    == 0) { cmd_i2c_();    return true; }
  if (strncmp(line, "mux ", 4) == 0) return cmd_mux_(line + 4);
  if (strcmp(line, "hall")   == 0) { g_hall_stream = true; g_hall_next_ms = 0; return true; }
  if (strcmp(line, "flow")   == 0) { cmd_flow_();   return true; }
  if (strcmp(line, "status") == 0) { cli_print_status(); return true; }
  if (strcmp(line, "help")   == 0) { cmd_help_();   return true; }
  if (strcmp(line, "dry on")  == 0) {
    safety_dry_set(true);
    hal_serial_write("dry=1 - every dose refused until `dry off`\n");
    return true;
  }
  if (strcmp(line, "dry off") == 0) {
    safety_dry_set(false);
    hal_serial_write("dry=0\n");
    return true;
  }
  if (strcmp(line, "clear contra") == 0) {     /* two literal tokens, no abbreviation */
    if (safety_contra_clear())
      hal_serial_write("contra cleared - the last dose said float OK and the meter saw "
                       "nothing. If you have not found out why, you have not fixed it.\n");
    else
      hal_serial_write("contra=0 already\n");
    return true;
  }
  if (strcmp(line, "stop") == 0) {
    /* A dose in progress never reaches here: the dosing loop blocks and matches this
       word byte-wise itself (§2.12). This arm is the idle console's answer, and it exists
       so that `stop` is never `? unknown` -- it is the one command an operator reaches
       for in an emergency. It also clears a stale request left by a matched-but-
       unconsumed word, so the NEXT dose is not aborted by a stop typed before it. */
    cli_stop_clear();
    hal_serial_write("stop: no dose running\n");
    return true;
  }
#if PB_BRINGUP
  if (cli_dispatch_bringup_(line)) return true;
#endif
#if PB_SIM_CLI
  if (strncmp(line, "sim ", 4) == 0) return cmd_sim_(line + 4);
#endif
  hal_serial_write("? unknown; type help\n");
  return false;
}

/* Printed at the end of every dose, from every path (spec §6's pitch deliverable). Outside
   #if PB_BRINGUP: exec.cpp (task 26) calls this same function for the backend's doses in
   the bench build, so the printer itself ships in both binaries even though only the
   bring-up console's pump/calib reach it today. */
void cli_print_dose_summary(void) {
  static char line[PB_LINE_CAP];
  uint32_t ms  = dose_last_ms();
  uint32_t ml  = dose_flow_ml();
  uint32_t t10 = ms ? (ml * 10000u) / ms : 0u;      /* ml/s x 10, in integer tenths */
  /* §6 prints r=ok for a successful dose while err_of(DOSE_OK) is the wire's "none".
     One conditional, here: `ok` must NOT be added to err_of()'s enum, which is tested
     against butler.py's own parser. */
  const char *r = (dose_last_result() == DOSE_OK) ? "ok" : err_of(dose_last_result());
  snprintf(line, sizeof line,
           "dose outlet=%lu ms=%lu pulses=%lu ml=%lu mls=%lu.%lu r=%s\n",
           (unsigned long)dose_last_outlet(), (unsigned long)ms,
           (unsigned long)dose_last_pulses(), (unsigned long)ml,
           (unsigned long)(t10 / 10u), (unsigned long)(t10 % 10u), r);
  hal_serial_write(line);
}

void cli_print_status(void) {
  char b[160];
  note_memory_();

  snprintf(b, sizeof b, "build=%s controller=%u\n", PB_BUILD_NAME,
           (unsigned)PB_CONTROLLER);
  hal_serial_write(b);

#ifdef PB_RELAY_ACTIVE_LOW
  const char *pol = "ACTIVE_LOW";
#else
  const char *pol = "ACTIVE_HIGH";
#endif
  snprintf(b, sizeof b, "pump_on_level=%u polarity=%s\n",
           (unsigned)hal_pump_level_on(), pol);
  hal_serial_write(b);

  /* The citation is spelled out by section TITLE, not number: "2.5)" is a digit, a dot
     and a digit, which is exactly the shape test_cli.cpp's float-formatting guard scans
     every printed line for (spec §12 item 1's rule against floating-point format
     specifiers anywhere in this program) - a numeric subsection here would make this
     line indistinguishable from a stray float and fail the very check it exists to
     satisfy. */
  /* hal_wdt_alive() and hal_wdt_last_delta() may NOT be passed inline as two arguments
     of the same call: C++ leaves function-argument evaluation order unspecified, and
     hal_wdt_alive() is destructive (it probes, then caches the delta hal_wdt_last_delta()
     reads back) -- on this toolchain gcc evaluates arguments right-to-left on ARM, which
     would read the delta BEFORE the probe that is supposed to have just produced it, so
     alive= and delta= would describe two different probes on the same printed line. Call
     hal_wdt_alive() first, into a local, THEN read the delta it just cached. */
  const bool     alive = hal_wdt_alive();      /* destructive: probes, then caches the delta */
  const uint32_t delta = hal_wdt_last_delta();
  snprintf(b, sizeof b,
           "wdt=%s granted=%lums alive=%s delta=%lu "
           "(WDT, not IWDT - DECISIONS #10 says IWDT; see design spec "
           "section \"The watchdog\")\n",
           hal_wdt_granted() ? "on" : "off", (unsigned long)hal_wdt_granted(),
           alive ? "yes" : "no", (unsigned long)delta);
  hal_serial_write(b);

  snprintf(b, sizeof b, "icufilter=%s %s irq_armed=%s %s\n",
           hal_irq_filtered(PIN_FLOW) ? "yes" : "no",
           hal_irq_filtered(PIN_HALL_SCREW) ? "yes" : "no",
           hal_irq_armed(PIN_FLOW) ? "yes" : "no",
           hal_irq_armed(PIN_HALL_SCREW) ? "yes" : "no");
  hal_serial_write(b);

  cli_printf_u32("uptime=%lus\n", hal_millis() / 1000u);

  /* spec §6's two widths, both real: adc_req is what we asked the core to map to, adc_hw
     is the fixed hardware width read back through seam 1 (task 3's hal_adc_bits()). A
     mismatch silently rescales every raw count on the wire, so main.cpp latches err=adc
     and disables the network for it (task 12). */
  snprintf(b, sizeof b, "adc_req=%lu adc_hw=%lu adc_ok=%s\n",
           (unsigned long)PB_ADC_BITS, (unsigned long)hal_adc_bits(),
           hal_adc_width_ok() ? "yes" : "no");
  hal_serial_write(b);

  snprintf(b, sizeof b, "screw=%lu flow_total=%lu flow_hz=%lu leak=%lu\n",
           (unsigned long)pulses_screw(), (unsigned long)pulses_flow(),
           (unsigned long)pulses_flow_rate(), (unsigned long)pulses_leak_count());
  hal_serial_write(b);

  snprintf(b, sizeof b, "i2c errors=%lu txn_per_min=%lu healthy=%s\n",
           (unsigned long)sensors_i2c_errors(), (unsigned long)sensors_i2c_txn_per_min(),
           sensors_i2c_healthy() ? "yes" : "no");
  hal_serial_write(b);

#if PB_PULSES_PER_GATE == 0
  hal_serial_write("cart=UNCALIBRATED (PB_PULSES_PER_GATE=0) - goto refuses, pos never ok\n");
#else
  cli_printf_u32("cart pulses_per_gate=%lu\n", (uint32_t)PB_PULSES_PER_GATE);
#endif
#ifdef PB_ALLOW_UNCALIBRATED
  hal_serial_write("cart=UNCALIBRATED BUILD (PB_ALLOW_UNCALIBRATED set; bring-up 6 removes it)\n");
#endif
  snprintf(b, sizeof b, "cart pos=%s%u pulses=%lu parked=%u busy=%u err=%s\n",
           cart_pos_known() ? "" : "?", (unsigned)cart_pos(),
           (unsigned long)cart_pulses(), (unsigned)cart_parked(),
           (unsigned)cart_busy(), cart_err());
  hal_serial_write(b);
#if PB_REPORT_POS_UNKNOWN
  hal_serial_write("pos: FORCED unknown (PB_REPORT_POS_UNKNOWN=1)\n");
#endif
  cli_printf_u32("parked=%lu\n", (uint32_t)(cart_parked() ? 1u : 0u));
  /* §2.12: the dosing loop blocks and net_poll() cannot run while it does, and enqueue() returns
     409 while the water command it would abort is still 'sent'. Say so, so nobody reaches for
     it in an emergency. The live aborts are the console `stop`, `dry on`, the float, the two
     flow rules, the plausibility ceiling, the cap and the watchdog. */
  hal_serial_write("note: a backend stop=1 CANNOT interrupt a running dose; type `stop`\n");

  /* §4.2 requires the drop to be loud: a truncated body is a DROPPED report, not a 400, and
     the console is the only place that failure is otherwise visible at all. */
  cli_printf_u32("report: last_body=%lu bytes\n", (uint32_t)report_last_len());
  cli_printf_u32("report: cap=%lu bytes\n", (uint32_t)PB_BODY_CAP);
  cli_printf_u32("report: DROPPED on truncation (err=txcap) x%lu\n", report_txcap_drops());

  /* A rebuilt or restored backend database restarts commands.id at 1, and the board then
     refuses EVERY command as a replay until a COLD boot (power cycle, not RESET). Without this
     line that is silent and unexplainable. Spec §4.3, §16.5.9. */
  cli_printf_u32("cmd_high_water=%lu (recovery: cold boot)\n", g_nv.cmd_high_water);

  cli_printf_u32("link=%lu\n", (uint32_t)net_link());         /* 0 down, 1 joining, 2 up */
  cli_printf_i32("rssi=%ld dBm\n", (int32_t)net_rssi());      /* task 11's signed printer */
  hal_serial_write("ip="); hal_serial_write(net_ip()); hal_serial_write("\n");
  cli_printf_u32("http_last=%lu\n", (uint32_t)net_last_status());
  cli_printf_u32("reports_ok=%lu\n", net_reports_ok());
  cli_printf_u32("reports_failed=%lu\n", net_reports_failed());
  cli_printf_u32("modem_ran=%lu\n", (uint32_t)(net_modem_ran_this_pass() ? 1u : 0u));
  /* The connect form is printed rather than asserted: this package proves only the command
     SELECTION, and whether _CLIENTCONNECT still resolves HOST_NAME as a hostname is a bring-up
     question task 27's real driver settles, not something a host test can check (spec §3
     change 4). modem_timeout_ms/conn_timeout_ms are both PB_NET_STEP_MS because a single
     modem.timeout() call sets the ONE budget every AT round trip in this file shares -- there
     is no separate connect-specific timeout to print a different number for. */
  cli_printf_u32("modem_timeout_ms=%lu\n", (uint32_t)PB_NET_STEP_MS);
  cli_printf_u32("conn_timeout_ms=%lu\n", (uint32_t)PB_NET_STEP_MS);
  hal_serial_write("connect_form=_CLIENTCONNECT to HOST_NAME as a NAME"
                   " (setConnectionTimeout != 0; unit unverified off-bench)\n");
  cli_printf_u32("desyncs=%lu\n", (uint32_t)net_desyncs());   /* rides out as ch206 */
  if (net_disabled()) { hal_serial_write("net=DISABLED ("); hal_serial_write(net_disabled());
                        hal_serial_write(")\n"); }

  snprintf(b, sizeof b,
           "arena=%lu (min %lu max %lu) ordblks=%lu break=0x%lx stack_hwm=%lu (max %lu) "
           "headroom=%lu\n",
           (unsigned long)hal_heap_arena(), (unsigned long)g_arena_min,
           (unsigned long)g_arena_max, (unsigned long)hal_heap_ordblks(),
           (unsigned long)hal_heap_break(), (unsigned long)hal_stack_hwm(),
           (unsigned long)g_hwm_max,
           (unsigned long)(hal_stack_limit() - hal_heap_break()));
  hal_serial_write(b);

  /* §2.11: the boot banner and `status` both print dry=. This is safety_dry()'s verdict,
     not a re-read of g_nv.dry_latched -- the two agree today, but this line is what the
     operator and bring-up 6/7c actually read, so it goes through the same accessor the
     dosing entry point's own ladder does. (Not spelled literally: §9's count of that call
     in this file must stay exactly one.) */
  cli_printf_u32("dry=%lu\n", (uint32_t)(safety_dry() ? 1u : 0u));

  /* §2.7. The loudest fact this board can report about itself: two independent sensors
     disagree and the rig has refused to water since. `clear contra` is the only way back. */
  if (safety_contra())
    hal_serial_write("contra=1 *** CONTRADICTION LATCHED - float said OK, meter saw "
                     "nothing. `clear contra` to release.\n");
  else
    hal_serial_write("contra=0\n");

  /* The raw .noinit struct: bring-up 7c' reads exactly this line after a forced reset.
     cold= and resetmid= are noinit.cpp's two accessors (task 4) and this is their only
     consumer -- without them the struct's numbers are readable but the VERDICT the boot
     drew from them is not, and 7c's pass criterion is the verdict. */
  snprintf(b, sizeof b,
           "nv magic=0x%lx boots=%lu chw=%lu dry=%u contra=%u inflight=%u "
           "pattern=0x%lx sum=0x%lx cold=%u resetmid=%u\n",
           (unsigned long)g_nv.magic, (unsigned long)g_nv.boots,
           (unsigned long)g_nv.cmd_high_water, (unsigned)g_nv.dry_latched,
           (unsigned)g_nv.contra_latched, (unsigned)g_nv.dose_in_flight,
           (unsigned long)g_nv.pattern, (unsigned long)g_nv.sum,
           (unsigned)noinit_was_cold(), (unsigned)noinit_reset_mid());
  hal_serial_write(b);

  cli_printf_u32("pulses_per_l=%lu\n", (uint32_t)cfg_pulses_per_l_get());
  cli_printf_u32("prime_ms=%lu\n",     (uint32_t)PB_PRIME_MS_DEFAULT);
  cli_printf_u32("stall_ms=%lu\n",     (uint32_t)PB_STALL_MS_DEFAULT);
#if PB_ML_PER_S_MEASURED > 0
  cli_printf_u32("cap=clamped to 2x the requested ml at %lu ml/s\n",
                 (uint32_t)PB_ML_PER_S_MEASURED);
#else
  hal_serial_write("cap=UNCLAMPED (PB_ML_PER_S_MEASURED=0; bring-up 7b commits it)\n");
#endif
  /* No numeric section citation on this line: test_no_float_formatting_appears_in_any_
     printed_line scans every printed line for a bare digit-dot-digit run, the shape of a
     stray float specifier, and "2.12)" is exactly that shape -- the same trap the wdt line
     above already dodges by spelling its own citation out as a section TITLE. */
  hal_serial_write("stop: `stop` and `dry on` abort a running dose; a backend stop=1 CANNOT "
                   "- net_poll() does not run while the pump is asserted (design spec "
                   "section \"What a backend stop=1 can and cannot do\")\n");
  hal_serial_write("last=");
  hal_serial_write(safety_last_err());     /* one bare token of 4.1's fixed enum */
  hal_serial_write("\n");
}
