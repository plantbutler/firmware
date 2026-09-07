/* test_cli.cpp: the console line reader and commands, and the two screen renderers, on the host. */
#include "../support/bodies.h"
#include "../support/harness.h"
#include "cart.h"
#include "cli.h"
#include "config.h"
#include "hal.h"
#include "noinit.h"
#include "pins.h"
#include "pulses.h"
#include "safety.h"
#include "sim.h"
#include "ui.h"
#include <stdlib.h>
#include <string.h>
#include <unity.h>

void setUp(void)    { pb_test_setup(); }
void tearDown(void) { pb_test_teardown(); }

static ui_state_t base_state(void) {
  ui_state_t s;
  memset(&s, 0, sizeof s);
  strcpy(s.build, "bench");
  /* the controller as ui_fill_() prints it: PB_CONTROLLER, an integer */
  strcpy(s.controller, "0");
  strcpy(s.ip, "192.168.1.42");
  s.uptime_min = 83; s.pos_known = false; s.screw_pulses = 1290;
  s.float_ok = true; s.pump_on = false; s.parked = true;
  s.flow_hz = 0; s.flow_total = 5881;
  s.link = 2; s.rssi = -52; s.http_status = 200; s.next_s = 60;
  s.cmd_id = 17; s.cmd_text = "ok 248ml";
  s.lcd_state = "IDLE"; s.lcd_detail = "next 35s";
  return s;
}

static void test_ui_render_fills_eight_rows_of_sixteen_characters(void) {
  ui_state_t s = base_state();
  char rows[8][17];
  memset(rows, 'X', sizeof rows);
  ui_render(&s, rows);
  for (int r = 0; r < 8; ++r) {
    TEST_ASSERT_EQUAL_CHAR('\0', rows[r][16]);
    TEST_ASSERT_EQUAL_UINT(16, strlen(rows[r]));    /* padded, so no stale glyphs remain */
  }
  TEST_ASSERT_EQUAL_STRING("PB 0  1h23m     ", rows[0]);
}

static void test_ui_render_lcd_shows_the_contradiction_banner(void) {
  ui_state_t s = base_state();
  s.contra = true;
  s.lcd_state = "CONTRA LATCH";
  s.lcd_detail = "float ok,no flow";
  char rows[2][17];
  ui_render_lcd(&s, rows);
  TEST_ASSERT_EQUAL_STRING("CONTRA LATCH    ", rows[0]);
  TEST_ASSERT_EQUAL_STRING("float ok,no flow", rows[1]);

  /* the banner overrides the caller's rows, and outranks an HTTP status */
  s.contra = true; s.lcd_state = "IDLE"; s.lcd_detail = "next 35s"; s.http_status = 400;
  ui_render_lcd(&s, rows);
  TEST_ASSERT_EQUAL_STRING("CONTRA LATCH    ", rows[0]);
  TEST_ASSERT_EQUAL_STRING("float ok,no flow", rows[1]);
  s.sim = true;
  ui_render_lcd(&s, rows);
  TEST_ASSERT_EQUAL_STRING("*** SIM NO D6 **", rows[0]);   /* and SIM outranks the latch */
}

static void test_ui_render_lcd_prose_is_never_the_wire_error_token(void) {
  /* the wire's fixed err= tokens; row 1 is prose and must never be one of them */
  static const char *const tokens[] = {
    "none", "float", "pos", "noflow", "noise", "cap", "stop", "wdt", "dry", "contra",
    "boot", "range", "cal", "i2c", "busy", "cooldown", "leak", "adc", "stuck", "txcap",
    "resetmid", "heap", "goto", "recv"
  };
  static const char *const details[] = {
    "float NOT OK", "float ok,no flow", "HTTP 400", "next 35s", "p 1290/1450"
  };
  char rows[2][17];
  for (unsigned d = 0; d < sizeof details / sizeof details[0]; ++d) {
    ui_state_t s = base_state();
    s.lcd_detail = details[d];
    ui_render_lcd(&s, rows);
    char trimmed[17];
    strcpy(trimmed, rows[1]);
    for (int i = 15; i >= 0 && trimmed[i] == ' '; --i) trimmed[i] = '\0';
    for (unsigned t = 0; t < sizeof tokens / sizeof tokens[0]; ++t)
      TEST_ASSERT_NOT_EQUAL(0, strcmp(trimmed, tokens[t]));
  }
}

/* A 400/401 loop is invisible to anyone not on the serial port unless the LCD shows it. The
   renderer decides, not the caller: ui_fill_() picks lcd_detail for a dozen other reasons. */
static void test_ui_render_lcd_shows_the_last_http_status_on_a_four_hundred(void) {
  ui_state_t s = base_state();
  s.http_status = 400;
  s.lcd_detail = "next 35s";
  char rows[2][17];
  ui_render_lcd(&s, rows);
  TEST_ASSERT_EQUAL_STRING("HTTP 400        ", rows[1]);
  s.http_status = 200;                           /* a healthy exchange leaves the prose */
  ui_render_lcd(&s, rows);
  TEST_ASSERT_EQUAL_STRING("next 35s        ", rows[1]);
  s.http_status = 0;                             /* and so does "nothing has happened yet" */
  ui_render_lcd(&s, rows);
  TEST_ASSERT_EQUAL_STRING("next 35s        ", rows[1]);
}

static void test_ui_poll_is_a_noop_while_the_pump_is_asserted(void) {
  ui_state_t s = base_state();
  s.pump_on = false;
  ui_poll(&s);                                   /* first paint: every row changes */
  uint16_t after_first = ui_paints_for_test();
  TEST_ASSERT_TRUE(after_first > 0);

  s.pump_on = true;
  s.uptime_min = 99; s.flow_total = 6000;        /* plenty changed... */
  ui_poll(&s);
  TEST_ASSERT_EQUAL_UINT16(after_first, ui_paints_for_test());   /* ...and nothing painted */
}

/* A pass that issued a modem command has spent up to 2.4 s of a 5592 ms grant, and one wedged
   LCD row can cost 102 s. net_poll() calls ui_modem_ran(); this keeps that call alive. */
static void test_ui_poll_is_a_noop_in_a_pass_where_a_modem_command_ran(void) {
  ui_state_t s = base_state();
  ui_poll(&s);                                   /* first paint fills the shadow */
  uint16_t after_first = ui_paints_for_test();
  s.uptime_min = 99; s.flow_total = 6000;        /* plenty changed... */
  ui_modem_ran();
  ui_poll(&s);
  TEST_ASSERT_EQUAL_UINT16(after_first, ui_paints_for_test());   /* ...and nothing painted */
  ui_poll(&s);                                   /* the flag is consumed, not sticky */
  TEST_ASSERT_TRUE(ui_paints_for_test() > after_first);
}

static void drain_tx(void) { char b[2048]; sim_serial_tx(b, sizeof b); }

/* cli_poll() reads at most 32 bytes per call and the overlong-line case pushes ~136, so one
   call would never reach the newline; loop, with a fixed bound. */
static size_t feed(const char *line, char *out, size_t cap) {
  drain_tx();
  sim_serial_rx(line);
  for (unsigned i = 0; i < 16u; ++i) cli_poll();
  return sim_serial_tx(out, cap);
}

/* Every command the bench binary has; none of these may sit behind the bring-up guard. */
static void test_parses_every_bench_command(void) {
  pb_test_setup();
  TEST_ASSERT_TRUE(cli_dispatch("i2c"));
  TEST_ASSERT_TRUE(cli_dispatch("mux 3"));
  TEST_ASSERT_TRUE(cli_dispatch("mux all"));
  TEST_ASSERT_TRUE(cli_dispatch("hall"));
  TEST_ASSERT_TRUE(cli_dispatch("flow"));
  TEST_ASSERT_TRUE(cli_dispatch("status"));
  TEST_ASSERT_TRUE(cli_dispatch("help"));
  TEST_ASSERT_TRUE(cli_dispatch("dry on"));
  TEST_ASSERT_TRUE(cli_dispatch("dry off"));
  TEST_ASSERT_TRUE(cli_dispatch("stop"));
  TEST_ASSERT_TRUE(cli_dispatch("clear contra"));
  TEST_ASSERT_FALSE(cli_dispatch("dry"));          /* no bare form, and no abbreviation */
  TEST_ASSERT_FALSE(cli_dispatch("mux 16"));      /* out of range */
  TEST_ASSERT_FALSE(cli_dispatch("nonsense"));
}

static void test_an_overlong_line_is_dropped_whole_not_truncated_into_a_command(void) {
  pb_test_setup();
  char line[PB_LINE_CAP + 40];
  memset(line, 'x', sizeof line);
  memcpy(line, "flow ", 5);                        /* a real command hiding at the front */
  line[sizeof line - 2] = '\n';
  line[sizeof line - 1] = '\0';
  char out[2048];
  size_t n = feed(line, out, sizeof out);
  out[n] = '\0';
  TEST_ASSERT_NOT_NULL(strstr(out, "line too long"));
  TEST_ASSERT_NULL(strstr(out, "flow hz="));       /* the prefix did NOT become a command */
}

/* Every word the dispatcher accepts, in cmd_help_'s column layout (the needle is the word plus
   its padding, so "home" cannot pass on HALL_HOME); the bring-up ones only in the bring-up
   binary, which native is. It once listed six and hid `dry off`, the one command an
   operator needs after a mid-dose reset. */
static void test_help_names_every_command_this_binary_has(void) {
  pb_test_setup();
  char out[2048];
  size_t n = feed("help\n", out, sizeof out);
  out[n] = '\0';
  static const char *const common[] = {
    "i2c   ", "mux <0-15>|all", "hall   ", "flow   ", "status   ", "stop   ",
    "dry on|off", "clear contra", "help   ",
  };
  for (unsigned i = 0; i < sizeof common / sizeof common[0]; ++i)
    TEST_ASSERT_NOT_NULL_MESSAGE(strstr(out, common[i]), common[i]);
#if PB_BRINGUP
  static const char *const bringup[] = {
    "servo <1000-2000> <ms>", "home   ", "goto <1-5>", "pump <ms> [prime] [hang]",
    "calib   ", "cal <pulses per litre>", "noinit pattern",
  };
  for (unsigned i = 0; i < sizeof bringup / sizeof bringup[0]; ++i)
    TEST_ASSERT_NOT_NULL_MESSAGE(strstr(out, bringup[i]), bringup[i]);
#endif
}

static void test_status_reports_the_watchdog_grant_liveness_and_the_pump_active_level(void) {
  pb_test_setup();
  char out[2048];
  size_t n = feed("status\n", out, sizeof out);
  out[n] = '\0';
  TEST_ASSERT_TRUE(pb_has_tok(out, "granted=5592ms"));
  TEST_ASSERT_TRUE(pb_has_tok(out, "alive=yes"));
  TEST_ASSERT_NOT_NULL(strstr(out, "WDT, not IWDT"));
  TEST_ASSERT_TRUE(pb_has_key(out, "pump_on_level="));
  sim_wdt_stop();
  n = feed("status\n", out, sizeof out);
  out[n] = '\0';
  TEST_ASSERT_TRUE(pb_has_tok(out, "alive=no"));
}

/* Both banners, both states. The needle for the latched case is the whole sentence: the raw
   .noinit dump line further down status also contains "contra=1", so a shorter needle could
   match either line. */
static void test_status_prints_the_correct_contra_banner_for_each_state(void) {
  pb_test_setup();
  char out[4096];
  size_t n = feed("status\n", out, sizeof out);
  out[n] = '\0';
  TEST_ASSERT_NOT_NULL_MESSAGE(strstr(out, "contra=0\n"), out);
  TEST_ASSERT_NULL_MESSAGE(strstr(out, "contra=1 ***"), out);

  pb_latch_contra();
  n = feed("status\n", out, sizeof out);
  out[n] = '\0';
  TEST_ASSERT_NOT_NULL_MESSAGE(
      strstr(out, "contra=1 *** CONTRADICTION LATCHED - float said OK, meter saw "
                  "nothing. `clear contra` to release.\n"), out);
  TEST_ASSERT_NULL_MESSAGE(strstr(out, "contra=0\n"), out);
}

static uint32_t parse_delta_(const char *out) {
  const char *p = strstr(out, "delta=");
  TEST_ASSERT_NOT_NULL(p);
  return (uint32_t)strtoul(p + 6, 0, 10);
}

/* Two distinct watchdog rates through two status calls: delta= must come from that call's
   probe, not the previous one's. Brackets, not exact integers: the probe loop's own clock
   reads add about a millisecond on top of the nominal 40 ms window. */
static void test_status_delta_reflects_the_probe_that_produced_it(void) {
  pb_test_setup();
  char out[2048];

  sim_wdt_rate_hz(2929);                          /* PCLKB/8192: ~120 counts per probe */
  size_t n = feed("status\n", out, sizeof out);
  out[n] = '\0';
  uint32_t delta_fast = parse_delta_(out);
  TEST_ASSERT_TRUE(delta_fast >= 100u && delta_fast <= 140u);

  sim_wdt_rate_hz(1000);                          /* distinctly slower: ~41 counts/probe */
  n = feed("status\n", out, sizeof out);
  out[n] = '\0';
  uint32_t delta_slow = parse_delta_(out);
  TEST_ASSERT_TRUE(delta_slow >= 30u && delta_slow <= 55u);

  TEST_ASSERT_TRUE(delta_slow < delta_fast);
}

/* newlib's float formatting is the deepest stack consumer in the program, so the float
   conversions are banned; a float-formatted number shows as digit.digit. Two exemptions: the
   ip= line's dotted quad (the whole line), and the dose summary's mls= field, computed in
   integer tenths -- field-scoped, because that line also carries outlet=, ms=, pulses=, ml=
   and r=. */
static void scan_line_for_float_formatting_(const char *line) {
  if (strncmp(line, "ip=", 3) == 0) return;
  const char *mv = strstr(line, "mls=");
  size_t lo = mv ? (size_t)((mv - line) + 4) : (size_t)-1;    /* mls='s first digit */
  size_t hi = lo;
  if (mv) while (line[hi] != '\0' && line[hi] != ' ') ++hi;   /* one past its last digit */
  for (size_t i = 1; line[i] != '\0' && line[i + 1] != '\0'; ++i) {
    if (mv && i >= lo && i < hi) continue;                    /* inside mls='s own value */
    if (line[i] == '.' && line[i - 1] >= '0' && line[i - 1] <= '9' &&
        line[i + 1] >= '0' && line[i + 1] <= '9')
      TEST_FAIL_MESSAGE(line);
  }
}

static void test_no_float_formatting_appears_in_any_printed_line(void) {
  /* the two needles are built character by character: the build check greps this tree, test/
     included, for a percent sign followed by a float conversion letter */
  pb_test_setup();
  char out[4096];
  size_t n = feed("status\n", out, sizeof out);
  out[n] = '\0';
  const char pct = '%';
  char needle[3] = { pct, 'f', '\0' };
  TEST_ASSERT_NULL(strstr(out, needle));
  needle[1] = 'g';
  TEST_ASSERT_NULL(strstr(out, needle));
  char *line = strtok(out, "\n");
  while (line) {
    scan_line_for_float_formatting_(line);
    line = strtok(0, "\n");
  }

  /* status alone never prints mls=, so the exemption went unexercised above. A real dose to
     DOSE_OK is the only route to the summary line: the console's pump and calib are by time
     and never reach it. */
  pb_advance(PB_BOOT_GAP_MS + 1u);
  sim_set_float(true);
  sim_set_flow_ml_s(85u);
  dose_req_t q = {0};
  q.ml = (uint16_t)PB_DOSE_RIG_MAX_ML;
  q.cap_ms = PB_DOSE_CAP_MS_MAX;
  q.long_prime = true;
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_OK, dose_run(&q), "arrange: a granted dose reaching target");
  (void)sim_serial_tx(out, sizeof out);
  cli_print_dose_summary();
  n = sim_serial_tx(out, sizeof out); out[n] = '\0';
  TEST_ASSERT_TRUE_MESSAGE(pb_has_key(out, "mls="), out);   /* the exemption under test */
  line = strtok(out, "\n");
  while (line) {
    scan_line_for_float_formatting_(line);
    line = strtok(0, "\n");
  }
}

/* The console's last-resort abort. The dose loop polls it once per iteration, so the word
   arrives in whatever fragments the UART hands over: here st and op. */
static void test_stop_is_matched_byte_by_byte_across_two_reads(void) {
  pb_test_setup();
  cli_stop_clear();
  sim_serial_rx("st");
  TEST_ASSERT_FALSE(cli_stop_requested());     /* half a word is not a stop */
  sim_serial_rx("op\n");
  TEST_ASSERT_TRUE(cli_stop_requested());
  TEST_ASSERT_TRUE(cli_stop_requested());      /* it LATCHES until cli_stop_clear() */
  cli_stop_clear();
  TEST_ASSERT_FALSE(cli_stop_requested());
}

/* sta must leave three bytes for the line buffer: a matcher that swallowed st would turn
   status into atus, an unknown command that looks like a console fault. */
static void test_a_non_matching_byte_is_pushed_to_the_line_buffer_unread(void) {
  pb_test_setup();
  cli_stop_clear();
  char out[512];
  (void)sim_serial_tx(out, sizeof out);
  sim_serial_rx("status\n");
  TEST_ASSERT_FALSE(cli_stop_requested());   /* not a stop, and not consumed either */
  cli_poll();                                /* reads the pushback FIRST */
  size_t n = sim_serial_tx(out, sizeof out); out[n] = '\0';
  TEST_ASSERT_TRUE_MESSAGE(pb_has_key(out, "granted="), out);   /* status actually ran */
}

/* dry on typed mid-dose sets the latch and raises the stop request: the word means the same
   during a dose as before one. */
static void test_dry_on_mid_dose_raises_the_stop_request_and_sets_the_latch(void) {
  pb_test_setup();
  cli_stop_clear();
  TEST_ASSERT_FALSE(safety_dry());
  sim_serial_rx("dry ");
  TEST_ASSERT_FALSE(cli_stop_requested());
  sim_serial_rx("on\n");
  TEST_ASSERT_TRUE(cli_stop_requested());
  TEST_ASSERT_TRUE(safety_dry());
}

/* Near misses: sto is short, stopp is long, xstop is not the line, and dry off is a different
   command that clears a latch and must not stop water. All four leave the bytes recoverable. */
static void test_a_near_miss_token_does_not_raise_the_stop_request(void) {
  const char *misses[] = { "sto\n", "stopp\n", "xstop\n", "dry off\n" };
  for (unsigned i = 0; i < 4u; ++i) {
    pb_test_setup();
    cli_stop_clear();
    sim_serial_rx(misses[i]);
    TEST_ASSERT_FALSE_MESSAGE(cli_stop_requested(), misses[i]);
    TEST_ASSERT_FALSE_MESSAGE(safety_dry(), misses[i]);
  }
}

/* The line matcher, not the latch (test_contra.cpp owns that): two literal tokens, no
   abbreviation. */
static void test_clear_requires_both_literal_tokens(void) {
  const char *misses[] = { "clear", "contra", "clearcontra", "clear  contra", "CLEAR CONTRA" };
  for (unsigned i = 0; i < 5u; ++i)
    TEST_ASSERT_FALSE_MESSAGE(cli_dispatch(misses[i]), misses[i]);
  TEST_ASSERT_TRUE(cli_dispatch("clear contra"));
}

static void test_goto_rejects_zero_and_six(void) {
#if PB_BRINGUP
  pb_test_setup();
  char out[256];
  const char *bad[] = { "goto 0", "goto 6", "goto x" };
  for (unsigned i = 0; i < 3u; ++i) {
    (void)sim_serial_tx(out, sizeof out);
    TEST_ASSERT_TRUE(cli_dispatch(bad[i]));            /* the command exists... */
    size_t n = sim_serial_tx(out, sizeof out); out[n] = '\0';
    TEST_ASSERT_NOT_NULL_MESSAGE(strstr(out, "1..5"), bad[i]);   /* ...and says the range */
  }
#else
  TEST_IGNORE_MESSAGE("bench build: goto is not a command");
#endif
}

static void test_pump_without_an_argument_is_refused(void) {
#if PB_BRINGUP
  pb_test_setup();
  char out[256];
  (void)sim_serial_tx(out, sizeof out);
  TEST_ASSERT_TRUE(cli_dispatch("pump"));
  size_t n = sim_serial_tx(out, sizeof out); out[n] = '\0';
  TEST_ASSERT_NOT_NULL_MESSAGE(strstr(out, "usage"), "a bare `pump` must never assert D6");
  TEST_ASSERT_FALSE(sim_pump_is_on());
#else
  TEST_IGNORE_MESSAGE("bench build: pump is not a command");
#endif
}

static void test_pump_ms_is_clamped_to_the_hard_cap(void) {
#if PB_BRINGUP
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  sim_set_float(true); sim_set_flow_ml_s(30);
  TEST_ASSERT_TRUE(cli_dispatch("pump 600000"));                  /* ten minutes typed */
  TEST_ASSERT_LESS_OR_EQUAL_UINT32(PB_DOSE_CAP_MS_MAX + 200u, sim_pump_on_ms());
#else
  TEST_IGNORE_MESSAGE("bench build");
#endif
}

/* The parser directly, not through a dose: "pump 500 hanging" has a 500 ms cap below the
   3000 ms PB_HANG_MS, so the cap abort fires before a wrongly-true hang flag could ever be
   observed, and no host case may risk a real hang. */
static void test_pump_flag_parser_requires_whole_tokens(void) {
#if PB_BRINGUP
  bool prime, hang;
  cli_pump_flags_for_test_("hanging", &prime, &hang);
  TEST_ASSERT_FALSE_MESSAGE(hang, "hanging");
  cli_pump_flags_for_test_("primed", &prime, &hang);
  TEST_ASSERT_FALSE_MESSAGE(prime, "primed");
  cli_pump_flags_for_test_("hang", &prime, &hang);
  TEST_ASSERT_TRUE_MESSAGE(hang, "hang");
  cli_pump_flags_for_test_("prime", &prime, &hang);
  TEST_ASSERT_TRUE_MESSAGE(prime, "prime");
  cli_pump_flags_for_test_("prime hang", &prime, &hang);
  TEST_ASSERT_TRUE(prime); TEST_ASSERT_TRUE(hang);
#else
  TEST_IGNORE_MESSAGE("bench build");
#endif
}

/* " hanging" contains " hang": a bare strstr would accept it. */
static void test_pump_hang_requires_the_literal_third_token(void) {
#if PB_BRINGUP
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  sim_set_float(true); sim_set_flow_ml_s(30);
  uint32_t f0 = sim_feeds();
  TEST_ASSERT_TRUE(cli_dispatch("pump 500 hanging"));
  TEST_ASSERT_GREATER_THAN_UINT32(f0, sim_feeds());   /* the dog was fed throughout */
#else
  TEST_IGNORE_MESSAGE("bench build");
#endif
}

static void test_cal_rejects_zero_and_absurd_values(void) {
#if PB_BRINGUP
  pb_test_setup();
  uint16_t before = cfg_pulses_per_l_get();
  char out[256];
  const char *bad[] = { "cal 0", "cal 999", "cal 20001", "cal 4294967295", "cal -5", "cal x" };
  for (unsigned i = 0; i < 6u; ++i) {
    (void)sim_serial_tx(out, sizeof out);
    TEST_ASSERT_TRUE(cli_dispatch(bad[i]));
    size_t n = sim_serial_tx(out, sizeof out); out[n] = '\0';
    TEST_ASSERT_NOT_NULL_MESSAGE(strstr(out, "1000..20000"), bad[i]);
    TEST_ASSERT_EQUAL_UINT16_MESSAGE(before, cfg_pulses_per_l_get(), bad[i]);
  }
  TEST_ASSERT_TRUE(cli_dispatch("cal 5880"));
  TEST_ASSERT_EQUAL_UINT16(5880u, cfg_pulses_per_l_get());
#else
  TEST_IGNORE_MESSAGE("bench build");
#endif
}

/* The write must have happened, not merely the command been recognised: a true dispatch
   cannot tell "wrote the pattern" from "did nothing". */
static void test_noinit_pattern_writes_the_known_word_and_recomputes_the_checksum(void) {
#if PB_BRINGUP
  pb_test_setup();
  g_nv.pattern = 0u;
  noinit_commit();
  TEST_ASSERT_TRUE(cli_dispatch("noinit pattern"));
  TEST_ASSERT_EQUAL_HEX32(0xC0FFEE01u, g_nv.pattern);
  TEST_ASSERT_EQUAL_HEX32_MESSAGE(noinit_sum(&g_nv), g_nv.sum,
      "the checksum must be recomputed on this write too, or a warm reset reads the "
      "pattern back as a corrupt struct and 7c' would prove nothing");
#else
  TEST_IGNORE_MESSAGE("bench build");
#endif
}

/* r=ok for DOSE_OK and the real token otherwise, never the wire's "none". The printer ships in
   both binaries, and the console's pump and calib are by time and never reach DOSE_OK, so a
   direct dose is the only route to that branch. */
static void test_dose_summary_line_prints_r_ok_only_for_a_successful_dose(void) {
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  sim_set_float(true);
  sim_set_flow_ml_s(85u);
  dose_req_t q = {0};
  q.ml = (uint16_t)PB_DOSE_RIG_MAX_ML;
  q.cap_ms = PB_DOSE_CAP_MS_MAX;
  q.long_prime = true;
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_OK, dose_run(&q), "arrange: a granted dose reaching target");
  char out[512];
  (void)sim_serial_tx(out, sizeof out);
  cli_print_dose_summary();
  size_t n = sim_serial_tx(out, sizeof out); out[n] = '\0';
  TEST_ASSERT_TRUE_MESSAGE(pb_has_tok(out, "r=ok"), out);
  TEST_ASSERT_FALSE_MESSAGE(pb_has_tok(out, "r=none"), out);   /* err_of(DOSE_OK) is "none" on
                                                                   the wire; the SUMMARY must say ok */

  /* And the negative: a refused dose prints its real token, never "ok". */
  pb_advance(PB_DOSE_MIN_GAP_MS + 1u);      /* clear the 10 s cooldown between callers */
  sim_set_float(false);
  TEST_ASSERT_EQUAL_MESSAGE(DOSE_REFUSED_FLOAT, dose_run(&q), "arrange: a float refusal");
  (void)sim_serial_tx(out, sizeof out);
  cli_print_dose_summary();
  n = sim_serial_tx(out, sizeof out); out[n] = '\0';
  TEST_ASSERT_TRUE_MESSAGE(pb_has_tok(out, "r=float"), out);
  TEST_ASSERT_FALSE_MESSAGE(pb_has_tok(out, "r=ok"), out);
}

static void test_dose_summary_line_carries_outlet_ms_pulses_ml_and_mls(void) {
#if PB_BRINGUP
  pb_test_setup();
  pb_advance(PB_BOOT_GAP_MS + 1u);
  sim_set_float(true); sim_set_flow_ml_s(30);
  TEST_ASSERT_TRUE(cli_dispatch("cal 5880"));
  char out[512];
  (void)sim_serial_tx(out, sizeof out);
  TEST_ASSERT_TRUE(cli_dispatch("pump 4000"));
  size_t n = sim_serial_tx(out, sizeof out); out[n] = '\0';
  TEST_ASSERT_NOT_NULL_MESSAGE(strstr(out, "dose outlet="), out);
  TEST_ASSERT_TRUE(pb_has_key(out, "ms="));
  TEST_ASSERT_TRUE(pb_has_key(out, "pulses="));
  TEST_ASSERT_TRUE(pb_has_key(out, "ml="));
  TEST_ASSERT_TRUE(pb_has_key(out, "mls="));
  TEST_ASSERT_TRUE(pb_has_key(out, "r="));
  /* mls is integer tenths printed around a literal dot: the float conversions are banned */
  const char *mls = strstr(out, " mls=");
  TEST_ASSERT_NOT_NULL(strchr(mls, '.'));
#else
  TEST_IGNORE_MESSAGE("bench build");
#endif
}

/* "pump 60000 prime hang" is one typed line that asserts D6, suppresses the no-flow abort and
   starves the watchdog. Over an unauthenticated USB CDC line a serial-monitor reconnect or a
   cat into the wrong /dev/cu.* is enough. A token's spelling is not a gate; a different
   binary is. */
static void test_bringup_commands_are_absent_from_the_bench_build(void) {
  pb_test_setup();
#if PB_BRINGUP
  TEST_ASSERT_TRUE(cli_dispatch("servo 1600 200"));
  TEST_ASSERT_TRUE(cli_dispatch("home"));
  TEST_ASSERT_TRUE(cli_dispatch("goto 3"));
  TEST_ASSERT_TRUE(cli_dispatch("pump"));          /* exists; refuses without an argument */
  TEST_ASSERT_TRUE(cli_dispatch("calib"));
  TEST_ASSERT_TRUE(cli_dispatch("cal 5880"));
  TEST_ASSERT_TRUE(cli_dispatch("noinit pattern"));
#else
  /* not refused: not a command. "? unknown; type help" is how an operator proves which binary
     is flashed */
  TEST_ASSERT_FALSE(cli_dispatch("servo 1600 200"));
  TEST_ASSERT_FALSE(cli_dispatch("home"));
  TEST_ASSERT_FALSE(cli_dispatch("goto 3"));
  TEST_ASSERT_FALSE(cli_dispatch("pump 2000"));
  TEST_ASSERT_FALSE(cli_dispatch("calib"));
  TEST_ASSERT_FALSE(cli_dispatch("cal 5880"));
  TEST_ASSERT_FALSE(cli_dispatch("noinit pattern"));
  /* the four that ship in both binaries: an unattended board must still be stoppable,
     dry-able and releasable */
  TEST_ASSERT_TRUE(cli_dispatch("stop"));
  TEST_ASSERT_TRUE(cli_dispatch("dry on"));
  TEST_ASSERT_TRUE(cli_dispatch("dry off"));
  TEST_ASSERT_TRUE(cli_dispatch("clear contra"));
#endif
}

static void test_every_sim_command_is_parsed_and_dispatched(void) {
  pb_test_setup();
#if PB_SIM_CLI
  /* A routing-only check cannot tell "float 0" from "float 1": both return true down one
     path. Each argument-differentiated pair below is followed by a read of its observable
     effect through an existing host-visible route. */
  TEST_ASSERT_TRUE(cli_dispatch("sim float 0"));
  TEST_ASSERT_EQUAL_INT_MESSAGE(PB_HIGH, hal_pin_read(PIN_HALL_FLOAT),
                                 "float 0 must read as NOT ok (spec 2.10: LOW == OK)");
  TEST_ASSERT_TRUE(cli_dispatch("sim float 1"));
  TEST_ASSERT_EQUAL_INT_MESSAGE(PB_LOW, hal_pin_read(PIN_HALL_FLOAT),
                                 "float 1 must read as ok (spec 2.10: LOW == OK)");

  TEST_ASSERT_TRUE(cli_dispatch("sim flow 30"));
  TEST_ASSERT_TRUE(cli_dispatch("sim flow storm"));

  TEST_ASSERT_TRUE(cli_dispatch("sim i2c fail"));
  TEST_ASSERT_FALSE_MESSAGE(hal_i2c_probe(I2C_ADDR_OLED), "i2c fail must fail every probe");
  TEST_ASSERT_TRUE(cli_dispatch("sim i2c ok"));
  TEST_ASSERT_TRUE_MESSAGE(hal_i2c_probe(I2C_ADDR_OLED), "i2c ok must restore the bus");

  TEST_ASSERT_TRUE(cli_dispatch("sim mux stuck"));

  /* the stall's effect lives on the screw emitter, which has no getter: turn the screw and
     count real pulses */
  pulses_begin();
  sim_set_screw_pulse_ms(50);           /* 20 Hz -- 0 would itself read as "not turning" */
  hal_servo_us(1600);                   /* off the 1500 stop point, either direction */
  pb_advance(100);
  TEST_ASSERT_TRUE_MESSAGE(pulses_screw() > 0u, "arrange: the screw must turn unstalled");
  TEST_ASSERT_TRUE(cli_dispatch("sim stall on"));
  pulses_begin();
  pb_advance(100);
  TEST_ASSERT_EQUAL_UINT32_MESSAGE(0u, pulses_screw(), "stall on must freeze the screw");
  TEST_ASSERT_TRUE(cli_dispatch("sim stall off"));
  pulses_begin();
  pb_advance(100);
  TEST_ASSERT_TRUE_MESSAGE(pulses_screw() > 0u, "stall off must let the screw turn again");
  hal_servo_us(1500);                   /* back to stopped */
  sim_set_screw_pulse_ms(0);            /* back to "does not turn", the model's own default */

  TEST_ASSERT_TRUE(cli_dispatch("sim leak on"));
  TEST_ASSERT_TRUE(cli_dispatch("sim wdt stop"));
  TEST_ASSERT_TRUE(cli_dispatch("sim wdt slow 100"));
  TEST_ASSERT_TRUE(cli_dispatch("sim noinit clobber"));
  TEST_ASSERT_TRUE(cli_dispatch("sim ch 2 8123"));

  /* resp only changes what a later sock_read() drains, so it is read back through
     sock_open()/sock_read() as the consumer would */
  {
    /* double-escaped to match the dispatch line byte for byte: the resp handler copies its
       body verbatim, so the two-character backslash-n the console line carries is what
       sock_read() must drain */
    static const char body[] = "next=60\\ncmd=7 water=3 ml=120 cap_s=11\\n";
    link_fake_reset();
    link_begin(1);
    link_fake_set_state(LINK_UP);
    TEST_ASSERT_TRUE(cli_dispatch("sim resp \"next=60\\ncmd=7 water=3 ml=120 cap_s=11\\n\""));
    TEST_ASSERT_TRUE_MESSAGE(sock_open(), "arrange: the fake socket must open");
    uint8_t got[64];
    int n = sock_read(got, sizeof got);
    TEST_ASSERT_EQUAL_INT_MESSAGE((int)(sizeof body - 1u), n,
                                   "resp must queue the body WITHOUT its closing quote");
    TEST_ASSERT_EQUAL_MEMORY(body, got, (size_t)n);
  }

  /* reset warm|cold share one path and differ only in the argument; g_nv is the real noinit
     struct: warm keeps it, cold clears it */
  g_nv.pattern = 0xABCD1234u;
  noinit_commit();
  TEST_ASSERT_TRUE(cli_dispatch("sim reset warm"));
  TEST_ASSERT_EQUAL_UINT32_MESSAGE(0xABCD1234u, g_nv.pattern, "reset warm must keep .noinit");

  g_nv.pattern = 0xABCD1234u;
  noinit_commit();
  TEST_ASSERT_TRUE(cli_dispatch("sim reset cold"));
  TEST_ASSERT_EQUAL_UINT32_MESSAGE(0u, g_nv.pattern, "reset cold must clear .noinit");

  TEST_ASSERT_FALSE(cli_dispatch("sim ch 9 1"));       /* channel out of 0..5 */
  TEST_ASSERT_FALSE(cli_dispatch("sim nonsense"));
#else
  TEST_IGNORE_MESSAGE("PB_SIM_CLI is undefined: `sim ...` is not a command at all");
#endif
}

/* native_nosimcli undefines PB_SIM_CLI alone (not PB_SIM, which would leave the suite with no
   HAL), so this case is compiled once per env. */
static void test_sim_commands_are_absent_from_the_bench_and_bringup_builds(void) {
#ifdef PB_SIM_CLI
  TEST_ASSERT_TRUE(cli_dispatch("sim float 0"));
#else
  TEST_ASSERT_FALSE(cli_dispatch("sim float 0"));      /* not a command at all */
#endif
}

/* Exact, not a substring: a substring check for "SIM" would ship a truncated banner. */
static void test_the_sim_banner_holds_row_zero_on_both_screens(void) {
  ui_state_t s = base_state();
  s.sim = true;
  char oled[8][17], lcd[2][17];
  ui_render(&s, oled);
  ui_render_lcd(&s, lcd);
  TEST_ASSERT_EQUAL_STRING("*** SIM NO D6 **", oled[0]);
  TEST_ASSERT_EQUAL_STRING("*** SIM NO D6 **", lcd[0]);
  TEST_ASSERT_NULL(strstr(oled[0], "PB "));            /* the banner WINS row 0 */
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_ui_render_fills_eight_rows_of_sixteen_characters);
  RUN_TEST(test_ui_render_lcd_shows_the_contradiction_banner);
  RUN_TEST(test_ui_render_lcd_prose_is_never_the_wire_error_token);
  RUN_TEST(test_ui_render_lcd_shows_the_last_http_status_on_a_four_hundred);
  RUN_TEST(test_ui_poll_is_a_noop_while_the_pump_is_asserted);
  RUN_TEST(test_ui_poll_is_a_noop_in_a_pass_where_a_modem_command_ran);
  RUN_TEST(test_parses_every_bench_command);
  RUN_TEST(test_help_names_every_command_this_binary_has);
  RUN_TEST(test_an_overlong_line_is_dropped_whole_not_truncated_into_a_command);
  RUN_TEST(test_status_reports_the_watchdog_grant_liveness_and_the_pump_active_level);
  RUN_TEST(test_status_prints_the_correct_contra_banner_for_each_state);
  RUN_TEST(test_status_delta_reflects_the_probe_that_produced_it);
  RUN_TEST(test_no_float_formatting_appears_in_any_printed_line);
  RUN_TEST(test_stop_is_matched_byte_by_byte_across_two_reads);
  RUN_TEST(test_a_non_matching_byte_is_pushed_to_the_line_buffer_unread);
  RUN_TEST(test_dry_on_mid_dose_raises_the_stop_request_and_sets_the_latch);
  RUN_TEST(test_a_near_miss_token_does_not_raise_the_stop_request);
  RUN_TEST(test_clear_requires_both_literal_tokens);
  RUN_TEST(test_goto_rejects_zero_and_six);
  RUN_TEST(test_pump_without_an_argument_is_refused);
  RUN_TEST(test_pump_ms_is_clamped_to_the_hard_cap);
  RUN_TEST(test_pump_flag_parser_requires_whole_tokens);
  RUN_TEST(test_pump_hang_requires_the_literal_third_token);
  RUN_TEST(test_cal_rejects_zero_and_absurd_values);
  RUN_TEST(test_noinit_pattern_writes_the_known_word_and_recomputes_the_checksum);
  RUN_TEST(test_dose_summary_line_prints_r_ok_only_for_a_successful_dose);
  RUN_TEST(test_dose_summary_line_carries_outlet_ms_pulses_ml_and_mls);
  RUN_TEST(test_bringup_commands_are_absent_from_the_bench_build);
  RUN_TEST(test_every_sim_command_is_parsed_and_dispatched);
  RUN_TEST(test_sim_commands_are_absent_from_the_bench_and_bringup_builds);
  RUN_TEST(test_the_sim_banner_holds_row_zero_on_both_screens);
  return UNITY_END();
}
