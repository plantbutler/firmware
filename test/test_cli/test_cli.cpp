/* test/test_cli/test_cli.cpp -- the console and the two renderers.
   Task 11 adds the line-reader and command cases to this same file; tasks 15, 16, 19, 20
   and 29 add more. The fixture is the SHARED one from the first line: every later case in
   this file calls pb_test_setup() (which starts the watchdog), and a suite whose setUp is
   empty makes task 11's granted=/alive= case unpassable. */
#include "../support/harness.h"
#include "cart.h"
#include "cli.h"
#include "config.h"
#include "hal.h"
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
  strcpy(s.controller, "bench1");
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
    TEST_ASSERT_EQUAL_CHAR('\0', rows[r][16]);      /* terminated AT index 16 */
    TEST_ASSERT_EQUAL_UINT(16, strlen(rows[r]));    /* padded, so no stale glyphs remain */
  }
  TEST_ASSERT_EQUAL_STRING("PB bench1  1h23m", rows[0]);
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
}

static void test_ui_render_lcd_prose_is_never_the_wire_error_token(void) {
  /* spec §4.1's fixed err= enum. Row 1 is human prose and must never be one of these. */
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

/* spec §4.2: "the last HTTP status is on the LCD, not only in `status`: a 400/401 loop is
   otherwise invisible to anyone not on the serial port". The renderer decides this, not the
   caller: main.cpp's ui_fill_() selects lcd_detail for a dozen other reasons, and a rule
   that depended on it happening to choose the right one would be a rule in name only. */
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

/* spec §3, §5: a pass that issued a modem command has already spent up to 2.4 s of a
   5592 ms grant, and one wedged LCD row is up to 102 s. net_poll() calls ui_modem_ran()
   directly (task 24); this is the assertion that keeps that call from being deleted. */
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

/* cli_poll() reads at most sizeof(buf) == 32 bytes per call (step 4), and the overlong-line
   case below pushes ~136 bytes at it. ONE cli_poll() would consume 32 of them and never
   reach the newline, so "line too long" would never be printed and the case would fail for
   a reason that has nothing to do with the line reader. Loop, with a fixed bound so that a
   bug here cannot hang the suite. */
static size_t feed(const char *line, char *out, size_t cap) {
  drain_tx();
  sim_serial_rx(line);
  for (unsigned i = 0; i < 16u; ++i) cli_poll();
  return sim_serial_tx(out, cap);
}

/* THE BENCH COMMAND SET of spec §6, and this case must end up carrying ALL of it.
   Six commands exist today. Four more arrive later and each is added to THIS case by the
   task that adds the command: `dry on` / `dry off` (task 15), `stop` (task 16),
   `clear contra` (task 19). Every one of them is present in the BENCH binary as well as
   the bringup one, so none of them may be wrapped in #if PB_BRINGUP here or there. */
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

static void test_status_reports_the_watchdog_grant_liveness_and_the_pump_active_level(void) {
  pb_test_setup();
  char out[2048];
  size_t n = feed("status\n", out, sizeof out);
  out[n] = '\0';
  TEST_ASSERT_NOT_NULL(strstr(out, "granted=5592ms"));
  TEST_ASSERT_NOT_NULL(strstr(out, "alive=yes"));
  TEST_ASSERT_NOT_NULL(strstr(out, "WDT, not IWDT"));
  TEST_ASSERT_NOT_NULL(strstr(out, "pump_on_level="));
  sim_wdt_stop();
  n = feed("status\n", out, sizeof out);
  out[n] = '\0';
  TEST_ASSERT_NOT_NULL(strstr(out, "alive=no"));
}

static uint32_t parse_delta_(const char *out) {
  const char *p = strstr(out, "delta=");
  TEST_ASSERT_NOT_NULL(p);
  return (uint32_t)strtoul(p + 6, 0, 10);
}

/* No case in this tree pins WHICH probe a printed delta= came from. "present" or "above
   PB_WDT_PROBE_MIN_COUNTS" would pass against a STALE delta from a prior probe at a
   similar rate just as well as a fresh one -- exactly the shape of the bug fix round 2
   found in cli_print_status() (hal_wdt_alive() and hal_wdt_last_delta() passed as two
   arguments of one unspecified-order snprintf() call). Drive two DISTINCT watchdog
   rates through two separate `status` calls and assert delta= tracks the rate that was
   active for THAT call, not the previous one. Brackets, not exact integers, the same
   way test_dose.cpp:120-121 already brackets a probe delta against the ~41 ms the probe
   loop's own hal_millis() reads add on top of the nominal 40 ms window. */
static void test_status_delta_reflects_the_probe_that_produced_it(void) {
  pb_test_setup();
  char out[2048];

  sim_wdt_rate_hz(2929);                          /* PCLKB/8192 (§7): ~120 counts/probe */
  size_t n = feed("status\n", out, sizeof out);
  out[n] = '\0';
  uint32_t delta_fast = parse_delta_(out);
  TEST_ASSERT_TRUE(delta_fast >= 100u && delta_fast <= 140u);

  sim_wdt_rate_hz(1000);                          /* distinctly slower: ~41 counts/probe */
  n = feed("status\n", out, sizeof out);
  out[n] = '\0';
  uint32_t delta_slow = parse_delta_(out);
  TEST_ASSERT_TRUE(delta_slow >= 30u && delta_slow <= 55u);

  /* The ranges above are already disjoint; this is the assertion that actually pins
     the ordering bug, spelled out rather than left implicit in the two brackets. */
  TEST_ASSERT_TRUE(delta_slow < delta_fast);
}

static void test_no_float_formatting_appears_in_any_printed_line(void) {
  /* newlib's float formatting is the deepest stack consumer in the program (spec §12),
     so the float conversions are banned. A float-formatted number shows as
     <digit>.<digit>. Two exemptions, and only two: the ip= line's dotted quad, and -
     from task 20 - the mls= field of the dose summary line, computed in integer tenths.

     THE TWO NEEDLES ARE BUILT CHARACTER BY CHARACTER ON PURPOSE. make check greps this
     tree for a percent sign followed by a float conversion letter, and it scans string
     literals in test/ exactly as it scans code; writing the needles out would make this
     file the one hit that fails the check it exists to defend. */
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
    if (strncmp(line, "ip=", 3) != 0)
      for (size_t i = 1; line[i] != '\0' && line[i + 1] != '\0'; ++i)
        if (line[i] == '.' && line[i - 1] >= '0' && line[i - 1] <= '9' &&
            line[i + 1] >= '0' && line[i + 1] <= '9')
          TEST_FAIL_MESSAGE(line);
    line = strtok(0, "\n");
  }
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
  RUN_TEST(test_an_overlong_line_is_dropped_whole_not_truncated_into_a_command);
  RUN_TEST(test_status_reports_the_watchdog_grant_liveness_and_the_pump_active_level);
  RUN_TEST(test_status_delta_reflects_the_probe_that_produced_it);
  RUN_TEST(test_no_float_formatting_appears_in_any_printed_line);
  return UNITY_END();
}
