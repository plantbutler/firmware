/* test/test_cli/test_cli.cpp -- the console and the two renderers.
   Task 11 adds the line-reader and command cases to this same file; tasks 15, 16, 19, 20
   and 29 add more. The fixture is the SHARED one from the first line: every later case in
   this file calls pb_test_setup() (which starts the watchdog), and a suite whose setUp is
   empty makes task 11's granted=/alive= case unpassable. */
#include "../support/harness.h"
#include "cart.h"
#include "config.h"
#include "ui.h"
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

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_ui_render_fills_eight_rows_of_sixteen_characters);
  RUN_TEST(test_ui_render_lcd_shows_the_contradiction_banner);
  RUN_TEST(test_ui_render_lcd_prose_is_never_the_wire_error_token);
  RUN_TEST(test_ui_render_lcd_shows_the_last_http_status_on_a_four_hundred);
  RUN_TEST(test_ui_poll_is_a_noop_while_the_pump_is_asserted);
  RUN_TEST(test_ui_poll_is_a_noop_in_a_pass_where_a_modem_command_ran);
  return UNITY_END();
}
