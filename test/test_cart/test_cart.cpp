#include <unity.h>
#include <string.h>
#include "../support/harness.h"
#include "cart.h"
#include "config.h"
#include "sensors.h"
#include "sim.h"

void setUp(void)    { pb_test_setup(); }
void tearDown(void) { pb_test_teardown(); }

/* §2.15. With the pitch unknown the natural arithmetic finds the target already satisfied
   at home, returns true and sets g_pos = outlet; both position guards then pass and the
   pump dead-heads against a CLOSED manifold with pos=ok on the wire. A comment saying
   "0 means always refuse" is not a mechanism. */
void test_goto_refuses_when_pulses_per_gate_is_zero(void) {
  TEST_ASSERT_TRUE(cart_begin());
#if PB_PULSES_PER_GATE == 0
  TEST_ASSERT_FALSE(cart_goto(1));
  TEST_ASSERT_FALSE(cart_goto(3));
  TEST_ASSERT_EQUAL_STRING("uncal", cart_err());
  TEST_ASSERT_FALSE(cart_pos_known());
#else
  TEST_IGNORE_MESSAGE("calibrated arm: see native_cal");
#endif
}

/* The regression on Manifold::reset(): it drove backwards for ONE gate-width and declared
   position 0, which from gate five is about 80 s short of the threadless start of the
   screw -- so the cart was left over gate four, holding it open under the reservoir head,
   while the firmware believed it was parked. */
void test_home_from_outlet_five_actually_reaches_home(void) {
#if PB_PULSES_PER_GATE == 0
  TEST_IGNORE_MESSAGE("uncalibrated arm: goto is compiled out");
#else
  TEST_ASSERT_TRUE(cart_begin());
  sim_set_screw_pulse_ms(2);
  sim_set_home_region(0, 40);
  sim_set_cart_at(PB_PULSES_HOME_TO_1 + 4u * PB_PULSES_PER_GATE);   /* over gate five */
  TEST_ASSERT_TRUE(cart_home());
  TEST_ASSERT_TRUE(cart_parked());
  TEST_ASSERT_EQUAL_UINT32(0u, cart_pulses());
  TEST_ASSERT_EQUAL_UINT8(0u, cart_pos());
  /* Not just cart.cpp's own bookkeeping: a live read of the PHYSICAL home hall, so a
     move_() call site that stops on a fixed pulse count instead of the hall (the exact
     shape of the old bug) cannot pass this case merely by zeroing g_pulses/g_pos on any
     "successful" move -- it has to have actually left the magnet over the sensor. */
  bool home = false;
  TEST_ASSERT_TRUE(sensors_home_hall(&home));
  TEST_ASSERT_TRUE(home);
#endif
}

/* cart_begin() promises "servo stopped, position UNKNOWN, NO MOVEMENT" -- checked two
   ways: cart_pos_known() itself, and that every SIM_EV_SERVO write on record (there is
   exactly one: cart_begin()'s own stop) carries 1500 and nothing else. The self-home is
   exec_pending()'s, PB_BOOT_HOME_MS after reset -- not cart_begin()'s. */
void test_position_is_unknown_after_boot_until_homed(void) {
  TEST_ASSERT_TRUE(cart_begin());
  TEST_ASSERT_FALSE(cart_pos_known());
  const sim_ev_t *ev;
  size_t n = sim_events(&ev);
  uint32_t servo_writes = 0;
  for (size_t i = 0; i < n; ++i) {
    if (ev[i].kind == SIM_EV_SERVO) {
      servo_writes++;
      TEST_ASSERT_EQUAL_UINT16_MESSAGE((uint16_t)PB_SERVO_STOP_US, (uint16_t)ev[i].arg,
                                        "cart_begin() must not move the cart");
    }
  }
  TEST_ASSERT_TRUE(servo_writes >= 1u);
}

/* Only meaningful while the pitch is unknown: under native_cal a successful cart_home()
   DOES make cart_pos_known() true, so this is guarded the opposite way from the refusal
   case above. Seeing the home hall (cart_home() succeeding) is not the same fact as being
   able to deliver to a numbered outlet (cart_pos_known()) -- that needs the pitch too. */
void test_pos_is_never_ok_before_calibration(void) {
#if PB_PULSES_PER_GATE == 0
  TEST_ASSERT_TRUE(cart_begin());
  sim_set_screw_pulse_ms(2);
  sim_set_home_region(0, 40);
  TEST_ASSERT_TRUE(cart_home());
  TEST_ASSERT_FALSE(cart_pos_known());
#else
  TEST_IGNORE_MESSAGE("this property only holds while PB_PULSES_PER_GATE == 0; see native");
#endif
}

/* The whole difference between this file and the one it replaces: the OLD Manifold moved
   by minutes of blocking waits; this one counts pulses, so a slower screw takes longer
   wall time to cover the SAME distance. Re-homed between the two goto(2) calls so each
   one actually traverses rather than finding the target already satisfied. */
void test_goto_counts_pulses_not_milliseconds(void) {
#if PB_PULSES_PER_GATE == 0
  TEST_IGNORE_MESSAGE("uncalibrated arm: goto is compiled out");
#else
  TEST_ASSERT_TRUE(cart_begin());
  sim_set_home_region(0, 40);

  sim_set_screw_pulse_ms(2);
  TEST_ASSERT_TRUE(cart_home());
  uint32_t t0 = hal_millis();
  TEST_ASSERT_TRUE(cart_goto(2));
  uint32_t fast_ms      = hal_millis() - t0;
  uint32_t fast_pulses  = cart_pulses();

  TEST_ASSERT_TRUE(cart_home());
  sim_set_screw_pulse_ms(8);
  uint32_t t1 = hal_millis();
  TEST_ASSERT_TRUE(cart_goto(2));
  uint32_t slow_ms      = hal_millis() - t1;
  uint32_t slow_pulses  = cart_pulses();

  TEST_ASSERT_EQUAL_UINT32(fast_pulses, slow_pulses);
  TEST_ASSERT_TRUE(slow_ms > fast_ms);
#endif
}

/* "cart_pulses() is not zeroed" is only a real assertion if it started non-zero: under
   the uncalibrated arm g_pulses can only ever be 0 or freshly re-zeroed by a successful
   cart_home(), so this case needs cart_goto() (native_cal) to move it off zero first,
   then proves a FAILED cart_home() leaves that value alone rather than re-zeroing it. */
void test_home_zeroes_the_count_only_when_the_hall_asserts(void) {
#if PB_PULSES_PER_GATE == 0
  TEST_IGNORE_MESSAGE("g_pulses can only be moved off zero by cart_goto(), which is compiled out");
#else
  TEST_ASSERT_TRUE(cart_begin());
  sim_set_screw_pulse_ms(2);
  sim_set_home_region(0, 40);
  TEST_ASSERT_TRUE(cart_home());
  TEST_ASSERT_TRUE(cart_goto(3));
  uint32_t before = cart_pulses();
  TEST_ASSERT_TRUE(before > 0u);
  sim_set_home_region(9000u, 9001u);        /* moved out of reach */
  TEST_ASSERT_FALSE(cart_home());
  TEST_ASSERT_EQUAL_UINT32(before, cart_pulses());   /* NOT zeroed */
  TEST_ASSERT_FALSE(cart_pos_known());
#endif
}

/* An unreadable hall must not read as "not home", which is what would drive the cart
   blind into the end of the screw. Runs the same in both arms: neither cart_home() nor
   the i2c error depends on the pitch. */
void test_an_i2c_error_on_the_home_hall_is_unknown_not_not_home(void) {
  TEST_ASSERT_TRUE(cart_begin());
  sim_set_screw_pulse_ms(2);
  sim_set_home_region(0, 40);
  sim_set_i2c_fail(true);
  TEST_ASSERT_FALSE(cart_home());
  TEST_ASSERT_EQUAL_STRING("i2c", cart_err());
  TEST_ASSERT_FALSE(cart_pos_known());
}

/* sim_set_clock_ms() (task 14 step 1) starts the fake within PB_MOVE_CAP_MS of the wrap.
   The home region is put out of reach so the ONLY way this ends is the deadline, proving
   move_()'s unsigned differences hold across the millis() rollover rather than the hall. */
void test_move_deadline_holds_across_a_millis_rollover(void) {
  TEST_ASSERT_TRUE(cart_begin());
  sim_set_screw_pulse_ms(2);
  sim_set_home_region(9000u, 9001u);
  sim_set_clock_ms(0xFFFFF000u);
  uint32_t t0 = hal_millis();
  TEST_ASSERT_FALSE(cart_home());
  uint32_t elapsed = hal_millis() - t0;   /* unsigned: valid across the wrap this rides through */
  TEST_ASSERT_EQUAL_STRING("timeout", cart_err());
  TEST_ASSERT_TRUE(elapsed >= (uint32_t)PB_MOVE_CAP_MS);
  TEST_ASSERT_TRUE(elapsed < (uint32_t)PB_MOVE_CAP_MS + 1000u);   /* generous, not tick-tight */
}

/* A traverse that never saw the hall has not found home, and must not be allowed to claim
   it did. */
void test_home_that_times_out_leaves_position_unknown(void) {
  TEST_ASSERT_TRUE(cart_begin());
  sim_set_screw_pulse_ms(2);
  sim_set_home_region(9000u, 9001u);
  sim_set_cart_at(600u);
  TEST_ASSERT_FALSE(cart_home());
  TEST_ASSERT_EQUAL_STRING("timeout", cart_err());
  TEST_ASSERT_FALSE(cart_pos_known());
  TEST_ASSERT_FALSE(cart_parked());
}

/* The stall window, not the move cap, is what ends a stalled traverse. */
void test_stall_aborts_within_the_stall_window_and_loses_position(void) {
#if PB_PULSES_PER_GATE == 0
  TEST_IGNORE_MESSAGE("uncalibrated arm: goto is compiled out");
#else
  TEST_ASSERT_TRUE(cart_begin());
  sim_set_screw_pulse_ms(2);
  sim_set_home_region(0, 40);
  TEST_ASSERT_TRUE(cart_home());
  sim_set_stall(true);
  uint32_t t0 = hal_millis();
  TEST_ASSERT_FALSE(cart_goto(3));
  uint32_t elapsed = hal_millis() - t0;
  TEST_ASSERT_EQUAL_STRING("stall", cart_err());
  TEST_ASSERT_FALSE(cart_pos_known());
  TEST_ASSERT_TRUE(elapsed < (uint32_t)PB_STALL_WINDOW_MS + (uint32_t)PB_MOVE_CAP_MS / 10u);
#endif
}

/* A rejected outlet must not start the servo at all, so outlet == 0 (which butler accepts,
   spec §4.5) cannot cost a traverse. */
void test_goto_rejects_an_outlet_outside_one_to_five(void) {
#if PB_PULSES_PER_GATE == 0
  TEST_IGNORE_MESSAGE("uncalibrated arm: goto is compiled out");
#else
  TEST_ASSERT_TRUE(cart_begin());
  uint32_t stops_before = sim_servo_stops();
  TEST_ASSERT_FALSE(cart_goto(0));
  TEST_ASSERT_EQUAL_STRING("range", cart_err());
  TEST_ASSERT_FALSE(cart_goto(6));
  TEST_ASSERT_EQUAL_STRING("range", cart_err());
  TEST_ASSERT_EQUAL_UINT32(0u, sim_servo_stops() - stops_before);
#endif
}

/* Four exits -- success, stall, timeout, a bus error -- looped rather than asserting only
   the happy one. Uses cart_home() only, so it runs the same in both arms. */
void test_servo_is_stopped_on_every_exit_path(void) {
  const char *names[] = { "ok", "stall", "timeout", "i2c" };
  for (unsigned k = 0; k < 4u; ++k) {
    pb_test_setup();
    TEST_ASSERT_TRUE(cart_begin());
    sim_set_screw_pulse_ms(2);
    sim_set_home_region(0, 40);
    sim_set_cart_at(600u);
    if (k == 1u) sim_set_stall(true);
    if (k == 2u) sim_set_home_region(9000u, 9001u);
    if (k == 3u) sim_set_i2c_fail(true);
    uint32_t stops = sim_servo_stops();
    (void)cart_home();
    TEST_ASSERT_EQUAL_UINT16_MESSAGE((uint16_t)PB_SERVO_STOP_US, sim_servo_us(), names[k]);
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(stops + 1u, sim_servo_stops(), names[k]);
  }
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_goto_refuses_when_pulses_per_gate_is_zero);
  RUN_TEST(test_home_from_outlet_five_actually_reaches_home);
  RUN_TEST(test_position_is_unknown_after_boot_until_homed);
  RUN_TEST(test_pos_is_never_ok_before_calibration);
  RUN_TEST(test_goto_counts_pulses_not_milliseconds);
  RUN_TEST(test_home_zeroes_the_count_only_when_the_hall_asserts);
  RUN_TEST(test_an_i2c_error_on_the_home_hall_is_unknown_not_not_home);
  RUN_TEST(test_move_deadline_holds_across_a_millis_rollover);
  RUN_TEST(test_home_that_times_out_leaves_position_unknown);
  RUN_TEST(test_stall_aborts_within_the_stall_window_and_loses_position);
  RUN_TEST(test_goto_rejects_an_outlet_outside_one_to_five);
  RUN_TEST(test_servo_is_stopped_on_every_exit_path);
  return UNITY_END();
}
