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

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_goto_refuses_when_pulses_per_gate_is_zero);
  RUN_TEST(test_home_from_outlet_five_actually_reaches_home);
  return UNITY_END();
}
