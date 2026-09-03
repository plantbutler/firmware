/* test/test_dose/test_dose.cpp
   The dose suite. Today it carries one case: that a native Unity runner links and runs.
   Tasks 3-6 and 15-19 fill it with the pump-pin, watchdog and refusal-ladder cases. */
#include <unity.h>

void setUp(void) {}
void tearDown(void) {}

static void test_the_native_runner_links_and_runs(void) {
  /* PB_CONTROLLER comes from [env:native]'s build_flags, so this also proves the flag
     reached the compiler. */
  TEST_ASSERT_EQUAL_STRING("test1", PB_CONTROLLER);
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_the_native_runner_links_and_runs);
  return UNITY_END();
}
