/* test/support/harness.h -- the Unity fixture over hal_sim.
   This is a HEADER. test/support/ must not become a suite directory with no runner
   (spec §10). Task 3 fills it in; task 19 adds the contradiction bracket, task 24
   pb_net_passes(), and task 28 the on-device arm. */
#pragma once
#include <unity.h>

static inline void pb_test_setup(void)    {}
static inline void pb_test_teardown(void) {}
