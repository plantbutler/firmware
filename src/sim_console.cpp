/* src/sim_console.cpp -- DEVICE ONLY, and only in [env:uno_r4_wifi_sim]. Three functions,
   and nothing else will ever be added here: no pin number, no ISR, no D6. PIN_PUMP_EN is
   not even reachable from this file -- pins.h defines it only under PB_PUMP_OWNER, and
   hal_uno.cpp is the only file in the tree that defines that, and hal_uno.cpp is not in
   this env's file set at all.
   It exists so that hal_sim.cpp needs no Arduino header of its own: §9's Arduino.h rule
   stays a FILE rule (four named files) rather than becoming a file-plus-guard rule that no
   grep can express. Step 6 widens that grep's exclusion list to name this file, in this
   task's own commit. */
#include <Arduino.h>
#include "sim_console.h"

void sim_console_begin(void) { Serial.begin(115200); }   /* §7's project-wide baud */

size_t sim_console_read(char *buf, size_t cap) {
  size_t n = 0;
  while (n < cap && Serial.available() > 0) buf[n++] = (char)Serial.read();
  return n;                                   /* bounded by cap AND by available() */
}

/* The `SIM ` prefix lives HERE, at the one point every console line in the sim binary
   passes through, and not on the host, where the suites compare whole lines. */
void sim_console_write(const char *s) {
  Serial.write("SIM ");
  Serial.write(s);
}
