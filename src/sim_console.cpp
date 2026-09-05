/* src/sim_console.cpp -- DEVICE ONLY, and only in [env:uno_r4_wifi_sim]. No D6, no ISR:
   PIN_PUMP_EN is not even reachable from this file -- pins.h defines it only under
   PB_PUMP_OWNER, and hal_uno.cpp is the only file in the tree that defines that, and
   hal_uno.cpp is not in this env's file set at all. LED_BUILTIN is the one deliberate
   exception to "no pin number" (fix round 1): the loudness LED needs REAL GPIO the same
   way the console needs a REAL UART, for the identical reason this file exists at all --
   hal_sim.cpp's hal_pin_write() only appends to the fake rig's event trace and drives no
   pin in this build (build_src_filter excludes hal_uno.cpp, the only file that ever turns
   a hal_pin_write into a real GPIO write), so a blink routed through the HAL never
   reaches an actual pin, and hal_millis() advances the FAKE clock 1 ms per call rather
   than tracking wall time -- a device arm added to those two HAL stub functions instead
   would put real pin I/O inside the fake rig, which is the opposite of the split this
   file exists to make.
   It exists so that hal_sim.cpp needs no Arduino header of its own: §9's Arduino.h rule
   stays a FILE rule (four named files) rather than becoming a file-plus-guard rule that no
   grep can express. Step 6 widens that grep's exclusion list to name this file, in this
   task's own commit. */
#include <Arduino.h>
#include "sim_console.h"

void sim_console_begin(void) {
  Serial.begin(115200);          /* §7's project-wide baud */
  pinMode(LED_BUILTIN, OUTPUT);  /* the blink's own pinMode, at begin (fix round 1) --
                                     never through hal_pin_mode(): that would configure the
                                     fake rig's event log, not a real pin, in this build */
}

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

/* 200 on / 200 off / 200 on / 1400 off: a rhythm no bench binary produces, readable
   across a room. REAL millis()/digitalWrite() -- fix round 1's fix: main.cpp used to call
   hal_millis()/hal_pin_write(PIN_LED, ...) here, which (see the file header above) drives
   no physical pin at all in this build and would in any case have moved on a
   loop-iteration count rather than wall-clock time, since hal_millis() advances the fake
   rig's own clock by 1 ms per call. Called once per loop() pass from main.cpp under
   #if PB_SIM; there is no separate "begin" call because sim_console_begin() above already
   configures LED_BUILTIN as an output. */
void sim_console_blink_tick(void) {
  const uint32_t p = millis() % 2000u;
  digitalWrite(LED_BUILTIN, (p < 200u || (p >= 400u && p < 600u)) ? HIGH : LOW);
}
