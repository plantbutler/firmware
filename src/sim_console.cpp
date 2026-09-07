/* sim_console.cpp: the sim binary's real UART and real LED, the only hardware the fake rig ever touches.
   Sim env only. It cannot reach D6: the pump pin exists only in hal_uno.cpp, which this env does not compile. */
#include <Arduino.h>
#include "sim_console.h"

void sim_console_begin(void) {
  Serial.begin(115200);          /* the project-wide baud */
  pinMode(LED_BUILTIN, OUTPUT);  /* not hal_pin_mode(): in this build that writes the fake
                                     rig's event log, not a pin */
}

size_t sim_console_read(char *buf, size_t cap) {
  size_t n = 0;
  while (n < cap && Serial.available() > 0) buf[n++] = (char)Serial.read();
  return n;                                   /* bounded by cap AND by available() */
}

/* The `SIM ` prefix lives HERE, where every console line in the sim binary passes, and
   not on the host, where the suites compare whole lines. */
void sim_console_write(const char *s) {
  Serial.write("SIM ");
  Serial.write(s);
}

/* 200 on / 200 off / 200 on / 1400 off: a rhythm no bench binary produces, readable
   across a room. REAL clock and REAL pin, not the HAL's: in this build hal_pin_write()
   only appends to the fake rig's event trace and hal_millis() advances the fake clock
   1 ms per call. Once per loop() pass; sim_console_begin() already set the pin as an output. */
void sim_console_blink_tick(void) {
  const uint32_t p = millis() % 2000u;
  digitalWrite(LED_BUILTIN, (p < 200u || (p >= 400u && p < 600u)) ? HIGH : LOW);
}
