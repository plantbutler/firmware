/* src/sim_console.h -- the internal seam between hal_sim.cpp/main.cpp and the device-only
   console-and-LED shim. NOT a public header: exactly three files include it, all in src/
   -- hal_sim.cpp (the device arm of the console HAL functions), sim_console.cpp (its own
   declarations) and main.cpp (fix round 1: the LED blink call site, which needs a real
   pinMode/millis/digitalWrite and so can no longer live behind hal_pin_write). It names
   no framework header of its own, which is why §9's header-location grep needs no
   exclusion for this file -- only for the .cpp beside it. */
#pragma once
#include <stddef.h>

void   sim_console_begin(void);
size_t sim_console_read(char *buf, size_t cap);
void   sim_console_write(const char *s);
/* Fix round 1: the loudness LED, driven by REAL millis()/digitalWrite() -- see
   sim_console.cpp. main.cpp calls this once per loop() pass under #if PB_SIM; there is no
   separate "begin" call because sim_console_begin() already configures the pin. */
void   sim_console_blink_tick(void);
