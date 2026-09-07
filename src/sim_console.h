/* sim_console.h: the internal seam between hal_sim.cpp/main.cpp and the device-only console-and-LED shim.
   Names no framework header, so the header-location grep needs no exclusion for it. */
#pragma once
#include <stddef.h>

void   sim_console_begin(void);
size_t sim_console_read(char *buf, size_t cap);
void   sim_console_write(const char *s);
/* the loudness LED, on the REAL clock and pin; once per loop() pass, no separate begin */
void   sim_console_blink_tick(void);
