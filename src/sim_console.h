/* src/sim_console.h -- the internal seam between hal_sim.cpp and the device-only console
   shim. NOT a public header: exactly two files include it, both in src/. It names no
   framework header of its own, which is why §9's header-location grep needs no exclusion
   for this file -- only for the .cpp beside it. */
#pragma once
#include <stddef.h>

void   sim_console_begin(void);
size_t sim_console_read(char *buf, size_t cap);
void   sim_console_write(const char *s);
