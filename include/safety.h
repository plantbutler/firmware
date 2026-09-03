/* include/safety.h — D6's whole story. READ THIS FILE FIRST.
   Includes neither link.h, Network.h nor WiFiS3.h: this layer *cannot* make a network
   call, and tools/check.sh greps for it (§3, §9). */
#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Pump idle re-asserted, then the dog fed — in that order, in one function, with nothing
   between them (§2.4). Called at the top of loop(), inside safety_wait_ms()'s loop, and
   inside dose_run()'s loop. Nowhere else. */
void safety_tick(void);

/* A bounded wait that calls safety_tick() on every iteration. Every loop in the program
   that can iterate over an I2C transfer, a modem call or a millisecond of wall clock
   uses this or its own safety_tick() (§3). */
void safety_wait_ms(uint32_t ms);

bool safety_dosing(void);

/* The only production writer is dose_run() (task 17), which sets g_dosing directly; this
   exists so the host suites can reproduce a dose in flight — sensors.cpp's recovery guard
   (§2.13) has to be testable before dose_run() is written. */
void safety_set_dosing(bool on);

/* §2.10. PB_FLOAT_OK_SAMPLES (3) consecutive OK readings to GRANT; ONE bad sample refuses
   IMMEDIATELY. The asymmetry is the whole design: refusing on one bad sample is safe,
   granting on one is not, because D5 runs up to a metre to the reservoir alongside a 12 V
   pump lead. The wait between samples is safety_wait_ms(), which calls safety_tick() on
   every iteration, so the dog is fed and the pump idle-re-asserted throughout. */
bool safety_float_ok_debounced(void);
