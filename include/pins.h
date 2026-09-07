/* pins.h: the wiring contract as constants; nothing outside this header may hold a pin
   number. Source of truth: cad/wiring/nets.py and the pin table it generates. */
#pragma once

/* The relay is active-high. Its polarity used to be a build flag with no default, so that a
   board could not be flashed before someone had READ THE MODULE; the flag is still demanded
   for that reason, and `status` prints the compiled level so bring-up can confirm it. */
#ifndef PB_RELAY_ACTIVE_HIGH
#  error "Define PB_RELAY_ACTIVE_HIGH in platformio.ini build_flags after you have READ THE MODULE."
#endif

/* ---- direct pins (cad/wiring/README.md pin table) ---- */
#define PIN_FLOW           2   /* YF-S401 pulse. 1 k series at the board; INPUT_PULLUP is the
                                  software half of the missing pull, R4 is the other */
#define PIN_HALL_SCREW     3   /* WPSE313 screw hall, interrupt pin: an edge missed while a mux
                                  sits on another channel is lost cart position */
#define PIN_HALL_FLOAT     5   /* WPSE313 float hall, 10 k pull-up R2. Direct pin, NOT the
                                  expander: shortest path for the safety input */
#define PIN_SERVO          9   /* SG90 continuous servo, manifold 1. A servo needs its 50 Hz
                                  train unbroken, so it is a direct pin, never the expander.
                                  Check the wiring bring-up note before trusting this number. */
#define PIN_MUX_ADC       14   /* A0 == PIN_A0 == 14u (variants/UNOWIFIR4/pins_arduino.h).
                                  MUX1 SIG lands here. */

/* ---- I2C addresses (A4/A5; the bus that gates the pump) ---- */
#define I2C_ADDR_EXPANDER  0x20  /* PCF8575, A0-A2 low. P0..P3 = MUX1 S0..S3,
                                    P4 = HALL_HOME (input, 10 k pull-up R3) */
#define I2C_ADDR_LCD       0x27  /* LCD1602 backpack, A0-A2 bridged ... */
#define I2C_ADDR_LCD_ALT   0x3F  /* ... or open: the same PCF8574 backpack ships either way.
                                    Screen::probe() asks 0x27 first, then 0x3F, and keeps
                                    whichever answers. */
#define I2C_ADDR_OLED      0x3C  /* SensorKit OLED (u8x8) */

/* ---- D6, and only for its owner ----
   PIN_PUMP_EN exists ONLY in the translation unit that owns the pump, src/hal_uno.cpp. In
   a sim or host build no translation unit can even NAME the pin: D6 is never made an
   output, stays an input from reset, and R1 holds the relay's OFF level in hardware even
   with 12 V on COM. */
#ifdef PB_PUMP_OWNER
#  define PIN_PUMP_EN 6
   /* PFS level bits, not BSP_IO_LEVEL_*: these are ORed into a whole-word PmnPFS write.
      IOPORT_CFG_PORT_OUTPUT_HIGH = 0x1, IOPORT_CFG_PORT_OUTPUT_LOW = 0 (r_ioport_api.h). */
#  define PUMP_ON_PFS_LEVEL  IOPORT_CFG_PORT_OUTPUT_HIGH
#  define PUMP_OFF_PFS_LEVEL 0
#endif
