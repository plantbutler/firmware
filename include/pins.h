/* include/pins.h — the wiring contract as constants. Source of truth for the numbers:
   cad/wiring/nets.py and the pin table it generates in cad/wiring/README.md.
   Spec §2.2. Nothing outside this header may hold a pin number. */
#pragma once

/* There is NO default relay polarity: a board cannot be flashed before someone has READ
   THE MODULE (bring-up 4a). `status` prints the compiled level so 4a can confirm it.
   The invariant is EXACTLY ONE of the two, not "at least one": defining both would make
   PUMP_ON_PFS_LEVEL and PUMP_OFF_PFS_LEVEL below resolve to the same value (whichever
   #ifdef wins), so PUMP_ON and PUMP_OFF would drive the pin identically and the pump's
   rest level would be a coin toss decided by macro-expansion order rather than the
   module actually in hand. Both #errors exist so neither failure mode compiles. */
#if !defined(PB_RELAY_ACTIVE_LOW) && !defined(PB_RELAY_ACTIVE_HIGH)
#  error "Relay polarity is unknown until bring-up 4a. Define PB_RELAY_ACTIVE_LOW or \
PB_RELAY_ACTIVE_HIGH in platformio.ini build_flags after you have READ THE MODULE."
#endif
#if defined(PB_RELAY_ACTIVE_LOW) && defined(PB_RELAY_ACTIVE_HIGH)
#  error "Define exactly one of PB_RELAY_ACTIVE_LOW / PB_RELAY_ACTIVE_HIGH."
#endif

/* ---- direct pins (cad/wiring/README.md pin table) ---- */
#define PIN_FLOW           2   /* YF-S401 pulse. 1 k series at the board; INPUT_PULLUP is the
                                  software half of the missing pull, R4 is the other (§2.14) */
#define PIN_HALL_SCREW     3   /* WPSE313 screw hall, interrupt pin: an edge missed while a mux
                                  sits on another channel is lost cart position */
#define PIN_HALL_FLOAT     5   /* WPSE313 float hall, 10 k pull-up R2. Direct pin, NOT the
                                  expander: shortest path for the safety input */
#define PIN_SERVO          9   /* SG90 continuous servo, manifold 1. A servo needs its 50 Hz
                                  train unbroken, so it is a direct pin, never the expander.
                                  SEE THE NOTE IN STEP 6 BEFORE YOU TRUST THIS NUMBER. */
/* PIN_LED (13, the board's own LED_BUILTIN) was deleted here, task 30: the plan's original
   consumer -- hal_pin_mode(PIN_LED, ...)/hal_pin_write(PIN_LED, ...), so that exactly one
   file held the number -- was superseded by fix round 1 (task 29), which blinks through
   real pinMode()/digitalWrite()/LED_BUILTIN in the device-only src/sim_console.cpp
   instead (see include/hal.h's comment on hal_pin_mode's own deletion). Nothing reads
   PIN_LED any more; LED_BUILTIN is the Arduino core's own name for pin 13 and needs no
   second one here. */
#define PIN_MUX_ADC       14   /* A0 == PIN_A0 == 14u
                                  (framework-arduinorenesas-uno/variants/UNOWIFIR4/
                                   pins_arduino.h:15). MUX1 SIG lands here. */

/* ---- I2C addresses (A4/A5; the bus that gates the pump) ---- */
#define I2C_ADDR_EXPANDER  0x20  /* PCF8575, A0-A2 low. P0..P3 = MUX1 S0..S3,
                                    P4 = HALL_HOME (input, 10 k pull-up R3) */
#define I2C_ADDR_LCD       0x27  /* LCD1602 backpack */
#define I2C_ADDR_OLED      0x3C  /* SensorKit OLED (u8x8) */

/* ---- D6, and only for its owner ----
   PIN_PUMP_EN exists ONLY in the translation unit that defines PB_PUMP_OWNER, which is
   src/hal_uno.cpp and nothing else. In a sim or host build no translation unit can even
   NAME the pin: D6 is never made an output, stays an input from reset, and R1 holds the
   relay's OFF level in hardware even with 12 V on COM (§8). */
#ifdef PB_PUMP_OWNER
#  define PIN_PUMP_EN 6
#  ifdef PB_RELAY_ACTIVE_LOW
     /* PFS level bits, not BSP_IO_LEVEL_*: these are ORed into a whole-word PmnPFS write.
        IOPORT_CFG_PORT_OUTPUT_HIGH = 0x1, IOPORT_CFG_PORT_OUTPUT_LOW = 0
        (r_ioport_api.h:185-186). */
#    define PUMP_ON_PFS_LEVEL  0
#    define PUMP_OFF_PFS_LEVEL IOPORT_CFG_PORT_OUTPUT_HIGH
#  else
#    define PUMP_ON_PFS_LEVEL  IOPORT_CFG_PORT_OUTPUT_HIGH
#    define PUMP_OFF_PFS_LEVEL 0
#  endif
#endif
