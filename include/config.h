/* config.h: every tunable and measured constant, with the derivation of each limit. */
#pragma once

/* ---- fixed by the wiring package ---- */
#define PB_OUTLETS               5
#define PB_CHANNELS              6      /* C0-C4 moisture, C5 LDR */
#define PB_CANARY_CHANNEL       15      /* unwired by the mux table: the stuck-mux canary */
/* analogReadResolution(bits) does NOT change the hardware width: the core fixes the ADC at
   14 bits at open time and map()s hardware -> requested, so 14 is an identity map today --
   but the default request is 10, and a core bump that changed the fixed width would
   silently rescale every raw count on the wire. hal_begin() therefore ASSERTS the readback
   equals PB_ADC_BITS, and `status` prints both the requested and the hardware width. */
#define PB_ADC_BITS             14

/* ---- the watchdog. PCLKB = 24 MHz (HOCO 48 / PCLKB_DIV 2, bsp_clock_cfg.h).
   RL_16384 * PR_8192 / (PCLKB/1000) = 16384*8192/24000 = 5592 ms (WDT.cpp).
   The wdt_cfg_t overload is used for stop_control = DISABLE, and that overload NEVER
   assigns _timeout, so the core's getTimeout() would return 0 on a running dog;
   hal_wdt_granted() computes this number instead.
   The counter is a DOWN-counter at PCLKB/8192 = 2929.7 Hz = 2.93 counts/ms, which is
   what hal_wdt_alive() measures across an UNFED window. ---- */
#define PB_WDT_GRANTED_MS     5592     /* the host test derives it from the registers */
#define PB_WDT_PROBE_MS         40     /* the ONE unfed window in the program. 40/5592 = 0.7%. */
#define PB_WDT_PROBE_MIN_COUNTS 58     /* half of 40 * 2929.7/1000 = 117: no false negative on
                                          jitter, no false positive on a frozen counter */

/* ---- the network. The modem's default timeout is 10000 ms (Modem.h), nearly twice the
   WDT window. The worst net_poll() pass is CONNECT = 2 AT commands (_BEGINCLIENT +
   _CLIENTCONNECT), which holds ONLY because (a) the socket close always ran in a PRIOR
   pass and left the client without a socket, (b) every error exit routes through the
   SOCK_CLOSE state instead of closing inline, and (c) the socket read is a bare
   client.read() -- no available(), no connected().
   2 * 1200 = 2400 < 5592, and 2400 + SLACK = 4400 is what setup() asserts. ---- */
#define PB_NET_STEP_MS        1200
#define PB_NET_SLACK_MS       2000     /* setup() asserts granted >= 2*STEP + SLACK */
#define PB_NET_DEADLINE_MS    5000     /* RECV: also the closed-socket detector, since the FSM
                                          never calls connected() (2 ATs) */
#define PB_NET_BACKOFF_MS     { 2000, 4000, 8000, 16000, 30000 }
#define PB_RETRY_DEADLINE_MS 30000     /* << RETRY_WINDOW_S = 300 in butler.py.
                                          Measured on g_t_ms (RAW millis), never on g_t_wire. */

/* ---- the dose. Protocol ceilings MATCH butler.py: MAX_DOSE_ML 1000, MAX_CAP_S 60.
   The RIG ceiling is smaller -- a reservoir small enough that a full dump is a mop-up --
   and the backend keeps the same number: change both together. ---- */
#define PB_DOSE_MAX_ML        1000     /* == MAX_DOSE_ML: protocol parity */
#define PB_DOSE_RIG_MAX_ML     250     /* what the dose loop actually enforces */
static_assert(PB_DOSE_RIG_MAX_ML <= PB_DOSE_MAX_ML, "the rig ceiling is inside the protocol ceiling");
#define PB_DOSE_CAP_MS_MAX   60000     /* == MAX_CAP_S * 1000 */
#define PB_DOSE_MIN_GAP_MS   10000     /* every caller */
#define PB_BOOT_GAP_MS       10000     /* no dose in the first 10 s after boot */
#define PB_BOOT_HOME_MS      15000
#define PB_POS_RECHECK_MS     1000     /* live expander read inside the dose loop */
#define PB_COAST_MS           2000     /* impeller spin-down is not a leak */
/* cap_for(ml) = min(60, ml//FLOW_FLOOR_ML_S + 5) with FLOW_FLOOR_ML_S = 20 (butler.py) is a
   GUESS. At a real 30 ml/s, cap_for(500) authorises 1.8x the requested water, so the cap is
   not a bound exactly when the meter -- the thing it stands in for -- has failed. Once the
   rate is measured, this clamps the cap to 2x the requested millilitres. */
/* -D-overridable, same reason and same shape as PB_PULSES_PER_GATE below: [env:native_measured]
   defines this at 30 on the command line to compile the measured-clamp arm. Without the guard
   this unconditional #define silently WINS over the command-line -D (GCC keeps the later
   definition in translation order), and the clamp arm compiles OUT, untested. */
#ifndef PB_ML_PER_S_MEASURED
#  define PB_ML_PER_S_MEASURED   0     /* measured on the bench; 0 == clamp disabled, status says so */
#endif
#define PB_CAP_SLACK_NUM         2
#define PB_CAP_SLACK_DEN         1
/* Delivered-vs-elapsed plausibility on the DOSE_OK path: a dose that reaches its target in
   less than 1/4 of the time the measured rate says it needs is noise, not a fast pump.
   Armed only when PB_ML_PER_S_MEASURED > 0. */
#define PB_PLAUS_NUM             4
#define PB_PLAUS_DEN             1

/* ---- flow. GUESSES until measured. YF-S401 nominal ~5880 pulses/L; the -0207 floor is
   0.2 L/min and the -3507 ceiling 6 L/min = 588 pulses/s. `cal` sets pulses_per_l at
   runtime so measuring needs no reflash; the measured numbers get COMMITTED here. ---- */
#define PB_PULSES_PER_L_DEFAULT 5880
#define PB_PULSES_PER_L_MIN     1000   /* `cal` and the dose ladder both enforce this range */
#define PB_PULSES_PER_L_MAX    20000
#define PB_PRIME_MS_DEFAULT     3000
#define PB_PRIME_LONG_MS       15000   /* `prime` EXTENDS the window; it never removes it */
#define PB_PRIME_CAP_MS        20000   /* and caps the whole dose regardless of the typed ms */
#define PB_STALL_MS_DEFAULT     1500
#define PB_PRIME_MIN_PULSES        5
#define PB_FLOW_MIN_GAP_US       500   /* ISR reject; honest: only bites above 2 kHz */
#define PB_FLOW_MAX_HZ          1200   /* 2x the meter's 588/s ceiling -> DOSE_ABORT_NOISE */
#define PB_FLOW_IDLE_MAX_HZ        2   /* non-zero with the pump off -> DOSE_REFUSED_NOISE */
/* pulses_flow_rate()'s window, STATED so the rate rules are falsifiable: at the ISR's own
   2 kHz ceiling a 250 ml target (1250 pulses at cfg=5000) is reached in ~625 ms, so any
   estimator slower than that loses the race. 100 ms wins it 6x over. */
#define PB_FLOW_RATE_WINDOW_MS   100
#define PB_HANG_MS              3000   /* `pump <ms> hang`: run the dose this long, THEN stop
                                          feeding the watchdog. Bring-up build only. */

/* ---- the cart. Pulses-per-gate is unmeasured; until it is, cart_goto() and
   cart_pos_known() are COMPILED OUT to hard false. ---- */
/* Both are -D-overridable, because [env:native_cal] defines PB_PULSES_PER_GATE=1450 on the
   command line to compile the cart's calibrated arm. Without the guard that is a
   -Wmacro-redefined on every native_cal run, and a warning nobody can fix is a warning
   everybody learns to scroll past. */
#ifndef PB_PULSES_PER_GATE
#  define PB_PULSES_PER_GATE     0     /* measured at bring-up; 0 == goto compiles to `return false` */
#endif
#ifndef PB_PULSES_HOME_TO_1
#  define PB_PULSES_HOME_TO_1    0
#endif
/* The binary that runs unattended may not ship with an uncalibrated gate pitch.
   PB_ALLOW_UNCALIBRATED sits in [env:uno_r4_wifi]'s build_flags until the measured
   PB_PULSES_PER_GATE is committed, and is deleted in that same commit: an escape hatch
   that has to be deleted by hand, and that `status` prints, is what lets the device build
   stay green meanwhile. */
#if !defined(PB_BRINGUP) && !defined(PB_SIM) && !defined(PB_NATIVE) && \
    !defined(PB_ALLOW_UNCALIBRATED)
#  if PB_PULSES_PER_GATE == 0
#    error "PB_PULSES_PER_GATE is 0: bring-up 6 has not committed the gate pitch. Run it, \
commit the number, and delete -DPB_ALLOW_UNCALIBRATED from [env:uno_r4_wifi]."
#  endif
#endif
#define PB_MOVE_CAP_MS       45000
#define PB_STALL_WINDOW_MS    2500
#define PB_SERVO_CAP_MS      10000
#define PB_SCREW_MIN_GAP_US   2000

/* A continuous-rotation servo needs one value each side of the 1500 us stop point.
   1600/1400 is a deliberate slow crawl: if the sign is wrong, bring-up sees a slow wrong
   direction rather than a fast one. If forward turns out to be the other way, SWAP THESE
   TWO VALUES -- do not add a sign flip inside cart.cpp, or the direction of travel stops
   being readable from one place. */
#define PB_SERVO_FWD_US       1600   /* toward gate 1..5 */
#define PB_SERVO_REV_US       1400   /* toward home */
#define PB_SERVO_STOP_US      1500   /* hal_servo_us(1500) == stop (hal.h) */

/* ---- the float ---- */
#define PB_FLOAT_OK_SAMPLES      3
#define PB_FLOAT_SAMPLE_MS      20
/* After this many CONSECUTIVE DOSE_REFUSED_FLOAT results, the report forces float=0 and
   err=float regardless of the report-time debounce, and water_rules goes dark. Cleared by
   any granted dose. Without it a float flapping at the waterline can grant in the report
   and refuse in the dose -- separate samples, minutes apart -- and the acked refusal sets
   the pot's cooldown and pages HIGH, forever. */
#define PB_FLOAT_FLAP_LIMIT      3

/* ---- I2C. The bus library's transfer timeout is a fixed 1000 ms (private, no setter);
   its flush() spins forever and is never called. ---- */
#define PB_I2C_FAIL_LIMIT        3
#define PB_I2C_BACKOFF_MS     5000
#define PB_I2C_RECOVER_CLOCKS    9     /* a FIXED count, never "until SDA releases" */

/* ---- the screens (lib/Screen). safety_tick() can only run BETWEEN calls into
   LiquidCrystal_I2C/u8x8, never inside them, so the unfed span is bounded by whichever
   library call is in flight, not by us.

   LCD: one command()/write()/setCursor() is 2x write4bits() x 3x expanderWrite() = 6
   Wire.endTransmission() calls, each capped at the bus's fixed 1000 ms -- so ONE call can
   block 6000 ms with zero feeds, past PB_WDT_GRANTED_MS on its own, and those helpers
   are private: the floor cannot be lowered without forking the library.

   OLED: drawGlyph() on this panel (U8X8_SSD1306_128X64_NONAME_HW_I2C, the fast cad) is
   3 Wire.endTransmission() calls per glyph -- worst case 3000 ms, under the grant.
   ClearLine()/clearDisplay() are NOT used: ClearLine asks for 16 tile-columns and the fast
   cad opens a fresh transaction for every one, 2 + 16 = 18 transactions (18000 ms) per
   line, and clearDisplay() runs that 8 times. lib/Screen therefore clears the OLED by
   painting blank glyphs through the same bounded per-glyph path.

   PB_SCREEN_PAINT_BUDGET_MS is the line: any single unit slower than this is treated as a
   wedged bus, and the panel goes permanently not-present rather than risk a second one.
   A healthy unit is low-single-digit ms, so 100 ms separates "healthy" from "degraded"
   without false-tripping on bus jitter. It turns the LCD's 6000 ms floor from "every
   paint" into "at most once per boot"; it does not make that single worst case fit under
   the grant. ---- */
#define PB_SCREEN_PAINT_BUDGET_MS 100

/* ---- going live. Ships DEFINED; flipping it turns on backend watering. ---- */
#ifndef PB_REPORT_POS_UNKNOWN
#  define PB_REPORT_POS_UNKNOWN  1
#endif

/* ---- buffers. Main stack is 1024 B (bsp_cfg.h) -- every one of these is FILE-STATIC,
   never a stack local.

   PB_BODY_WORST_SUM is the report body's worst case with `c=` and its value EXCLUDED,
   summed term by term at the maximum width the grammar permits, with every diagnostic
   clamped to six digits; PB_BODY_WORST_FIXED is that sum rounded up to the next multiple
   of 32, derived below rather than typed. PB_DIAG_CHANNELS is the count on the third
   line, and report.cpp static_asserts its diag array against it, so the array cannot grow
   without this sum being re-done. ch210 (the flap latch) and ch211 (the dry latch) are
   summed at the clamp width like ch207/ch208, not at their real 0/1 width:

       t=4294967295                        13
       six wired channels, chN=16383     6*10 =  60   (14-bit ADC: 5 digits)
       twelve diagnostics, chNNN=999999 12*13 = 156   (clamped; unclamped, a uint32_t's ten
                                                       digits make it 12*17 = 204)
       float=1                              8
       pos=unknown                         12
       ack=4294967295                      15
       flow_ml=1000                        13   (bounded by PB_DOSE_MAX_ML)
       err=resetmid                        13   (longest token is 8 chars)
       the trailing newline                 1
                                          ---
                                          291, rounded up to 320

   THE SUM IS PINNED TO THE BYTES: test_report builds a body with every field at its
   maximum, swaps in the diagnostic block at full clamp width (three producers are host
   constants and four are booleans, so report_build() alone cannot reach it), and asserts
   the total EQUALS PB_BODY_WORST_SUM -- a thirteenth channel, a wider token or a
   hand-edit of the number below fails that case. A silent overflow here is err=txcap with
   the report DROPPED, a reporting blackout rather than a 400. PB_CONTROLLER_WIRE is what
   c= can cost on the wire: 0..255 is three characters whatever anybody sets. netfsm.cpp
   and report.cpp static_assert PB_CONTROLLER_WIRE + 2 + PB_BODY_WORST_FIXED <=
   PB_BODY_CAP, and that PB_CONTROLLER is inside 0..255 -- a RANGE, not a "not empty",
   because board 0 is a real board and is the one the app fills in by default. ---- */
#define PB_CONTROLLER_WIRE       3     /* strlen("255"); c= is 0..255 (butler.MAX_CONTROLLER) */
#define PB_DIAG_CHANNELS        12     /* ch200..ch211: the "twelve diagnostics" line above */
#define PB_BODY_WORST_SUM      291     /* the table above, to the byte; test_report pins it */
#define PB_BODY_WORST_FIXED    (((PB_BODY_WORST_SUM + 31) / 32) * 32)   /* 320 */
#define PB_BODY_CAP            384
#define PB_DIAG_CLAMP       999999     /* every chN diagnostic is min(v, this) on the way out:
                                          chN must be < MAX_RAW = 2**31, and a storming D2
                                          pushes ch205 past 2**31 in ~12.4 days */
#define PB_TX_CAP              768     /* PB_HDR_FIXED + HOST_NAME + BUTLER_TOKEN + PB_BODY_CAP */
#define PB_RX_CAP              256
#define PB_HDR_FIXED           128     /* the fixed part of the request line + headers */
#define PB_LINE_CAP             96
#define PB_STACK_MARGIN       2048     /* the break must stay this far below __StackLimit.
                                          _sbrk is UNCHECKED, so this is the only bound
                                          that exists; crossing it latches err=heap. */

/* ---- .noinit ---- */
#define PB_NOINIT_MAGIC   0x50423031u  /* "PB01" */
