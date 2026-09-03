/* include/config.h — every tunable and measured constant, with the citation that
   justifies it. A constant without one is a bug report waiting to be written.
   Source: docs/superpowers/specs/2026-09-03-bench-sketch-design.md §7. */
#pragma once

/* ---- fixed by the wiring package ---- */
#define PB_OUTLETS               5
#define PB_CHANNELS              6      /* C0-C4 moisture, C5 LDR */
#define PB_CANARY_CHANNEL       15      /* unwired by the mux table: the stuck-mux canary (§5) */
/* analogReadResolution(bits) does NOT change the hardware width: ANALOG_READ_HARDWARE_
   RESOLUTION_FIXED is defined (analog.cpp:599), the hardware is fixed at open time to
   BSP_FEATURE_ADC_MAX_RESOLUTION_BITS = 14 (:34-45), and adcConvert map()s hw -> requested
   (:495). So 14 is an identity map TODAY -- but the default requested value is 10 (:11), and
   a core bump that changed the fixed width would silently rescale every raw count on the
   wire. hal_begin() therefore ASSERTS analogReadResolution() == PB_ADC_BITS after setting it
   (:698), and `status` prints both the requested and the hardware width. */
#define PB_ADC_BITS             14

/* ---- the watchdog. PCLKB = 24 MHz (bsp_clock_cfg.h:8,14: HOCO 48 / PCLKB_DIV 2).
   RL_16384 * PR_8192 / (PCLKB/1000) = 16384*8192/24000 = 5592 ms (WDT.cpp:105-113).
   We use the wdt_cfg_t overload for stop_control = DISABLE (WDT.cpp:67, r_wdt_api.h:115-116),
   and that overload NEVER assigns _timeout (WDT.cpp:32-46), so getTimeout() would
   return 0 on a running dog. hal_wdt_granted() computes this number instead.
   The counter is a DOWN-counter at PCLKB/8192 = 2929.7 Hz = 2.93 counts/ms, which is
   what hal_wdt_alive() measures across an UNFED window (§2.5). ---- */
#define PB_WDT_GRANTED_MS     5592
#define PB_WDT_PROBE_MS         40     /* the ONE unfed window in the program. 40/5592 = 0.7%. */
#define PB_WDT_PROBE_MIN_COUNTS 58     /* half of 40 * 2929.7/1000 = 117: no false negative on
                                          jitter, no false positive on a frozen counter */

/* ---- the network. MODEM_TIMEOUT default is 10000 (Modem.h:12), nearly twice the
   WDT window. The worst net_poll() pass is CONNECT = 2 AT commands (_BEGINCLIENT +
   _CLIENTCONNECT), which holds ONLY because (a) sock_close() always ran in a PRIOR
   pass and left _sock == -1 (WiFiClient.cpp:31,217), (b) every error exit routes
   through the SOCK_CLOSE state instead of closing inline, and (c) sock_read() is a
   bare client.read() -- no available(), no connected(). See §3's per-pass table.
   2 * 1200 = 2400 < 5592, and 2400 + SLACK = 4400 is what setup() asserts. ---- */
#define PB_NET_STEP_MS        1200
#define PB_NET_SLACK_MS       2000     /* setup() asserts granted >= 2*STEP + SLACK */
#define PB_NET_DEADLINE_MS    5000     /* RECV: also the closed-socket detector, since the FSM
                                          never calls connected() (2 ATs, WiFiClient.cpp:224-238) */
#define PB_NET_BACKOFF_MS     { 2000, 4000, 8000, 16000, 30000 }
#define PB_RETRY_DEADLINE_MS 30000     /* << RETRY_WINDOW_S = 300 (butler.py RETRY_WINDOW_S).
                                          Measured on g_t_ms (RAW millis), never on g_t_wire. */

/* ---- the dose. Protocol ceilings MATCH butler.py: MAX_DOSE_ML 1000, MAX_CAP_S 60.
   The RIG ceiling is smaller, per DECISIONS #7's "a reservoir small enough that a full
   dump is a mop-up" -- and butler does NOT know about it, which is a going-live
   precondition (§4.6) and an owed backend change (§16.5.8). ---- */
#define PB_DOSE_MAX_ML        1000     /* == MAX_DOSE_ML: protocol parity */
#define PB_DOSE_RIG_MAX_ML     250     /* what dose_run() actually enforces */
#define PB_DOSE_CAP_MS_MAX   60000     /* == MAX_CAP_S * 1000 */
#define PB_DOSE_MIN_GAP_MS   10000     /* every caller. See §15.3 for why not 60 s. */
#define PB_BOOT_GAP_MS       10000     /* DECISIONS #5 "minimum gap since boot" */
#define PB_BOOT_HOME_MS      15000
#define PB_POS_RECHECK_MS     1000     /* live expander read inside the dose loop */
#define PB_COAST_MS           2000     /* impeller spin-down is not a leak */
/* cap_for(ml) = min(60, ml//FLOW_FLOOR_ML_S + 5) with FLOW_FLOOR_ML_S = 20 (butler.py) is a
   GUESS. At a real 30 ml/s, cap_for(500) authorises 1.8x the requested water, so the cap is
   not a bound exactly when the meter -- the thing it stands in for -- has failed. Once 7b
   measures the rate, this clamps the cap to 2x the requested millilitres. It is ALSO the
   constant -DPB_DOSE_BY_TIME=1 uses; there is no second ml/s constant (§6). */
#define PB_ML_PER_S_MEASURED     0     /* 7b fills this in; 0 == clamp disabled, status says so */
#define PB_CAP_SLACK_NUM         2
#define PB_CAP_SLACK_DEN         1
/* Delivered-vs-elapsed plausibility on the DOSE_OK path (§2.8): a dose that reaches its
   target in less than 1/4 of the time the measured rate says it needs is noise, not a fast
   pump. Armed only when PB_ML_PER_S_MEASURED > 0. */
#define PB_PLAUS_NUM             4
#define PB_PLAUS_DEN             1

/* ---- flow. GUESSES until bring-up 7b. YF-S401 nominal ~5880 pulses/L; the
   -0207 floor is 0.2 L/min and the -3507 ceiling 6 L/min = 588 pulses/s.
   `cal` sets pulses_per_l at runtime so 7b needs no reflash; 7b's numbers get
   COMMITTED here. ---- */
#define PB_PULSES_PER_L_DEFAULT 5880
#define PB_PULSES_PER_L_MIN     1000   /* `cal` and dose_run() both enforce this range */
#define PB_PULSES_PER_L_MAX    20000
#define PB_PRIME_MS_DEFAULT     3000
#define PB_PRIME_LONG_MS       15000   /* `prime` EXTENDS the window; it never removes it */
#define PB_PRIME_CAP_MS        20000   /* and caps the whole dose regardless of the typed ms */
#define PB_STALL_MS_DEFAULT     1500
#define PB_PRIME_MIN_PULSES        5
#define PB_FLOW_MIN_GAP_US       500   /* ISR reject; honest: only bites above 2 kHz */
#define PB_FLOW_MAX_HZ          1200   /* 2x the meter's 588/s ceiling -> DOSE_ABORT_NOISE */
#define PB_FLOW_IDLE_MAX_HZ        2   /* non-zero with the pump off -> DOSE_REFUSED_NOISE */
/* pulses_flow_rate()'s window, and it has to be STATED or the rate rules are unfalsifiable:
   at the ISR's own 2 kHz ceiling a 250 ml target (1250 pulses at cfg=5000) is reached in
   ~625 ms, so any estimator slower than that loses the race. 100 ms wins it 6x over. */
#define PB_FLOW_RATE_WINDOW_MS   100
#define PB_HANG_MS              3000   /* `pump <ms> hang`: run the dose this long, THEN stop
                                          feeding (bring-up 7c). PB_BRINGUP only. */

/* ---- the cart. Pulses-per-gate DOES NOT EXIST YET: today's Manifold is purely
   time-based, so there is nothing to port. Bring-up 6 measures it. Until then
   cart_goto() and cart_pos_known() are COMPILED OUT to hard false (§2.15). ---- */
/* Both are -D-overridable, because [env:native_cal] defines PB_PULSES_PER_GATE=1450 on the
   command line to compile the cart's calibrated arm. Without the guard that is a
   -Wmacro-redefined on every native_cal run, and a warning nobody can fix is a warning
   everybody learns to scroll past. */
#ifndef PB_PULSES_PER_GATE
#  define PB_PULSES_PER_GATE     0     /* 6 fills this in; 0 == goto compiles to `return false` */
#endif
#ifndef PB_PULSES_HOME_TO_1
#  define PB_PULSES_HOME_TO_1    0
#endif
#define PB_MOVE_CAP_MS       45000
#define PB_STALL_WINDOW_MS    2500
#define PB_SERVO_CAP_MS      10000
#define PB_SCREW_MIN_GAP_US   2000

/* ---- the float ---- */
#define PB_FLOAT_OK_SAMPLES      3
#define PB_FLOAT_SAMPLE_MS      20
/* After this many CONSECUTIVE DOSE_REFUSED_FLOAT results, the report forces float=0 and
   err=float regardless of the report-time debounce, and water_rules goes dark. Cleared by
   any granted dose. Without it a float flapping at the waterline can grant in the report
   and refuse in the dose -- separate samples, minutes apart -- and the acked refusal sets
   the pot's cooldown and pages HIGH, forever (§2.10). */
#define PB_FLOAT_FLAP_LIMIT      3

/* ---- I2C. TwoWire's transfer timeout is a fixed 1000 ms (Wire.cpp:194, private,
   no setter); TwoWire::flush() (Wire.cpp:833) spins forever and is never called. ---- */
#define PB_I2C_FAIL_LIMIT        3
#define PB_I2C_BACKOFF_MS     5000
#define PB_I2C_RECOVER_CLOCKS    9     /* a FIXED count, never "until SDA releases" */

/* ---- the screens (lib/Screen). safety_tick() can only run BETWEEN calls into
   LiquidCrystal_I2C/u8x8, never inside them, so the unfed span is bounded by whichever
   library call is in flight, not by us.

   LCD: LiquidCrystal_I2C::send() is 2x write4bits() x 3x expanderWrite() = 6
   Wire.endTransmission() calls per command()/write()/setCursor()
   (LiquidCrystal_I2C.cpp:9-21,247-269), each independently capped at Wire's fixed
   1000 ms (Wire.cpp:194) -- so ONE such call can block up to 6000 ms with zero feeds,
   already past PB_WDT_GRANTED_MS on its own, and send/write4bits/expanderWrite are
   private: this floor cannot be lowered without forking the library.

   OLED: drawGlyph() is the finer public call in place of the opaque whole-string draw
   and whole-panel clear calls. It is NOT one Wire transaction: this OLED is
   U8X8_SSD1306_128X64_NONAME_HW_I2C, which selects u8x8_cad_ssd13xx_FAST_i2c
   (U8x8lib.h:806-808), and tracing the DRAW_TILE case through that cad
   (u8x8_d_ssd1306_128x64_noname.c's SendCmd/SendCmd/SendArg/SendArg/SendData sequence)
   gives 3 Wire.endTransmission() calls per glyph -- worst case 3000 ms, comfortably
   under the 5592 ms grant. u8x8_ClearLine()/clearDisplay() are NOT used for the same
   reason at a larger scale: ClearLine's one DRAW_TILE call asks for 16 tile-columns, and
   the fast cad's SEND_DATA case opens a fresh transaction on every one of the 16
   SendData calls regardless of the "fast" merge, for 2 + 16 = 18 transactions (18000 ms)
   per line, and clearDisplay() runs that 8 times (144 total). lib/Screen therefore
   clears the OLED by painting blank glyphs through the same bounded per-glyph path
   row() already uses (Screen.cpp).

   Either way, PB_SCREEN_PAINT_BUDGET_MS is the line: any single unit slower than this
   is treated as a wedged bus, and the panel goes permanently not-present rather than
   risk a second one. A healthy unit is low-single-digit ms; 100 ms is one to two orders
   of magnitude above that and a tiny fraction of the 5592 ms grant, so it separates
   "healthy" from "anything degraded" fast without false-tripping on bus jitter. This
   converts the LCD's unavoidable 6000 ms floor from "will recur every paint for the
   rest of the run" into "at most once, ever, per boot, self-terminating" -- it does not
   make a single LCD unit's worst case fit under the grant, which is not achievable
   without forking LiquidCrystal_I2C or changing which code feeds the watchdog, and
   neither was in scope for this fix. The OLED's per-glyph path, unlike the LCD's, is a
   real closed bound: 3000 ms is comfortably under 5592 ms even in the single worst
   case. ---- */
#define PB_SCREEN_PAINT_BUDGET_MS 100

/* ---- going live. See §4.6. Ships DEFINED; flipping it turns on backend watering. ---- */
#ifndef PB_REPORT_POS_UNKNOWN
#  define PB_REPORT_POS_UNKNOWN  1
#endif

/* ---- buffers. Main stack is 1024 B (bsp_cfg.h:26) -- every one of these is
   FILE-STATIC, never a stack local.

   PB_BODY_WORST_FIXED is the report body's worst case with `c=` and its value EXCLUDED,
   summed term by term at the maximum width the grammar permits, with every diagnostic
   clamped to six digits (§4.1):

       t=4294967295                        13
       six wired channels, chN=16383     6*10 =  60   (14-bit ADC: 5 digits)
       ten diagnostics, chNNN=999999    10*13 = 130   (clamped; unclamped it is 10*16 = 160)
       float=1                              8
       pos=unknown                         12
       ack=4294967295                      15
       flow_ml=1000                        13   (bounded by PB_DOSE_MAX_ML)
       err=resetmid                        13   (longest token is 8 chars)
                                          ---
                                          264, rounded up to 288

   The old PB_BODY_CAP of 288 therefore had NO margin at all: any PB_CONTROLLER longer
   than six characters overflowed, and the failure mode is err=txcap with the report
   DROPPED -- a silent reporting blackout, not a 400. netfsm.cpp static_asserts
   sizeof(PB_CONTROLLER) + 2 + PB_BODY_WORST_FIXED <= PB_BODY_CAP, and that
   sizeof(PB_CONTROLLER) > 1 (an empty c= is a permanent 400). ---- */
#define PB_BODY_WORST_FIXED    288
#define PB_BODY_CAP            384
#define PB_DIAG_CLAMP       999999     /* every chN diagnostic is min(v, this) on the way out:
                                          chN must be < MAX_RAW = 2**31, and a storming D2
                                          pushes ch205 past 2**31 in ~12.4 days */
#define PB_TX_CAP              768     /* PB_HDR_FIXED + HOST_NAME + BUTLER_TOKEN + PB_BODY_CAP */
#define PB_RX_CAP              256
#define PB_HDR_FIXED           128     /* the fixed part of the request line + headers */
#define PB_LINE_CAP             96
#define PB_STACK_MARGIN       2048     /* the break must stay this far below __StackLimit.
                                          _sbrk is UNCHECKED (§12), so this is the only bound
                                          that exists; crossing it latches err=heap. */

/* ---- .noinit ---- */
#define PB_NOINIT_MAGIC   0x50423031u  /* "PB01" */

/* ---- §6's seconds fallback. A by-time dose against an unmeasured rate is an unbounded
   run in a costume, so the flag and the measurement are welded together. ---- */
#if defined(PB_DOSE_BY_TIME) && (PB_ML_PER_S_MEASURED == 0)
#  error "PB_DOSE_BY_TIME needs PB_ML_PER_S_MEASURED committed non-zero by bring-up 7b \
(spec §6). Measure the rate, commit it in config.h, THEN uncomment the build flag."
#endif
