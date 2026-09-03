/* src/hal_uno.cpp — DEVICE ONLY. The only translation unit in the project that includes
   <Arduino.h>, names a pin, owns an ISR, or writes D6. Filtered out of [env:native] and
   [env:uno_r4_wifi_sim] by build_src_filter. Spec §2.1, §2.5, §2.13, §2.14, §12. */

#define PB_PUMP_OWNER 1     /* the ONLY definition in the tree. pins.h defines
                               PIN_PUMP_EN for this translation unit and no other. */

#include <Arduino.h>        /* declares g_pin_cfg at :60-66, inside extern "C". Do NOT
                               re-declare it: a second declaration in a .cpp without
                               extern "C" is a hard linkage error (§2.1). */
#include <Wire.h>
#include <Servo.h>
#include <WDT.h>
#include <malloc.h>
#include <unistd.h>

#include "hal.h"
#include "pins.h"
#include "config.h"
#include "noinit.h"
#include "pulses.h"

extern uint32_t __StackLimit;
extern uint32_t __StackTop;

static Servo    g_servo;
static bool     g_wdt_running;
static uint32_t g_wdt_delta;
static bool     g_adc_ok;

/* ---------------------------------------------------------------- D6, and only here */
#define PB_PUMP_PFS_OFF ((uint32_t)(IOPORT_CFG_PORT_DIRECTION_OUTPUT | PUMP_OFF_PFS_LEVEL))
#define PB_PUMP_PFS_ON  ((uint32_t)(IOPORT_CFG_PORT_DIRECTION_OUTPUT | PUMP_ON_PFS_LEVEL))

void hal_boot_pump_off(void) {          /* setup()'s FIRST statement */
  /* ONE register write. Direction = output AND level = the module's OFF level, atomically.
     BSP_CFG_PARAM_CHECKING_ENABLE is 0 (bsp_cfg.h:28), so the NULL p_ctrl digital.cpp
     passes is safe, and we match it. */
  R_IOPORT_PinCfg(NULL, g_pin_cfg[PIN_PUMP_EN].pin, PB_PUMP_PFS_OFF);
}

void hal_pump_write(bool on) {
  /* THE SAME whole-word form as the boot write, deliberately. R_IOPORT_PinCfg ->
     R_BSP_PinCfg is one unconditional `PmnPFS = cfg` (bsp_io.h:391-395), so every pump
     write re-states the DIRECTION as well as the level — which is what makes
     safety_tick()'s idle re-assert a REPAIR of a stray pinMode on D6. */
  R_IOPORT_PinCfg(NULL, g_pin_cfg[PIN_PUMP_EN].pin, on ? PB_PUMP_PFS_ON : PB_PUMP_PFS_OFF);
}

bool hal_pump_level_on(void) { return PUMP_ON_PFS_LEVEL != 0; }

/* ------------------------------------------------------------------------ the clock */
uint32_t hal_millis(void) { return (uint32_t)millis(); }
uint32_t hal_micros(void) { return (uint32_t)micros(); }
void     hal_delay_us(uint16_t us) { delayMicroseconds(us); }

/* ------------------------------------------------------------------- ordinary pins */
void hal_pin_mode(uint8_t pin, uint8_t mode) {
  pinMode(pin, mode == PB_OUT ? OUTPUT : INPUT_PULLUP);
}
int  hal_pin_read(uint8_t pin) { return digitalRead(pin) == HIGH ? PB_HIGH : PB_LOW; }
void hal_pin_write(uint8_t pin, uint8_t level) { digitalWrite(pin, level ? HIGH : LOW); }

/* -------------------------------------------------------------------- ADC and I2C */
uint16_t hal_adc_read(void) { return (uint16_t)analogRead(PIN_MUX_ADC); }

bool hal_i2c_write16(uint8_t addr, uint16_t bits) {
  Wire.beginTransmission(addr);
  Wire.write((uint8_t)(bits & 0xFFu));        /* PCF8575: P0..P7 first, then P8..P15 */
  Wire.write((uint8_t)(bits >> 8));
  return Wire.endTransmission() == 0;
}

bool hal_i2c_read16(uint8_t addr, uint16_t *bits) {
  if (Wire.requestFrom((int)addr, 2) != 2) return false;
  uint16_t lo = (uint16_t)Wire.read();
  uint16_t hi = (uint16_t)Wire.read();
  *bits = (uint16_t)(lo | (uint16_t)(hi << 8));
  return true;
}

bool hal_i2c_probe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

bool hal_i2c_recover(void) {
  /* EXACTLY PB_I2C_RECOVER_CLOCKS clocks, a FIXED loop count, never an "until SDA
     releases" condition — that condition is an unbounded loop on a bus a stuck device is
     holding down. The refuse-while-dosing guard is sensors.cpp's, one line above the
     call (§2.13). Nothing here calls the unbounded flush helper of TwoWire: Wire.cpp:833
     spins with no bound. */
  Wire.end();
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, OUTPUT);
  for (uint8_t i = 0; i < PB_I2C_RECOVER_CLOCKS; ++i) {
    digitalWrite(SCL, HIGH); hal_delay_us(5);
    digitalWrite(SCL, LOW);  hal_delay_us(5);
  }
  digitalWrite(SCL, HIGH);
  Wire.begin();
  return true;
}

/* ---------------------------------------------------------------------- the servo */
void hal_servo_us(uint16_t us) {
  if (us == 0u) { g_servo.detach(); return; }
  if (!g_servo.attached()) g_servo.attach(PIN_SERVO);
  g_servo.writeMicroseconds(us);
}

/* ------------------------------------------------------------------- the watchdog */
bool hal_wdt_start(void) {
  wdt_cfg_t cfg = {};                             /* NINE members (r_wdt_api.h:147-160):
                                                     p_callback, p_context and p_extend
                                                     follow stop_control. `= {}` so no
                                                     stack garbage reaches R_WDT_Open. */
  cfg.timeout        = WDT_TIMEOUT_16384;
  cfg.clock_division = WDT_CLOCK_DIVISION_8192;
  cfg.window_start   = WDT_WINDOW_START_100;
  cfg.window_end     = WDT_WINDOW_END_0;          /* refresh legal at any point */
  cfg.reset_control  = WDT_RESET_CONTROL_RESET;
  cfg.stop_control   = WDT_STOP_CONTROL_DISABLE;  /* the reason for this overload:
                                                     the uint32_t one hardcodes ENABLE
                                                     (WDT.cpp:67) — a dog a future __WFI
                                                     could silently stop */
  g_wdt_running = (WDT.begin(cfg) == 1);
  return g_wdt_running;
}

uint32_t hal_wdt_granted(void) {
  /* NEVER the timeout getter: _timeout is assigned only inside getReload(), which only
     begin(uint32_t) calls (WDT.cpp:59,153), so under that overload the getter returns 0
     on a perfectly running dog. Compute the grant instead. */
  if (!g_wdt_running) return 0u;
  return (16384u * 8192u) / (R_FSP_SystemClockHzGet(FSP_PRIV_CLOCK_PCLKB) / 1000u);
}

uint32_t hal_wdt_counter(void) {
  /* the raw 14-bit down-counter (R7FA4M1AB.h:17811-17812, CNTVAL mask 0x3FFF) */
  return (uint32_t)(R_WDT->WDTSR & R_WDT_WDTSR_CNTVAL_Msk);
}

void hal_wdt_feed(void) { WDT.refresh(); }

uint32_t hal_wdt_last_delta(void) { return g_wdt_delta; }

/* The ONE place in the program that deliberately does not feed. Precondition: not dosing
   — it is called from setup() and as dose_run()'s first guard, after the g_dosing check.
   Spec §2.5, verbatim; hal_sim.cpp carries the same body. */
bool hal_wdt_alive(void) {
  hal_wdt_feed();                              /* start from a known reload */
  uint32_t a  = hal_wdt_counter();
  uint32_t t0 = hal_millis();
  while (hal_millis() - t0 < PB_WDT_PROBE_MS)  /* 40 ms, UNFED, pump already idle-OFF */
    hal_pump_write(false);                     /* the safety half of safety_tick(), without the feed */
  uint32_t b = hal_wdt_counter();
  hal_wdt_feed();                              /* and immediately back in the window */
  g_wdt_delta = (a > b) ? (a - b) : 0u;        /* a DOWN-counter: b must be smaller */
  return g_wdt_delta >= PB_WDT_PROBE_MIN_COUNTS;
}

/* -------------------------------------------------- D2 and D3, configured ONCE */
static void isr_flow_(void)  { pulses_isr_flow(); }
static void isr_screw_(void) { pulses_isr_screw(); }

static void hal_icu_enable_filter_(uint8_t pin) {
  /* attachInterrupt hardcodes filter_enable = false (Interrupts.cpp:151) but ALREADY
     sets pclk_div = EXTERNAL_IRQ_PCLK_DIV_BY_64 (:150), so the only thing missing is
     FLTEN. Set the bit directly: IRQManager::addPeripheral would allocate a SECOND NVIC
     vector on the same ICU channel, unbounded (§2.14). */
  auto cfg = getPinCfgs(pin, PIN_CFG_REQ_INTERRUPT);   /* variant.h:32 — public */
  if (cfg[0] == 0) return;                             /* not an IRQ-capable pin */
  uint8_t ch = GET_CHANNEL(cfg[0]);                    /* variant.h:120 */
  R_ICU->IRQCR[ch] |= (uint8_t)(R_ICU_IRQCR_FLTEN_Msk
                     | (EXTERNAL_IRQ_PCLK_DIV_BY_64 << R_ICU_IRQCR_FCLKSEL_Pos));
}

static void hal_arm_pulse_pins_(void) {          /* the ONLY place D2 and D3 are configured */
  pinMode(PIN_FLOW,       INPUT_PULLUP);         /* attachInterrupt PRESERVES this */
  pinMode(PIN_HALL_SCREW, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_FLOW),       isr_flow_,  FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_HALL_SCREW), isr_screw_, FALLING);
  hal_icu_enable_filter_(PIN_FLOW);
  hal_icu_enable_filter_(PIN_HALL_SCREW);
}

bool hal_irq_armed(uint8_t pin) {
  /* There is no per-channel enable bit reachable from a pin number — the file-static
     IrqPool owns the allocated vector — so find the vector: scan IELSR for the pin's ICU
     event, then ask the NVIC. PROG_IRQ_NUM is BSP_ICU_VECTOR_MAX_ENTRIES
     (IRQManager.cpp:7); the ICU IRQ events are contiguous (bsp_elc.h:51-66). */
  auto cfg = getPinCfgs(pin, PIN_CFG_REQ_INTERRUPT);
  if (cfg[0] == 0) return false;
  uint8_t ch = GET_CHANNEL(cfg[0]);
  for (uint8_t i = 0; i < 32u; ++i) {
    if ((R_ICU->IELSR[i] & 0xFFu) != (uint32_t)(ELC_EVENT_ICU_IRQ0 + ch)) continue;
    return NVIC_GetEnableIRQ((IRQn_Type)i) != 0u;
  }
  return false;
}

bool hal_irq_filtered(uint8_t pin) {
  auto cfg = getPinCfgs(pin, PIN_CFG_REQ_INTERRUPT);
  if (cfg[0] == 0) return false;
  uint8_t ch = GET_CHANNEL(cfg[0]);
  return (R_ICU->IRQCR[ch] & R_ICU_IRQCR_FLTEN_Msk) != 0u;
}

/* --------------------------------------------------------------------- the console */
size_t hal_serial_read(char *buf, size_t cap) {
  size_t n = 0;
  while (n < cap && Serial.available() > 0) buf[n++] = (char)Serial.read();
  return n;
}
void hal_serial_write(const char *s) { Serial.write(s); }
void hal_serial_drain(void) { while (Serial.available() > 0) (void)Serial.read(); }

/* ------------------------------------------------------------ memory, and the paint */
uint32_t hal_heap_arena(void)   { struct mallinfo m = mallinfo(); return (uint32_t)m.arena; }
uint32_t hal_heap_ordblks(void) { struct mallinfo m = mallinfo(); return (uint32_t)m.ordblks; }
uint32_t hal_heap_break(void)   { return (uint32_t)sbrk(0); }
uint32_t hal_stack_limit(void)  { return (uint32_t)&__StackLimit; }

static void hal_paint_stack_(void) {
  /* 0xA5 from __StackLimit up to just below the live frame — painting through our own
     frame would corrupt the return address. §12: nothing else bounds this stack. */
  uint32_t sp;
  __asm volatile ("mov %0, sp" : "=r" (sp));
  uint32_t *p = &__StackLimit;
  uint32_t *stop = (uint32_t *)(sp - 64u);
  while (p < stop) *p++ = 0xA5A5A5A5u;
}

uint32_t hal_stack_hwm(void) {
  uint32_t *p = &__StackLimit;
  while (p < &__StackTop && *p == 0xA5A5A5A5u) p++;
  return (uint32_t)((uint8_t *)&__StackTop - (uint8_t *)p);
}

uint32_t hal_boot_salt(void) { return g_nv.boots * PB_BOOT_SALT_STRIDE; }

/* The HARDWARE width, which analogReadResolution() does not change: the core fixes it at
   open time to BSP_FEATURE_ADC_MAX_RESOLUTION_BITS (analog.cpp:34-45). `status` prints
   adc_hw= from this and main.cpp's boot banner prints adc=14/14 from it beside PB_ADC_BITS.
   ACCEPTED FALLBACK (§9's not-tested-on-the-host list): if that feature macro does not
   resolve on the installed core, return PB_ADC_BITS here and make task 11's status line read
   `adc_hw=unverifiable` instead of a number — and say which shipped in the commit message. */
uint8_t hal_adc_bits(void) { return (uint8_t)BSP_FEATURE_ADC_MAX_RESOLUTION_BITS; }

bool hal_adc_width_ok(void) { return g_adc_ok; }   /* `status` prints adc_req/adc_hw/adc_ok */

void hal_begin(void) {
  /* FIRST, and nothing before it: main.cpp may not include the Arduino header (spec §9) and
     hal.h has no serial-begin, so this is where spec §12's "Serial at 115200" lands. Without
     this line nothing on the console prints and bring-up step 0 has no banner at all. The
     short settle is the USB CDC bridge coming up; it is bounded and it is not a delay() in
     the sense §9 greps for -- see the invariant's hal_uno.cpp exemption. */
  Serial.begin(115200);
  delay(50);
  analogReadResolution(PB_ADC_BITS);
  /* §7: analogReadResolution() does NOT change the hardware width, so 14 is an identity
     map TODAY — but the default requested value is 10 (analog.cpp:11) and a core bump
     that changed the fixed width would silently rescale every raw count on the wire.
     Assert the readback (analog.cpp:698) instead of trusting it. */
  g_adc_ok = (analogReadResolution() == PB_ADC_BITS);
  hal_paint_stack_();
  pinMode(PIN_HALL_FLOAT, INPUT_PULLUP);
  Wire.begin();
  g_servo.attach(PIN_SERVO);
  hal_arm_pulse_pins_();
}
