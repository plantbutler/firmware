/* hal_uno.cpp: seam 1 on the board -- the only file that includes the Arduino core header,
   owns an ISR, or writes D6. Device only: filtered out of the native and sim envs. */

#define PB_PUMP_OWNER 1     /* the only definition in the tree: pins.h gives PIN_PUMP_EN to this unit alone */

#include <Arduino.h>
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
  /* One register write: direction = output AND level = OFF, atomically. Parameter checking
     is compiled out of the BSP, so the NULL p_ctrl the core itself passes is safe.
     g_pin_cfg is the core's, declared inside extern "C": never re-declare it here. */
  R_IOPORT_PinCfg(NULL, g_pin_cfg[PIN_PUMP_EN].pin, PB_PUMP_PFS_OFF);
}

void hal_pump_write(bool on) {
  /* The same whole-word form as the boot write: R_IOPORT_PinCfg is one unconditional PFS
     register write, so every pump write re-states the direction as well as the level --
     which is what makes safety_tick()'s idle re-assert a repair of a stray pinMode on D6. */
  R_IOPORT_PinCfg(NULL, g_pin_cfg[PIN_PUMP_EN].pin, on ? PB_PUMP_PFS_ON : PB_PUMP_PFS_OFF);
}

bool hal_pump_level_on(void) { return PUMP_ON_PFS_LEVEL != 0; }

/* ------------------------------------------------------------------------ the clock */
uint32_t hal_millis(void) { return (uint32_t)millis(); }
uint32_t hal_micros(void) { return (uint32_t)micros(); }
void     hal_delay_us(uint16_t us) { delayMicroseconds(us); }

/* ------------------------------------------------------------------- ordinary pins */
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
  /* Exactly PB_I2C_RECOVER_CLOCKS clocks, a fixed count, never "until SDA releases": that
     is an unbounded loop on a bus a stuck device holds down. The refuse-while-dosing guard
     is sensors.cpp's. TwoWire's own flush spins with no bound; never call it. */
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
  wdt_cfg_t cfg = {};                             /* nine members; `= {}` so no stack
                                                     garbage reaches R_WDT_Open */
  cfg.timeout        = WDT_TIMEOUT_16384;
  cfg.clock_division = WDT_CLOCK_DIVISION_8192;
  cfg.window_start   = WDT_WINDOW_START_100;
  cfg.window_end     = WDT_WINDOW_END_0;          /* refresh legal at any point */
  cfg.reset_control  = WDT_RESET_CONTROL_RESET;
  cfg.stop_control   = WDT_STOP_CONTROL_DISABLE;  /* the reason for this overload: the
                                                     uint32_t one hardcodes ENABLE -- a
                                                     dog a future __WFI could stop */
  g_wdt_running = (WDT.begin(cfg) == 1);
  return g_wdt_running;
}

uint32_t hal_wdt_granted(void) {
  /* Never the timeout getter: under this overload it returns 0 on a running dog, because
     its field is set only by the uint32_t begin(). Compute the grant instead. */
  if (!g_wdt_running) return 0u;
  return (16384u * 8192u) / (R_FSP_SystemClockHzGet(FSP_PRIV_CLOCK_PCLKB) / 1000u);
}

uint32_t hal_wdt_counter(void) {
  /* the raw 14-bit down-counter (CNTVAL mask 0x3FFF) */
  return (uint32_t)(R_WDT->WDTSR & R_WDT_WDTSR_CNTVAL_Msk);
}

void hal_wdt_feed(void) { WDT.refresh(); }

uint32_t hal_wdt_last_delta(void) { return g_wdt_delta; }

/* The one place in the program that deliberately does not feed. Precondition: not dosing;
   hal_sim.cpp carries the same body. */
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
  /* attachInterrupt hardcodes filter_enable = false but already sets pclk_div to /64, so
     only FLTEN is missing. Set the bit directly: IRQManager::addPeripheral would allocate a
     second NVIC vector on the same ICU channel, unbounded. */
  auto cfg = getPinCfgs(pin, PIN_CFG_REQ_INTERRUPT);   /* public in the variant */
  if (cfg[0] == 0) return;                             /* not an IRQ-capable pin */
  uint8_t ch = GET_CHANNEL(cfg[0]);
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
  /* No per-channel enable bit is reachable from a pin number -- the core's IrqPool owns the
     vector -- so scan IELSR for the pin's ICU event and ask the NVIC. The ICU IRQ events
     are contiguous. */
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
  /* 0xA5 from __StackLimit up to just below the live frame: painting through our own frame
     would corrupt the return address. Nothing else bounds this stack. */
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

/* The hardware width, which analogReadResolution() does not change: the core fixes it at
   open time to BSP_FEATURE_ADC_MAX_RESOLUTION_BITS. */
uint8_t hal_adc_bits(void) { return (uint8_t)BSP_FEATURE_ADC_MAX_RESOLUTION_BITS; }

bool hal_adc_width_ok(void) { return g_adc_ok; }

void hal_begin(void) {
  /* First, and nothing before it: main.cpp may not include the core header and hal.h has no
     serial-begin, so this is where the console starts. The 50 ms settle is the USB CDC
     bridge coming up: bounded, and the one wait exempted by name from the no-delay rule. */
  Serial.begin(115200);
  delay(50);
  analogReadResolution(PB_ADC_BITS);
  /* analogReadResolution() does not change the hardware width, so 14 is an identity map
     today -- but the core's default request is 10, and a core bump that changed the fixed
     width would silently rescale every raw count on the wire. Assert the readback. */
  g_adc_ok = (analogReadResolution() == PB_ADC_BITS);
  hal_paint_stack_();
  pinMode(PIN_HALL_FLOAT, INPUT_PULLUP);
  Wire.begin();
  g_servo.attach(PIN_SERVO);
  hal_arm_pulse_pins_();
}
