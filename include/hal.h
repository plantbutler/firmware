/* hal.h — the hardware seam. NOTHING here names an Arduino type. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#define PB_LOW 0
#define PB_HIGH 1
#define PB_IN 0
#define PB_OUT 1

uint32_t hal_millis(void);
uint32_t hal_micros(void);                 /* ADDED to the printed seam: the ISR gap rejects of
                                              §2.14 are 500 us and 2000 us, not milliseconds */
void     hal_delay_us(uint16_t us);        /* the ONLY sub-ms wait; never > 200 us */
void     hal_pin_mode(uint8_t pin, uint8_t mode);   /* NEVER for D2, D3 or D6 */
int      hal_pin_read(uint8_t pin);
void     hal_pin_write(uint8_t pin, uint8_t level);

void     hal_boot_pump_off(void);          /* ONE PFS write: direction AND level. setup()'s 1st stmt */
void     hal_pump_write(bool on);          /* the ONLY route to D6 after boot */
bool     hal_pump_level_on(void);          /* what PUMP_ON compiled to; `status` prints it */

uint16_t hal_adc_read(void);               /* A0, 14-bit */
uint8_t  hal_adc_bits(void);               /* ADDED: the HARDWARE width, read back. `status`
                                              prints adc_hw= from it and main.cpp's boot banner
                                              prints adc=%lu/%lu from it (§6, §7) */
bool     hal_adc_width_ok(void);           /* ADDED: the verdict hal_begin() drew from the
                                              analogReadResolution() readback; `status` prints
                                              adc_ok= and main.cpp latches err=adc on it (§7) */
bool     hal_i2c_write16(uint8_t addr, uint16_t bits);  /* false == bus error */
bool     hal_i2c_read16(uint8_t addr, uint16_t *bits);  /* false == bus error, NOT zero */
bool     hal_i2c_probe(uint8_t addr);
bool     hal_i2c_recover(void);            /* EXACTLY nine clocks, fixed count */
void     hal_servo_us(uint16_t us);        /* 1500 == stop; 0 == detach */

bool     hal_wdt_start(void);              /* wdt_cfg_t overload; false if the core rejected it */
uint32_t hal_wdt_granted(void);            /* OUR computed grant, never the timeout getter — §2.5 */
uint32_t hal_wdt_counter(void);            /* the raw down-counter; the sim makes it settable */
bool     hal_wdt_alive(void);              /* counter DECREASED across an UNFED window — §2.5 */
uint32_t hal_wdt_last_delta(void);         /* what the last probe measured; rides out as ch209 */
void     hal_wdt_feed(void);               /* ONE caller: safety_tick(). Not called from inside
                                              hal_wdt_alive()'s probe measurement window — the
                                              probe itself brackets that window with a feed on
                                              either side (§2.5). */

bool     hal_irq_armed(uint8_t pin);       /* IELSR scan + NVIC enable for the pin's ICU channel */
bool     hal_irq_filtered(uint8_t pin);    /* IRQCR[ch] FLTEN; `status` prints icufilter= from it */

size_t   hal_serial_read(char *buf, size_t cap);
void     hal_serial_write(const char *s);
void     hal_serial_drain(void);           /* discard the RX ring */
uint32_t hal_heap_arena(void);             /* mallinfo().arena  — break growth */
uint32_t hal_heap_ordblks(void);           /* mallinfo().ordblks — free-chunk count */
uint32_t hal_heap_break(void);             /* (uint32_t)sbrk(0) — the ONLY real heap bound (§12) */
uint32_t hal_stack_limit(void);            /* (uint32_t)&__StackLimit */
uint32_t hal_stack_hwm(void);              /* bytes of the 1024 used, from the boot paint */
uint32_t hal_boot_salt(void);              /* per-boot, from the .noinit boot counter */
void     hal_begin(void);                  /* ADC width, input pins, ISRs, Wire, servo, stack paint */
