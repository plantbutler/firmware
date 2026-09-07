/* hal.h: the hardware seam; nothing here names an Arduino type. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#define PB_LOW 0
#define PB_HIGH 1
#define PB_IN 0
#define PB_OUT 1

uint32_t hal_millis(void);
uint32_t hal_micros(void);                 /* the ISR gap rejects are 500 us and 2000 us, not ms */
void     hal_delay_us(uint16_t us);        /* the ONLY sub-ms wait; never > 200 us */
int      hal_pin_read(uint8_t pin);
void     hal_pin_write(uint8_t pin, uint8_t level);

void     hal_boot_pump_off(void);          /* ONE PFS write: direction AND level. setup()'s 1st stmt */
void     hal_pump_write(bool on);          /* the ONLY route to D6 after boot */
bool     hal_pump_level_on(void);          /* what PUMP_ON compiled to; `status` prints it */

uint16_t hal_adc_read(void);               /* A0, 14-bit */
uint8_t  hal_adc_bits(void);               /* the HARDWARE width, read back; `status` prints
                                              adc_hw= from it */
bool     hal_adc_width_ok(void);           /* hal_begin()'s verdict on the width readback;
                                              main.cpp latches err=adc on it */
bool     hal_i2c_write16(uint8_t addr, uint16_t bits);  /* false == bus error */
bool     hal_i2c_read16(uint8_t addr, uint16_t *bits);  /* false == bus error, NOT zero */
bool     hal_i2c_probe(uint8_t addr);
bool     hal_i2c_recover(void);            /* EXACTLY nine clocks, fixed count */
void     hal_servo_us(uint16_t us);        /* 1500 == stop; 0 == detach */

bool     hal_wdt_start(void);              /* wdt_cfg_t overload; false if the core rejected it */
uint32_t hal_wdt_granted(void);            /* OUR computed grant, never the core's timeout getter */
uint32_t hal_wdt_counter(void);            /* the raw down-counter; the sim makes it settable */
bool     hal_wdt_alive(void);              /* counter DECREASED across an UNFED window.
                                              DESTRUCTIVE, not a getter: probes for
                                              PB_WDT_PROBE_MS with the dog deliberately unfed,
                                              then re-feeds. Each call is its own independent
                                              probe: call it ONCE into a local, THEN read
                                              hal_wdt_last_delta(). Never call it twice
                                              expecting the same verdict, and never pass it
                                              and hal_wdt_last_delta() as two arguments of one
                                              call — argument evaluation order is unspecified,
                                              so the delta could come from a DIFFERENT probe
                                              than the verdict beside it. */
uint32_t hal_wdt_last_delta(void);         /* what the LAST hal_wdt_alive() probe measured;
                                              rides out as ch209. Read it only after that call
                                              has returned and been stored. */
void     hal_wdt_feed(void);               /* ONE caller: safety_tick(). Never from inside the
                                              alive probe's window — the probe brackets that
                                              window with its own feed on either side. */

bool     hal_irq_armed(uint8_t pin);       /* IELSR scan + NVIC enable for the pin's ICU channel */
bool     hal_irq_filtered(uint8_t pin);    /* IRQCR[ch] FLTEN; `status` prints icufilter= from it */

size_t   hal_serial_read(char *buf, size_t cap);
void     hal_serial_write(const char *s);
void     hal_serial_drain(void);           /* discard the RX ring */
uint32_t hal_heap_arena(void);             /* mallinfo().arena  — break growth */
uint32_t hal_heap_ordblks(void);           /* mallinfo().ordblks — free-chunk count */
uint32_t hal_heap_break(void);             /* (uint32_t)sbrk(0) — the ONLY real heap bound */
uint32_t hal_stack_limit(void);            /* (uint32_t)&__StackLimit */
uint32_t hal_stack_hwm(void);              /* bytes of the 1024 used, from the boot paint */
uint32_t hal_boot_salt(void);              /* per-boot, from the .noinit boot counter */
void     hal_begin(void);                  /* ADC width, input pins, ISRs, Wire, servo, stack paint */
