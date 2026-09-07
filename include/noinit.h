/* noinit.h: the 24 bytes that survive a warm reset. SRAM survives a watchdog or RESET-button
   reset; startup clears only .bss, and fsp.ld's .noinit NOLOAD section escapes it. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "config.h"

/* The salt's stride: the 32-bit golden-ratio multiplier 0x9E3779B1. Odd, so distinct boot
   counts never collide, and large enough that t = hal_boot_salt() + hal_millis() is above
   2^31 on ordinary boots — which is why the %d ban on the wire is load-bearing. */
#define PB_BOOT_SALT_STRIDE 2654435761u

typedef struct {
  uint32_t magic;            /* PB_NOINIT_MAGIC; anything else == cold boot, zero the struct */
  uint32_t boots;            /* incremented every warm boot; feeds hal_boot_salt() */
  uint32_t cmd_high_water;   /* highest command id ever accepted: the replay guard */
  bool     dry_latched;      /* the operator's `dry on` */
  bool     contra_latched;   /* the float/flow contradiction latch */
  bool     dose_in_flight;   /* set before the ON write, cleared after the OFF write */
  uint8_t  _pad;
  uint32_t pattern;          /* the `noinit pattern` console command's spare word */
  uint32_t sum;              /* XOR of every field above; recomputed on EVERY write */
} pb_noinit_t;

extern pb_noinit_t g_nv;

uint32_t noinit_sum(const pb_noinit_t *n);
void     noinit_begin(void);     /* setup() calls this once, before anything reads a latch */
void     noinit_commit(void);    /* call after EVERY write to g_nv. No exceptions. */
bool     noinit_was_cold(void);  /* what noinit_begin() decided this boot was */

/* The reset-taken-with-the-pump-asserted verdict, latched by noinit_begin() BEFORE setup()
   clears dose_in_flight, so the fact outlives the flag. The ONLY producer of the `resetmid`
   token: status prints resetmid= from it and safety_last_err() turns it into err=resetmid. */
bool     noinit_reset_mid(void);
