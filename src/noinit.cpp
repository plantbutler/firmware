/* src/noinit.cpp — the warm-reset struct's storage and its four rules. Spec §2.3. */
#include "noinit.h"
#include <string.h>

/* On the board the struct must land in fsp.ld's uncleared NOLOAD section. On the host
   there is no such section and an ordinary definition is the whole model — which is why
   survival across a REAL warm reset is a measurement bring-up 7c' owes, not a claim this
   file makes. */
#ifdef PB_NATIVE
pb_noinit_t g_nv;
#else
pb_noinit_t g_nv __attribute__((section(".noinit")));
#endif

static bool g_was_cold;
static bool g_reset_mid;      /* §2.3's verdict, latched once per boot by noinit_begin() */

uint32_t noinit_sum(const pb_noinit_t *n) {
  return n->magic ^ n->boots ^ n->cmd_high_water ^ n->pattern
       ^ ((uint32_t)n->dry_latched)
       ^ ((uint32_t)n->contra_latched   << 8)
       ^ ((uint32_t)n->dose_in_flight   << 16);
}

void noinit_commit(void) { g_nv.sum = noinit_sum(&g_nv); }

bool noinit_was_cold(void)  { return g_was_cold; }
bool noinit_reset_mid(void) { return g_reset_mid; }

void noinit_begin(void) {
  g_reset_mid = false;
  if (g_nv.magic != PB_NOINIT_MAGIC || g_nv.sum != noinit_sum(&g_nv)) {
    /* a cold boot, or a partial clobber: start clean */
    memset(&g_nv, 0, sizeof g_nv);
    g_nv.magic = PB_NOINIT_MAGIC;
    g_nv.boots = 1u;
    g_was_cold = true;
  } else {
    /* dry_latched, contra_latched, cmd_high_water and pattern are KEPT untouched.
       dose_in_flight is kept SET as well, and the VERDICT is latched here rather than left
       for setup() to re-derive: setup() clears the flag (task 12), and g_reset_mid is what
       survives that clear so `status` and safety_last_err() can still see the fact. */
    if (g_nv.dose_in_flight) { g_nv.dry_latched = true; g_reset_mid = true; }
    g_nv.boots++;
    g_was_cold = false;
  }
  noinit_commit();
}
