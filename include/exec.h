/* exec.h: the ack cycle, the boot self-home and the park. No pending-flag accessor on
   purpose: the UI reads its state from the cart and the pump instead. */
#pragma once
#include <stdint.h>

void        exec_begin(void);
void        exec_pending(void);
uint32_t    exec_last_cmd_id(void);     /* 0 before any command; the OLED's `cmd N` row */
const char *exec_last_cmd_text(void);   /* "ok 248ml" | "REF float" | 0; the OLED's row 7 */

#ifdef PB_NATIVE
#include "safety.h"
/* The dose request exec_pending() last built, so a test can assert on what was BUILT
   rather than on what it timed out into. */
dose_req_t exec_test_last_req_(void);
#endif
