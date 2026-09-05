/* include/exec.h — the ack cycle, the boot self-home and the park. Spec §2.9, §2.11, §4.3.
   Four functions, not five: there is no exec_has_pending(). exec.cpp reads its own file
   static; main.cpp calls exec_pending() unconditionally; ui_fill_() selects its LCD state
   from the cart and the pump rather than from a pending flag. */
#pragma once
#include <stdint.h>

void        exec_begin(void);
void        exec_pending(void);
uint32_t    exec_last_cmd_id(void);     /* 0 before any command; the OLED's `cmd N` row */
const char *exec_last_cmd_text(void);   /* "ok 248ml" | "REF float" | 0 -- §5's row 7 */

#ifdef PB_NATIVE
#include "safety.h"
/* The dose request exec_pending() last handed to dose_run(), so a test can assert on what was
   BUILT rather than on what it timed out into. Compiled out of every device build. */
dose_req_t exec_test_last_req_(void);
#endif
