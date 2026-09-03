/* include/cli.h -- the console. Bench commands always; bring-up commands under
   #if PB_BRINGUP (task 20). A flat if-chain, so cli_dispatch() is testable. */
#pragma once
#include <stdbool.h>
#include <stdint.h>

void cli_begin(void);
void cli_poll(void);
bool cli_dispatch(const char *line);
void cli_print_status(void);
void cli_printf_u32(const char *fmt, uint32_t v);
/* RSSI is the one signed figure `status` prints; %d is banned in report.cpp and
   netfsm.cpp only, and cli.cpp is outside that grep (task 24 step 17). */
void cli_printf_i32(const char *fmt, int32_t v);
