#!/usr/bin/env bash
# tools/check.sh -- the mechanical invariants of the bench-sketch spec §9, as greps.
#
# THIS IS NOT THE WHOLE CONTRACT YET. The invariants that need drop-2 and drop-3 code
# (the safety.h/dose_run/hal_pump_write greps over netfsm and ui, the single dose_run(
# in cli.cpp, the %d grep over report.cpp and netfsm.cpp, the preprocessed-cli grep and
# the safety.o/hal_uno.o hash equality) are added by task 30. See
# docs/superpowers/specs/2026-09-03-bench-sketch-design.md §9.
#
# Patterns use POSIX classes, never \b or \s: BSD grep is the default on macOS.
set -uo pipefail
cd "$(dirname "$0")/.."

SCAN=(include src lib test)      # deliberately NOT tools/: this file contains the patterns
fails=0

fail() { printf 'FAIL  %s\n' "$*" >&2; fails=$((fails + 1)); }
ok()   { printf 'ok    %s\n' "$*"; }

# count <pattern> [paths...] -> matching LINES
count() { local pat="$1"; shift; grep -rEn "$pat" "$@" 2>/dev/null | wc -l | tr -d ' '; }
# files <pattern> [paths...] -> matching FILES
files() { local pat="$1"; shift; grep -rlE "$pat" "$@" 2>/dev/null | wc -l | tr -d ' '; }
# expect <want> <got> <description>
expect() { if [ "$2" = "$1" ]; then ok "$3 ($1)"; else fail "$3: expected $1, found $2"; fi; }

# ---- invariants land here (task 13, then task 30) ----

# ---- D6: spec §2.1, §2.2 ----
# PB_PUMP_OWNER is DEFINED in exactly one file. The pattern is the DEFINITION, never the
# bare token: include/pins.h necessarily *tests* it with #ifdef, so a bare-token file count
# would be 2 forever and this invariant could never hold.
expect 1 "$(files 'define[[:space:]]+PB_PUMP_OWNER' "${SCAN[@]}")" \
  "exactly one file defines PB_PUMP_OWNER, so exactly one file gets PIN_PUMP_EN"
expect 0 "$(count 'pinMode\(PIN_PUMP_EN' "${SCAN[@]}")" \
  "pinMode never touches D6 (it would latch PODR=0 and drive the pin LOW)"
expect 2 "$(count 'R_IOPORT_PinCfg.*PIN_PUMP_EN' "${SCAN[@]}")" \
  "exactly two whole-word PFS writes to D6 (hal_boot_pump_off, hal_pump_write)"
expect 0 "$(count 'R_IOPORT_PinWrite.*PIN_PUMP_EN|digitalWrite.*PIN_PUMP_EN' "${SCAN[@]}")" \
  "no unverifiable write form on D6"

# ---- the watchdog, first two of four: spec §2.5. The other two are task 13 step 3's. ----
expect 1 "$(count 'WDT\.refresh' "${SCAN[@]}")" \
  "one refresh call site, inside hal_wdt_feed"
expect 0 "$(count 'WDT\.getTimeout' "${SCAN[@]}")" \
  "the timeout getter is never used (it returns 0 under the wdt_cfg_t overload)"

if [ "$fails" -gt 0 ]; then
  printf '\n%s invariant(s) FAILED\n' "$fails" >&2
  exit 1
fi
printf '\nall invariants hold\n'
exit 0
