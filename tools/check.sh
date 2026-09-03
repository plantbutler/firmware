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

# ---- D6, task 13's own closed gap: a bare pin-6 literal bypasses every PIN_PUMP_EN grep
# above. The pattern requires the call's FIRST argument to be the literal 6 (open paren,
# optional space, "6", optional space, then a comma or close paren) so it never fires on
# an unrelated "6" -- a channel count, an array size, ch206 -- or on a different pin whose
# number happens to start with 6 (pinMode(60,... has a "0" where this pattern requires a
# terminator). Nothing in the tree does this today; the macro is what every write site uses.
expect 0 "$(grep -rEn '(pinMode|digitalWrite)\([[:space:]]*6[[:space:]]*[,)]' "${SCAN[@]}" \
              2>/dev/null | wc -l | tr -d ' ')" \
  "no bare-literal pin 6 write bypassing PIN_PUMP_EN (pinMode(6,...) or digitalWrite(6,...))"

# ---- D2/D3: spec §2.14. Both hits must be inside hal_arm_pulse_pins_. ----
pp_all=$(count 'pinMode\((PIN_FLOW|PIN_HALL_SCREW)' "${SCAN[@]}")
pp_fn=$(awk '/hal_arm_pulse_pins_/ {f=1}
             f && /pinMode\((PIN_FLOW|PIN_HALL_SCREW)/ {c++}
             f && /^}/ {f=0}
             END {print c+0}' src/hal_uno.cpp 2>/dev/null)
expect "$pp_all" "$pp_fn" \
  "every pinMode on D2/D3 is inside hal_arm_pulse_pins_ (a later one detaches the interrupt)"

# ---- the watchdog, first two of four: spec §2.5. The other two are task 13 step 3's. ----
expect 1 "$(count 'WDT\.refresh' "${SCAN[@]}")" \
  "one refresh call site, inside hal_wdt_feed"
expect 0 "$(count 'WDT\.getTimeout' "${SCAN[@]}")" \
  "the timeout getter is never used (it returns 0 under the wdt_cfg_t overload)"

# ---- the watchdog, continued: spec §2.4, §2.5 ----
expect 1 "$(count 'hal_wdt_feed\(' src/safety.cpp)" \
  "safety_tick is the ONLY feeder in safety.cpp"
# hal.h's declaration is excluded: a seam has to declare what it seams.
expect 0 "$(grep -rEn 'hal_wdt_feed\(' "${SCAN[@]}" \
              --exclude=safety.cpp --exclude=hal_uno.cpp --exclude=hal_sim.cpp \
              --exclude=hal.h 2>/dev/null | wc -l | tr -d ' ')" \
  "nothing outside safety.cpp and the two HALs feeds the dog"

# ---- lib/Screen: spec §5 ----
# spec §5: TwoWire::flush() spins with no bound and is never called by us.
expect 0 "$(count 'Wire\.flush' "${SCAN[@]}")" \
  "no Wire flush anywhere"

# spec §5: the library's row printer offers no hook between characters, so an LCD row is
# painted one character at a time with safety_tick() between them.
expect 0 "$(count 'lcd\.print|lcd\.println' "${SCAN[@]}")" \
  "no library row printer on the LCD"

# task 13's own closed gap: task 9 replaced the OLED's opaque whole-string draw and
# whole-panel/per-row clear calls with the bounded per-glyph path in Screen::clear()/
# row() specifically because those calls run 144 (whole-panel) or 18 (per-row) unfed
# Wire transactions against a 5592 ms watchdog grant. Nothing greps for their return
# today, so a future edit could reinstate any of them and reintroduce a span that long
# without a single invariant noticing. Scoped to lib/Screen only: the library names are
# real English words elsewhere (config.h's own accounting of why they are NOT called
# describes them without spelling any of the three contiguously -- see that file).
expect 0 "$(count 'drawString|clearDisplay|clearLine' lib/Screen)" \
  "the opaque whole-string draw and whole-panel/per-row clear calls do not creep back into lib/Screen"

# ---- blocking, buffers and formatting: spec §3, §9, §12 ----
# lib/Manifold/src/Manifold.cpp and lib/Network/src/Network.cpp are the pre-bench-sketch
# sketch: task 1 already orphaned both (nothing under src/ or lib/ includes either header
# any more, so the LDF never builds them -- "main.cpp: a bare sketch" says so directly),
# but neither file is deleted yet. Task 14 deletes Manifold.cpp, and the drop-3 seam-2 task
# deletes Network.cpp; each should remove its own --exclude below in the same commit, the
# way task 14 must also remove lib/Manifold/library.json's srcFilter once the file it
# excludes is gone (see 190b56d). Excluded by FILE NAME, never by directory, so a real file
# added to either library -- cart.cpp today, link_wifi.cpp later -- is still checked.
expect 0 "$(grep -rEn '(^|[^[:alnum:]_])delay\(' "${SCAN[@]}" \
              --exclude=hal_uno.cpp --exclude=Manifold.cpp --exclude=Network.cpp \
              2>/dev/null | wc -l | tr -d ' ')" \
  "no unbounded blocking wait outside hal_uno.cpp's power-on settles"
expect 0 "$(grep -rEn '%[0-9.]*[fgeFGE]([^[:alnum:]]|$)' "${SCAN[@]}" --exclude=Network.cpp \
              2>/dev/null | wc -l | tr -d ' ')" \
  "no float formatting anywhere (newlib float printf is the deepest stack consumer)"
expect 0 "$(grep -rEn 'for[[:space:]]*\([[:space:]]*;[[:space:]]*;[[:space:]]*\)|while[[:space:]]*\([[:space:]]*(true|1)[[:space:]]*\)' \
              "${SCAN[@]}" --exclude=safety.cpp --exclude=Network.cpp 2>/dev/null | wc -l | tr -d ' ')" \
  "the program's only intentional unbounded loop is in the function that owns D6"
# `malloc[[:space:]]*\(` -- the CALL, never the bare word. src/hal_uno.cpp writes
# `#include <malloc.h>` for mallinfo(), which is how the heap diagnostics of ch200/ch201
# exist at all; a bare-word pattern would match that include and fail this check forever.
expect 0 "$(grep -rEn 'String|std::map|std::string|(^|[^[:alnum:]_])new([^[:alnum:]_]|$)|malloc[[:space:]]*\(' \
              include src test lib/Manifold 2>/dev/null | wc -l | tr -d ' ')" \
  "no dynamic allocation outside lib/Network and lib/Screen"
# Task 29 widens this ONE exclusion to sim_console.cpp, the device-only console shim the
# sim binary needs, and records it as a deviation from §9's table in its own commit. It is
# the only widening this line ever takes. Manifold.cpp is excluded for the same reason as
# the block above -- it is dead, orphaned, pre-bench-sketch code task 14 deletes -- and the
# same rule applies: whoever deletes it removes this --exclude too.
expect 0 "$(grep -rEn 'Arduino\.h' include src lib/Manifold \
              --exclude=hal_uno.cpp --exclude=Manifold.cpp 2>/dev/null | wc -l | tr -d ' ')" \
  "the Arduino header lives only in hal_uno.cpp, lib/Network and lib/Screen"
expect 0 "$(count 'WiFi\.ping' "${SCAN[@]}")" \
  "ping is never called (it resets the modem timeout to 10 s)"

# ---- the two binaries: spec §6 ----
# SCOPED TO src AND lib, not "${SCAN[@]}", and that is a deliberate deviation from §9's
# table recorded in this task's commit message and again in task 20 step 13. Two files
# outside src/ and lib/ name PB_BRINGUP on purpose and neither is a console affordance,
# which is what this invariant exists to bound: include/config.h's uncalibrated-build
# guard (task 20 step 13) tests !defined(PB_BRINGUP), and test/test_cli/test_cli.cpp
# compiles BOTH arms of the bench-vs-bringup cases. Widening this back to include/ and
# test/ makes make check fail on every task from 20 onward.
expect 2 "$(files 'PB_BRINGUP' src lib)" \
  "PB_BRINGUP appears in src/cli.cpp and src/main.cpp, and in no other source file"

if [ "$fails" -gt 0 ]; then
  printf '\n%s invariant(s) FAILED\n' "$fails" >&2
  exit 1
fi
printf '\nall invariants hold\n'
exit 0
