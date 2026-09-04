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

# count <pattern> [grep-args-and-paths...] -> OCCURRENCE count (grep -o), not matching
# lines: two hits written on the same physical line must count as 2, or "exactly N" stops
# meaning what it says the moment someone writes two statements on one line (task 13 fix
# round 1 -- verified: a second hal_wdt_feed(); appended to the SAME line as the existing
# call in safety.cpp used to read as 1 under a line-count; it reads as 2 now).
count() { local pat="$1"; shift; grep -rohE "$pat" "$@" 2>/dev/null | wc -l | tr -d ' '; }
# files <pattern> [grep-args-and-paths...] -> matching FILE count
files() { local pat="$1"; shift; grep -rlE "$pat" "$@" 2>/dev/null | wc -l | tr -d ' '; }

# expect <want> <got> <description> -- the primitive comparator. Kept for the one
# invariant below (D2/D3) that compares two independently-derived numbers rather than a
# single grep's count against a literal, so a single call to check()/check_files() cannot
# express it.
expect() { if [ "$2" = "$1" ]; then ok "$3 ($1)"; else fail "$3: expected $1, found $2"; fi; }

# check <want> <pattern> [grep-args-and-paths...] -- <description>
# Occurrence-counts <pattern> (via count(), above) across the grep args/paths that follow
# -- may include --exclude=... flags mixed with paths, exactly as count()/files() always
# accepted -- and compares to <want>. On a mismatch, ALSO prints `grep -n` output for the
# same pattern/args, indented, under the FAIL line: task 13 fix round 1's finding that a
# developer who trips an invariant had to reconstruct and re-run the grep by hand to find
# the offending line, which is the tax every future violation used to pay.
check() {
  local want="$1" pat="$2"; shift 2
  local -a gargs=()
  while [ "$#" -gt 0 ] && [ "$1" != "--" ]; do gargs+=("$1"); shift; done
  shift            # drop the -- sentinel
  local desc="$1" got
  got=$(count "$pat" "${gargs[@]}")
  if [ "$got" = "$want" ]; then
    ok "$desc ($want)"
  else
    fail "$desc: expected $want, found $got"
    grep -rnE "$pat" "${gargs[@]}" 2>/dev/null | sed 's/^/      /' >&2
  fi
}

# check_files <want> <pattern> [grep-args-and-paths...] -- <description>
# Same shape as check(), but the unit is FILES (via files(), above), for the two "exactly
# N files contain..." invariants. The failure diagnostic is still `grep -n`: which files,
# and which line in each, is more useful than a bare file list.
check_files() {
  local want="$1" pat="$2"; shift 2
  local -a gargs=()
  while [ "$#" -gt 0 ] && [ "$1" != "--" ]; do gargs+=("$1"); shift; done
  shift
  local desc="$1" got
  got=$(files "$pat" "${gargs[@]}")
  if [ "$got" = "$want" ]; then
    ok "$desc ($want)"
  else
    fail "$desc: expected $want, found $got"
    grep -rnE "$pat" "${gargs[@]}" 2>/dev/null | sed 's/^/      /' >&2
  fi
}

# To count the invariants below (e.g. before adding more): almost all of them are one
# check() or check_files() call each; exactly one (D2/D3, immediately below) is a bespoke
# expect() because it compares two independently-derived numbers, not a single grep's
# count against a literal. Count all three:
#   grep -cE '^[[:space:]]*(check|check_files|expect)[[:space:]]' tools/check.sh
# A bare `grep -c 'expect '` over-counts: it also matches expect()'s own doc comment.
# ---- invariants land here (task 13, then task 30) ----

# ---- D6: spec §2.1, §2.2 ----
# PB_PUMP_OWNER is DEFINED in exactly one file. The pattern is the DEFINITION, never the
# bare token: include/pins.h necessarily *tests* it with #ifdef, so a bare-token file count
# would be 2 forever and this invariant could never hold.
check_files 1 'define[[:space:]]+PB_PUMP_OWNER' "${SCAN[@]}" -- \
  "exactly one file defines PB_PUMP_OWNER, so exactly one file gets PIN_PUMP_EN"
check 0 'pinMode\(PIN_PUMP_EN' "${SCAN[@]}" -- \
  "pinMode never touches D6 (it would latch PODR=0 and drive the pin LOW)"
# `[^;]*`, never `.*`: a C statement ends at `;`, and the two writes below can never
# share one, so bounding the wildcard at the statement boundary is enough to stop it
# matching two calls as one. `.*` is greedy and grep -o returns ONE span per match --
# fix round 2 found that a second, unverified R_IOPORT_PinCfg(...PIN_PUMP_EN...) call
# appended to the SAME LINE as an existing one used to merge into the first call's match
# under `.*`, so three real writes on disk still counted as two. `[^;]*` still matches
# any argument order or spacing WITHIN one statement -- it only refuses to reach past the
# semicolon that ends it, so it is not narrowed to the exact `NULL, g_pin_cfg[...]` call
# form the two real sites happen to share today.
check 2 'R_IOPORT_PinCfg[^;]*PIN_PUMP_EN' "${SCAN[@]}" -- \
  "exactly two whole-word PFS writes to D6 (hal_boot_pump_off, hal_pump_write)"
check 0 'R_IOPORT_PinWrite.*PIN_PUMP_EN|digitalWrite.*PIN_PUMP_EN' "${SCAN[@]}" -- \
  "no unverifiable write form on D6"

# ---- D6, task 13's own closed gap: a bare pin-6 literal bypasses every PIN_PUMP_EN grep
# above. The pattern requires the call's FIRST argument to be the literal 6 (open paren,
# optional space, "6", optional space, then a comma or close paren) so it never fires on
# an unrelated "6" -- a channel count, an array size, ch206 -- or on a different pin whose
# number happens to start with 6 (pinMode(60,... has a "0" where this pattern requires a
# terminator). Nothing in the tree does this today; the macro is what every write site uses.
check 0 '(pinMode|digitalWrite)\([[:space:]]*6[[:space:]]*[,)]' "${SCAN[@]}" -- \
  "no bare-literal pin 6 write bypassing PIN_PUMP_EN (pinMode(6,...) or digitalWrite(6,...))"

# ---- D2/D3: spec §2.14. Both hits must be inside hal_arm_pulse_pins_. ----
# f=1 is anchored to the FUNCTION DEFINITION line, not any line containing the substring
# "hal_arm_pulse_pins_" -- task 13 fix round 1: a comment on the same line as a violating
# pinMode(), describing the violation in words that happened to include the function's own
# name, used to set f=1 on that same line and hide the violation from pp_fn. A definition-
# anchored pattern is not fooled by a comment naming the function; it takes the literal
# `static void hal_arm_pulse_pins_(void) {` line to open the window.
#
# pp_fn counts OCCURRENCES (gsub), not matching lines, for the same reason count() does
# (fix round 1's finding 3) -- fix round 2: pp_all already came from count(), which is
# occurrence-based, but pp_fn still incremented once per matching LINE. Two entirely
# legitimate pulse-pin pinMode() calls merged onto one physical line inside
# hal_arm_pulse_pins_ used to read pp_fn=1 against pp_all=2 and FAIL SAFE (a false
# positive, never a missed violation) rather than pass. gsub(pat, "&") substitutes each
# match with itself -- a no-op on the text -- and returns the number of substitutions, so
# both counters now agree on what "one" means.
# The pattern is threaded through ENVIRON, not `awk -v`: POSIX -v assignments undergo the
# SAME backslash-escape processing as a string constant, and this awk (macOS's) silently
# drops an unrecognized escape -- `-v pat='pinMode\('` arrives inside awk as the STRING
# pinMode( (no backslash), which is not a balanced regex and is a syntax error, not a
# quiet miscount. ENVIRON entries are not escape-processed, so the backslash survives.
D2D3_PAT='pinMode\((PIN_FLOW|PIN_HALL_SCREW)'
pp_all=$(count "$D2D3_PAT" "${SCAN[@]}")
export PB_CHECK_D2D3_PAT="$D2D3_PAT"
pp_fn=$(awk 'BEGIN { pat = ENVIRON["PB_CHECK_D2D3_PAT"] }
             /^static void hal_arm_pulse_pins_\(void\)[[:space:]]*\{/ {f=1}
             f {c += gsub(pat, "&")}
             f && /^}/ {f=0}
             END {print c+0}' src/hal_uno.cpp 2>/dev/null)
unset PB_CHECK_D2D3_PAT
expect "$pp_all" "$pp_fn" \
  "every pinMode on D2/D3 is inside hal_arm_pulse_pins_ (a later one detaches the interrupt)"
if [ "$pp_all" != "$pp_fn" ]; then
  grep -rnE "$D2D3_PAT" "${SCAN[@]}" 2>/dev/null | sed 's/^/      /' >&2
fi

# ---- the watchdog, first two of four: spec §2.5. The other two are task 13 step 3's. ----
check 1 'WDT\.refresh' "${SCAN[@]}" -- \
  "one refresh call site, inside hal_wdt_feed"
check 0 'WDT\.getTimeout' "${SCAN[@]}" -- \
  "the timeout getter is never used (it returns 0 under the wdt_cfg_t overload)"

# ---- the watchdog, continued: spec §2.4, §2.5 ----
check 1 'hal_wdt_feed\(' src/safety.cpp -- \
  "safety_tick is the ONLY feeder in safety.cpp"
# hal.h's declaration is excluded: a seam has to declare what it seams.
check 0 'hal_wdt_feed\(' "${SCAN[@]}" \
  --exclude=safety.cpp --exclude=hal_uno.cpp --exclude=hal_sim.cpp --exclude=hal.h -- \
  "nothing outside safety.cpp and the two HALs feeds the dog"

# ---- lib/Screen: spec §5 ----
# spec §5: TwoWire::flush() spins with no bound and is never called by us.
check 0 'Wire\.flush' "${SCAN[@]}" -- \
  "no Wire flush anywhere"

# spec §5: the library's row printer offers no hook between characters, so an LCD row is
# painted one character at a time with safety_tick() between them.
check 0 'lcd\.print|lcd\.println' "${SCAN[@]}" -- \
  "no library row printer on the LCD"

# task 13's own closed gap: task 9 replaced the OLED's opaque whole-string draw and
# whole-panel/per-row clear calls with the bounded per-glyph path in Screen::clear()/
# row() specifically because those calls run 144 (whole-panel) or 18 (per-row) unfed
# Wire transactions against a 5592 ms watchdog grant. Nothing greps for their return
# today, so a future edit could reinstate any of them and reintroduce a span that long
# without a single invariant noticing. Scoped to lib/Screen only: the library names are
# real English words elsewhere (config.h's own accounting of why they are NOT called
# describes them without spelling any of the three contiguously -- see that file).
check 0 'drawString|clearDisplay|clearLine' lib/Screen -- \
  "the opaque whole-string draw and whole-panel/per-row clear calls do not creep back into lib/Screen"

# ---- seam isolation, spec §9 item 16: the safety layer cannot reach the network stack.
# include/safety.h already promises this ("Includes neither link.h, Network.h nor
# WiFiS3.h ... and tools/check.sh greps for it") -- this is the grep that makes that
# promise true rather than aspirational. Decidable today: safety.cpp and lib/Manifold
# both exist in a drop-1 tree, so this does not wait for task 30 the way the netfsm/
# report.cpp-scoped rows of spec §9's table do. ----
check 0 'WiFiS3|link\.h|Network\.h' src/safety.cpp lib/Manifold -- \
  "the safety layer never names the network stack (safety.cpp, lib/Manifold)"

# ---- blocking, buffers and formatting: spec §3, §9, §12 ----
# lib/Network/src/Network.cpp -- the pre-bench-sketch sketch, orphaned since task 1 and
# excluded here by name because it was still on disk -- is deleted by this task (drop 3,
# seam 2), in the same commit that removes the --exclude=Network.cpp that used to sit on
# each check below, the way task 14 removed lib/Manifold/src/Manifold.cpp's own --exclude
# here (and its library.json srcFilter, see 190b56d) in the same commit that deleted that
# file. Excluded by FILE NAME, never by directory, so a real file added to either library
# -- cart.cpp already, link_wifi.cpp in task 27 -- is still checked.
check 0 '(^|[^[:alnum:]_])delay\(' "${SCAN[@]}" --exclude=hal_uno.cpp -- \
  "no unbounded blocking wait outside hal_uno.cpp's power-on settles"
check 0 '%[0-9.]*[fgeFGE]([^[:alnum:]]|$)' "${SCAN[@]}" -- \
  "no float formatting anywhere (newlib float printf is the deepest stack consumer)"
check 0 'for[[:space:]]*\([[:space:]]*;[[:space:]]*;[[:space:]]*\)|while[[:space:]]*\([[:space:]]*(true|1)[[:space:]]*\)' \
  "${SCAN[@]}" --exclude=safety.cpp -- \
  "the program's only intentional unbounded loop is in the function that owns D6"
# `malloc[[:space:]]*\(` -- the CALL, never the bare word. src/hal_uno.cpp writes
# `#include <malloc.h>` for mallinfo(), which is how the heap diagnostics of ch200/ch201
# exist at all; a bare-word pattern would match that include and fail this check forever.
check 0 'String|std::map|std::string|(^|[^[:alnum:]_])new([^[:alnum:]_]|$)|malloc[[:space:]]*\(' \
  include src test lib/Manifold -- \
  "no dynamic allocation outside lib/Network and lib/Screen"
# spec §9 names three homes for the Arduino header; src/sim_console.cpp is a FOURTH, added
# by task 29 and recorded as a deviation in that task's commit message. It exists so that
# hal_sim.cpp stays byte-identical between the sim binary and the host suites, which is the
# whole reason one fake serves both. It is the only widening this line ever takes.
expect 0 "$(grep -rEn 'Arduino\.h' include src test lib/Manifold \
              --exclude=hal_uno.cpp --exclude=sim_console.cpp \
              2>/dev/null | wc -l | tr -d ' ')" \
  "the Arduino header lives only in hal_uno.cpp, sim_console.cpp, lib/Network and lib/Screen"
check 0 'WiFi\.ping' "${SCAN[@]}" -- \
  "ping is never called (it resets the modem timeout to 10 s)"

# ---- the two binaries: spec §6 ----
# SCOPED TO src AND lib, not "${SCAN[@]}", and that is a deliberate deviation from §9's
# table recorded in this task's commit message and again in task 20 step 13. Two files
# outside src/ and lib/ name PB_BRINGUP on purpose and neither is a console affordance,
# which is what this invariant exists to bound: include/config.h's uncalibrated-build
# guard (task 20 step 13) tests !defined(PB_BRINGUP), and test/test_cli/test_cli.cpp
# compiles BOTH arms of the bench-vs-bringup cases. Widening this back to include/ and
# test/ makes make check fail on every task from 20 onward.
check_files 2 'PB_BRINGUP' src lib -- \
  "PB_BRINGUP appears in src/cli.cpp and src/main.cpp, and in no other source file"

# ---- sim: spec §8. The sim binary compiles no pump driver at all. ----
# It greps the file set the env compiles, not a linker map: PIN_PUMP_EN is a macro and
# leaves no symbol, so a map cannot prove this.
if [ -d .pio/build/uno_r4_wifi_sim ]; then
  if [ -e .pio/build/uno_r4_wifi_sim/src/hal_uno.cpp.o ]; then
    fail "the sim env compiled hal_uno.cpp: D6 could be driven with 12 V on COM"
  else
    ok "the sim env compiles no pump driver"
  fi
  [ -e .pio/build/uno_r4_wifi_sim/src/hal_sim.cpp.o ] || \
    fail "the sim env compiled no HAL at all - check build_src_filter"
else
  printf 'skip  sim file set (run: pio run -e uno_r4_wifi_sim)\n'
fi

if [ "$fails" -gt 0 ]; then
  printf '\n%s invariant(s) FAILED\n' "$fails" >&2
  exit 1
fi
printf '\nall invariants hold\n'
exit 0
