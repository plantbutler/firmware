#!/usr/bin/env bash
# tools/check.sh -- the mechanical invariants of the bench-sketch spec §9, as greps.
# Completed by task 30: spec §9's full table now has a check below it, plus a handful of
# carried findings from tasks 23-29 that the spec's table does not print (the seam-2
# mediator rule, the every-non-sim-env-builds-lib/Network rule, and the single-setter
# rule over g_nv.contra_latched). See docs/superpowers/specs/2026-09-03-bench-sketch-
# design.md §9.
#
# EVERY GREP BELOW MATCHES A COMMENT JUST AS READILY AS IT MATCHES CODE. grep cannot tell
# the two apart, and eight false positives were found this way on this branch before this
# file settled -- a comment describing a violation, in the same words the pattern looks
# for, trips the same grep the violation itself would. The fix is never a file exemption:
# it is rewording the COMMENT to describe the thing without spelling it (see WiFi.ping's
# own treatment below, and netfsm.h/config.h/include/sim.h's, task 30) -- "said in words,
# not spelled". Before adding a new invariant, write the sentence that would trip it and
# run this file; if an existing comment does, reword that comment, not the pattern's scope.
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
# check() or check_files() call each; three are bespoke expect() calls, each because it
# compares two independently-derived numbers rather than a single grep's count against a
# literal. D2/D3 (immediately below) compares two pinMode counts derived two different
# ways. The sim file-set pair (task 29, spec §8) each compare a want-bit against a file's
# on-disk existence: an object file's PRESENCE has no text pattern to grep -- only its own
# `-e` test -- so check()/check_files() cannot express it and expect() is the honest fit,
# not a workaround. Count all three:
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
# report.cpp-scoped rows of spec §9's table do.
# WIDENED, task 30: `netfsm\.h` added. The original three names catch an include of the
# seam header or the library wrapper, but include/netfsm.h is the network stack too --
# it is seam 2's OWN state machine and the only sanctioned way to reach net_poll(),
# net_disable() and the four cached link accessors (221ac9a's "netfsm.cpp owns the AT
# budget, so it owns the seam" is precisely what makes a stray #include "netfsm.h" in
# safety.cpp a real path to a network call, not just an include of a header that talks
# about one). include/netfsm.h's own file-header comment used to spell `sock_close()`
# and `sock_read()` in prose, which would have tripped this pattern the moment it was
# widened; task 30 reworded that comment (and two more, in config.h and sim.h) to drop
# the parentheses rather than exempt the file -- said in words, not spelled, exactly
# like the WiFi.ping treatment below. Re-run after the reword: netfsm.h needs no
# exemption. ----
check 0 'WiFiS3|link\.h|Network\.h|netfsm\.h' src/safety.cpp lib/Manifold -- \
  "the safety layer never names the network stack (safety.cpp, lib/Manifold)"

# ---- the mirror direction, spec §9 items 17-18: the network layer and the UI painter
# cannot assert D6. Item 16 above is the direction that was decidable in a drop-1 tree;
# this is the direction that actually matters, because it is the one where a mistake is
# invisible right up until it waters something -- one #include "safety.h" added to
# netfsm.cpp during a later change puts a dose_run() call one edit away from a state
# whose socket is open, with the build and make check both green (spec §3). Proven able
# to fire (task 30): `printf '#include "safety.h"\n' >> src/netfsm.cpp` turns this FAIL,
# reverted after. ----
check 0 'safety\.h|dose_run|hal_pump_write' src/netfsm.cpp src/ui.cpp lib/Network -- \
  "the network layer and the painter cannot assert D6"
# src/exec.cpp carries the tree's SECOND dose_run( call site (its exec_pending() ->
# dose_run(&q), task 24) and is expected to: this invariant is scoped to cli.cpp on
# purpose, because cli.cpp's console `pump` command is the one call site a human can
# trigger directly and is where "exactly one" is worth enforcing on its own. Widen the
# scope comment if a future task adds a third call site somewhere legitimate; never
# widen the invariant's own count past 1 to make room for it.
check 1 'dose_run\(' src/cli.cpp -- \
  "cli.cpp has exactly one dose_run call site"

# ---- carried finding, task 30 (not in spec §9's printed table): nothing outside
# src/netfsm.cpp and lib/Network/src/link_wifi.cpp may call a seam-2 function directly.
# netfsm.cpp owns the AT budget and therefore owns the seam (221ac9a); link_wifi.cpp is
# the driver behind it. Eleven names: the ten primitives include/link.h declares
# (link_begin, link_join, link_state, link_rssi, link_ip, link_reset, sock_open,
# sock_write, sock_read, sock_close) plus link_desyncs -- confirmed by reading the header
# rather than assuming the count. Word-anchored on the left with the same
# `(^|[^[:alnum:]_])` idiom the delay()/new() checks already use below, so `g_link_state(`
# could never be mistaken for a call to `link_state`; the trailing `\(` is boundary enough
# on the right, since none of the eleven is a prefix of another.
#
# FIVE files are legitimate, not three. include/link.h (the declarations),
# src/link_fake.cpp (the host fake, which defines them) and test/test_net/
# test_netfsm.cpp (which drives them deliberately) were the three anticipated ahead of
# time. Running this grep before committing it also found two more, both real, both
# already shipped: test/test_cli/test_cli.cpp's `sim resp` case reads the fake's queued
# response back through the seam's own sock_open()/sock_read() on purpose (task 29 --
# see that file's own comment on why), and test/test_device/test_device.cpp calls all
# eleven directly because its whole job is driving the REAL WiFiS3 stack, the one thing
# no host suite can do (task 28). Both are as deliberate as test_netfsm.cpp, so both are
# exclusions, not violations -- widening this list to seven rather than the anticipated
# three is recorded here and in this task's commit message, not silently.
#
# include/netfsm.h needs NO exemption (confirmed by running this grep against the tree
# before adding any exclusion for it) -- but its own file-header comment, and one each in
# include/config.h and include/sim.h, used to spell `sock_close()`/`sock_read()`/
# `sock_write()` in prose and would have tripped this the moment it was added. All three
# were reworded to drop the parentheses (task 30), the same "said in words, not spelled"
# treatment WiFi.ping's own comments already get below. ----
LINK_SEAM_PAT='(^|[^[:alnum:]_])(link_begin|link_join|link_state|link_rssi|link_ip|link_reset|link_desyncs|sock_open|sock_write|sock_read|sock_close)\('
check 0 "$LINK_SEAM_PAT" "${SCAN[@]}" \
  --exclude=link.h --exclude=netfsm.cpp --exclude=link_wifi.cpp --exclude=link_fake.cpp \
  --exclude=test_netfsm.cpp --exclude=test_cli.cpp --exclude=test_device.cpp -- \
  "nothing outside netfsm.cpp and link_wifi.cpp calls a seam-2 function directly"

# ---- carried finding, task 30 (not in spec §9's printed table): contra_latched is SET
# in exactly one place. include/safety.h already promises this in prose ("SET in exactly
# one place -- dose_end_ml_(), under five conditions each doing one job... There is
# deliberately no safety_contra_set_(): a test hook that set the latch directly would be
# a second setter, which is the very thing this design exists to prevent"); this is the
# grep that makes the promise mechanical. Scoped to src/safety.cpp, not "${SCAN[@]}":
# test/test_dose/test_dose.cpp sets g_nv.contra_latched = true directly, twice, as
# fixture arrangement for two latch-refusal cases, and that is legitimate test setup, not
# a second production setter -- widening this past src/safety.cpp would count those and
# make "exactly one" false forever, the same trap PB_PUMP_OWNER's own check.sh comment
# already warns about for a bare-token count. ----
check 1 'g_nv\.contra_latched[[:space:]]*=[[:space:]]*true' src/safety.cpp -- \
  "contra_latched is set to true in exactly one place (dose_end_ml_, spec section 2.7)"

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
# WIDENED, task 30: the original `%[0-9.]*[fgeFGE]` never matched a flag character
# between `%` and the width/precision digits -- `%-8.2f`, `% .3f`, `%+.1f`, `%#.1f` and a
# dynamic `%.*f` all read straight past it, because none of `-`, ` `, `+`, `#` or `*` was
# in the character class. Confirmed against the four literal forms above: the old pattern
# matched none of them, the widened one below matches all four. No such format string
# exists in the tree today (this stays 0), so the widening only closes a blind spot, it
# does not surface a live bug.
check 0 '%[-+ #*0-9.]*[fgeFGE]([^[:alnum:]]|$)' "${SCAN[@]}" -- \
  "no float formatting anywhere (newlib float printf is the deepest stack consumer)"
# ---- the wire's integers: spec §9, §15.2. Every t=/ack=/chN= site is %lu with an
#      explicit (unsigned long) cast. -Wformat-signedness in build_src_flags is the early
#      warning; this grep is the enforcement, and it is LOAD-BEARING, not belt-and-braces:
#      t = hal_boot_salt() + hal_millis() is above 2^31 on ordinary boots (the salt is the
#      boot counter times PB_BOOT_SALT_STRIDE, a large odd stride), so a single %d against
#      that uint32_t prints a leading `-`, _int_in rejects it, and every report 400s from
#      the FIRST one, not from day 25 -- with the console looking healthy throughout,
#      because report.cpp/netfsm.cpp's OWN %lu sites are unaffected. -Wall -Wextra does
#      not diagnose a %d against an unsigned argument. ----
check 0 '%[-+ #*0-9.]*[di]([^[:alnum:]]|$)' src/report.cpp src/netfsm.cpp -- \
  "no signed integer conversion in the report or the framing"
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
# check(), not expect(): this IS a single grep's occurrence count against a literal (0),
# exactly the shape check() exists for, and check()'s own `grep -n` diagnostic dump on a
# mismatch (the tax task 13 fix round 1 removed project-wide) is worth keeping here too.
check 0 'Arduino\.h' include src test lib/Manifold \
  --exclude=hal_uno.cpp --exclude=sim_console.cpp -- \
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

# ---- carried finding, task 30 (not in spec §9's printed table): every non-sim device
# env actually compiles lib/Network's driver. This is the invariant that would have
# caught 8e7df87's bug before task 27's first device build ever ran: lib_deps named
# Network in [env:uno_r4_wifi], but nothing under src/ includes any of THAT LIBRARY's
# own headers (seam 2's header, include/link.h, is a project header), so PlatformIO's
# dependency finder never had a reason to build lib/Network at all -- src/link_wifi.cpp
# compiled into no binary for five tasks, invisible to every host suite (native never
# builds lib/Network either) and invisible to make check (no grep over source text can
# tell "compiled" from "not compiled"). An object file's PRESENCE has no pattern to
# grep, only its own on-disk existence, so this follows the sim file-set block's own
# shape below: expect(), not check()/check_files(), and a clean skip when the env has
# not been built. `find`, not a fixed path, because a library's objects land under a
# PlatformIO-numbered `lib<hash>/<LibName>/` directory (confirmed: `lib65b/Network/
# link_wifi.cpp.o` on this machine, and that hash is not a promise). Three explicit
# blocks, not a loop over the three envs: a loop's `expect` call is ONE line in this
# file's own text but would print up to THREE ok/skip lines at runtime, which is exactly
# the static-count-vs-runtime-count mismatch the sim file-set block's own history
# warns about (fix round 1: "make check reported 22 while the grep said 21"). ----
if [ -d .pio/build/uno_r4_wifi ]; then
  expect 1 "$(find .pio/build/uno_r4_wifi -name 'link_wifi.cpp.o' 2>/dev/null | wc -l | tr -d ' ')" \
    "uno_r4_wifi compiles lib/Network's driver (link_wifi.cpp.o present)"
else
  printf 'skip  uno_r4_wifi builds lib/Network (run: pio run -e uno_r4_wifi)\n'
fi
if [ -d .pio/build/uno_r4_wifi_bringup ]; then
  expect 1 "$(find .pio/build/uno_r4_wifi_bringup -name 'link_wifi.cpp.o' 2>/dev/null | wc -l | tr -d ' ')" \
    "uno_r4_wifi_bringup compiles lib/Network's driver (link_wifi.cpp.o present)"
else
  printf 'skip  uno_r4_wifi_bringup builds lib/Network (run: pio run -e uno_r4_wifi_bringup)\n'
fi
if [ -d .pio/build/uno_r4_wifi_test ]; then
  expect 1 "$(find .pio/build/uno_r4_wifi_test -name 'link_wifi.cpp.o' 2>/dev/null | wc -l | tr -d ' ')" \
    "uno_r4_wifi_test compiles lib/Network's driver (link_wifi.cpp.o present)"
else
  printf 'skip  uno_r4_wifi_test builds lib/Network (run: pio test -e uno_r4_wifi_test -f test_device --without-uploading --without-testing)\n'
fi

# ---- the two binaries, continued: spec §6, §9. Proved on the PREPROCESSED bench
# source, because the tokens this forbids are console AFFORDANCES (a human typing `pump
# 60000 hang` at a live prompt), not symbols -- a source grep sees `#if PB_BRINGUP`
# around cmd_pump_'s hang/prime branches and the whole block either compiles in or it
# does not, so only the preprocessor's own output tells the truth about which happened.
# `hang`/`prime`/`cal` are ordinary English words too (sensors_float_change_age_s,
# netfsm's exchange, any unchanged) and preprocessing expands every included header into
# the one file grep sees, so the four patterns below quote the literal C string forms,
# never the bare words -- a bare `hang` would match inside "unchanged" before it ever
# reached the console table this exists to check. long_prime is deliberately absent:
# it is a dose_req_t member read by dose_run() in safety.cpp, compiled into BOTH envs,
# so a whole-tree grep for it could never return zero (spec §6) -- the object-hash check
# below is what replaces that whole-tree grep instead.
#
# idedata.json's own keys (cxx_path, defines, includes.build) are read from PlatformIO
# rather than retyped, because retyped flags drift the first time someone upgrades the
# platform; if a PlatformIO upgrade renames one of those keys, this block prints
# idedata.json and the fix is one line, not a rewritten compiler invocation. ----
pp="$(mktemp)"
idedata=.pio/build/uno_r4_wifi/idedata.json
if pio run -e uno_r4_wifi -t idedata >/dev/null 2>&1 && [ -f "$idedata" ]; then
  cxx="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["cxx_path"])' "$idedata")"
  flags="$(python3 -c '
import json, sys
d = json.load(open(sys.argv[1]))
print(" ".join(["-D" + x for x in d["defines"]] + ["-I" + x for x in d["includes"]["build"]]))
' "$idedata")"
  # shellcheck disable=SC2086
  "$cxx" -E $flags src/cli.cpp > "$pp" 2>/dev/null
  expect 0 "$(grep -cE '"hang"|" hang"|"prime"|" prime"|"cal "|"noinit pattern"' "$pp" | tr -d ' ')" \
    "the bench binary carries no hang, prime, cal or noinit console token"
else
  printf 'skip  preprocessed bench console (run: pio run -e uno_r4_wifi -t idedata)\n'
fi
rm -f "$pp"

# ---- the two binaries, continued: spec §6. safety.o and hal_uno.o compile IDENTICALLY
# in both envs -- this is the stronger check that stands in for a whole-tree long_prime
# grep (which, as the block above explains, could never return zero) and is what lets
# bring-up 7c prove the watchdog on the BRINGUP binary and have that verdict still mean
# something for the BENCH binary: if the object files match byte for byte, the code that
# ran on the bench is the code bring-up exercised. `find`, not a fixed path: both files
# are project sources, not library sources, so today they land directly under
# `.pio/build/<env>/src/`, but this does not assume that stays true. expect(), not
# check()/check_files(): two independently-derived sha1 sums is exactly the "two
# independently-derived numbers" shape expect() exists for (see the D2/D3 check above,
# and the sim file-set block below) -- converted to a 0/1 boolean the same way the sim
# block converts a file's existence, so a mismatch reads as "expected 1, found 0"
# through this file's own primitive rather than a bespoke fail() string that this file's
# own audit grep cannot see. ----
sha_a="$(find .pio/build/uno_r4_wifi         -name 'safety*.o'  2>/dev/null | head -1)"
sha_b="$(find .pio/build/uno_r4_wifi_bringup -name 'safety*.o'  2>/dev/null | head -1)"
sha_c="$(find .pio/build/uno_r4_wifi         -name 'hal_uno*.o' 2>/dev/null | head -1)"
sha_d="$(find .pio/build/uno_r4_wifi_bringup -name 'hal_uno*.o' 2>/dev/null | head -1)"
if [ -n "$sha_a" ] && [ -n "$sha_b" ] && [ -n "$sha_c" ] && [ -n "$sha_d" ]; then
  expect 1 "$([ "$(shasum "$sha_a" | cut -d' ' -f1)" = "$(shasum "$sha_b" | cut -d' ' -f1)" ] && echo 1 || echo 0)" \
    "safety.o is identical in bench and bringup (the safety layer is not the same code otherwise)"
  expect 1 "$([ "$(shasum "$sha_c" | cut -d' ' -f1)" = "$(shasum "$sha_d" | cut -d' ' -f1)" ] && echo 1 || echo 0)" \
    "hal_uno.o is identical in bench and bringup (the pin layer is not the same code otherwise)"
else
  printf 'skip  object-hash equality (run: pio run -e uno_r4_wifi -e uno_r4_wifi_bringup)\n'
fi

# ---- sim: spec §8. The sim binary compiles no pump driver at all. ----
# Through expect(), not raw fail/ok: it tests the file set the env compiled, not a grep
# over source text (PIN_PUMP_EN is a macro and leaves no symbol, so a map or a source grep
# cannot prove this either) -- an object file's PRESENCE has no pattern to count, only its
# own `-e` test, which is exactly the "two independently-derived numbers" shape expect()
# exists for. Fix round 1: this used to call fail/ok directly, invisible to this file's own
# audit grep (the header comment, above) -- make check reported 22 while the grep said 21.
if [ -d .pio/build/uno_r4_wifi_sim ]; then
  expect 0 "$([ -e .pio/build/uno_r4_wifi_sim/src/hal_uno.cpp.o ] && echo 1 || echo 0)" \
    "the sim env compiles no pump driver (hal_uno.cpp absent, or D6 could be driven with 12 V on COM)"
  expect 1 "$([ -e .pio/build/uno_r4_wifi_sim/src/hal_sim.cpp.o ] && echo 1 || echo 0)" \
    "the sim env compiled its HAL (hal_sim.cpp present -- check build_src_filter if not)"
else
  printf 'skip  sim file set (run: pio run -e uno_r4_wifi_sim)\n'
fi

if [ "$fails" -gt 0 ]; then
  printf '\n%s invariant(s) FAILED\n' "$fails" >&2
  exit 1
fi
printf '\nall invariants hold\n'
exit 0
