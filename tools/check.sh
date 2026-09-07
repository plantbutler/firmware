#!/usr/bin/env bash
# The firmware's mechanical invariants, as greps over include/ src/ lib/ test/: which file
# may touch the pump pin, who feeds the watchdog, what the network layer may not call,
# what the bench binary must not carry. `make check` runs it.
# The check_nc family strips // and /* */ comments before grepping, so a comment that
# states a rule in the rule's own words cannot trip it (string literals are not stripped).
# Seven invariants read objects under .pio/build and are skipped until the three board
# environments are built; each skip line names the build. Exit 1: an invariant is broken.
# Exit 2: one was skipped, and an unrun gate has not passed. Patterns use POSIX classes,
# never \b or \s: BSD grep is the default on macOS.
set -uo pipefail
# Absolute, because the verdict greps this file after the cd below.
self_="$(cd "$(dirname "$0")" && pwd)/$(basename "$0")"
cd "$(dirname "$0")/.."

SCAN=(include src lib test)      # not tools/: this file spells every pattern
fails=0
oks=0
skips=0
skipped=()

fail() { printf 'FAIL  %s\n' "$*" >&2; fails=$((fails + 1)); }
ok()   { printf 'ok    %s\n' "$*"; oks=$((oks + 1)); }

# built <env>: that environment has compiled at least one object. The directory alone is not
# proof: the idedata step below creates it with only a json file in it.
built() { [ -n "$(find ".pio/build/$1" -name '*.o' -print -quit 2>/dev/null)" ]; }
# skip <n-invariants> <what> <how to run it>; n keeps ok + FAIL + skipped equal to TOTAL below.
skip() {
  skips=$((skips + $1))
  printf 'skip  %s (run: %s)\n' "$2" "$3"
  skipped+=("$2
        run: $3")
}

# Occurrences (grep -o), not lines: two hits on one line must count as 2. files() counts files.
count() { local pat="$1"; shift; grep -rohE "$pat" "$@" 2>/dev/null | wc -l | tr -d ' '; }
files() { local pat="$1"; shift; grep -rlE "$pat" "$@" 2>/dev/null | wc -l | tr -d ' '; }

# Usage: expect <want> <got> <description>. For the checks that compare two derived numbers.
expect() { if [ "$2" = "$1" ]; then ok "$3 ($1)"; else fail "$3: expected $1, found $2"; fi; }

# Usage: check <want> <pattern> [grep args and paths...] -- <description>. A FAIL lists the hits.
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

# Same shape as check, counting files.
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

# Prints file:line:content with every // and /* */ span removed, across line boundaries.
# A state machine, not a per-line filter: code sharing a line with a comment survives.
strip_spans_() {
  awk '
    FNR == 1 { incomment = 0 }
    {
      line = $0; out = ""; i = 1; n = length(line)
      while (i <= n) {
        if (incomment) {
          p = index(substr(line, i), "*/")
          if (p == 0) { i = n + 1 }
          else { i = i + p + 1; incomment = 0 }
        } else {
          rest = substr(line, i)
          pl = index(rest, "//")
          pb = index(rest, "/*")
          if (pl == 0 && pb == 0) { out = out substr(line, i); i = n + 1 }
          else if (pb == 0 || (pl > 0 && pl < pb)) {
            out = out substr(line, i, pl - 1); i = n + 1
          } else {
            out = out substr(line, i, pb - 1); i = i + pb + 1; incomment = 1
          }
        }
      }
      print FILENAME ":" FNR ":" out
    }
  ' "$@" 2>/dev/null
}

# count() over comment-stripped files; grep resolves the paths and --exclude args to files.
count_nc() {
  local pat="$1"; shift
  local -a fl=()
  while IFS= read -r f; do [ -n "$f" ] && fl+=("$f"); done < <(grep -rlE '.' "$@" 2>/dev/null)
  [ "${#fl[@]}" -eq 0 ] && { echo 0; return; }
  strip_spans_ "${fl[@]}" | sed -E 's/^[^:]+:[0-9]+://' | grep -ohE "$pat" 2>/dev/null \
    | wc -l | tr -d ' '
}

# Same shape as check, over comment-stripped files; the diagnostic shows the stripped line.
check_nc() {
  local want="$1" pat="$2"; shift 2
  local -a gargs=()
  while [ "$#" -gt 0 ] && [ "$1" != "--" ]; do gargs+=("$1"); shift; done
  shift
  local desc="$1" got
  got=$(count_nc "$pat" "${gargs[@]}")
  if [ "$got" = "$want" ]; then
    ok "$desc ($want)"
  else
    fail "$desc: expected $want, found $got"
    local -a fl=()
    while IFS= read -r f; do [ -n "$f" ] && fl+=("$f"); done < <(grep -rlE '.' "${gargs[@]}" 2>/dev/null)
    strip_spans_ "${fl[@]}" | grep -E "$pat" | sed 's/^/      /' >&2
  fi
}

# ---- D6, the pump pin ----
# The definition, not the bare token: pins.h has to #ifdef-test it, so a token count is 2.
check_files 1 'define[[:space:]]+PB_PUMP_OWNER' "${SCAN[@]}" -- \
  "exactly one file defines PB_PUMP_OWNER, so exactly one file gets PIN_PUMP_EN"
check_nc 0 'pinMode\(PIN_PUMP_EN' "${SCAN[@]}" -- \
  "pinMode never touches D6 (it would latch PODR=0 and drive the pin LOW)"
# `[^;]*` stops at the statement end, so two writes on one line count as two.
check_nc 2 'R_IOPORT_PinCfg[^;]*PIN_PUMP_EN' "${SCAN[@]}" -- \
  "exactly two whole-word PFS writes to D6 (hal_boot_pump_off, hal_pump_write)"
check_nc 0 'R_IOPORT_PinWrite.*PIN_PUMP_EN|digitalWrite.*PIN_PUMP_EN' "${SCAN[@]}" -- \
  "no unverifiable write form on D6"
# The first argument must be the literal 6, so pinMode(60, ...) and ch206 do not match.
check_nc 0 '(pinMode|digitalWrite)\([[:space:]]*6[[:space:]]*[,)]' "${SCAN[@]}" -- \
  "no bare-literal pin 6 write bypassing PIN_PUMP_EN (pinMode(6,...) or digitalWrite(6,...))"

# ---- D2/D3, the pulse pins: every pinMode on them is inside hal_arm_pulse_pins_ ----
# The window opens on the definition line only; gsub counts occurrences like count() does;
# ENVIRON carries the pattern because `awk -v` would eat the backslash.
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

# ---- the watchdog ----
check_nc 1 'WDT\.refresh' "${SCAN[@]}" -- \
  "one refresh call site, inside hal_wdt_feed"
check_nc 0 'WDT\.getTimeout' "${SCAN[@]}" -- \
  "the timeout getter is never used (it returns 0 under the wdt_cfg_t overload)"
check_nc 1 'hal_wdt_feed\(' src/safety.cpp -- \
  "safety_tick is the ONLY feeder in safety.cpp"
# hal.h is excluded: a seam has to declare what it seams.
check_nc 0 'hal_wdt_feed\(' "${SCAN[@]}" \
  --exclude=safety.cpp --exclude=hal_uno.cpp --exclude=hal_sim.cpp --exclude=hal.h -- \
  "nothing outside safety.cpp and the two HALs feeds the dog"

# ---- lib/Screen ----
# TwoWire::flush() spins with no bound.
check_nc 0 'Wire\.flush' "${SCAN[@]}" -- \
  "no Wire flush anywhere"
# The row printer has no hook between characters; rows go out one glyph per safety_tick().
check_nc 0 'lcd\.print|lcd\.println' "${SCAN[@]}" -- \
  "no library row printer on the LCD"
# Up to 144 unfed Wire transactions each. lib/Screen only: the names are English elsewhere.
check_nc 0 'drawString|clearDisplay|clearLine' lib/Screen -- \
  "the opaque whole-string draw and whole-panel/per-row clear calls do not creep back into lib/Screen"

# ---- seam isolation ----
# netfsm.h is the network stack too: it is the only sanctioned way to reach net_poll().
check_nc 0 'WiFiS3|link\.h|Network\.h|netfsm\.h' src/safety.cpp lib/Manifold -- \
  "the safety layer never names the network stack (safety.cpp, lib/Manifold)"
# One #include "safety.h" in netfsm.cpp puts a dose_run() call one edit from an open socket.
check_nc 0 'safety\.h|dose_run|hal_pump_write' src/netfsm.cpp src/ui.cpp lib/Network -- \
  "the network layer and the painter cannot assert D6"
# exec.cpp has the other call site; cli.cpp's `pump` is the one a human types, so it is bounded.
check_nc 1 'dose_run\(' src/cli.cpp -- \
  "cli.cpp has exactly one dose_run call site"
# netfsm.cpp owns the AT budget and so owns the seam; link_wifi.cpp is the driver behind it.
# The excluded tests and the host fake drive the seam on purpose.
LINK_SEAM_PAT='(^|[^[:alnum:]_])(link_begin|link_join|link_state|link_rssi|link_ip|link_reset|link_desyncs|sock_open|sock_write|sock_read|sock_close)\('
check_nc 0 "$LINK_SEAM_PAT" "${SCAN[@]}" \
  --exclude=link.h --exclude=netfsm.cpp --exclude=link_wifi.cpp --exclude=link_fake.cpp \
  --exclude=test_netfsm.cpp --exclude=test_cli.cpp --exclude=test_device.cpp -- \
  "nothing outside netfsm.cpp and link_wifi.cpp calls a seam-2 function directly"
# Scoped to safety.cpp: test_dose.cpp sets the latch directly as fixture arrangement.
check_nc 1 'g_nv\.contra_latched[[:space:]]*=[[:space:]]*true' src/safety.cpp -- \
  "contra_latched is set to true in exactly one place (dose_end_ml_, spec section 2.7)"

# ---- blocking, buffers and formatting ----
check_nc 0 '(^|[^[:alnum:]_])delay\(' "${SCAN[@]}" --exclude=hal_uno.cpp -- \
  "no unbounded blocking wait outside hal_uno.cpp's power-on settles"
# The flag class covers %-8.2f, % .3f, %+.1f, %#.1f and %.*f.
check_nc 0 '%[-+ #*0-9.]*[fgeFGE]([^[:alnum:]]|$)' "${SCAN[@]}" -- \
  "no float formatting anywhere (newlib float printf is the deepest stack consumer)"
# t= exceeds 2^31 on ordinary boots: a %d prints `-`, every report 400s, and -Wall is silent.
check_nc 0 '%[-+ #*0-9.]*[di]([^[:alnum:]]|$)' src/report.cpp src/netfsm.cpp -- \
  "no signed integer conversion in the report or the framing"
check_nc 0 'for[[:space:]]*\([[:space:]]*;[[:space:]]*;[[:space:]]*\)|while[[:space:]]*\([[:space:]]*(true|1)[[:space:]]*\)' \
  "${SCAN[@]}" --exclude=safety.cpp -- \
  "the program's only intentional unbounded loop is in the function that owns D6"
# The malloc call, not the word: hal_uno.cpp includes <malloc.h> for mallinfo().
check_nc 0 'String|std::map|std::string|(^|[^[:alnum:]_])new([^[:alnum:]_]|$)|malloc[[:space:]]*\(' \
  include src test lib/Manifold -- \
  "no dynamic allocation outside lib/Network and lib/Screen"
# sim_console.cpp takes the header so hal_sim.cpp stays identical between sim and host.
check_nc 0 'Arduino\.h' include src test lib/Manifold \
  --exclude=hal_uno.cpp --exclude=sim_console.cpp -- \
  "the Arduino header lives only in hal_uno.cpp and sim_console.cpp, of the files this check scans (lib/Screen also uses it, out of scope here; lib/Network does not use it at all)"
check_nc 0 'WiFi\.ping' "${SCAN[@]}" -- \
  "ping is never called (it resets the modem timeout to 10 s)"

# ---- the two binaries ----
# src and lib only: config.h's guard and test_cli.cpp also name it, and neither is a console.
check_files 2 'PB_BRINGUP' src lib -- \
  "PB_BRINGUP appears in src/cli.cpp and src/main.cpp, and in no other source file"

# Nothing under src/ includes a lib/Network header, so the dependency finder alone never
# builds it; only the object (under a hashed lib<n>/ dir, hence find) proves lib_deps did.
if built uno_r4_wifi; then
  expect 1 "$(find .pio/build/uno_r4_wifi -name 'link_wifi.cpp.o' 2>/dev/null | wc -l | tr -d ' ')" \
    "uno_r4_wifi compiles lib/Network's driver (link_wifi.cpp.o present)"
else
  skip 1 'uno_r4_wifi builds lib/Network' 'pio run -e uno_r4_wifi'
fi
if built uno_r4_wifi_bringup; then
  expect 1 "$(find .pio/build/uno_r4_wifi_bringup -name 'link_wifi.cpp.o' 2>/dev/null | wc -l | tr -d ' ')" \
    "uno_r4_wifi_bringup compiles lib/Network's driver (link_wifi.cpp.o present)"
else
  skip 1 'uno_r4_wifi_bringup builds lib/Network' 'pio run -e uno_r4_wifi_bringup'
fi
if built uno_r4_wifi_test; then
  expect 1 "$(find .pio/build/uno_r4_wifi_test -name 'link_wifi.cpp.o' 2>/dev/null | wc -l | tr -d ' ')" \
    "uno_r4_wifi_test compiles lib/Network's driver (link_wifi.cpp.o present)"
else
  skip 1 'uno_r4_wifi_test builds lib/Network' \
    'pio test -e uno_r4_wifi_test -f test_device --without-uploading --without-testing'
fi

# Console affordances behind #if PB_BRINGUP: only preprocessed output says whether they
# compiled in. Quoted forms, since a bare `hang` matches inside "unchanged"; flags from idedata.
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
  skip 1 'the bench binary carries no hang, prime, cal or noinit console token' \
    'pio run -e uno_r4_wifi -t idedata'
fi
rm -f "$pp"

# Byte-identical objects are what let bring-up's watchdog proof stand for the bench binary.
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
  skip 2 'safety.o and hal_uno.o are byte-identical in bench and bringup' \
    'pio run -e uno_r4_wifi -e uno_r4_wifi_bringup'
fi

# PIN_PUMP_EN is a macro and leaves no symbol; only the compiled file set proves no pump driver.
if built uno_r4_wifi_sim; then
  expect 0 "$([ -e .pio/build/uno_r4_wifi_sim/src/hal_uno.cpp.o ] && echo 1 || echo 0)" \
    "the sim env compiles no pump driver (hal_uno.cpp absent, or D6 could be driven with 12 V on COM)"
  expect 1 "$([ -e .pio/build/uno_r4_wifi_sim/src/hal_sim.cpp.o ] && echo 1 || echo 0)" \
    "the sim env compiled its HAL (hal_sim.cpp present -- check build_src_filter if not)"
else
  skip 2 'the sim env compiles no pump driver, and compiled its own HAL' \
    'pio run -e uno_r4_wifi_sim'
fi

# ---- the verdict ----
# TOTAL is this file's own audit grep: an invariant added without a matching skip() count fails.
TOTAL=$(grep -cE '^[[:space:]]*(check|check_files|check_nc|expect)[[:space:]]' "$self_")
if [ "$((oks + fails + skips))" != "$TOTAL" ]; then
  printf '\nthis file cannot count itself: %s ok + %s FAIL + %s skipped != %s invariants\n' \
    "$oks" "$fails" "$skips" "$TOTAL" >&2
  fails=$((fails + 1))
fi

if [ "$fails" -gt 0 ]; then
  printf '\n%s invariant(s) FAILED (%s ran, %s skipped, of %s)\n' \
    "$fails" "$((oks + fails))" "$skips" "$TOTAL" >&2
  exit 1
fi

# 2, not 0: an unrun gate has not passed, and 1 is reserved for a broken invariant.
if [ "$skips" -gt 0 ]; then
  printf '\n%s of %s invariants ran; %s were SKIPPED and this gate is incomplete:\n' \
    "$oks" "$TOTAL" "$skips"
  for s in "${skipped[@]}"; do printf '  - %s\n' "$s"; done
  printf '\nRun the commands above (they populate .pio/build) and re-run this script.\n'
  exit 2
fi

printf '\nall %s invariants hold\n' "$TOTAL"
exit 0
