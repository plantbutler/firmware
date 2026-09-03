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

if [ "$fails" -gt 0 ]; then
  printf '\n%s invariant(s) FAILED\n' "$fails" >&2
  exit 1
fi
printf '\nall invariants hold\n'
exit 0
