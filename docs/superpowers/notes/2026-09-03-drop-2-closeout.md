# Drop 2 close-out: what this drop has NOT proved

Drop 2 (tasks 15-20) is complete on the host: `safety.h`/`safety.cpp` (the sixteen-rung
ladder, the dry latch, the contradiction latch, the float debounce), `cli.h`/`cli.cpp`
(`status`, `stop`, `dry on|off`, `clear contra`, and now — task 20 — the bring-up console:
`servo`, `home`, `goto`, `pump`, `calib`, `cal`, `hang`, `noinit pattern`), the two-binary
split (`uno_r4_wifi` vs `uno_r4_wifi_bringup`), and `config.h`'s `#error` against an
uncalibrated gate pitch. 142 host cases, 134 passing, 8 skipped by design; `make check`
holds all 21 invariants; every environment builds, including both device envs.

**None of that touches silicon.** Nothing in tasks 15-20 substitutes for running bring-up
on the actual board, and no host test can. Below is spec §13's bring-up table, step by
step, with what still has to happen and why a green `pio test -e native` does not stand in
for it. Run every step on the **bringup** binary — `status` must say `build=bringup`.

- **`servo`, `home` and `goto` against the real screw, the real reduction gears and the
  real home hall — bring-up 6.** This is also what produces `PB_PULSES_PER_GATE` and
  `PB_PULSES_HOME_TO_1`, and it deletes `-DPB_ALLOW_UNCALIBRATED` from
  `[env:uno_r4_wifi]` in the same commit. Until it lands, `cart_goto()` is compiled to
  `return false` and `cart_pos_known()` to `return false` (§2.15) — `goto` at the console
  always answers `goto FAILED: uncal`, and `pos` can never be `ok` on the wire.

- **`pump 2000` clicking a dry relay with no 12 V on COM, and COM-NO staying open across a
  power cycle AND across a `hang`-forced watchdog reset — bring-up 4a.** This is what
  proves spec §2.1's boot write actually holds the coil open on a cold and a warm boot; the
  wiring README's old level-then-direction recipe would have failed it on an active-LOW
  module. Nothing in the host suite drives real 12 V or a real relay coil.

- **`DOSE_REFUSED_FLOAT` with the float below the line, and `DOSE_ABORT_FLOAT` with the
  hall unplugged mid-dose, `contra=0` afterwards — bring-up 5a and 5b.** The host suite
  proves the logic (`safety_float_ok_debounced()`, the abort rung inside `dose_run()`'s
  loop); it cannot prove the physical hall, the physical float arm, or the wiring between
  D5 and the expander.

- **`calib` producing a real `mls=` figure and `cal <n>` retuning without a reflash —
  bring-up 7b.** Its output commits `PB_PULSES_PER_L_DEFAULT`, `PB_PRIME_MS_DEFAULT`,
  `PB_STALL_MS_DEFAULT` and `PB_ML_PER_S_MEASURED`. Until that last one is committed,
  `status` says `cap=UNCLAMPED` and the measured cap clamp does not exist — the host suite
  can only exercise the clamp's *arithmetic* under `[env:native_measured]`'s
  `-DPB_ML_PER_S_MEASURED=30`, a number nobody has measured on the real rig yet. If the
  meter does not pulse at the rig's real flow, `-DPB_DOSE_BY_TIME=1` is the stated
  fallback (spec §6).

- **`pump 3000 hang` actually resetting the chip — bring-up 7c.** This cannot be
  simulated: the host's fake clock has no watchdog silicon behind it, so there is no way
  for a host case to observe a real WDT bite, a real reset, or D6 actually dropping inside
  the reset window. `dose_req_t.hang` is proven only as "the dog stops being fed and the
  loop does not return" on the host (`test_pump_hang_requires_the_literal_third_token` is
  the *negative* case — no host case ever sets `hang = true`, because that would hang the
  suite by construction). Afterwards `status` must say `dry=1` and `last=resetmid`.

- **pump start and dead-head current, measured with a clamp meter during `pump 5000
  prime`, and F1 sized from the number — bring-up 7d.** This is pure hardware. Nothing in
  the firmware and nothing in the host suite bears on it at all, and no test anywhere in
  this plan comes near it.

- **`noinit pattern` surviving that reset — bring-up 7c′.** `noinit pattern` writes
  `0xC0FFEE01` into `g_nv.pattern` and commits the checksum; after the watchdog reset,
  `status` must print both the pattern and a checksum that still validates. **If it fails,
  every `.noinit` guarantee in this spec is void**: the dry latch, the contradiction latch,
  `err=resetmid` and the replay high-water mark all become cold-boot-only, and spec
  §16.5.3 (the backend's durable half) stops being a follow-up and becomes a going-live
  blocker. The host suite can prove `.noinit`'s *checksum arithmetic* and its *warm vs.
  cold* decision (`sim_reset(true)` vs `sim_reset(false)`); it cannot prove that SRAM
  actually survives a real watchdog reset on this specific board with its specific
  bootloader, because `__noinit_start` sits at 0x200000cc, the bottom of SRAM, and the
  installed core's bootloader SRAM use is not documented.

Do not read a green `pio test -e native` as coverage of any line above. Task 20's own
deliverable is that bring-up 4a-7d are now *runnable* — the commands exist, are bounded,
and are excluded from the binary that runs unattended — not that they have been *run*.
