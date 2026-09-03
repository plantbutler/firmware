# Bench bring-up firmware — implementation spec

**2026-09-03.** This is the merged design **as corrected** by 31 adversarial findings and six human
decisions. Where it differs from the design it supersedes, it differs on purpose; §15 lists the
findings that were deliberately *not* applied and why.

> Every platform claim below was re-verified for this document against the installed core at
> `~/.platformio/packages/framework-arduinorenesas-uno`, every protocol claim against
> `/Users/jcanton/projects/plant-butler/backend/butler.py`, every wiring claim against
> `/Users/jcanton/projects/plant-butler/cad/wiring/{README.md,nets.py}`, and the RAM figures against
> `.pio/build/uno_r4_wifi/firmware.map`. **Two of the design's platform claims did not survive that
> re-check** (the D6 boot order and `WDT.getTimeout()` under the config overload); both are corrected
> below with the citation that corrects them. Citations belong in `include/config.h` next to the
> constants they justify — a constant without one is a bug report waiting to be written.

Sections keep the original numbering 0–14. §15 is new (findings declined). The design's old §15
(owed changes) is now §16, so the document still ends with what other repos owe.

---

## Read this first, in this order

An implementer who reads these seven things, in this order, will not have to guess at anything else
in the document.

1. **`cad/wiring/README.md`** — the sections *Pump driver* (THE GAP), the *what stops the pump*
   table, *Where the float sits*, and the *Bring-up order* table. That file is the contract; this one
   is an implementation of it. Note as you read it that **its D6 boot recipe is wrong for this
   silicon** — see §2 and §16.1.
2. **`DECISIONS.md` #5, #7, #10, #11, #12.** #10 is the one that makes three firmware measures
   mandatory rather than nice to have. It also says "IWDT", and that word is currently false (§2).
3. **§2 of this document, whole, before anything else here.** It is the only section where a mistake
   puts water on the floor. Everything else is plumbing around it.
4. **§15, findings declined.** Seven things were deliberately not done, and three of the eight
   declines exist because a *reviewer* was wrong rather than the design. Knowing which, and why, is
   cheaper than rediscovering them.
5. **`backend/butler.py`**, about 150 lines: `_int_in` (:192-201), `parse_report` (:204-257),
   `cap_for` (:261-266), `handle_report`'s ack/expire/dedup block (:828-:873), **`water_rules`'s gate
   (:680-681)**, and the dose alert (:1361-:1369). Every trap in §4 comes from those lines.

   > Every `butler.py:` citation in this document was re-derived mechanically with `grep -n` on
   > 2026-09-03. They are line numbers into a file that is still being edited: pin them by **name**
   > when you copy them into `include/config.h`, and re-grep before trusting one. A one-line edit
   > upstream reshuffles every number below it.
6. **§7 (`config.h`) and §10 (`platformio.ini`).** Every number and every build knob, with its
   citation.
7. **Then §1, §3–§6, §8–§14** in any order.

Then, **before writing a single module**, prove `pio test -e native` links (§10). It is a
thirty-minute experiment that either validates the two seams or kills them, and everything else in
this spec depends on the answer.

---

## 0. What this is, and what it is not

A bench-rig firmware for one manifold, five outlets. It must:

- run the nine serial commands bring-up 0–7d depends on, all live — **in a separate `bringup`
  binary, which is not the binary left running unattended** (§6);
- post five raw channels plus ten diagnostic channels once a minute to `butler.py`, reconnect on
  drop, retry once inside a bounded deadline, discard;
- execute at most one bounded water command and ack it on the next report;
- never lie in `float=`, `pos=`, `ack=` and `flow_ml=` — which is the whole of the phone-notification
  feature;
- **latch refuse-all-watering when the float and the flow meter contradict each other** (§2, new);
- keep both screens and print what it is doing;
- produce the pitch's deliverables: **ml/s per outlet** and a **verdict on flow-meter accuracy**.

It is **not** a product. No RTOS, no scheduler, no virtual HAL, no event queue, no DHT, no EEPROM,
no TLS/JSON/OTA. ~2,300 lines of firmware in the bench binary, ~1,400 of host tests.

**It also does not water anything yet.** `PB_REPORT_POS_UNKNOWN` ships defined (§4, §7), so every
report says `pos=unknown`, so `water_rules` returns at `butler.py:680-681` and no backend water
command is ever queued. Flipping that constant is a deliberate, separate act after the 48-hour run
passes. **It has a visible price on the phone**, stated in §4.6: butler raises exactly one HIGH
`pos:` page about two minutes after first boot, and that page stands for the whole bench programme.

### Verified platform facts that shape everything

| fact | value | source |
| --- | --- | --- |
| PCLKB | 24 MHz (HOCO 48 MHz, `BSP_CFG_PCLKB_DIV` = `_DIV_2`) | `variants/UNOWIFIR4/includes/ra_gen/bsp_clock_cfg.h:8,14` |
| WDT max window | `16384 × 8192 / 24000` = **5592 ms** | `libraries/WDT/src/WDT.cpp:110`, arithmetic re-run below |
| WDT refresh window | `WDT_WINDOW_START_100` / `WDT_WINDOW_END_0` — refresh legal any time | `WDT.cpp:64-65` |
| **`WDT.begin(wdt_cfg_t)` never sets `_timeout`** | so `getTimeout()` returns **0** under the config overload even when running | `WDT.cpp:32-46` vs `:59,153` — **corrects the design** |
| `WDT.getCounter()` | `R_WDT_CounterGet`, exists; **this design's liveness probe** | `WDT.cpp:91-100` |
| the WDT counter is a **down**-counter clocked at PCLKB/8192 | 24 MHz / 8192 = **2929.7 Hz**, i.e. ~2.93 counts/ms; a refresh reloads it to 16384 | `WDT.cpp:32-45` (`R_WDT_Refresh` inside `begin`), `bsp_clock_cfg.h:8,14` — **this is what makes `hal_wdt_alive()` measurable, and why the probe must not feed (§2.5)** |
| `begin(uint32_t)` sets `stop_control = WDT_STOP_CONTROL_ENABLE` | "count will automatically stop when device enters sleep mode" | `WDT.cpp:67`, `r_wdt_api.h:113-114` |
| `wdt_cfg_t` has **nine** members, not six | `p_callback`, `p_context`, `p_extend` follow `stop_control` | `r_wdt_api.h:147-160` — initialise with `= {}` (§2.5) |
| **`pinMode(pin, OUTPUT)` writes the whole PFS word and latches PODR = 0** | so it drives the pin **LOW**, discarding a preceding `digitalWrite` | `cores/arduino/digital.cpp:12-14` → `R_IOPORT_PinCfg` → `R_BSP_PinCfg` = `PmnPFS = cfg` (`bsp_io.h:391-395`); `IOPORT_CFG_PORT_DIRECTION_OUTPUT = 0x4`, `IOPORT_CFG_PORT_OUTPUT_HIGH = 0x1` (`r_ioport_api.h:184,186`) — **corrects the design and the wiring README** |
| the core's own idiom proves it | `IOPORT_CFG_PORT_DIRECTION_OUTPUT \| IOPORT_CFG_PORT_OUTPUT_HIGH` | `Arduino_LED_Matrix.h:125`, `SoftwareSerial.cpp:228,232`, `pin_data.c:14` |
| `BSP_CFG_PARAM_CHECKING_ENABLE` is **0** | so passing `NULL` for `p_ctrl`, as `digital.cpp` does, is safe | `ra_cfg/fsp_cfg/bsp/bsp_cfg.h:28` |
| **`R_IOPORT_PinWrite`'s body is NOT in the installed package** | FSP ships precompiled here (`r_ioport.c` is absent; only `r_ioport.h`/`r_ioport_api.h` are present). `R_BSP_PinWrite` — the inline the FSP call is documented against — *does* read-modify-write and re-force `PDR = output` (`bsp_io.h:373-382`), but that is an inference, not a verified call chain | so §2.1 does **not** rely on it: `hal_pump_write()` uses the same one-word `R_IOPORT_PinCfg` form as the boot write, which **is** verified (`R_BSP_PinCfg`, `bsp_io.h:391-395`) |
| `attachInterrupt` hardcodes `filter_enable = false` | no hardware debounce from the Arduino wrapper | `cores/arduino/Interrupts.cpp:151` |
| …and already sets `pclk_div = EXTERNAL_IRQ_PCLK_DIV_BY_64` | so the filter **clock** needs no change; only `FLTEN` is missing | `Interrupts.cpp:150` — **corrects the design's §2.14** |
| the `CIrq`/`external_irq_cfg_t` objects live in a **file-static** `IrqPool` | nothing outside `Interrupts.cpp` can re-open the channel the core opened | `Interrupts.cpp:41-67` (`static IrqPool IrqChannel{}`) |
| `IRQManager::addPeripheral(IRQ_EXTERNAL_PIN, cfg)` with a fresh cfg **allocates a second NVIC vector** on the same ICU channel, with no `last_interrupt_index` bound check | unlike the SPI branch at `:769-771` | `IRQManager.cpp:747-756` — **this is why §2.14 sets `FLTEN` in `R_ICU->IRQCR[ch]` directly** |
| `R_ICU->IRQCR[ch]`: `FLTEN` bit 7, `FCLKSEL` bits 5:4 | the one register write the filter needs | `R7FA4M1AB.h:5794`, `:14783-14786` |
| `attachInterrupt` **preserves an existing pull-up** | so `pinMode(INPUT_PULLUP)` *before* it works | `Interrupts.cpp:182-183`, `has_pullup` read of `PmnPFS` |
| a later `pinMode` on an IRQ pin drops `IOPORT_CFG_IRQ_ENABLE` | silently detaching the interrupt | `digital.cpp:3-19` vs `Interrupts.cpp:183` |
| **external pin IRQ is IPL 12; the `millis()` AGT tick is IPL 8** | so a pin-edge storm **cannot** starve `millis()` — lower number wins on Cortex-M | `IRQManager.cpp:13` (`EXTERNAL_PIN_PRIORITY 12`), `time.cpp:43` (`agt_timer.setup_overflow_irq(8)`) — **corrects adversary 1** |
| `millis()` | `agt_time_ms`, incremented in the AGT0 underflow callback | `cores/arduino/time.cpp` |
| **`NO_USB` is defined for this board** | so `Serial` is `_UART1_` (bridged to the host's USB CDC through the ESP32), `Serial1` is `_UART2_`, and **`Serial2` is `_UART3_`** — the modem's UART | `platforms/renesas-ra/boards/uno_r4_wifi.json:10`, `boards.txt:122`, `Arduino.h:112-118` — see §12 |
| `analogReadResolution(bits)` only stores `_analogRequestedReadResolution` | the hardware width is fixed at open time to `BSP_FEATURE_ADC_MAX_RESOLUTION_BITS` (14) and `adcConvert` `map()`s hw→requested; the **default requested value is 10** | `analog.cpp:11,34-45,495,595-607` — so `hal_begin()` asserts `analogReadResolution() == PB_ADC_BITS` after setting it (`analog.cpp:698`) |
| `adcConvert` busy-waits on `ADC_STATE_SCAN_IN_PROGRESS` with **no timeout** | an unbounded core loop the §9 `for(;;)` grep cannot see; listed in "not tested on the host" | `analog.cpp:486-489` |
| `MODEM_TIMEOUT` | 10000 ms default; `ModemClass::timeout()` is **public** | `WiFiS3/src/Modem.h:12,66` |
| `CWifi::begin()` | `modem.begin()` + 2 × `modem.write()`, **then** polls `status()` while `millis()-start < _timeout` | `WiFi.cpp:43-67` |
| `WiFi.setTimeout(0)` | the poll loop body never runs; `begin()` returns `WL_CONNECT_FAILED` immediately | `WiFi.cpp:61,563` |
| `WiFi.ping()` | **resets `modem.timeout()` to 10000 ms** | `WiFi.cpp:585-593` — never call it |
| **`WiFiClient::connect()` on a stale socket is 5 AT round trips** | `_AVAILABLE` + `_CLIENTCONNECTED` + `_CLIENTCLOSE` + `_BEGINCLIENT` + `_CLIENTCONNECTNAME` | `WiFiClient.cpp:29-44, 53-70, 95-114, 211-220, 224-238` — **corrects the design's 3-write budget** |
| …but **2** when `_sock == -1` | `getSocket()`'s `if(_sock >= 0 && !connected())` short-circuits, leaving only `_BEGINCLIENT` | `WiFiClient.cpp:31` |
| `stop()` sets `_sock = -1` | so an always-called `sock_close()` guarantees the 2-round-trip path | `WiFiClient.cpp:217` |
| **`available()` is 1 AT** (`_AVAILABLE`) when the rx FIFO is empty, 0 when it is not | | `WiFiClient.cpp:95-114` |
| **`connected()` is 2 ATs**: it calls `available()` first, then `_CLIENTCONNECTED` | so the net FSM **never calls it** — the RECV deadline is the closed-socket detector instead (§3) | `WiFiClient.cpp:224-238` |
| **`read(buf,n)` is at most 1 AT**: `read_if_needed(n)` → `_read()` → `_CLIENTRECEIVE`, then the FIFO is drained locally | so `sock_read()` is `client.read(buf,cap)` and **nothing else** — one AT per RECV pass | `WiFiClient.cpp:145-182` |
| `WiFiClient::_read()` | asks for `freePositions()-1` = up to 1023 B into a `std::string`, every read | `WiFiClient.cpp:117-142`, size at `:122` |
| **`write(buf,n)` is 1 bounded round trip**: `write_nowait(_CLIENTSEND)` (no wait) + `passthrough()` (one `buf_read`) | so a SEND pass is one `PB_NET_STEP_MS` unit | `WiFiClient.cpp:79-91`, `Modem.cpp:51-70` |
| `setConnectionTimeout(int)` is **public**, and a non-zero value selects `_CLIENTCONNECT` over `_CLIENTCONNECTNAME`, appending the value to the AT command | **that is all the driver proves.** `_CLIENTCONNECT` and `_CLIENTCONNECTNAME` are two distinct ESP32-side entry points (`WiFiCommands.h:52-53`) and the ESP32 AT firmware is not in this package, so **neither the unit of the timeout nor whether `_CLIENTCONNECT` still resolves a hostname is checkable here** — both are bring-up questions (§3 change 3, §13) | `WiFiClient.h:63-64,74`, `WiFiClient.cpp:57-67` |
| `buf_read` leaves a late answer in the UART FIFO on timeout and does not drain before a command | AT session can go one answer out of phase | `Modem.cpp:100,182-198`; the `+`-branch strict check is commented out at `:235`, though `:267` does check a mismatched command name |
| **`ModemClass::end()` does NOT clear `beginned`** | so `modem.end(); modem.begin();` closes Serial2 and then **declines to reopen it** — `link_reset()` must write `modem.beginned = false` between them (`beginned` is a public member) | `Modem.cpp:45-48` vs `:35`, `Modem.h:40` — **corrects the design; this is the 48-hour run's recovery path** |
| `WiFiClient` ctor | `new FifoBuffer<uint8_t,1024>` on the heap, every construction | `WiFiClient.cpp:6-8`, `.h:76-77` |
| `TwoWire::flush()` | `while(bus_status != ...){}` — **unbounded** | `libraries/Wire/Wire.cpp:833-835` |
| `TwoWire` transfer timeout | fixed **1000 ms**, private member, no setter | `Wire.cpp:194`, `.h:172`, used at `:645,:690` |
| `LiquidCrystal_I2C::init_priv()` calls `Wire.begin()` and `delay(1000)` | so it must run **before** `sensors_begin()` | `LiquidCrystal_I2C.cpp:66,85,89` |
| LCD one character | `send` → 2 × `write4bits` → 3 × `expanderWrite` = **6 Wire transactions**; a 16-character row plus its `setCursor` (itself a `command()` → `send()`) is **102** | `LiquidCrystal_I2C.cpp:255-278` |
| `LiquidCrystal_I2C::print()` offers **no hook between characters** | so `ui.cpp` writes rows as a loop of `lcd.write(c)` with `safety_tick()` between characters, never `lcd.print()` (§5) | `Print.cpp`; enforced by a `make check` grep |
| neither `LiquidCrystal_I2C` nor u8x8 calls `Wire.flush()` | grepped `.pio/libdeps/uno_r4_wifi`: clean today | re-grep any library later added to this bus |
| main stack | **1024 B fixed** (`BSP_CFG_STACK_MAIN_BYTES 0x400`) | `bsp_cfg.h:26` |
| **`__HeapLimit == __StackLimit`, and `_sbrk` never reads either** | the linked `_sbrk` is libnosys's **unchecked** 28-byte version. Disassembled at 0xfd08 it is `ldr r2; ldr r1; ldr r3,[r2]; cmp #0; it eq; moveq r3,r1; add r0,r3; str r0,[r2]; mov r0,r3; bx lr` — no limit compare, no `-1` return. `__HeapLimit` is referenced by **nothing** in the image (one hit in `firmware.map`, its own definition). **So there is no heap bound at all**: `malloc` never fails from exhaustion, it hands out addresses inside and above the 1 KB stack. | `fsp.ld:260-261`; `firmware.map:257-258,3610-3611,4150-4151`; `arm-none-eabi-objdump -d` — **corrects the design's "comfortable" and is why §12 adds a break check** |
| a `.noinit` section exists and is not zeroed at startup | `KEEP(*(.noinit*))`, NOLOAD, placed **immediately after `.data`** at `__noinit_start = 0x200000cc` — the very bottom of SRAM | `fsp.ld:222-231`, `firmware.map:3950`. **That it survives the bootloader across a warm reset is NOT verified here** — the UNO R4 WiFi runs a bootloader before the sketch (sketch `.text` at 0x4000; `variant.cpp:119` says it configures LED_BUILTIN as PWM, i.e. it executes with its own stack and variables) and nothing in the package says what SRAM it uses. **Bring-up 7c measures it** (§2.3, §13). |
| heap arena | `__HeapBase` 0x20001650 … `__HeapLimit` 0x20007b00 = **25,776 B** — a distance, **not** an enforced bound (row above) | `firmware.map:4150` |
| today's static | `.bss` **0x1568 = 5480 B** + `.data` 0xcc = 204 B (NOT the 14980 `size` prints — that folds in `.heap` 0x2000 and `.stack_dummy` 0x400) | measured `firmware.map` |

**The WDT grant, re-derived rather than quoted.** This design does **not** call `begin(uint32_t)`
(§2.5 says why), so `getPrescaler`/`getReload` never run and no `PB_WDT_MS` constant exists. The grant
is fixed by the two enums the `wdt_cfg_t` carries:

```
RL_16384 × PR_8192 / (PCLKB / 1000) = 16384 × 8192 / 24000 = 5592 ms
```

which is exactly what `hal_wdt_granted()` computes at run time and what `PB_WDT_GRANTED_MS` asserts
against (`WDT.cpp:105-113`, `bsp_clock_cfg.h:8,14`). The counter therefore decrements at
`PCLKB / 8192 = 2929.7 Hz` — **2.93 counts per millisecond**, the number `hal_wdt_alive()` measures
against (§2.5).

### Verified backend facts

| fact | consequence |
| --- | --- |
| `_int_in` bounds are **half-open** `[low, high)`, ASCII digits only (`butler.py:192-201`) | `float=` is `_int_in(v,"float",0,2)` → **0 or 1 only**. A leading `-`, a `+`, or any non-digit refuses the **whole report**, so no field may ever carry a sentinel. |
| `ack` is `_int_in(value,"ack",1,2**63)` (`:230`) | `ack=0` 400s the whole report. Never emit it; reject `cmd=0`. |
| `t` is `_int_in(value,"t",0,2**63)` (`:226`) | a `uint32_t` printed `%lu` is **always** in range — see §15.2 |
| `flow_ml=` without `ack=` raises (`:256-257`) | structurally paired in the builder |
| **every key may appear at most once**: a repeated `c`, `t`, `ack`, `flow_ml`, `float`, `pos` or `chN` refuses the whole report (`:220-221,:224-225,:228-229,:232-233,:236-237,:240-241,:249-250`) | the body is assembled from four independent sources into one buffer — §4.1 makes it a rule and §9 a test |
| `chN` requires `key[2:].isdigit()` and `_int_in(v, key, 0, MAX_RAW)`, `MAX_CHANNEL = 255`, `MAX_RAW = 2**31` (`:248-251,:87,:88`) | no negatives; **ch200–ch209 are legal and free**; every diagnostic must be clamped below 2^31 |
| `no chN= in the report` raises (`:254-255`); `no c= in the report` raises (`:252-253`) | a report with no channel, or an empty `c=`, is a permanent 400 |
| unknown keys ignored on purpose (docstring `:16`) | `err=` rides free — and reaches no storage and no alert rule (§16.5.4) |
| `readings` is only ever ordered `ts DESC, rowid DESC` (`:716,:1079`) | **`t` need not be monotonic** — only unique per controller inside the dedup window |
| ack UPDATE (`:829-833`) runs **before** the unconditional expire (`:837-841`) | a pending ack may safely ride a later report — but only while the row is still `sent`; see the TTL row below |
| dedup is `(controller, t)` matched against the **original's arrival `ts`**, `ts >= now - RETRY_WINDOW_S` (`:846-854`) | byte-identity is **sufficient, not necessary**: same `c=`, same `t=`, inside 300 s of the first one's *arrival*. A duplicate suppresses only the `readings` insert and `water_rules` — it still updates `last_seen`, the whole `status` row (`float_bad`/`float_bad_prev`, `pos_bad`/`pos_bad_prev`), applies the ack UPDATE, runs the unconditional expire, and can be handed a queued command. |
| **six expiry paths, not one**: the unconditional `state='sent'` expire (`:837-841`); `queued AND created_ts < now - cmd_ttl` (`:842-846`); `queued\|sent AND COALESCE(sent_ts, created_ts) < now - cmd_ttl` in `enqueue` (`:887-891`) and `approve` (`:1011-1015`); the `proposed` TTL sweep in `water_rules` (`:675-678`) and `approve` (`:994-998`) | the ack-carry window is bounded by `BUTLER_CMD_TTL_S` (900 s default, enforced ≥ 2 × `BUTLER_NEXT_S` at startup, `:586-599`), not by the report interval — §4.3 |
| `MAX_DOSE_ML = 1000` (`:89`), `MAX_CAP_S = 60` (`:90`), `cap_for(ml) = min(60, ml//20+5)` (`:261-266`) | firmware protocol ceilings must be ≥ these. Nothing tells butler the **rig** ceiling — §4.6, §16.5.8. |
| `water=` is `_int_in(value,"water",0,MAX_CHANNEL+1)` and the `outlet is None` guard does not catch 0 (`:291,:311-312`) | **`water=0` is a legal backend command** — see §2.8 on why `dose_req_t.outlet == 0` must not be an overloaded sentinel |
| `FLOW_FLOOR_ML_S = 20` is "worst-case pump flow; bench-rig-tunable" (`:96`) | at a real 30 ml/s, `cap_for(500) = 30 s` authorises **1.8×** the requested water — §2's measured clamp exists for this |
| alert: `acked_ts is None` → **HIGH** "handed to the board and never acknowledged" (`:1361-1364`) | never let a received command expire unacked |
| alert: `2 * flow_ml < ml` → **HIGH** (`:1367-1369`) | acking a refusal with `flow_ml=0` is the loud path. **Same priority and same `warning,droplet` tags as the row above** — the difference is the message text and the daily-cap charge, not loudness. |
| daily cap sums `COALESCE(flow_ml, ml)` over `sent_ts IS NOT NULL` (`:745-751`) | an **acked** refusal charges 0 ml; an **expired** one charges the full `ml`. An `ack=` **without** `flow_ml=` stores NULL (`:830`) and is therefore charged the full `ml` and skips the `2*flow_ml < ml` branch entirely — so every `ack=` must carry a `flow_ml=`. |
| cooldown is `COALESCE(acked_ts, sent_ts) > now - cooldown_s` (`:736-742`) | an acked refusal **does** set the pot's cooldown — see the float-flap trap in §4 |
| `water_rules` returns early on `r.float_ok != 1 or r.pos != "ok"` (`:680-681`) | `pos=unknown` keeps the whole rules ladder dark |
| the `float:` and `pos:` alerts raise on `flapped` — `bad` **and** `bad_prev` both set and both inside `FLAP_WINDOW_S` (`:1267-1275`) | a condition present on **every** report sets both from the second report onward, raises **once**, and then **stands forever** (`raised(key)` suppresses a re-raise). See §4.6 for what `PB_REPORT_POS_UNKNOWN` does to this. |
| the `float:` alert text is fixed: "the reservoir on X is empty or at the waterline: watering is on hold" (`:1246-1249`) | so a contradiction latch surfaced as `float=0` says "reservoir empty" whatever the cause — §2.7 |
| 503 is only `sqlite3.OperationalError` → whole `BEGIN IMMEDIATE` rolled back (`:1638-1639`) | a 503 is provably safe to retry once. **Any other non-200, 500 included, is not**: anything else raised in the threadpool becomes a FastAPI 500 with no rollback guarantee. |
| the silence rule iterates `controllers WHERE last_seen > 0` (`:1137`) | **there is no retirement** — see §16.5 |
| `commands.id` is `INTEGER PRIMARY KEY` — a rowid alias, **no `AUTOINCREMENT`** | ids are monotonic only because `schema.sql` says rows are never deleted. A rebuilt or restored database restarts at 1 — see §4.3's replay guard and its recovery |
| the response is `next=<n>\n` then optionally `cmd=<id> water=<o> ml=<m> cap_s=<c>\n` or `cmd=<id> stop=1\n` (`:1640-1647`) | exactly what `response_parse` must accept |

---

## 1. Architecture

Two seams, both plain C free functions — no vtables, no templates, no dependency injection.
Implementations are selected by `build_src_filter`, not by a runtime flag.

```
                       ┌──────────────────────────────────────────────┐
   host tests  ────────│  report.cpp  netfsm.cpp  safety.cpp  cart.cpp │
   (env:native)        │  pulses.cpp  sensors.cpp  cli.cpp  ui.cpp     │   ALL LOGIC
                       └───────┬──────────────────────────┬───────────┘
                    seam 1 ────┤ include/hal.h            │ include/link.h ──── seam 2
                       ┌───────┴───────┐          ┌───────┴──────────┐
   env:uno_r4_wifi     │ hal_uno.cpp   │          │ link_wifi.cpp    │  (lib/Network)
   env:sim / native    │ hal_sim.cpp   │          │ link_fake.cpp    │
                       └───────────────┘          └──────────────────┘
```

**Every one of those modules is a `.cpp`, not a `.c`** — the seams are plain C free functions, the
files are not. A `.c` file gets `-std=gnu11` from the renesas-ra builder
(`builder/frameworks/arduino.py:100`), so it would have no `static_assert`, and any `-std=gnu++*` in
`build_flags` lands in SCons `CCFLAGS` and is applied to C compilation too, producing one
`cc1: warning: command line option '-std=gnu++1z' is valid for C++/ObjC++ but not for C` per core
`.c` file on every build — enough noise to bury a real warning. §10 keeps the standard out of
`build_flags` for the same reason.

`src/hal_uno.cpp` is the only translation unit in the project that includes `<Arduino.h>`, names a
pin, owns an ISR, or writes D6. `lib/Network/src/link_wifi.cpp` is the only one that names `WiFiS3`.
`lib/Screen` is the only one that names `LiquidCrystal_I2C` or `u8x8`. Everything else compiles on
the host unchanged.

### Module table

| module | purpose | native? |
| --- | --- | --- |
| **`include/pins.h`** | The wiring contract as constants. `PIN_PUMP_EN` is defined **only** when `PB_PUMP_OWNER` is defined, and `#error`s otherwise. One polarity knob, and it is **mandatory**. | yes |
| **`include/config.h`** | Every tunable, every measured constant, and the citation that justifies it (§7). | yes |
| **`include/hal.h`** (seam 1) | 32 free functions. **Arduino-free**: its own `PB_LOW/PB_HIGH/PB_IN/PB_OUT`, numeric pins, no `min()` macro, no `HIGH`/`LOW`/`A0`. | yes |
| **`src/hal_uno.cpp`** | The real board. Pin numbers, polarity, two ISRs, Servo, ADC, Wire, WDT, the stack paint. No logic, no state machine. Not compiled in `sim` or `native`. | **no** |
| **`src/hal_sim.cpp`** | One fake, two jobs: the on-device sim **and** the host test double. Models tank, pump, flow pulses at a settable ml/s after a prime delay, screw + home region, expander, six channels, a settable WDT counter, a settable UART. | yes |
| **`include/safety.h` / `src/safety.cpp`** | **Read this file first.** Owns D6's whole story: `safety_tick()`, `safety_wait()`, the float debounce, the dry latch, **the contradiction latch**, and `dose_run()` — the only caller of `hal_pump_write()`. Includes neither `link.h`, `Network.h` nor `WiFiS3.h`. | yes |
| **`include/pulses.h` / `src/pulses.cpp`** | The two interrupt counters, per-pin minimum-gap reject in the ISR, torn-read-safe snapshots, pulses→ml, the rate estimator, the plausibility ceiling, the leak watch with a coast-down blanking window. | yes |
| **`include/sensors.h` / `src/sensors.cpp`** | PCF8575 (P4..P15 always written HIGH so the home hall stays readable), the mux discipline, the **open-channel canary**, the home-hall read, I2C health with back-off, the bounded nine-clock recovery, the `i2c` scan. Every failure returns `false`, never a value. | yes |
| **`lib/Manifold`** (reworked in place) | `class Cart` — position by counted screw pulses + the home hall. Every `delay()` becomes (target pulses, deadline, stall window). No `Screen*`. `test()` and `reset()`'s one-gate-width guess deleted. | yes |
| **`include/report.h` / `src/report.cpp`** | The wire protocol, pure. Builds the body into a caller's buffer; parses the response. Where the traps live (§4). | yes |
| **`include/netfsm.h` / `src/netfsm.cpp`** | The report state machine and the HTTP framing, above seam 2. **One AT-bounded link/socket step per `net_poll()`.** Socket closed on **every** exit, including a failed open. | yes |
| **`include/link.h`** (seam 2) | 10 network primitives. | yes |
| **`lib/Network`** (reworked in place) | `link_wifi.cpp`: the WiFiS3 driver, and nothing else. `NetworkClient`, its `std::map`/`String` API, the two `while(true)` spins and the leaked socket are gone. | **no** |
| **`lib/Screen`** (kept, reworked) | Same class, same two types, plus `probe()`/`present()`. A screen that does not answer becomes a no-op forever. Never calls `Wire.flush()`. | **no** |
| **`include/ui.h` / `src/ui.cpp`** | Pure `ui_render()` / `ui_render_lcd()` fill `char[8][17]` and `char[2][17]`; `ui_poll()` paints changed rows, on the **coarsened** cadence of §5. | yes |
| **`include/cli.h` / `src/cli.cpp`** | The bench commands always; the bring-up commands under `#if PB_BRINGUP`. A flat if-chain. | yes |
| **`src/main.cpp`** | Setup order, the five-line loop, `exec_pending()`. | **no** |

**`include/hal.h`** — seam 1, verbatim:

```c
/* hal.h — the hardware seam. NOTHING here names an Arduino type. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#define PB_LOW 0
#define PB_HIGH 1
#define PB_IN 0
#define PB_OUT 1

uint32_t hal_millis(void);
void     hal_delay_us(uint16_t us);        /* the ONLY sub-ms wait; never > 200 us */
void     hal_pin_mode(uint8_t pin, uint8_t mode);   /* NEVER for D2, D3 or D6 */
int      hal_pin_read(uint8_t pin);
void     hal_pin_write(uint8_t pin, uint8_t level);

void     hal_boot_pump_off(void);          /* ONE PFS write: direction AND level. setup()'s 1st stmt */
void     hal_pump_write(bool on);          /* the ONLY route to D6 after boot */
bool     hal_pump_level_on(void);          /* what PUMP_ON compiled to; `status` prints it */

uint16_t hal_adc_read(void);               /* A0, 14-bit */
bool     hal_i2c_write16(uint8_t addr, uint16_t bits);  /* false == bus error */
bool     hal_i2c_read16(uint8_t addr, uint16_t *bits);  /* false == bus error, NOT zero */
bool     hal_i2c_probe(uint8_t addr);
bool     hal_i2c_recover(void);            /* EXACTLY nine clocks, fixed count; refuses while dosing */
void     hal_servo_us(uint16_t us);        /* 1500 == stop; 0 == detach */

bool     hal_wdt_start(void);              /* wdt_cfg_t overload; false if the core rejected it */
uint32_t hal_wdt_granted(void);            /* OUR computed grant, not WDT.getTimeout() — see §2 */
uint32_t hal_wdt_counter(void);            /* the raw down-counter; the sim makes it settable */
bool     hal_wdt_alive(void);              /* counter DECREASED across an UNFED window — §2.5 */
uint32_t hal_wdt_last_delta(void);         /* what the last probe measured; rides out as ch209 */
void     hal_wdt_feed(void);               /* ONE caller: safety_tick(). NOT called by the probe. */

bool     hal_irq_armed(uint8_t pin);       /* IELSR scan + NVIC enable for the pin's ICU channel */
bool     hal_irq_filtered(uint8_t pin);    /* IRQCR[ch] FLTEN; `status` prints icufilter= from it */

size_t   hal_serial_read(char *buf, size_t cap);
void     hal_serial_write(const char *s);
void     hal_serial_drain(void);           /* discard the RX ring; returns nothing, prints a count */
uint32_t hal_heap_arena(void);             /* mallinfo().arena  — break growth */
uint32_t hal_heap_ordblks(void);           /* mallinfo().ordblks — free-chunk count */
uint32_t hal_heap_break(void);             /* (uint32_t)sbrk(0) — the ONLY real heap bound (§12) */
uint32_t hal_stack_limit(void);            /* (uint32_t)&__StackLimit */
uint32_t hal_stack_hwm(void);              /* bytes of the 1024 used, from the boot paint */
uint32_t hal_boot_salt(void);              /* per-boot, from the .noinit boot counter */
void     hal_begin(void);                  /* ADC width, input pins, ISRs, Wire, servo, stack paint */
```

`hal_heap_break()` is `(uint32_t)sbrk(0)`. If that will not link, the fallback is
`(uint32_t)&__HeapBase + hal_heap_arena()` — nano-malloc's `arena` is the total obtained from `sbrk`
— and `status` says which form shipped.

**`include/link.h`** — seam 2, verbatim:

```c
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

typedef enum { LINK_DOWN, LINK_JOINING, LINK_UP } link_state_t;

void         link_begin(uint32_t step_ms);  /* modem.timeout(); modem.begin(); WiFi.setTimeout(0) */
void         link_join(void);               /* 2 ATs, returns; does NOT spin. Return value ignored. */
link_state_t link_state(void);              /* ONE bounded status query */
int8_t       link_rssi(void);
const char  *link_ip(void);                 /* into a static char[16] */
void         link_reset(void);              /* end(); beginned = false; begin(); counts a desync */
uint16_t     link_desyncs(void);
bool         sock_open(void);               /* HOST_NAME:HTTP_PORT; PRECONDITION: sock_close() ran */
int          sock_write(const uint8_t *b, size_t n);   /* one write, one modem round trip */
int          sock_read(uint8_t *b, size_t cap);        /* -1 closed, 0 nothing yet */
void         sock_close(void);              /* idempotent; called on EVERY exit, failed open included */
```

`link_begin()` calls `modem.begin()` **explicitly, once, in `setup()`**. `ModemClass::begin` is
guarded by `beginned` (`Modem.cpp:35`) and issues a `_SOFTRESETWIFI` on its first call; draining that
one-time cost into `setup()` is what makes `link_join()` exactly two AT commands rather than three.

**`link_reset()` must clear `beginned` itself**, and this is the single line the 48-hour run depends
on. `ModemClass::end()` is `void ModemClass::end(){ _serial->end(); }` (`Modem.cpp:45-48`) — it
closes Serial2 and **never clears `beginned`** — while `begin()` is
`if(_serial != nullptr && !beginned) { ... }` (`:35`). So the obvious
`modem.end(); modem.begin();` closes the UART and then declines to reopen it: every subsequent AT
command writes into a closed UART, `buf_read` times out at `PB_NET_STEP_MS` forever, `ch206` climbs,
and the board silently stops reporting — exactly the failure §3 says this prevents. `beginned` is a
**public** member (`Modem.h:40`), so:

```c
void link_reset(void) {
  modem.end();
  modem.beginned = false;      /* Modem.cpp:45-48 never does this. Without it, begin() no-ops. */
  modem.begin();               /* re-issues _SOFTRESETWIFI, which is the point of the reset */
  g_desyncs++;
}
```

§9 has a host test that a **second** `link_reset()` still produces a working AT round trip, and the
device test that pulls the AP mid-connect is the one that would have caught this.

### Two things that are deliberately NOT here

- **No DHT.** It is not in the wire protocol (`chN` requires `isdigit()`), and a bit-banged DHT11
  frame masks interrupts for ~20 ms, dropping flow edges (6 L/min = 588 pulses/s, 1.7 ms apart) and
  screw edges. A dropped screw edge is lost cart position, silently. D7 stays free.
- **No leak *latch*.** Pulses with the pump off raise `ch205` and `err=leak` in `status`; they do
  **not** block a dose. A latch with no clear bricks the rig on the first stray pulse during 7b
  priming. (The contradiction latch of §2 is a different thing: it is armed by two sensors
  disagreeing during an actual dose, and it has a clear.)

---

## 2. The safety chain around D6

### 2.0 What a refusal is, and what it is not

**Every refusal in this document refuses watering and nothing else.** The dry latch, the
contradiction latch, an unknown cart position, a float that reads not-OK, a stale I2C read, a dog
that will not tick — each one stops a dose. None of them stops a sensor read, a mux sweep, a
report, a retry, an ack, a screen repaint or a `status`. Refusing is a decision about the pump,
never about the instruments.

This is load-bearing rather than tidy. A latch reached exactly when something has gone wrong is a
latch reached exactly when the backend most needs data: `float=`, `err=`, the diagnostic channels
and the readings themselves are how the phone learns there is a problem at all, and how the history
shows what the tank and the pots were doing on either side of it. A latch that silenced the report
would blind the alerting it exists to trigger, and would look — from the NAS — identical to a board
that had died.

Two consequences the rest of the document holds to. Homing is not watering: `cart_home()` and the
boot self-home run under every latch, because parking the cart off every gate is what you want after
a refusal, not something to withhold (§2.9, §2.11). And the report cycle has no latch-aware branch
anywhere in it — §4 never reads `g_nv.contra_latched` or `g_nv.dry_latched` to decide *whether* to
send, only to decide what `float=`, `err=` and `ch207` say inside a report that is sent regardless.


### 2.1 The boot order — the wiring README's recipe is wrong for this silicon

`cad/wiring/README.md` says, twice (line 23 and the *Pump driver* notes):

> In the sketch, set the level BEFORE the direction: `digitalWrite(D6, <OFF level>); pinMode(D6,
> OUTPUT)`. The other order glitches the pin to the ON level at boot and kicks the pump.

On the RA4M1 with this core that recipe **inverts**. `pinMode(pin, OUTPUT)` is
`R_IOPORT_PinCfg(NULL, g_pin_cfg[pin].pin, IOPORT_CFG_PORT_DIRECTION_OUTPUT)` (`digital.cpp:12-14`;
lines 14-16 are the `OUTPUT_OPENDRAIN` case), and
`R_IOPORT_PinCfg` bottoms out in `R_BSP_PinCfg`, which is an **unconditional whole-register
assignment**: `R_PFS->PORT[..].PIN[..].PmnPFS = cfg;` (`bsp_io.h:391-395`). `cfg` is `0x4` —
`IOPORT_CFG_PORT_DIRECTION_OUTPUT` alone. `IOPORT_CFG_PORT_OUTPUT_HIGH` is `0x1`, a bit in the same
word that `pinMode` never sets (`r_ioport_api.h:184,186`). So `pinMode(D6, OUTPUT)` latches PODR = 0
and **drives D6 LOW**, discarding whatever `digitalWrite` had just put there. The core's own code
proves the idiom by ORing the level bit in explicitly whenever it wants a pin to come up high:
`Arduino_LED_Matrix.h:125`, `SoftwareSerial.cpp:228,232`, `pin_data.c:14`.

If bring-up 4a reads an **active-LOW** relay module (`PUMP_ON = LOW`), the wiring README's own
sequence therefore asserts the pump as `setup()`'s first statement, and holds it through
`Serial.begin`, `hal_begin()`, the screens' `init_priv()` (which contains `delay(1000)`,
`LiquidCrystal_I2C.cpp:89`), `sensors_begin()` and `hal_wdt_start()` — hundreds of ms to several
seconds of unconditional pumping with no float check, no cap and no meter. And a watchdog reset is a
boot, so the last line of defence would become a pump-restart pulse generator.

**The correct sequence is one PFS write that carries direction and level together, and `pinMode`
never touches D6 at all:**

```c
/* src/hal_uno.cpp. Do NOT re-declare g_pin_cfg here: Arduino.h:60-66 already declares it,
   INSIDE an extern "C" block, and a second declaration in a .cpp without extern "C" is a
   hard error ("declaration of 'g_pin_cfg' with 'C++' linkage conflicts with previous
   declaration with 'C' linkage"). hal_uno.cpp includes <Arduino.h>; that is the declaration. */

#define PB_PUMP_PFS_OFF   ((uint32_t)(IOPORT_CFG_PORT_DIRECTION_OUTPUT | PUMP_OFF_PFS_LEVEL))
#define PB_PUMP_PFS_ON    ((uint32_t)(IOPORT_CFG_PORT_DIRECTION_OUTPUT | PUMP_ON_PFS_LEVEL))
/* PUMP_*_PFS_LEVEL is IOPORT_CFG_PORT_OUTPUT_HIGH (0x1) or 0, from pins.h's polarity knob. */

void hal_boot_pump_off(void) {          /* setup()'s FIRST statement */
  /* ONE register write. Direction = output AND level = the module's OFF level, atomically.
     BSP_CFG_PARAM_CHECKING_ENABLE is 0 (bsp_cfg.h:28), so the NULL p_ctrl digital.cpp
     passes is safe, and we match it. */
  R_IOPORT_PinCfg(NULL, g_pin_cfg[PIN_PUMP_EN].pin, PB_PUMP_PFS_OFF);
}

void hal_pump_write(bool on) {
  /* THE SAME whole-word form as the boot write, deliberately. R_IOPORT_PinCfg -> R_BSP_PinCfg
     is one unconditional `PmnPFS = cfg` (bsp_io.h:391-395), so every pump write re-states the
     DIRECTION as well as the level — which is what makes safety_tick()'s idle re-assert a
     REPAIR of a stray pinMode on D6 and not merely a re-statement of the level. */
  R_IOPORT_PinCfg(NULL, g_pin_cfg[PIN_PUMP_EN].pin, on ? PB_PUMP_PFS_ON : PB_PUMP_PFS_OFF);
}
```

**Why not `digitalWrite` or `R_IOPORT_PinWrite` here.** Not because they are wrong — because they are
**unverifiable from the installed package**. `r_ioport.c` is not shipped (FSP is precompiled in this
package; only the headers are present), so what `R_IOPORT_PinWrite` actually does to `PDR` cannot be
read. `R_BSP_PinWrite` — the inline the FSP call is documented against — *does* re-force
`BSP_IO_PFS_PDR_OUTPUT` (`bsp_io.h:373-382`), but "the FSP wrapper must call the BSP inline" is an
inference. `R_IOPORT_PinCfg`'s path to a whole-word `PmnPFS = cfg` **is** readable, and the pin that
starts the pump is the wrong place to spend an inference. The cost of the choice is nil: one register
write either way.

`tools/check.sh` gains **zero hits for `pinMode(PIN_PUMP_EN`**, **zero for
`digitalWrite.*PIN_PUMP_EN` and `R_IOPORT_PinWrite.*PIN_PUMP_EN`**, and **exactly two
`R_IOPORT_PinCfg` call sites naming `PIN_PUMP_EN`** — `hal_boot_pump_off` and `hal_pump_write`, and
no others.

**What actually makes the boot write safe is not that it is first.** The core runs `initVariant()`
and `Serial.begin(115200)` before `setup()` (`main.cpp:107-115`), so "before `Serial.begin()`" is not
the property being relied on. The property is that **nothing before `setup()` writes D6**:
`initVariant`'s pin-6 call is `set_initial_timer_channel_as_pwm`, which only writes a bookkeeping
array (`FspTimer.h:132-139`). That is worth stating because it is the thing a core bump could break,
and a core bump is exactly what bring-up 4a's power-cycle criterion re-tests.

Bring-up 4a gains a second pass criterion (§13): COM–NO must stay
open across a **power cycle** *and* across a `hang`-forced watchdog reset, not only across a
`pump 2000`. `cad/wiring/README.md` owes the corrected recipe (§16.1) — this is the single most
important line this work sends back to another repo.

### 2.2 Where D6 is written

`src/hal_uno.cpp` is the **only file in the tree** that contains `PB_PUMP_OWNER`, and therefore the
only file for which `include/pins.h` defines `PIN_PUMP_EN` at all:

```c
/* include/pins.h */
#if !defined(PB_RELAY_ACTIVE_LOW) && !defined(PB_RELAY_ACTIVE_HIGH)
#  error "Relay polarity is unknown until bring-up 4a. Define PB_RELAY_ACTIVE_LOW or \
PB_RELAY_ACTIVE_HIGH in platformio.ini build_flags after you have READ THE MODULE."
#endif
#ifdef PB_PUMP_OWNER
#  define PIN_PUMP_EN 6
#  ifdef PB_RELAY_ACTIVE_LOW
     /* PFS level bits, not BSP_IO_LEVEL_*: these are ORed into a whole-word PmnPFS write.
        IOPORT_CFG_PORT_OUTPUT_HIGH = 0x1, IOPORT_CFG_PORT_OUTPUT_LOW = 0 (r_ioport_api.h:185-186). */
#    define PUMP_ON_PFS_LEVEL  0
#    define PUMP_OFF_PFS_LEVEL IOPORT_CFG_PORT_OUTPUT_HIGH
#  else
#    define PUMP_ON_PFS_LEVEL  IOPORT_CFG_PORT_OUTPUT_HIGH
#    define PUMP_OFF_PFS_LEVEL 0
#  endif
#endif
```

There is **no default** for the polarity. A board cannot be flashed before someone has read the relay
module. `status` prints the compiled level so bring-up 4a can confirm it.

> **The boot readback is not a defence and is not implemented.** R1 already holds D6 at the module's
> OFF level, so a readback agrees with `PUMP_OFF` whether the pin drives correctly, whether the
> `PinCfg` silently failed, or whether the polarity is inverted-but-consistent. The `#error`, the
> `status` line and bring-up 4a done dry with no 12 V on COM are the real defence.

### 2.3 The `.noinit` block — what survives a warm reset

SRAM survives a watchdog or RESET-button reset; only the startup code clears `.bss`, and `fsp.ld`
provides a `.noinit` NOLOAD section (`:222-231`) that it does not clear. One struct, ~44 bytes, with
a magic word **and a checksum**:

```c
typedef struct {
  uint32_t magic;            /* PB_NOINIT_MAGIC; anything else == cold boot, zero the struct */
  uint32_t boots;            /* incremented every warm boot; feeds hal_boot_salt() */
  uint32_t cmd_high_water;   /* highest command id ever accepted (§4 replay guard) */
  bool     dry_latched;      /* the operator's `dry on` */
  bool     contra_latched;   /* the float/flow contradiction latch, §2.7 */
  bool     dose_in_flight;   /* set before the ON write, cleared after the OFF write */
  uint8_t  _pad;
  uint32_t sum;              /* XOR of every word above; recomputed on EVERY write */
} pb_noinit_t;
static pb_noinit_t g_nv __attribute__((section(".noinit")));
```

At boot: if `magic` mismatches **or the checksum does not verify**, zero the struct and set both — a
cold boot, or a partial clobber, starts clean. If it verifies, **restore** `dry_latched`,
`contra_latched` and `cmd_high_water`, and if `dose_in_flight` was set, **latch dry** and set
`g_last_err = "resetmid"`, which rides out as `err=resetmid` until a human types `dry off`. A reset
with the pump asserted is the single loudest thing this rig can discover about itself, and without
this it discovers nothing.

**Survival across a warm reset is a MEASUREMENT, not a claim, and it has not been made yet.**
`__noinit_start = 0x200000cc` (`firmware.map:3950`) is immediately above `.data`, at the very bottom
of SRAM — which is exactly where a second-stage image puts its own `.data`/`.bss`. The UNO R4 WiFi
runs a bootloader before the sketch (sketch `.text` sits at 0x4000, and `variant.cpp:119` says in as
many words that the bootloader configures LED_BUILTIN as a PWM output, i.e. it executes code with its
own stack and variables), and **nothing in the installed package says what SRAM that bootloader
uses**. Four things rest on this: the dry latch, the contradiction latch, `dose_in_flight` →
`err=resetmid`, and the replay high-water mark.

So: **bring-up 7c gains a `.noinit` pattern test** (§13). Write a known pattern into the struct,
force the watchdog reset it already forces, and print the struct on the next boot. Until that passes,
every `.noinit` guarantee in this document is **best-effort**, the checksum above is what keeps a
partial clobber from reading as a valid latch, and **§16.5.3 — the backend's durable half of the
contradiction latch — is a requirement rather than a follow-up.**

**Even when it passes, this is not durability.** A power cycle, a brown-out and a reflash all clear
SRAM. `.noinit` buys the warm-reset case only — which is exactly the case the watchdog creates, and
exactly the case where losing an operator's `dry on` is most dangerous. See §2.7.

### 2.4 `safety_tick()` — the ordering that carries the argument

```c
static bool g_dosing;                    /* true only between the ON and OFF writes */

void safety_tick(void) {
  if (!g_dosing) hal_pump_write(false);  /* idle ACTIVELY re-asserts OFF, every pass */
  hal_wdt_feed();                        /* the ONE caller of WDT.refresh() */
}
```

You cannot feed the dog without having just re-asserted the pump's idle state, and because
`hal_pump_write()` is the whole-word `R_IOPORT_PinCfg` form of §2.1 — direction **and** level in one
`PmnPFS = cfg` — that re-assert repairs a stray `pinMode` on D6 as well as a stray level.
`safety_tick()` is called at the top of `loop()`, inside `safety_wait()`'s loop, and inside
`dose_run()`'s loop — nowhere else.

**There is exactly one place in the program where feeding is deliberately suspended**, and it is
`hal_wdt_alive()`'s probe (§2.5). Nothing else may skip a `safety_tick()`.

### 2.5 The watchdog — honest naming, and a grant we compute ourselves

`hal_wdt_start()` uses the **`wdt_cfg_t` overload** so that `stop_control` can be
`WDT_STOP_CONTROL_DISABLE`; the `uint32_t` overload hardcodes `WDT_STOP_CONTROL_ENABLE`
(`WDT.cpp:67`), which `r_wdt_api.h:114` defines as "count will automatically stop when device enters
sleep mode" — a dog that a future `__WFI` could silently stop.

```c
bool hal_wdt_start(void) {
  wdt_cfg_t cfg = {};                             /* NINE members (r_wdt_api.h:147-160): p_callback,
                                                     p_context and p_extend follow stop_control.
                                                     `= {}` so no stack garbage reaches R_WDT_Open. */
  cfg.timeout        = WDT_TIMEOUT_16384;         /* reload 16384 */
  cfg.clock_division = WDT_CLOCK_DIVISION_8192;   /* prescaler 8192 */
  cfg.window_start   = WDT_WINDOW_START_100;
  cfg.window_end     = WDT_WINDOW_END_0;          /* refresh legal at any point */
  cfg.reset_control  = WDT_RESET_CONTROL_RESET;
  cfg.stop_control   = WDT_STOP_CONTROL_DISABLE;  /* the reason for this overload */
  return WDT.begin(cfg) == 1;
}
```

**`WDT.getTimeout()` cannot be used here.** `_timeout` is assigned only inside `getReload()`, which
only `begin(uint32_t)` calls (`WDT.cpp:59,153`); the config overload (`:32-46`) never touches it. So
under this overload `getTimeout()` returns 0 even on a perfectly running dog — which, in the design
as written, would have made `dose_run()`'s first guard refuse **every dose forever**. `hal_wdt_granted()`
therefore computes the grant itself and asserts it against the hardware at boot:

```c
uint32_t hal_wdt_granted(void) {
  if (!g_wdt_running) return 0;
  return (16384u * 8192u) / (R_FSP_SystemClockHzGet(FSP_PRIV_CLOCK_PCLKB) / 1000u);  /* 5592 */
}
```

`setup()` asserts the returned value equals `PB_WDT_GRANTED_MS` (5592) and, if it does not (a core
bump, a different PCLKB), **disables the network and says why in `status`** — a board that reports
nothing is better than one that reset-loops through a router reboot.

**Liveness, not a constant — and the probe must NOT feed.** `getTimeout()` — even used correctly —
proves only that `R_WDT_Open` returned `FSP_SUCCESS`. `hal_wdt_alive()` proves the counter moves. But
"moves" has to be measured across a window in which **nothing reloads it**:

> `R_WDT_Refresh` reloads the down-counter to 16384. A probe that samples the counter, waits through
> `safety_wait_ms()` — which calls `safety_tick()` and therefore `hal_wdt_feed()` on **every**
> iteration, thousands of times faster than the counter's 341 µs tick — and then samples again reads
> ≈16384 both times, on a perfectly healthy dog. It would return **false always**, and this function
> is `dose_run()`'s first guard and `setup()`'s assertion: a false here disables the network, latches
> dry and refuses every dose forever. That is the same shape as the `getTimeout()`-returns-0 bug this
> section was written to correct, and it is why the probe below is the one place in the program where
> feeding is suspended.

```c
/* The ONE place in the program that deliberately does not feed. Precondition: !g_dosing —
   it is called from setup() and as dose_run()'s first guard, after the g_dosing check. */
bool hal_wdt_alive(void) {
  hal_wdt_feed();                              /* start from a known reload */
  uint32_t a  = hal_wdt_counter();
  uint32_t t0 = hal_millis();
  while (hal_millis() - t0 < PB_WDT_PROBE_MS)  /* 40 ms, UNFED, pump already idle-OFF */
    hal_pump_write(false);                     /* the safety half of safety_tick(), without the feed */
  uint32_t b = hal_wdt_counter();
  hal_wdt_feed();                              /* and immediately back in the window */
  g_wdt_last_delta = (a > b) ? (a - b) : 0;    /* a DOWN-counter: b must be smaller */
  return g_wdt_last_delta >= PB_WDT_PROBE_MIN_COUNTS;
}
```

**The arithmetic, and why 40 ms is safe.** The counter decrements at `PCLKB / 8192` = 24 MHz / 8192 =
**2929.7 Hz**, so 40 ms is `40 × 2929.7 / 1000 =` **117** counts. `PB_WDT_PROBE_MIN_COUNTS` is
**58** — half of that — so a slow or jittery `millis()` cannot produce a false negative, while a
frozen counter (delta 0) cannot produce a false positive. The unfed window is 40 ms against a granted
**5592 ms**: two orders of magnitude of margin, and the probe re-enters the window with a feed on the
line after the second sample.

Called in `setup()` (a failure disables the network **and** latches dry), and it is `dose_run()`'s
**first** guard in place of the design's `getTimeout() == 0` test. The last measured delta rides out
as **`ch209`** (§4.1) and prints in `status`, so the 48-hour run carries evidence rather than a
compile-time constant.

The host test that pins the polarity is
`test_wdt_alive_is_false_only_when_the_counter_is_frozen`, driven through `hal_sim`'s settable
counter, with the fake **asserting that no feed occurred inside the probe window**. Without that
second assertion the test passes against a feeding probe, which is precisely how this bug survived
review the first time.

**Naming, plainly.** This is the RA4M1 **WDT** — register-start, PCLKB-clocked. It is **not** the
IWDT that `DECISIONS.md` #10 and `cad/wiring/README.md` name. The IWDT auto-starts from OFS0 option
bytes that the Arduino core does not expose and that can lock the board out of uploads; **this work
does not write `.option_setting_ofs0`.** The WDT meets the intent (a hung loop resets the board, D6
reverts to an input, R1 opens the relay) but not the letter (a PCLKB failure stops it too). `status`
prints:

```
wdt=on granted=5592ms alive=yes  (WDT, not IWDT — DECISIONS #10 says IWDT; see docs/…/2026-09-03-bench-sketch-design.md §2.5)
```

The true IWDT becomes its own pitch, whose **first deliverable is a documented DFU recovery path**.
A dated `DECISIONS.md` amendment is owed either way (§16.4): the word "IWDT" in #10 is false today
and should not be allowed to go quietly false.

### 2.6 The five guards, plus three new ones

| # | guard | in `dose_run()` | source |
| --- | --- | --- | --- |
| 1 | **Watchdog alive** | `!hal_wdt_alive()` refuses | the counter **decreased** across a 40 ms **unfed** window (§2.5), not `getTimeout()`, and not a fed probe |
| 2 | **Hard cap** | `cap_ms` clamped twice (protocol ceiling, then the measured clamp), checked every pass | DECISIONS #10, wiring README THE GAP |
| 3 | **Float** | debounced at start, single-sample abort mid-dose | DECISIONS #12; bring-up 5a / 5b |
| 4 | **Position** | `need_pos` refusal + `cart_bus_check()` **live inside the dose loop** | README row "I2C hung, home hall unreadable" |
| 5 | **No flow** | prime and stall, **both armed on time, never on `got`** | pitch-28a903; bring-up 7b |
| 6 | **Meter plausibility** | idle rate non-zero refuses; in-dose rate over `PB_FLOW_MAX_HZ` aborts `err=noise` | an unplugged D2 counting garbage would otherwise "reach target" in ms |
| 7 | **Cooldown** | `PB_DOSE_MIN_GAP_MS` since the last dose ended, for **every** caller | queued console impatience, backend adjacency |
| 8 | **Contradiction latch** | refuses everything until explicitly cleared | §2.7 |

### 2.7 The float/flow contradiction latch — new, and the strongest thing in this file

**The situation.** The float said OK — permission was granted by the one input whose whole design
(DECISIONS #12) is that failure reads as refusal — and the dose that permission authorised produced
**no flow at all**. Two independent sensors contradict each other. The safe reading is *the tank is
empty and the float is stuck*, and the safe response is not "end this dose and let the next one
start" (which is all the design's no-flow abort does) but **refuse everything until a human looks**.

**Name.** The *contradiction latch*. `g_nv.contra_latched` in the `.noinit` block; `err=contra` on
the wire; `ch207=1`; `status` prints a banner.

**Exactly where it is set.** One place: inside `dose_end_ml_()`, the single function every
`dose_run()` exit passes through. Nothing else in the tree assigns it.

```c
/* inside dose_end_ml_(r, got_pulses, elapsed_ms, outlet, prime_ms, long_prime) */
if (g_float_granted                                   /* the debounce GRANTED at the top of this dose */
    && !long_prime                                    /* and this was not a console prime of a dry line */
    && hal_pin_read(PIN_HALL_FLOAT) == PB_LOW         /* and the float STILL says OK right now */
    && got_pulses == 0                                /* and NOTHING came out, at all, ever */
    && elapsed_ms >= prime_ms) {                      /* THIS dose's effective prime window, not cfg_ */
  g_nv.contra_latched = true;
  g_last_err = "contra";
}
```

Five conditions, each doing one job:

- `g_float_granted` — this was a *permitted* dose. A dose refused for any reason never reaches here.
- **`!long_prime`** — a console `pump <ms> prime` is the one case where "float OK, no flow" is the
  *expected* result rather than a diagnostic one: priming a dry line **is** running the pump into air
  on purpose. Bring-up 7a's own command, on a line that has never held water, satisfies every other
  condition on its **first** attempt; without this exemption 7a would latch, and §13's instruction to
  "run it again" would be wrong because the second run would return `DOSE_REFUSED_CONTRA`. The
  exemption costs nothing, because `long_prime` is compiled out of the `bench` binary entirely (§6),
  so the unattended board has no way to reach it.
- **the float still reads OK at the abort** — this is what separates a contradiction from an ordinary
  `DOSE_ABORT_FLOAT`. If the float dropped mid-dose, the two sensors *agree* (tank ran out); that is
  an ordinary abort and it does not latch.
- `got_pulses == 0` — **nothing at all**, for the whole dose. A dose that flowed and then stalled
  (hose off a pot, tank sucked dry mid-dose) has `got > 0`: the meter and the float agree that water
  was moving and then stopped. That is `DOSE_ABORT_NOFLOW`, ordinary, and it does not latch.
- `elapsed_ms >= prime_ms` — **the effective prime window of the dose that just ran**, passed into
  `dose_end_ml_()`, not the configured default. A `stop` typed 200 ms in is not evidence of anything;
  neither is a dose that never got as far as its own prime window.

**A kinked hose latches too, and that is intended.** With a full tank and a blocked line the float
says OK and nothing comes out, so the latch fires. The latch does not diagnose; it refuses. A kinked
hose and a stuck float both stop the rig until a human looks, and stopping the rig is the right
answer to both. What the latch will *not* do is fire on the two benign shapes above.

**Exactly where it is cleared.** One place: the console command `clear contra` — two literal tokens,
no abbreviation, present in **both** the `bench` and `bringup` builds because it is the only way back
and the unattended binary can latch. It prints a banner naming the dose that set it. It clears on
nothing else: no timer, no successful anything, no backend command, no `dry off`.

**Where it is checked.** `dose_run()`'s refusal ladder, *above* the dry latch so the more specific
reason is the one reported:

```c
if (g_nv.contra_latched) return dose_end_(DOSE_REFUSED_CONTRA);
```

It refuses every dose. It does **not** refuse `cart_home()` or the boot self-home — homing parks the
cart off every gate, which after a latch is exactly what you want (§2.9, §2.11).

**Where it is surfaced.**

- `err=contra` in every report until cleared.
- `ch207=1` — a stored, graphable reading, no backend change needed.
- **`float=0`.** Under the latch, the board's considered belief — from two sensors, not one raw
  sample — is that there is no usable water. §2.10's rule is that `float=` is *the debounced tank
  verdict ANDed with `!contra_latched`*; under the latch that is 0, and 0 is the honest value. It
  also, with **zero backend change**, makes `water_rules` return at `butler.py:680-681` and stop
  queuing immediately, and drives the existing `float:` alert.
- `status`: `CONTRADICTION LATCHED — float said OK, meter saw nothing. `clear contra` to release.`
- LCD row 0: `CONTRA LATCH`, row 1: `float ok, no flow`.

**What the phone actually says, and why that is not good enough yet.** The `float:` alert's text is
fixed at `butler.py:1246-1249`: *"the reservoir on X is empty or at the waterline: watering is on
hold"*. A kinked hose with a full tank latches too, deliberately — so **until §16.5.4 (store `err=`)
or a `ch207` alert rule lands, the phone says "reservoir empty" whatever the cause**, and the
operator must read `status` or `ch207` before topping up. Neither field reaches an alert rule today:
`err=contra` is an unknown key that `parse_report` discards, and `ch207` maps to no pot so no rule
reads it. §16.5.4 is therefore a **stated precondition of this latch's notification story**, not a
loose owed item. And because the raise needs `flapped` — two bad sightings inside `FLAP_WINDOW_S`
(`:1267-1275`) — the alert arrives on the **second** report after the latch, not the first.

**Across a board reset.** The latch lives in `.noinit`, so it survives a **warm** reset (watchdog,
RESET button) — which matters, because the watchdog reset is precisely the event that would otherwise
erase it. It does **not** survive a power cycle, a brown-out or a reflash, and it is not written to
flash. **The firmware latch is therefore not durable, and the durable half of this feature lives in
the backend**: on seeing `err=contra` (or `ch207=1`, or `float=0` persisting from a controller that
was reporting `float=1`), the backend must stop queuing water commands for that controller and keep
refusing until a human confirms in the app — not until the next report says something nicer. That is
owed backend and app work (§16.5), and until it lands the firmware half is what stands. A power cycle
after a latch silently rearms the rig; the bench notes must say so in as many words.

### 2.8 `dose_run()` — the whole of THE GAP, in one function

```c
typedef enum {
  DOSE_OK = 0,
  DOSE_REFUSED_WDT, DOSE_REFUSED_DRY, DOSE_REFUSED_CONTRA, DOSE_REFUSED_BOOT,
  DOSE_REFUSED_RANGE, DOSE_REFUSED_CAL, DOSE_REFUSED_FLOAT, DOSE_REFUSED_POS,
  DOSE_REFUSED_I2C, DOSE_REFUSED_BUSY, DOSE_REFUSED_COOLDOWN, DOSE_REFUSED_NOISE,
  DOSE_ABORT_CAP, DOSE_ABORT_NOFLOW, DOSE_ABORT_NOISE, DOSE_ABORT_FLOAT,
  DOSE_ABORT_POS, DOSE_ABORT_STOP
} dose_result_t;

typedef struct {
  uint8_t  outlet;      /* 1..PB_OUTLETS. NOT a sentinel: `need_pos` is what says whether it means
                           anything. `water=0` IS a legal backend command (_int_in(v,"water",0,256),
                           butler.py:291, and the `outlet is None` guard at :311 does not catch 0),
                           and pots.outlet is bounded (0,256) too — so outlet 0 arrives from the
                           wire, not only from the console. exec_pending() range-checks it before it
                           touches the cart (§4.3) and dose_run() refuses it again below. */
  uint16_t ml;          /* the millilitre target; meaningless when by_time */
  bool     by_time;     /* EXPLICIT. `ml` is never overloaded as a sentinel. */
  uint32_t cap_ms;      /* already clamped by the caller; clamped AGAIN below, twice */
  bool     need_pos;    /* backend water command: true. console `pump`: FALSE (bring-up 4a/5a/5b) */
  bool     long_prime;  /* console `pump <ms> prime`: EXTEND the prime window, never remove it */
} dose_req_t;

dose_result_t dose_run(const dose_req_t *q) {
  /* ---- every refusal, before the assert, in this order ---- */
  if (g_dosing)                                return dose_end_(DOSE_REFUSED_BUSY);
  if (!hal_wdt_alive())                        return dose_end_(DOSE_REFUSED_WDT);
  if (g_nv.contra_latched)                     return dose_end_(DOSE_REFUSED_CONTRA);
  if (g_nv.dry_latched)                        return dose_end_(DOSE_REFUSED_DRY);
  if (hal_millis() < PB_BOOT_GAP_MS)           return dose_end_(DOSE_REFUSED_BOOT);   /* DECISIONS #5 */
  if (hal_millis() - g_last_dose_end_ms < PB_DOSE_MIN_GAP_MS && g_last_dose_end_ms)
                                               return dose_end_(DOSE_REFUSED_COOLDOWN);
  if (cfg_pulses_per_l < PB_PULSES_PER_L_MIN ||
      cfg_pulses_per_l > PB_PULSES_PER_L_MAX)  return dose_end_(DOSE_REFUSED_CAL);
  if (!q->by_time && (q->ml == 0 || q->ml > PB_DOSE_RIG_MAX_ML))
                                               return dose_end_(DOSE_REFUSED_RANGE);
  if (q->cap_ms == 0)                          return dose_end_(DOSE_REFUSED_RANGE);
  if (q->need_pos && (q->outlet < 1 || q->outlet > PB_OUTLETS))
                                               return dose_end_(DOSE_REFUSED_RANGE);
  if (pulses_flow_rate() > PB_FLOW_IDLE_MAX_HZ) return dose_end_(DOSE_REFUSED_NOISE);
  if (!sensors_i2c_healthy())                  return dose_end_(DOSE_REFUSED_I2C);
  if (!safety_float_ok_debounced())            return dose_end_(DOSE_REFUSED_FLOAT);  /* 5a */
  if (q->need_pos && !cart_pos_known())        return dose_end_(DOSE_REFUSED_POS);
  if (q->need_pos && cart_pos() != q->outlet)  return dose_end_(DOSE_REFUSED_POS);
  g_float_granted = true;                      /* consumed by dose_end_ml_(), §2.7 */

  /* ---- the two caps, in the same function, five lines above the assert ---- */
  uint32_t cap_ms = q->cap_ms;
  if (cap_ms > PB_DOSE_CAP_MS_MAX) cap_ms = PB_DOSE_CAP_MS_MAX;   /* 60000 == butler MAX_CAP_S */
#if PB_ML_PER_S_MEASURED > 0
  if (!q->by_time) {   /* the cap may never authorise more than 2x the requested water */
    uint32_t bound = (uint32_t)q->ml * 1000u / PB_ML_PER_S_MEASURED
                     * PB_CAP_SLACK_NUM / PB_CAP_SLACK_DEN;
    if (bound && cap_ms > bound) cap_ms = bound;
  }
#endif
  uint32_t target = 0;
  if (!q->by_time) {
    /* MULTIPLY FIRST, DIVIDE SECOND. `ml * (cfg/1000)` truncates the calibration to whole
       pulses per millilitre: at the nominal cfg = 5880 it is 5 instead of 5.88, i.e. every
       metered dose stops 15% short, forever, and at the legal cfg = 1999 it is a 2x error.
       Neither is visible to butler's `2*flow_ml < ml` alert (2 x 212 > 250), so the pot is
       quietly short-watered while every field reports healthy.
       Overflow is impossible and the range check above is the proof, not the ordering:
       PB_DOSE_RIG_MAX_ML (250) x PB_PULSES_PER_L_MAX (20000) = 5e6, and even the protocol
       ceiling PB_DOSE_MAX_ML (1000) x 20000 = 2e7 — three orders below UINT32_MAX. */
    target = (uint32_t)q->ml * cfg_pulses_per_l / 1000u;
    if (target == 0) return dose_end_(DOSE_REFUSED_RANGE);   /* a metered dose NEVER runs uncapped */
  }
  uint32_t prime_ms = q->long_prime ? PB_PRIME_LONG_MS : cfg_prime_ms;
  if (q->long_prime && cap_ms > PB_PRIME_CAP_MS) cap_ms = PB_PRIME_CAP_MS;

  uint32_t flow0 = pulses_flow(), got = 0, last_got = 0;
  uint32_t t0 = hal_millis(), last_edge = t0, last_bus = t0;
  dose_result_t r = DOSE_ABORT_CAP;

  g_nv.dose_in_flight = true;           /* .noinit: a reset from here on latches dry at the next boot */
  g_dosing = true;
  hal_pump_write(true);                 /* <-- the ONLY assertion of D6 in the program */
  for (;;) {
    safety_tick();                                        /* feeds; g_dosing keeps the ON write */
    uint32_t now = hal_millis(), el = now - t0;           /* unsigned diff: rollover-safe */
    got = pulses_flow() - flow0;
    if (got != last_got) { last_got = got; last_edge = now; }

    /* BOTH NOISE RULES COME FIRST, ABOVE THE TARGET RULE. A D2 storming at the ISR's own
       2 kHz ceiling reaches a 250 ml target (1250 pulses at cfg = 5000) in ~625 ms; testing
       `got >= target` first would ack DOSE_OK and flow_ml=250 for water that never moved. The
       rate estimator's window is PB_FLOW_RATE_WINDOW_MS = 100 ms, so a 2 kHz storm is visible
       inside ~0.1 s against a >= 0.6 s target. The pre-dose PB_FLOW_IDLE_MAX_HZ guard only
       catches a storm that was ALREADY running; this catches one that starts with the pump. */
    if (pulses_flow_rate() > PB_FLOW_MAX_HZ)           { r = DOSE_ABORT_NOISE;  break; }
#if PB_ML_PER_S_MEASURED > 0
    /* Delivered-vs-elapsed plausibility: reaching the target in far less time than the rig can
       physically deliver it is noise, not a fast pump. Armed only once 7b commits the rate. */
    if (target && got >= target &&
        el * PB_ML_PER_S_MEASURED * PB_PLAUS_NUM < (uint32_t)q->ml * 1000u * PB_PLAUS_DEN)
                                                       { r = DOSE_ABORT_NOISE;  break; }
#endif
    if (target && got >= target)                       { r = DOSE_OK;           break; }
    if (el >= cap_ms)                                  { r = DOSE_ABORT_CAP;    break; }
    /* PRIME rule: nothing at all came out in the prime window. `prime` EXTENDS it, never removes it. */
    if (el >= prime_ms && got < PB_PRIME_MIN_PULSES)   { r = DOSE_ABORT_NOFLOW; break; }
    /* STALL rule: armed on TIME, not on `got`, so zero flow can never disarm it. */
    if (el >= prime_ms && (now - last_edge) >= cfg_stall_ms)
                                                       { r = DOSE_ABORT_NOFLOW; break; }
    if (hal_pin_read(PIN_HALL_FLOAT) != PB_LOW)        { r = DOSE_ABORT_FLOAT;  break; }  /* 5b */
    if (cli_stop_requested())                          { r = DOSE_ABORT_STOP;   break; }
    if ((now - last_bus) >= PB_POS_RECHECK_MS) {       /* the README's "I2C hung" row */
      last_bus = now;
      if (!cart_bus_check())                           { r = DOSE_ABORT_POS;    break; }
    }
  }
  hal_pump_write(false);                /* unconditional, ONE exit, before any bookkeeping */
  g_dosing = false;
  g_nv.dose_in_flight = false;
  g_last_dose_end_ms = hal_millis();
  g_leak_rearm_at = g_last_dose_end_ms + PB_COAST_MS;   /* impeller coast-down is not a leak */
  hal_serial_drain();                   /* impatience typed during the dose is DISCARDED, not executed */
  return dose_end_ml_(r, pulses_flow() - flow0, g_last_dose_end_ms - t0,
                      q->outlet, prime_ms, q->long_prime);   /* §2.7 needs both of the last two */
}
```

Three properties the reviewer should check by eye:

- **There is no `return` between the ON write and the OFF write.** The loop's only exit is a `break`.
- **`dose_end_()` and `dose_end_ml_()` always set** `g_last_result`, `g_last_flow_ml` and
  `g_last_err`, so a refusal can never ack the previous dose's millilitres.
- **The two flow rules are both armed on elapsed time.** The design armed the stall rule on
  `got >= PB_PRIME_MIN_PULSES`, so with zero flow it never armed at all — and `prime` suppressed the
  other one. `pump 60000 prime`, which is bring-up 7a's own command and `calib`'s implementation,
  was therefore an unconditional sixty-second dry run with the no-flow abort — one of DECISIONS #10's
  three mandatory measures — entirely absent. It is now a **bounded** run: `prime` extends the window
  to `PB_PRIME_LONG_MS` (15 s), still checked, and caps the whole dose at `PB_PRIME_CAP_MS` (20 s)
  regardless of the typed `ms`.

### 2.9 The cart is parked off every gate after every dose

`exec_pending()` calls `cart_home()` on **every exit at which a command has been consumed** — not
"after `dose_run()` returns". The distinction matters: a `cart_goto()` that stalls or times out never
reaches `dose_run()`, so a cart stalled part-way across the gates would never be parked. Concretely,
the park runs after: a successful dose, a refused dose, an aborted dose, a **failed or stalled
`cart_goto`**, a `CMD_STOP`, an out-of-range outlet refused before the cart moved, and the
`net_state() != NET_IDLE` early return **once a command has been consumed**. One `goto park;` target,
one call site, every terminal path routed through it — the same shape as `dose_end_()`.

The magnet cart lifts the gate it sits over (DECISIONS #9) and the reservoir sits
above the pump inlet (wiring README, *Hydraulic chain*), so a cart left over outlet N holds that gate
open under a head of water for however long it is until the next command — which may be six hours, or
never. A diaphragm pump's check valve would be the only thing in the way. Home is the threadless
start of the screw and is over no gate.

The cost is a traverse (≤ `PB_MOVE_CAP_MS` = 45 s) after each dose. Report deferral is then
goto (45) + dose (60) + park (45) ≈ **150 s**, against `butler.py`'s silence threshold of
`max(SILENT_AFTER_S = 600, 3 × interval)`. At `next=60` that never pages. The bench notes say `next`
below ~60 s will visibly stutter while doses are live.

`status` prints the parked state, and so does the report: **`ch208` = parked, 0 or 1** (§4.1's table
allocates it a real channel — "`chNNN`-adjacent" was vague, and §4.1's table is what an implementer
codes from). `pos=` still reports honestly.

**Neither latch blocks parking.** The boot self-home and `cart_home()` run under `dry on` and under a
contradiction latch alike — see §2.11, which makes the same argument once for both.

### 2.10 Float: asymmetric debounce, and what the wire says

```c
bool safety_float_ok_debounced(void) {          /* N consecutive OK to GRANT */
  for (uint8_t i = 0; i < PB_FLOAT_OK_SAMPLES; ++i) {         /* 3 */
    if (hal_pin_read(PIN_HALL_FLOAT) != PB_LOW) return false; /* one bad sample REFUSES */
    if (i + 1 < PB_FLOAT_OK_SAMPLES) safety_wait_ms(PB_FLOAT_SAMPLE_MS);  /* 20 ms, fed */
  }
  return true;
}
```

Refusing on one bad sample is safe; **granting** on one is not — D5 runs up to 1 m to the reservoir
alongside a 12 V pump lead. Mid-dose, a single HIGH aborts: that direction is dry.

**`float=` on the wire is built from this same function**, not from a raw pin read, so the wire says
what the board will actually do. The design's raw-sample version produced a specific, expensive lie:
a float bouncing at the waterline satisfies one sample and fails three, so the board reports
`float=1 pos=ok`, `water_rules` queues a dose, `dose_run()` refuses it, the refusal is acked with
`flow_ml=0`, `2*flow_ml < ml` raises a **HIGH** ntfy (`butler.py:1368`) and the acked refusal **sets
the pot's cooldown** (`:736-742`) — so the pattern repeats every `cooldown_h`, forever, while every
field the phone-notification design rests on says the rig is fine.

**`float=` has exactly one definition, and this is it.** `float=` is *the debounced tank verdict,
ANDed with `!g_nv.contra_latched`* — nothing else. It is deliberately **not** "what `dose_run()` would
decide": `dose_run()` refuses for eleven reasons that have nothing to do with the tank, and folding
them into `float=` would report an empty reservoir to the phone for a stalled cart or a busy dose.

Two consequences that have to be written down, because the loop they prevent is expensive:

- **A dry latch forces `pos=unknown`, not `float=0`.** Under `dry on`, `float=` still reports the
  tank honestly, so `water_rules` would queue, `dose_run()` would refuse `DOSE_REFUSED_DRY`, the
  refusal would be acked with `flow_ml=0`, `2*flow_ml < ml` would page HIGH and the acked refusal
  would set the pot's cooldown — repeating every `cooldown_h`, forever, for an operator who left
  `dry on` set (which now survives a warm reset, §2.11). So while `g_nv.dry_latched` is set,
  `report_build()` emits `pos=unknown`, the same zero-backend-change trick `PB_REPORT_POS_UNKNOWN`
  uses. The rules ladder goes dark at `butler.py:680-681` and nothing is ever queued.
- **Repeated float refusals force `float=0` on the wire.** The report's debounce and the dose's
  debounce are separate samples taken minutes apart, so a float flapping at the waterline can grant
  in the report and refuse in the dose — reproducing the same acked-refusal loop above once
  `PB_REPORT_POS_UNKNOWN` is cleared. The contradiction latch does **not** cover this: a refused dose
  never reaches `dose_end_ml_()`. So `safety.cpp` counts consecutive `DOSE_REFUSED_FLOAT` results;
  above `PB_FLOAT_FLAP_LIMIT` (3) the report emits `float=0` and `err=float` regardless of the
  report-time debounce, and the counter is cleared by any granted dose.

**Staleness is reported, never enforced.** `ch204 = <seconds since D5 last changed state>`, a **bare
non-negative integer, always** — `0` before D5 has ever moved, never a sentinel. `_int_in` rejects a
leading `-` and any non-digit, so `-1`, `unknown` or `never` in a `chN` would 400 the whole report.
The
firmware **never** refuses on staleness. `cad/wiring/README.md`'s rule — "a float input that never
changes state across a refill is presumed dead" — is not implementable on the board, because a refill
is a human act the board cannot observe. It becomes implementable once the app has a *refilled*
button and the backend has a refill log; then the backend, which knows both the refill time and
`ch204`, raises the alert. That is owed backend and app work (§16.5); the firmware's whole
contribution is `ch204`.

> **Naming conflict, recorded rather than silently resolved.** pitch-d155fe ("Trust the tank") states
> as its no-go that "the board's whole contribution is **`fstale=`** and the contradiction report".
> This spec emits **`ch204`** instead, and the difference is not cosmetic: `fstale` is neither a known
> key nor `ch`-prefixed, so `parse_report` ignores it (`butler.py:204-257`), stores it nowhere, and
> no alert rule can ever read it — a firmware that emitted `fstale=` would satisfy the pitch's letter
> and deliver nothing. `ch204` reaches `readings` and is graphable with **no backend change at all**.
> The pitch must be amended before it is bet on; §16.5.1 carries the item.

### 2.11 The dry latch

`dry on|off` sets `g_nv.dry_latched`; `dose_run()` then refuses everything. Used by bring-up 6 and
while plumbing. It is **strictly more refusing**, so unlike a sim flag it cannot fail dangerously. It
now survives a warm reset (§2.3), which is the case that mattered: a brown-out at pump start — the
wiring README warns about 3–5× inrush on a sagging brick — used to silently clear the latch while the
operator's hands were in the plumbing. It does not survive a power cycle; `PB_BOOT_GAP_MS` (10 s)
refuses right after one, and the boot banner and `status` both print `dry=`.

While it is set, `report_build()` emits `pos=unknown` (§2.10), so the backend queues nothing rather
than queuing doses the board will refuse and ack.

**Neither latch blocks the servo.** The boot self-home of §3 runs under `dry_latched` **and** under
`contra_latched`, and so does an explicit `home`. The argument is §2.7's, made once for both: homing
drives the servo, not D6; `safety_tick()` re-asserts pump-OFF on every pass of the move loop; and
parking the cart off every gate is *more* wanted after a latch, not less — a cart left over outlet N
holds that gate open under the reservoir head for as long as the latch stands, which may be days.
Gating the boot self-home on `!g_nv.dry_latched` would do exactly that after every watchdog reset
taken mid-dose, because §2.3 latches dry on precisely that case. `test_latch_does_not_refuse_homing`
covers both latches.

### 2.12 What a backend `stop=1` can and cannot do

`dose_run()` blocks and `net_poll()` cannot run while it does, so **a backend `stop=1` can never
interrupt a running dose** — and `enqueue()` returns 409 while the water command it would abort is
still `sent`, so it cannot even be queued. This is written into `status`'s help text so nobody
reaches for it in an emergency.

The live aborts are the console `stop`, the console `dry on`, the float, the two flow rules, the
plausibility ceiling, the cap and the watchdog.

**`cli_stop_requested()` is specified, not assumed.** The design left it undefined at the seam while
advertising it as the last-resort abort. It is a byte-at-a-time matcher over its own four-byte state,
with its own `hal_serial_read`, that consumes **only** the bytes of an exact `stop\n` or `dry on\n`
and pushes every other byte into `cli.cpp`'s line buffer unread. `dry on` typed mid-dose sets
`g_nv.dry_latched` **and** raises the stop request, so the word means the same thing during a dose as
before one. The host test drives it through the real `hal_serial_read` fake, byte by byte, split
across two reads — not by poking a flag.

### 2.13 I2C recovery may not run while D6 is asserted

`hal_i2c_recover()` bit-bangs A4/A5 — the mux select lines and the home hall, the input that gates
the pump. It therefore begins `if (g_dosing) return false;`, and a back-off that expires mid-dose
simply stays expired until the dose ends. It is **exactly nine clocks**, a fixed loop count with
`hal_delay_us(5)` per half-cycle — never an "until SDA releases" condition — with `safety_tick()`
before and after. It runs at boot and on each back-off expiry outside a dose.

### 2.14 D2 and D3 — configured once, and never touched again

```c
static void hal_arm_pulse_pins_(void) {          /* the ONLY place D2 and D3 are configured */
  pinMode(PIN_FLOW,       INPUT_PULLUP);         /* attachInterrupt PRESERVES this — Interrupts.cpp */
  pinMode(PIN_HALL_SCREW, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_FLOW),       isr_flow_,  FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_HALL_SCREW), isr_screw_, FALLING);
  hal_icu_enable_filter_(PIN_FLOW);              /* see below */
  hal_icu_enable_filter_(PIN_HALL_SCREW);
}
```

Three things this fixes:

1. **A floating D2.** `cad/wiring/README.md`'s pin table gives D2 a 1 k series resistor and **no
   pull** — it is the only one of D2/D3/D5 without one. An unplugged, cut or dead meter leaves it
   floating beside a servo lead and a 12 V pump leg. `INPUT_PULLUP` here is the software half; a
   physical 10 k on D2 is an owed `nets.py` change (§16.1) and is the half that actually holds the
   line.
2. **The ICU filter — set in the register, not through `IRQManager`.** `attachInterrupt` hardcodes
   `filter_enable = false` (`Interrupts.cpp:151`) but **already** sets
   `pclk_div = EXTERNAL_IRQ_PCLK_DIV_BY_64` (`:150`), so the only thing missing is `FLTEN`.

   > The design's route — re-opening the channel through
   > `IRQManager::getInstance().addPeripheral(IRQ_EXTERNAL_PIN, &cfg)` — **is not reachable and must
   > not be used.** The `external_irq_cfg_t` and `icu_instance_ctrl_t` the core actually opened live
   > in `CIrq` objects owned by a **file-static** `IrqPool` inside `Interrupts.cpp` (`:41-67`) with
   > no external accessor, so nothing outside that translation unit can re-open them. Handing
   > `addPeripheral` a *fresh* cfg takes the `if (p_cfg->irq == FSP_INVALID_VECTOR)` branch
   > (`IRQManager.cpp:747-756`) and allocates a **second NVIC vector** mapped to the same ICU
   > channel — and, unlike the SPI branch at `:768-771`, that branch has **no
   > `last_interrupt_index > PROG_IRQ_NUM` bounds check**.

   Set the bit directly, after `attachInterrupt` has opened the channel:

   ```c
   static void hal_icu_enable_filter_(uint8_t pin) {
     auto cfg = getPinCfgs(pin, PIN_CFG_REQ_INTERRUPT);   /* variant.h:32 — public */
     if (cfg[0] == 0) return;                            /* not an IRQ-capable pin */
     uint8_t ch = GET_CHANNEL(cfg[0]);                   /* variant.h:120 */
     R_ICU->IRQCR[ch] |= (uint8_t)(R_ICU_IRQCR_FLTEN_Msk                    /* bit 7 */
                        | (EXTERNAL_IRQ_PCLK_DIV_BY_64 << R_ICU_IRQCR_FCLKSEL_Pos)); /* bits 5:4 */
   }
   ```

   `PCLKB/64` = 375 kHz, three sample clocks, ≈ 8 µs minimum pulse width. Re-writing `FCLKSEL` to the
   value the core already programmed is a no-op and is kept for legibility. `hal_irq_filtered(pin)`
   reads the same bit back, and `status` prints `icufilter=yes|no` so which route shipped is never a
   guess. **If even this will not build cleanly**, the accepted fallback is the pull-up plus the
   plausibility ceiling alone, and `icufilter=no` says so.
3. **A later `pinMode` silently detaching the interrupt.** `Interrupts.cpp:183` configures the pin
   with `IOPORT_CFG_IRQ_ENABLE | ... | pullup`; `digital.cpp`'s `pinMode` writes the same register
   **without** `IOPORT_CFG_IRQ_ENABLE`. Any subsequent `pinMode` on D2 or D3 — from the `hall`
   command re-arming a pull-up, from a re-init after I2C recovery, from `hal_begin()` running twice —
   detaches it with no error and no symptom. `tools/check.sh` gains **zero hits for
   `pinMode(PIN_FLOW` and `pinMode(PIN_HALL_SCREW` outside `hal_arm_pulse_pins_`**, and
   `hal_irq_armed(pin)` answers the question in hardware. There is no per-channel "enable bit"
   reachable from a pin number (the pool owns the allocated vector, and `R_ICU_ExternalIrqEnable`
   goes to `R_BSP_IrqEnable` on it), so `hal_irq_armed` finds the vector instead: scan
   `R_ICU->IELSR[i]` for `i` in `[0, 32)` — `PROG_IRQ_NUM` is `BSP_ICU_VECTOR_MAX_ENTRIES`,
   `IRQManager.cpp:7` — for `(IELSR[i] & 0xFF) == ELC_EVENT_ICU_IRQ0 + ch` (the ICU IRQ events are
   contiguous, `bsp_elc.h:51-66`), then return `NVIC_GetEnableIRQ((IRQn_Type)i)`. `status` and the
   pre-dose guard can then say "the meter's interrupt is not armed" instead of running dry.

**The plausibility ceiling.** A YF-S401-3507 tops out at 6 L/min = 588 pulses/s at ~5880 pulses/L. So:

- `PB_FLOW_MAX_HZ = 1200` (2× the meter's rating): exceeded mid-dose → `DOSE_ABORT_NOISE`,
  `err=noise`.
- `PB_FLOW_IDLE_MAX_HZ = 2`: a non-zero idle rate refuses the dose outright, `err=noise`.
- `PB_FLOW_MIN_GAP_US = 500` stays as the ISR-level reject; it is honest about only rejecting above
  2 kHz, which is why the two rate rules exist above it.
- **`PB_FLOW_RATE_WINDOW_MS = 100`.** `pulses_flow_rate()` is a rate over a *bounded* window, and the
  bound has to be stated or the whole argument is unfalsifiable: at the ISR's own 2 kHz ceiling a
  250 ml target (1250 pulses at cfg = 5000) is reached in **~625 ms**, so any estimator slower than
  that loses the race. 100 ms wins it by a factor of six. Implementation: a 100 ms tumbling window,
  `(pulses_this_window * 1000) / elapsed_ms`, with the previous window's value returned while the
  current one fills — no floats, no ring buffer.
- **And the rate rules run ABOVE the target rule in the dose loop** (§2.8). Ordering is the whole
  fix: `got >= target` tested first would break out with `DOSE_OK` before either rate rule was
  reached.

Without these, an edge-storming D2 reaches a 500 ml `target` in milliseconds: `DOSE_OK`,
`flow_ml=500` acked, `2*flow_ml < ml` never fires, the daily cap is charged, and the pot silently gets
nothing while every guard reports healthy. Note that `PB_FLOW_IDLE_MAX_HZ` alone does **not** close
this: it catches a storm that is already running when the dose starts, and the scenario that matters
— a floating D2 running beside the 12 V pump leg — is a storm that begins **with** the pump. §9's
storm test therefore starts its storm at pump-on, not before it.

### 2.15 `PB_PULSES_PER_GATE = 0` refuses in code, not in prose

The design set it to 0 with the note "0 == goto always refuses" — a comment, not a mechanism. The
natural implementation computes `PB_PULSES_HOME_TO_1 + (outlet-1) * 0`, finds the target already
satisfied at home, returns true and sets `g_pos = outlet`; both position guards then pass and the
pump dead-heads against a closed manifold with `pos=ok` on the wire, from the first flash until
bring-up 6.

Structural instead:

```c
/* lib/Manifold/src/cart.cpp */
#if PB_PULSES_PER_GATE == 0
bool cart_goto(uint8_t outlet) { (void)outlet; g_err = "uncal"; return false; }
bool cart_pos_known(void)      { return false; }
#else
...
#endif
```

`status` prints `cart=UNCALIBRATED (PB_PULSES_PER_GATE=0)`. Once drop 2 lands, the `bench` env
`#error`s on a zero gate constant, so the number cannot be forgotten.

`pos=ok` therefore means "I can deliver to the requested outlet", not "I once saw the home hall".
Combined with `PB_REPORT_POS_UNKNOWN` (§4) and the dry latch's own `pos=unknown` (§2.10) it means
`water_rules` stays dark by construction through every drop, so the "one HIGH push per hour,
indefinitely, for doses the firmware always refuses" failure has no window in which to happen. (It
buys that at the cost of one standing HIGH `pos:` page for the whole bench programme — §4.6 says so
plainly, because a cost paid to the operator's phone is not a cost the spec may leave unstated.)

---

## 3. The loop

```c
void loop(void) {
  safety_tick();       /* pump idle re-asserted (and D6's direction repaired), then the dog fed */
  cli_poll();          /* one whole line; may block, but only through safety_wait */
  net_poll();          /* ONE bounded link/socket step; no-ops while g_dosing */
  exec_pending();      /* at most one command; runs only when the socket is closed */
  ui_poll();           /* no-ops while g_dosing, cart_busy(), or a modem command ran this pass */
}
```

Five lines. No `delay()` anywhere except inside `hal_uno.cpp`'s power-on settles — `make check` greps
for it.

### What blocks, and for how long

| phase | worst case | fed? |
| --- | --- | --- |
| `cli_poll()` line read | ~1 ms | n/a |
| `mux all` | ~40 ms (16 × [select + 1 ms + 2 ADC]) | yes, per channel |
| `sensors_sweep()` (+ canary) | ~18 ms healthy; **7 s** on a wedged bus (1 s/transfer) | yes, per channel |
| one net step | **2.4 s** (at most 2 AT commands × `PB_NET_STEP_MS`) | before and after each command |
| `cart_home` / `cart_goto` | `PB_MOVE_CAP_MS` = 45 s, stall-aborted at 2.5 s | yes, per pass |
| `dose_run` | `PB_DOSE_CAP_MS_MAX` = 60 s | yes, per pass |
| `hal_wdt_alive()` probe | 40 ms, **deliberately unfed** (§2.5) — the one exception | **no**, by design |
| one LCD row | ~25 ms healthy; **102 s** wedged (102 transactions × 1 s) | yes, per **character** — `lcd.write(c)` in a loop, never `lcd.print()` |

The rule is not "short functions" — it is **every loop that can iterate over an I2C transfer, a modem
call or a millisecond of wall clock calls `safety_tick()` on each iteration**. That is what makes a
60 s dose legal under a 5.6 s window.

### The net-step budget, re-derived (the design's was wrong)

The design budgeted "3 × 1200 = 3.6 s" from `CWifi::begin`'s three writes and never counted
`WiFiClient`'s round trips at all. Trace `WiFiClient.cpp` instead. `connect()` calls `getSocket()`,
which — **when `_sock >= 0`** — calls `connected()` (which itself first calls `available()`, one
`_AVAILABLE` write, then one `_CLIENTCONNECTED` write), then `stop()` (one `_CLIENTCLOSE`), then
`_BEGINCLIENT`, then `connect()`'s own `_CLIENTCONNECTNAME`. **Five** `modem.write` calls in one
un-fed span: 6000 ms at 1200 ms each, against a granted 5592 ms. That is a watchdog reset loop
triggered by exactly the stale socket a router reboot produces — the event the 48-hour run exists to
survive — and the design's own tripwire (`granted >= 4 * PB_NET_STEP_MS + 500` = 5300) passed while
the real worst step was 6000.

**Every AT command in a pass, counted.** The budget is a computed number or it is nothing, so here is
the whole cost model, with the line that proves each row:

| pass | AT commands issued | cite |
| --- | --- | --- |
| `JOIN_ISSUE` | `link_join()` = **2** (`modem.begin()` already paid in `setup()`) | `WiFi.cpp:43-67` |
| `JOIN_WAIT` | `link_state()` = **1** | `WiFi.cpp` `status()` |
| `SOCK_CLOSE` | `_CLIENTCLOSE`, or **0** when `_sock == -1` | `WiFiClient.cpp:211-220` |
| `CONNECT` | `_BEGINCLIENT` + `_CLIENTCONNECT` = **2** — and **only** 2 because `SOCK_CLOSE` ran first and left `_sock == -1`, so `getSocket()`'s `connected()`/`stop()` branch is skipped | `WiFiClient.cpp:29-44, 53-70` |
| `SEND` | `_CLIENTSEND` + `passthrough` = **1** bounded round trip | `WiFiClient.cpp:79-91` |
| `RECV` | `client.read(buf, cap)` = `_CLIENTRECEIVE`, **1** | `WiFiClient.cpp:145-182, 117-142` |

Five changes, and the driver is left alone (§15.4 says why):

1. **`sock_close()` runs on EVERY exit, a failed open included — but never in the same pass as
   another AT command.** `stop()` sets `_sock = -1` (`WiFiClient.cpp:217`), and `getSocket()`
   short-circuits its `connected()`/`stop()` branch when `_sock == -1` (`:31`). So the connect path
   is **2** round trips, always. The design listed the exits calling `sock_close()` as "success, 400,
   401, timeout, parse failure, `abort()`"; a failed open was not among them, and a failed
   `connect()` leaves `_sock >= 0` because `getSocket()` allocates before the connect runs.
2. **A dedicated `SOCK_CLOSE` state, and every error exit routes THROUGH it rather than closing
   inline.** `finish(err)` records the error and transitions to `SOCK_CLOSE`; it does not call
   `sock_close()` itself. This is load-bearing arithmetic, not tidiness: a failed CONNECT that closed
   inline would issue `_BEGINCLIENT` + `_CLIENTCONNECT` + `_CLIENTCLOSE` = **3** ATs = 3600 ms in one
   pass, and 3600 + `PB_NET_SLACK_MS` (2000) = 5600 > 5592. Routing the close into its own pass keeps
   the bound at 2 and the assertion honest.
3. **`sock_read()` is `client.read(buf, cap)` and nothing else — no `available()`, no `connected()`.**
   `read()` calls `read_if_needed(cap)` → `_read()` → one `_CLIENTRECEIVE`, then drains the local
   FIFO (`WiFiClient.cpp:145-182`). Calling `available()` first would add an `_AVAILABLE`; calling
   `connected()` to detect a closed socket costs **two** more (it calls `available()` itself,
   `:224-238`) and would make a RECV timeout a 3-to-4-AT pass. **The RECV deadline is the
   closed-socket detector instead**: `PB_NET_DEADLINE_MS` spread across passes, after which the
   exchange is a lost one. That is the same verdict `connected()` would have produced, for zero AT
   commands.
4. **`client.setConnectionTimeout(PB_NET_STEP_MS)`** in `link_begin()` (`WiFiClient.h:63`), which
   selects `_CLIENTCONNECT` and appends the value (`WiFiClient.cpp:57-61`). `_connectionTimeout`
   defaults to 0 (`WiFiClient.h:74`), so without this nothing bounds the ESP32's side inside our
   1200 ms window — which is what makes a modem timeout likely rather than rare. **But the driver
   proves only the command selection.** `_CLIENTCONNECT` and `_CLIENTCONNECTNAME` are distinct
   ESP32-side entry points (`WiFiCommands.h:52-53`) and that firmware is not in this package, so
   *neither the unit of the timeout nor whether `_CLIENTCONNECT` still resolves a hostname* is
   checkable here. Bring-up therefore asks **two** questions, in this order: (a) connect to
   `HOST_NAME` — a name, not an IP — with the timeout set, and confirm it still succeeds; (b) then
   time a connect to a blackholed address and check it returns near `PB_NET_STEP_MS`. If (a) fails,
   the fallback is `hostByName()` once at join time with the address cached, and `connect(IPAddress)`
   thereafter — an owed change (§16.1 is wiring; this one is firmware, and it is written down here so
   it is not rediscovered on the bench). `status` prints which form and which value were sent.
5. **`link_join()` is 2 ATs, not 3**, because `link_begin()` already paid `modem.begin()`'s
   `_SOFTRESETWIFI` in `setup()`.

Worst single step is therefore **2 × `PB_NET_STEP_MS` = 2400 ms** against 5592 ms. `setup()` asserts
`hal_wdt_granted() >= 2 * PB_NET_STEP_MS + PB_NET_SLACK_MS` (= 4400) and, if it fails, **disables the
network and says why in `status`**. `ui_poll()` and `sensors_sweep()` are both skipped in any pass
where a modem command ran, so the loop pass cannot stack a 7 s wedged sweep on top of 2.4 s of modem.

Two tests hold the bound. `test_no_pass_issues_more_than_two_at_commands` runs against
`link_fake.cpp` and can only see calls that cross seam 2 — by construction it cannot see
`WiFiClient`'s internal `modem.write` calls — so it proves the **FSM's** shape and nothing about the
driver. The driver half is on-device: `test_sock_open_from_a_stale_socket_completes_within_the_wdt_window`
and `test_a_recv_pass_against_a_slow_responder_completes_within_the_wdt_window`, both asserting wall
clock (§9).

**Never call `WiFi.ping()`**: `WiFi.cpp:585-593` resets `modem.timeout()` to 10000 ms and would
silently undo the entire margin. `make check` greps for it.

**A modem timeout poisons the link.** `buf_read` breaks out on `Timeout` (`Modem.cpp:185-187`) and
leaves the late answer sitting in Serial2's RX FIFO; `write()` clears its result string (`:100`) but
does not drain the UART. The FSM's `restart` logic (`:241,:267`) resyncs some shapes and not others,
and the strict `+`-branch check is commented out at `:235`. So: **any modem timeout is treated as
link poisoned.** Do not issue the next command. `link_reset()` — `end()`, **`beginned = false`**,
`begin()`, per §1 — back to `DOWN`, re-join on the backoff, and count it — `ch206 = desyncs`. The
`beginned` line is not optional: without it `link_reset()` is a function that closes the UART and
never reopens it, which converts a recoverable poisoning into a permanent one. Without the reset at
all, the session
goes permanently one answer out of phase, nothing hangs, the watchdog never bites, and `status` keeps
printing `wifi UP` while the board silently stops reporting for the rest of the 48 hours.

### How a WiFi call is prevented from blocking while D6 is asserted

Three mechanisms, the first of which is the compiler:

1. **Include hygiene, in both directions.** `src/safety.cpp` and `lib/Manifold` include neither
   `link.h`, `Network.h` nor `WiFiS3.h`; they *cannot* make a network call. **And the mirror is now a
   grep, not a sentence**: zero hits for `safety\.h|dose_run|hal_pump_write` in `src/netfsm.cpp`,
   `lib/Network/`, `src/ui.cpp`, and in `src/cli.cpp` outside its one `dose_run` call site. The
   design asserted the mirror in prose; that is the direction that matters, because one
   `#include "safety.h"` added to `netfsm.cpp` during a later change puts a `dose_run()` call one
   edit away from a state whose socket is open, with the build and `make check` both green.
2. **Call-site ordering.** `dose_run()` is reached only from `exec_pending()` and `cli_poll()`, both
   at the top level of `loop()`. `exec_pending()` runs only when `net_state() == NET_IDLE`, i.e.
   after `sock_close()` has already run.
3. **A runtime guard.** `net_poll()` and `ui_poll()` both begin
   `if (g_dosing || cart_busy()) return;`.

### `exec_pending()` — who homes the cart, and who acks

```c
static void exec_pending(void) {
  /* The boot self-home runs under BOTH latches — see §2.11. Gating it on !dry_latched would
     leave the cart wherever a mid-dose watchdog reset stopped it (§2.3 latches dry on exactly
     that case), holding gate N open under the reservoir head until a human types `dry off`. */
  if (g_boot_home_due && hal_millis() >= PB_BOOT_HOME_MS) {
    g_boot_home_due = false;          /* servo only; D6 is idle-OFF every safety_tick */
    cart_home();                      /* bounded, stall-aborted, watchdog-fed */
  }
  if (!g_cmd.pending || net_state() != NET_IDLE) return;   /* nothing consumed yet: no park owed */

  /* The ack already exists: response_parse() set (id, flow_ml = 0, err = "recv") on receipt.
     Every path below OVERWRITES it, and no report may be built while it still reads "recv"
     (§4.3). From here on the command is CONSUMED, so every exit goes through `park`. */
  if (g_cmd.kind == CMD_STOP)             { ack(g_cmd.id, 0, "stop");  goto park; }
  if (g_cmd.outlet < 1 ||
      g_cmd.outlet > PB_OUTLETS)          { ack(g_cmd.id, 0, "range"); goto park; }  /* water=0 too */
  if (!cart_goto(g_cmd.outlet))           { ack(g_cmd.id, 0, "goto");  goto park; }
  { dose_result_t r = dose_run(&req); ack(g_cmd.id, dose_flow_ml(), err_of(r)); }
park:
  g_cmd.pending = false;
  cart_home();                        /* §2.9: EVERY consumed command parks, goto failures included */
}
```

The outlet range check sits **above** `cart_goto()` on purpose: it is what makes §4.5's promise —
"an out-of-range outlet is refused with `err=range` and acked" — true, rather than the backend
receiving whichever cart error happened first. It is also what handles `water=0`, which butler
accepts (`:291,:311-312`).

**The cart homes itself once at boot**, `PB_BOOT_HOME_MS` (15 s) after reset. This is not optional:
`dose_run()` refuses when position is unknown, and `water_rules` returns early on `r.pos != 'ok'`, so
without a self-home a single overnight reset would leave the rig permanently dry — the backend would
never queue a command, so nothing would ever trigger a home. It moves the cart unattended after every
reset; that is bounded and stall-aborted, and it must be, because a manifold that never homes never
waters.

---

## 4. The report / ack cycle, against `butler.py`

### 4.1 The body

Built **once** per cycle into `static char g_body[PB_BODY_CAP]`, with `t` stamped at build time:

```
c=bench1 t=1931044931 ch0=8123 ch1=7902 ch2=8340 ch4=7711 ch5=1422
ch200=26112 ch201=9 ch202=612 ch203=0 ch204=86400 ch205=0 ch206=0 ch207=0 ch208=1 ch209=117
float=1 pos=unknown ack=17 flow_ml=248 err=none
```

| key | rule |
| --- | --- |
| `c=` | `PB_CONTROLLER` from `secrets.h`. **Never empty** — an empty `c=` is "no `c=` in the report" (`butler.py:252-253`), a permanent 400. `static_assert`ed non-empty. |
| `t=` | **`g_t_wire = hal_boot_salt() + hal_millis()`, `uint32_t`, printed `%lu`.** Stamped once; the retry resends the identical bytes. **A second, separate variable `g_t_ms = hal_millis()` (raw, unsalted) is stamped at the same moment and is the ONLY thing the retry deadline is measured against** — §4.4. See §15.2 for why not `uint64_t`. |
| `chN=` | last **complete** sweep, `ch0..ch5`. A channel whose select failed is **omitted**, never sent as 0. |
| `ch200..ch209` | the diagnostics (below). **At least one of these is always present**, so a legal report always exists. |
| `float=` | the **debounced tank verdict**, ANDed with `!contra_latched`, forced to 0 above `PB_FLOAT_FLAP_LIMIT` consecutive `DOSE_REFUSED_FLOAT` results (§2.10 — one definition, and that is it). **Never 2** — `_int_in(v,"float",0,2)` is half-open. |
| `pos=` | `unknown` unconditionally while `PB_REPORT_POS_UNKNOWN` is defined, **and unconditionally while `g_nv.dry_latched` is set** (§2.10). Otherwise `ok` only when the gate pitch is calibrated **and** a home has been seen since boot **and** the last expander read succeeded. |
| `ack=`/`flow_ml=` | structurally one optional pair in `report_in_t`, **mandatory in both directions**: `flow_ml` cannot be emitted without `ack`, and **every `ack=` carries a `flow_ml=`** — 0 for a refusal. Butler's ack UPDATE writes `flow_ml = ?` unconditionally (`:830`), so an `ack=` without one stores NULL, which charges the pot the **full `ml`** against its daily cap (`COALESCE(flow_ml, ml)`, `:747`) and skips the `2*flow_ml < ml` branch entirely (`:1367`) — the expired-command penalty applied to a command the board acked. `ack` is `uint32_t` end to end and is never 0. |
| `err=` | a **bare lowercase token from a fixed enum**, tested to contain no whitespace: `none float pos noflow noise cap stop wdt dry contra boot range cal i2c busy cooldown leak adc stuck txcap resetmid heap goto recv`. A space here would split into a non-`k=v` token and 400 the whole report at exactly the moment it matters. `recv` is in the enum because `g_last_err` can hold it, and is the one token that **must never reach the wire** (§4.3). The LCD text is a **separate** string. |
| **every key at most once** | `parse_report` refuses the whole report on a repeated `c`, `t`, `ack`, `flow_ml`, `float`, `pos` or `chN` (`:220-250`). The body is assembled from four independent sources — wired channels, diagnostics, safety fields, the ack pair — into one buffer, so this is a rule the builder has to hold, not a property it gets for free. `PB_CANARY_CHANNEL` and the diagnostic indices are chosen disjoint from `0..PB_CHANNELS-1` for the same reason. |

**The diagnostics ride as high-numbered channels, not as unknown keys.** `parse_report` ignores
unknown keys and *stores them nowhere*; `evaluate()` never reads them. A "48-hour fragmentation
instrument" that only exists if a human tails the serial console for 48 hours is the definition of
not-unattended. `MAX_CHANNEL = 255` and `chN` values are `[0, 2^31)` (`butler.py:67,251`), so:

| channel | carries | why |
| --- | --- | --- |
| `ch200` | `mallinfo().arena` | break growth |
| `ch201` | `mallinfo().ordblks` | free-chunk count — the actual fragmentation signal |
| `ch202` | stack high-water mark, bytes of 1024 | the binding constraint (§12) |
| `ch203` | I2C error count | |
| `ch204` | seconds since D5 last changed | the dead-float instrument (§2.10). **A bare integer, `0` before D5 has ever moved — never `-1`, `unknown` or `never`.** |
| `ch205` | leak pulses since boot | |
| `ch206` | modem desync count | |
| `ch207` | contradiction latch, 0 or 1 | |
| `ch208` | cart parked off every gate, 0 or 1 | §2.9 |
| `ch209` | last `hal_wdt_alive()` counter delta | §2.5 — expect ~117, refuse below 58 |

**Every diagnostic is clamped on the way out: `min(v, 999999)`.** `chN` values must be in
`[0, MAX_RAW)` = `[0, 2^31)` (`butler.py:88,251`), and `ch203` (I2C errors), `ch205` (leak pulses)
and `ch206` (desyncs) are boot-monotonic. At the ISR's own 2 kHz ceiling
(`PB_FLOW_MIN_GAP_US = 500`) a permanently storming D2 pushes `ch205` past 2^31 in **~12.4 days** —
inside the 48-hour run's own margin for error — after which every report is 400ed on that one token
and the board stops reporting entirely. The clamp also fixes the body-width sum below: six digits is
the widest any diagnostic can be.

No pot maps to those channels, so `water_rules` and the per-sensor alert ignore them; they are stored
in `readings` and are graphable. They cost ten rows per report — 48 h × 60 reports/h × 10 =
**~29,000 rows** across the run, which SQLite will not notice. This also closes the blackout:
`parse_report` raises "no `chN=` in the report" when no channel is present (`:254-255`), so a wedged
bus emptying the mux mask used
to mean **sending nothing at all** — throwing away the honest `float=`, `pos=unknown` and `err=i2c`
that the backend's alert rules exist to read, and suppressing any pending ack with them. Now every
report is legal by construction and a wedged bus produces a specific alarm instead of silence.

**`err=` is still an unknown key**, deliberately: it is a token, not an integer, so it cannot be a
`chN` value. It reaches the serial console and the LCD, and is the reason `status` exists. Making the
backend store it is a separate, small backend change (§16.5).

**Ceilings.** `PB_DOSE_CAP_MS_MAX = 60000` matches `MAX_CAP_S`; `PB_DOSE_MAX_ML = 1000` matches
`MAX_DOSE_ML`, so the protocol parity the design argued for is preserved. But the **rig** ceiling is
`PB_DOSE_RIG_MAX_ML = 250`, and that is what `dose_run()` enforces: DECISIONS #7 asks for "a reservoir
small enough that a full dump is a mop-up", and a 1000 ml command against a one-litre reservoir makes
a full dump a single legal command. A dose above either ceiling is **refused with `err=range` and
acked**, never clamped — a clamp would deliver half the water and, because `2 × flow_ml < ml` is the
alert test, sometimes not even page.

> **The rig ceiling is invisible to the backend, and that is a real cost.** `MAX_DOSE_ML` stays 1000
> and per-pot `dose_ml` is free to exceed 250, so a pot configured for 300 ml — or one manual
> `water=N ml=300` — is queued, refused, acked with `flow_ml=0`, charged 0 ml, given a full cooldown
> and paged HIGH, once per cooldown period, **forever, and never watered**: the same never-ending
> alert loop §2.10 and §2.15 were rewritten to eliminate, arriving by a different door. Two
> mitigations, and the firmware owns neither: §4.6 makes "no enabled pot's `dose_ml` exceeds
> `PB_DOSE_RIG_MAX_ML`" a stated going-live precondition, and §16.5.8 asks butler for a per-controller
> `max_dose_ml` rejected at **config** time rather than at dose time.

### 4.2 The exchange

`POST /report HTTP/1.1`, `Host:`, `X-Token:`, `Content-Type: text/plain`, `Content-Length:`,
`Connection: close`, headers **and** body assembled into one `static char g_tx[PB_TX_CAP]` and
written with **one** `sock_write`.

- **`Content-Length` is computed from the bytes actually placed in the buffer**, and a truncated
  assembly refuses to send: `err=txcap`, the report is dropped, and `status` says so loudly.
  `HOST_NAME` and `BUTLER_TOKEN` come from a gitignored `secrets.h` with no length bound, and
  `snprintf` truncates silently; a 64-character token overflowing the buffer would cut the body while
  `Content-Length` still claimed the full length, uvicorn would wait for bytes that never come,
  `sock_close()` would raise `ClientDisconnect` → 400 "client went away" (`butler.py:1618-1621`), and
  because a 4xx is never retried, **every report would 400 from the very first one, forever**, with
  the console looking healthy.
- **Three `static_assert`s, in `netfsm.cpp` and `report.cpp`**, so every buffer bound is a compile
  error rather than an `err=txcap` at 3 a.m.:
  - `static_assert(sizeof(HOST_NAME) + sizeof(BUTLER_TOKEN) + PB_HDR_FIXED + PB_BODY_CAP <= PB_TX_CAP)`
  - `static_assert(sizeof(PB_CONTROLLER) + 2 + PB_BODY_WORST_FIXED <= PB_BODY_CAP)` — the body
    against **its own maxima**, not just the tx buffer against the body cap. §7 carries the
    worst-case sum that `PB_BODY_WORST_FIXED` is derived from, term by term.
  - `static_assert(sizeof(PB_CONTROLLER) > 1)` — an empty `c=` is a permanent 400 (`:252-253`).
- `netfsm` states: `DOWN → JOIN_ISSUE → JOIN_WAIT → IDLE → SOCK_CLOSE → CONNECT → SEND → RECV →
  CLOSE`, **one bounded link/socket step per `net_poll()`, at most 2 AT commands** (§3's table gives
  the count for each state), each with its own deadline. `RECV` scans for CRLFCRLF then reads up to
  `Content-Length` into `static char g_rx[PB_RX_CAP]` with a `PB_NET_DEADLINE_MS` deadline spread
  across passes; the backend's real answer is under 60 bytes.
- **`g_rx` and `g_rx_len` are zeroed on entering `SOCK_CLOSE`**, every cycle, so no byte from an
  earlier round trip can survive into this one's parse.
- **Only a 200 body reaches `response_parse`.** Butler's 400 body echoes the board's own tokens
  (`f"{key}= out of range: {value}"`), so a 4xx body could otherwise be parsed for `cmd=`/`ml=`.
- **Every** exit — success, 400, 401, 413, timeout, parse failure, truncation, `abort()`, and a
  **failed open** — goes through one `finish(err)`, which records the error and **transitions to
  `SOCK_CLOSE`**. It does not call `sock_close()` inline; §3 change 2 explains why that one line is
  the difference between a 2-AT and a 3-AT worst pass.
- Exactly **one** `WiFiClient` for the life of the program (a `link_wifi.cpp` file static), so its
  1 KB `FifoBuffer` (`WiFiClient.cpp:6-8`) is heap-allocated once.
- The last HTTP status is on the **LCD**, not only in `status`: a 400/401 loop is otherwise invisible
  to anyone not on the serial port.

### 4.3 The ack offbeat

1. The response to report N may carry one command. **The moment `response_parse` yields one, the ack
   slot is set to `(id, flow_ml = 0, err = "recv")`.** It is stored in `g_cmd` and **executed on a
   later pass**, never inside `net_poll()`.
2. `exec_pending()` range-checks the outlet, runs `cart_goto(outlet)` (homing first if position is
   unknown), then `dose_run(...)`, then parks with `cart_home()`, and each step **overwrites** the
   ack slot with its real result (§3's `exec_pending()` listing).
3. Report N+1 carries `ack=17 flow_ml=248 err=none`. The slot clears **after** a 200.

**The invariant that makes step 1 safe, stated because it is load-bearing and was not.** *No report
may be built while `g_cmd.pending` is true and the ack slot still carries `err=recv`.* If the report
interval elapses in that window, **the report waits.** Without this rule the placeholder can reach
the wire, and butler then applies `state='acked', flow_ml=0` (`:829-833`), raises a HIGH "the meter
counted 0 of N ml" (`:1367-1369`), sets the pot's cooldown from `acked_ts` (`:736-742`) and charges
0 ml against the daily cap — **and then the board runs the full dose.** What saves it today is a
property of `loop()`'s order (`net_poll()` before `exec_pending()`, one FSM step per pass, so
`exec_pending()` always gets the pass in which the FSM first reaches `NET_IDLE`) — a true property,
but an emergent one, one edit away from being false. `test_no_report_is_built_between_receiving_a_command_and_executing_it`
and `test_err_recv_never_reaches_the_wire` pin it.

**The ack is a property of receiving a command, not of running a dose.** The design set it only in
`dose_run()`, so a `cart_goto` that failed — stall, timeout, or `PB_PULSES_PER_GATE == 0` — never
reached it, and `CMD_STOP` had no handler at all. A command the firmware definitely received and
definitely refused would then sit `sent`, be expired by the next report, charge the pot the full `ml`
against its daily cap (`butler.py:745-751`) and raise a **HIGH** "handed to the board and never
acknowledged" page (`:1361-1364`) — the exact outcome the ack-refusals rule was written to avoid. An
unacked `stop=1` is worse because it is invisible: the dose-judging query filters `kind = 'water'`
(`:1336`), so an expired stop produces no alert and the operator believes it was delivered.

`CMD_STOP` is handled explicitly: cancel any pending water command, ack with `flow_ml=0 err=stop`.

**A refused or aborted dose is still acked**, with the honest `flow_ml` (0 for a refusal) and an
`err=` token. Both alert branches are **`priority = "high"` with the same `warning,droplet` tags**
(`:1361-1364` and `:1367-1369`) — one is not louder than the other, and the design's claim that it
was is wrong. What actually differs is the message text and, decisively, the **daily-cap charge**: an
**acked** refusal charges 0 ml (`COALESCE(flow_ml, ml)`, `:745-751`) while an **expired** one charges
the full `ml`. That is the reason to prefer acking a refusal.

**A pending ack survives EVERY discard path** — 4xx, 401, 413, retry-exhausted, and a report
suppressed for any reason — and rides the next report. That is safe because the ack `UPDATE` runs
before the expire `UPDATE` (`:829-833`, `:837-841`) and a discarded report never reached the server.

> **But the carry window is bounded by the command TTL, not by the report interval.** It holds only
> while the row is still `sent`. Besides the unconditional expire there are five more sweeps
> (§0's table), and after `BUTLER_CMD_TTL_S` — 900 s by default, enforced ≥ 2 × `BUTLER_NEXT_S` at
> startup, `:586-599` — any `POST /command` or `/approve` will sweep the row (`:887-891`,
> `:1011-1015`), turning a late ack into a no-op that charges the full `ml` and pages HIGH. At
> `next=60` that is fifteen consecutive failed reports of headroom; at a longer interval, less.

**Replay guard.** `response_parse` rejects any `cmd=<id> <= g_nv.cmd_high_water` and bumps the
high-water mark the moment a command is accepted. Without it, a response body left over from an
earlier round trip — the exact thing a poisoned AT session produces — is parsed as this cycle's
answer, and a `cmd=17 water=3 ml=500` is executed a **second** time: the backend has already marked
17 acked or expired, the second `ack=17` lands on a row that is no longer `state='sent'` so the
UPDATE is a silent no-op (`:829-833`), the pot's cooldown and daily cap never see it, and the plant
gets double the water with no alert. The high-water mark lives in `.noinit` so a warm reset does not
reopen the window.

> **Its monotonicity assumption rests on a schema comment, and its failure is silent.**
> `commands.id` is `INTEGER PRIMARY KEY` — a rowid alias with **no `AUTOINCREMENT`** — so ids are
> monotonic only because `schema.sql` says rows are never deleted. A rebuilt or restored database (a
> lost `/data` bind mount, a fresh test DB, a prune) restarts ids at 1, and the board then rejects
> **every** command as a replay until a cold boot clears `.noinit`; each one expires unacked and
> pages HIGH, with nothing on the board saying why. So: **`status` prints `cmd_high_water=`**, the
> recovery is a cold boot (power cycle, not RESET), and §16.5.9 asks butler for `AUTOINCREMENT` so a
> prune cannot reuse an id.

### 4.4 Retry, and 4xx vs 5xx

**The retry-eligible set is exactly two cases, and nothing else:**

- **(a) zero response bytes arrived** — no answer, a reset socket, a socket that closed before the
  first byte; and
- **(b) a COMPLETE 503 response.**

Everything else is discarded: a 4xx, a **truncated reply**, a parse failure, a 500, and any other
non-200. The retry resends the **identical** bytes once — same `t=`, which is exactly what the 300 s
`(controller, t)` dedup is built for.

The four rules behind that set:

- **If ANY response bytes arrived, do not retry.** Treat the server as having the report. When the
  request lands and the *response* is lost, the backend has already moved a command `queued → sent`
  (`:870-873`); the retry then hits the unconditional expire (`:837-841`) and kills a command the
  board never saw — costing a HIGH "never acknowledged" page for a dose that never existed, and
  charging the pot the full `ml` (`flow_ml` NULL) against its daily cap. **Retrying is what destroys
  it.** This is why "truncated reply" and "parse failure" are on the *never-retry* side: a truncation
  is bytes that arrived, so retrying it is precisely the act this rule forbids. Their recovery is the
  ack-rides-the-next-report path of §4.3, which costs one interval and destroys nothing. The rule
  narrows the window; it cannot close it in firmware, so the backend follow-up is §16.5.7.
- **The retry has a deadline, measured on RAW `millis()`.** `PB_RETRY_DEADLINE_MS = 30000`, well
  inside `RETRY_WINDOW_S = 300` (`butler.py:86`). The test is
  `hal_millis() - g_t_ms >= PB_RETRY_DEADLINE_MS`, where **`g_t_ms` is the unsalted `hal_millis()`
  stamped alongside `g_t_wire`** (§4.1). It is *not* `hal_millis() - t_stamp` against the wire value:
  `t_wire = salt + millis`, so `millis() - t_wire` evaluates to `elapsed - salt` (mod 2^32), which
  for any non-trivial salt is a huge number — **every retry would be abandoned before it was sent**,
  and cycle 1's "retry once" requirement would quietly not exist (or, for an unlucky salt, never be
  bounded at all). Two variables, one purpose each.
  Past the deadline the report is **abandoned**, never sent: a retry deferred to the next interval
  falls outside the dedup window, is not recognised as a retry, and inserts the same reading twice
  with the same `t=` — polluting the 5-sample median window with duplicates of one sample and running
  `water_rules` on it a second time, which with `cooldown_h = 0` ("0 disables", `butler.py:94`) can
  queue a second dose on the strength of a duplicated reading. The byte-identical-retry argument is
  correct and useless if the timing is wrong.
  `test_a_retry_is_abandoned_rather_than_sent_outside_the_dedup_window` runs with a **non-zero boot
  salt**, so a single-stamp implementation fails it.
- **A 4xx is never retried**: the backend answered; the same body cannot get better. **A 503 is**,
  once: it is raised only on `sqlite3.OperationalError` (`:1638-1639`), which rolls the whole
  `BEGIN IMMEDIATE` transaction back, so retrying it is provably safe. **Every other non-200, 5xx
  included, is discarded like a 4xx** — anything else raised in the threadpool (a
  `sqlite3.DatabaseError` on a corrupt file, say) surfaces as a FastAPI 500 with no rollback
  guarantee, so it is not in the same class as a 503. This is a deliberate deviation from
  `fake_device.py`, which treats every `HTTPError` as terminal.
- **"The same report" means the same `c=` and `t=` inside 300 s of the FIRST one's arrival** — the
  dedup query compares against the original's arrival `ts` (`:846-854`), not against anything in the
  body. Byte-identity is *sufficient, not necessary*, and the firmware resends byte-identical bytes
  because that is the cheapest way to be sufficient. **A duplicate is not a no-op**: it still updates
  `controllers.last_seen`, the whole `status` row including `float_bad`/`float_bad_prev` and
  `pos_bad`/`pos_bad_prev`, applies the ack UPDATE, runs the unconditional `sent`-expire, and can be
  handed a queued command. Only the `readings` insert and `water_rules` are suppressed. So a retried
  `float=0` report counts as **two** bad sightings and can raise the flap alert off a single report —
  which is what makes "never retry once any response bytes arrived" mandatory rather than prudent.

### 4.5 Response parsing — where a fault becomes water

```c
typedef enum { CMD_NONE, CMD_WATER, CMD_STOP } cmd_kind_t;
typedef struct { uint32_t id; cmd_kind_t kind; uint8_t outlet; uint16_t ml; uint16_t cap_s; } cmd_t;
typedef struct { uint16_t next_s; cmd_t cmd; } response_t;
bool response_parse(const char *body, uint16_t len, response_t *out);   /* 200 bodies ONLY */
```

Rejections, each a named test:

- `cmd=0` → no command, and never acked (`ack` must be ≥ 1 or the whole report 400s).
- `cmd=<id> <= g_nv.cmd_high_water` → no command (replay).
- `water=` without `ml=` → no command. `water=` without `cap_s=` → no command (an absent cap is an
  unbounded run). `ml=0` → no command.
- a truncated body mid-token → no command. A half-read reply must never water.
- a body that did not arrive with a `Content-Length` whose bytes were fully read → no command.
- `next=` outside [5, 3600] → keep the previous interval.
- unknown keys ignored.
- an outlet outside 1..`PB_OUTLETS` — **including `water=0`, which butler accepts** (`:291,:311-312`)
  — **is** accepted here, then refused and **acked with `err=range`** by `exec_pending()`'s range
  check, which runs *before* `cart_goto()` for exactly this reason (§3). So the backend learns the
  real reason instead of receiving whichever step happened to fail first, and instead of the command
  expiring silently.

### 4.6 `PB_REPORT_POS_UNKNOWN` — the going-live switch

```c
/* include/config.h */
#ifndef PB_REPORT_POS_UNKNOWN
#  define PB_REPORT_POS_UNKNOWN 1      /* SHIPS DEFINED. See docs/…/2026-09-03-bench-sketch-design.md §4.6 */
#endif
```

set from `platformio.ini`'s `[env:uno_r4_wifi]` build flags as `-DPB_REPORT_POS_UNKNOWN=1`.

While it is 1, `report_build()` emits `pos=unknown` on **every** report regardless of what
`cart_pos_known()` says, and `status` prints `pos: FORCED unknown (PB_REPORT_POS_UNKNOWN=1)`.
`water_rules` returns at `butler.py:680-681`, so no dose is ever queued, so no backend water command
exists to execute, no matter what the bench does. Everything else — reporting, the ack path, the
console, `dry`, the contradiction latch — works normally.

**What it costs on the phone, which is not nothing.** `pos=unknown` on every report sets `pos_bad`
and `pos_bad_prev` each time (`:811-814`), so from the **second** report `flapped` is true
(`:1267-1275`) and butler pages **HIGH**: *"bench1 lost track of its manifold position: watering is
on hold"*. Expect it **about two minutes after first boot**. It raises once — `raised(key)`
suppresses a re-raise — and then **stands for the entire bench programme**, until
`PB_REPORT_POS_UNKNOWN` is flipped and a `pos=ok` report clears it. Two consequences to plan around:
the operator must not read that page as news, and **while it stands, a genuine position loss during
the 48-hour run cannot page** — the `pos:` rule is deaf for the duration. If that is unacceptable for
the run, §16.5.10 is the backend change that fixes it properly (suppress `pos:` while a controller
has never reported `pos=ok`). §13's 7e row repeats this so the operator meets it on the bench, not on
their phone.

**Flipping it to 0 turns on backend-decided watering.** It is a commit and a reflash, deliberately,
and its preconditions are:

1. bring-up 7d complete and the relay polarity confirmed on the bench;
2. `PB_PULSES_PER_GATE` and `PB_PULSES_HOME_TO_1` measured at bring-up 6 and committed non-zero;
3. `PB_ML_PER_S_MEASURED` committed from 7b, so the measured cap clamp is live;
4. the 48-hour unattended run passed on the `bench` binary, `build=bench` confirmed in `status`;
5. `butler.py`'s `FLOW_FLOOR_ML_S` updated from 7b's number (§16.5.6);
6. **every enabled pot's `dose_ml`, and any manual `water=N ml=…`, is ≤ `PB_DOSE_RIG_MAX_ML` (250).**
   The firmware ceiling is the one that binds, and butler does not know it: a pot configured above it
   is refused, acked, cooled down and paged HIGH once per cooldown period, forever, and never
   watered (§4.1). Check the `pots` table before flipping, and see §16.5.8.

### 4.7 Phone notifications

Zero firmware code. `butler.py`'s alert ticker already raises and pushes to ntfy on: controller
silent, a mapped channel gone missing, `float=0`, `pos=unknown`, a safety field that vanished, a dose
never acked or short on the meter. All of it is driven by `float=`, `pos=`, `ack=` and `flow_ml=`,
which this design sends honestly in every report. **The firmware's entire contribution to
"notifications on the phone" is never lying in those four fields**, and DECISIONS #2 keeps ntfy in the
backend.

---

## 5. Screens, and the I2C bus they share

### OLED (u8x8, 16 × 8)

```
PB bench1  1h23m       build id, uptime in MINUTES (not seconds — see the bus rule)
pos ?  p 1290          cart position (or "pos ?") and screw pulses since home
float OK   pump off    the two safety inputs, in words
flow 0/s  tot 5881     pulses/second and total since reset
wifi UP  -52 dBm       link state and RSSI
192.168.1.42           IP, or "joining..." / "no link"
rpt 200  next 60s      last HTTP status, the interval butler asked for
cmd 17 ok 248ml        last command, result and metered ml — or "cmd 17 REFUSED float"
```

### LCD (16 × 2) — the across-the-room view

```
row 0:  IDLE          | MOVE o3       | PUMP o3       | WIFI?     | REFUSED   | CONTRA LATCH
row 1:  next 35s      | p 1290/1450   | 248/500ml     | HTTP 400  | float NOT OK | float ok, no flow
```

Row 1's refusal text is **human prose** and is never the `err=` wire token.

### The bus rule, and the honest cost of keeping the screens

A4/A5 carry the mux select lines and the home hall — the input that gates the pump. So:

- **Neither screen is painted while `g_dosing`, `cart_busy()`, or a modem command ran this pass.**
  The LCD gets one static line before D6 goes hot; both screens freeze for the length of a dose and
  catch up after. That is the visible price of keeping them, and it is stated on the LCD.
- **The per-second clock is gone.** A 48-hour run is **172,800 seconds**, so a per-second repaint of
  one changed OLED row and one changed LCD row is 172,800 repaints and, at **102** Wire transactions
  per 16-character LCD row (16 characters × 6, plus a `setCursor` that is itself a `command()` →
  `send()` → 6 more; `LiquidCrystal_I2C.cpp:255-278`), roughly **17.6 million I2C transactions whose
  entire purpose is a ticking clock**, on the bus that gates the pump. So: uptime in **minutes**,
  `next` in **5 s** steps, and the LCD repaints on **state transitions only**. Per-second counters
  stay on the OLED, where u8x8 pushes 8-byte tiles (~17 transactions per row, not 102). Roughly a
  30× cut in bus traffic. `status` prints transactions/minute so the cost stays visible.
- **`ui.cpp` writes an LCD row as a loop of `lcd.write(c)` with `safety_tick()` between characters —
  never `lcd.print()`.** `Print::print(const char*)` offers no hook between characters, so a row
  painted with it is one un-fed span of up to 16 × 6 = 96 transactions; on a wedged bus at the core's
  fixed 1000 ms transfer timeout (`Wire.cpp:194`) that is 96 seconds without a feed, and the watchdog
  bites at 5592 ms. §3's "fed, per character" is only true of the `write()` loop. `make check` gains
  **zero hits for `lcd.print|lcd.println`**.
- **Nothing may call `Wire.flush()`** — `Wire.cpp:833-835` spins with no bound, unlike
  `endTransmission`/`requestFrom`, which the core bounds at a fixed 1000 ms (`Wire.cpp:194`, private,
  no setter). I grepped `.pio/libdeps/uno_r4_wifi`: `LiquidCrystal_I2C` and u8x8 are **both clean
  today**. `make check` keeps our code that way; any library later added to this bus must be
  re-grepped, because the greps only scan our source.
- **Init ordering is fixed.** `LiquidCrystal_I2C::init_priv()` calls `Wire.begin()` and `delay(1000)`
  (`:66,:89`) and would re-open the IIC peripheral if it ran after `sensors_begin()`. So the order in
  `setup()` is: `hal_i2c_probe(0x3C)` and `hal_i2c_probe(0x27)` **first**, then both screens'
  `begin()`/`init()`, **then** `sensors_begin()`. Those library `delay()`s are on the documented
  "not tested on the host" list (§9); the greps cannot see them.
- `Screen::probe()` at boot: a device that does not answer becomes a permanent no-op instead of
  wedging in `Oled.begin()` or `LiquidCrystal_I2C::init()`. This is also the root fix for the old
  `Manifold::log` null-screen hazard: the dependency is inverted — `ui.c` reads the cart, the cart no
  longer writes to a screen.
- After `PB_I2C_FAIL_LIMIT` (3) consecutive failures a device backs off 5 s; `sensors_i2c_healthy()`
  goes false, `pos=unknown` follows, and `dose_run()` refuses. The nine-clock SDA recovery (§2.13)
  runs at boot and on each back-off expiry — **never inside a dose**.
- **The stuck-mux canary.** `sensors_select()` returns true whenever the PCF8575 ACKs the port write,
  so an unpowered mux, a broken S-line or a floating EN gives the *same* ADC value on every channel
  with no error raised anywhere. The backend then stores five identical raw counts, the 5-sample
  median is perfectly stable, and `water_rules` waters five pots on a harness that is not connected —
  the one failure the "raw on the wire, calibration in the backend" split cannot catch, because from
  butler's side a stuck mux and five equally-dry pots are byte-identical. So every sweep reads one
  **unwired** channel (C6..C15 are unused by the mux table) as a canary. If the canary equals every
  wired channel, raise `err=stuck`, **omit the wired channels** from the report, and count it — the
  diagnostic channels keep the report legal, and the backend's existing "sensor stopped reporting"
  HIGH alert fires. One extra select per minute converts a silent lie into an alarm.

---

## 6. The console, and the two binaries

**The bring-up console does not ship in the binary that runs unattended.** `pump 60000 prime hang`
was a single typed line that removed all three of DECISIONS #10's mandatory measures at once: it
asserted D6, suppressed the no-flow abort (§2.8), and starved the watchdog. Over an unauthenticated
USB CDC line, a stray line from a serial-monitor reconnect, a `cat` of the wrong file into
`/dev/cu.*`, or an autocompleting terminal is enough. Gating on the spelling of a token is not a
gate.

| command | `bench` | `bringup` | does |
| --- | --- | --- | --- |
| `i2c` | ✓ | ✓ | scan Wire, list addresses (expect 0x20, 0x27, 0x3C; 0x38 if the kit has the DHT20) |
| `mux <0-15>\|all` | ✓ | ✓ | select, ≥1 ms, read twice, print the second; 14-bit raw |
| `hall` | ✓ | ✓ | stream screw / home / float; an I2C error prints `home unknown`. **Never calls `pinMode` on D2/D3.** |
| `flow` | ✓ | ✓ | pulses/second and total since reset |
| `status` | ✓ | ✓ | see below |
| `stop` | ✓ | ✓ | cuts a dose in progress; also matched byte-wise inside the dose loop (§2.12) |
| `dry on\|off` | ✓ | ✓ | latch; refuse every dose. `dry on` also raises the stop request. |
| **`clear contra`** | ✓ | ✓ | the **only** way to release the contradiction latch (§2.7) |
| `help` | ✓ | ✓ | one screen |
| `servo <±us> <ms>` | — | ✓ | bounded jog, ≤ `PB_SERVO_CAP_MS` |
| `home` | — | ✓ | run toward home until HALL_HOME, bounded, zero the count |
| `goto <1-5>` | — | ✓ | step to the outlet counting screw pulses, bounded, stall-aborted |
| `pump <ms> [prime] [hang]` | — | ✓ | `dose_run` with `need_pos = false`, `by_time = true`, `cap_ms = min(ms, 60000)`. `prime` **extends** the prime window and caps the dose at `PB_PRIME_CAP_MS`. `hang` starves the dog (7c). |
| `calib` | — | ✓ | `pump 10000 prime` plus the summary line — the 7b workhorse |
| `cal <pulses_per_l>` | — | ✓ | set the calibration at runtime, **1000..20000 only** |
| `noinit pattern` | — | ✓ | write a known word into the `.noinit` struct's spare slot; `status` prints the struct raw. Bring-up 7c′ — the one thing that turns §2.3 from a claim into a measurement. |
| `sim ...` | — | sim only | §8 |

`-DPB_BRINGUP=1` lives in `[env:uno_r4_wifi_bringup]`, which is never the env left running.
`status` prints `build=bench|bringup|sim`.

**The mechanical check, scoped so it can actually pass.** `tools/check.sh` asserts that the
**`bench` env's preprocessed `src/cli.cpp`** contains **zero** hits for `hang`, `" prime"` and
`"cal "`, and that `PB_BRINGUP` appears only in `src/cli.cpp` and `src/main.cpp`.

> `long_prime` is deliberately **not** in that grep. It is a member of `dose_req_t`, declared in
> `safety.h` and read by `dose_run()` in `safety.cpp` — both compiled into the bench env — so a grep
> over the whole preprocessed source could never return zero. The only ways to satisfy it would be
> to delete the grep (losing the guard) or to `#if PB_BRINGUP` the field, which breaks the property
> the next paragraph relies on. `long_prime` is a parameter of the safety layer, not a console
> affordance, and its value is only ever `true` from bring-up-gated code; the console tokens are what
> the grep is for.

The lost coverage is replaced by a **stronger** check, which is what §6 wanted all along:
**`safety.o` and `hal_uno.o` hash identically between `uno_r4_wifi` and `uno_r4_wifi_bringup`**
(`shasum` the two object files after `pio run -e … -e …`). That is a direct proof that the safety
layer and the pin layer compile the same in both envs — which is what lets bring-up 7c prove the
watchdog on the `bringup` binary and have that mean something for the `bench` one.

`clear contra` is a genuine disarm and it ships in both. It has to: the unattended binary can latch,
and a rig that cannot be released except by a reflash is worse. It is two literal tokens, it prints a
banner, and the residual risk — a stray serial line spelling exactly `clear contra` — is accepted and
written down here.

**`cal` is bounded.** `cal 0` — one token on the serial line, or a stray byte parsed as one — used to
make `target = 0` for every subsequent water command, so each dose ignored its millilitre target and
ran the full `cap_ms`; `pulses_to_ml` then divided by zero, and the Cortex-M4's UDIV returns 0 without
`DIV_0_TRP`, so it would have acked `flow_ml=0` — the flood happens and the report says nothing came
out. `cal` now accepts only `[PB_PULSES_PER_L_MIN, PB_PULSES_PER_L_MAX]` = 1000..20000 and prints a
refusal otherwise, and `dose_run()` re-checks the range above the assert (`DOSE_REFUSED_CAL`).
`target` is `ml * cfg / 1000` — **multiply first** (§2.8; the reverse order truncates the calibration
to whole pulses per millilitre and under-delivers 15% at the nominal 5880). There is no overflow to
avoid: the enforced ranges make it impossible, `PB_DOSE_MAX_ML (1000) × PB_PULSES_PER_L_MAX (20000)`
= 2 × 10^7, three orders of magnitude below `UINT32_MAX`.

**`status`** prints, on one line each: `build=`, compiled pump level and polarity, `wdt=on
granted=5592ms alive=yes delta=117 (WDT, not IWDT)`, `icufilter=`, modem timeout in force, the
`setConnectionTimeout` value and command form actually sent, desync count,
uptime, `dry=`, `contra=`, `sim=`, cart state / position / `parked=` / `cart=UNCALIBRATED` if so,
`pos: FORCED unknown` if so, screw and flow counters and `irq_armed=`, `pulses_per_l`, prime and stall
windows, the measured cap clamp or `cap=UNCLAMPED`, I2C error count and transactions/minute, link
state and RSSI, last HTTP status, reports ok/failed, **`cmd_high_water=`** (§4.3's replay guard —
without it a rebuilt backend database is a silent, unexplainable refusal of every command),
`adc_req=14 adc_hw=14` (§7), `arena`/`ordblks`/`break`/`stack_hwm` with their minima and maxima and
the headroom to `__StackLimit`, and `last=<err token>`.

**The per-dose summary line — the pitch's deliverable.** Printed at the end of every dose, from every
path:

```
dose outlet=3 ms=4120 pulses=706 ml=120 mls=29.1 r=ok
```

`mls` is the ml/s figure pitch-9f9267 buys. **It is computed in integer tenths and printed
`%lu.%lu`**; `%f`/`%g` are banned and grepped for (§12).

**The seconds fallback.** If 7b's verdict is that the meter does not pulse at the rig's real flow,
`-DPB_DOSE_BY_TIME=1` makes `exec_pending()` build the request with `by_time = true` and
`cap_ms = min(ml * 1000 / PB_ML_PER_S_MEASURED, cap_ms)` — **the same constant the cap clamp uses**,
not a second one; there is no `PB_ML_PER_S`. The flag is `#error`ed if `PB_ML_PER_S_MEASURED == 0`,
because a by-time dose against an unmeasured rate is an unbounded run in a costume. Four lines, one
build flag, and the pitch's stated fallback exists. It ships as a **commented** line in
`[env:uno_r4_wifi]` (§10) so enabling it is an edit and a commit, not a memory. `by_time` is an
explicit field, so nothing else in the program changes meaning.

---

## 7. `include/config.h` — every number, with its citation

```c
/* ---- fixed by the wiring package ---- */
#define PB_OUTLETS               5
#define PB_CHANNELS              6      /* C0-C4 moisture, C5 LDR */
#define PB_CANARY_CHANNEL       15      /* unwired by the mux table: the stuck-mux canary (§5) */
/* analogReadResolution(bits) does NOT change the hardware width: ANALOG_READ_HARDWARE_
   RESOLUTION_FIXED is defined (analog.cpp:599), the hardware is fixed at open time to
   BSP_FEATURE_ADC_MAX_RESOLUTION_BITS = 14 (:34-45), and adcConvert map()s hw -> requested
   (:495). So 14 is an identity map TODAY -- but the default requested value is 10 (:11), and
   a core bump that changed the fixed width would silently rescale every raw count on the
   wire. hal_begin() therefore ASSERTS analogReadResolution() == PB_ADC_BITS after setting it
   (:698), and `status` prints both the requested and the hardware width. */
#define PB_ADC_BITS             14

/* ---- the watchdog. PCLKB = 24 MHz (bsp_clock_cfg.h:8,14: HOCO 48 / PCLKB_DIV 2).
   RL_16384 * PR_8192 / (PCLKB/1000) = 16384*8192/24000 = 5592 ms (WDT.cpp:105-113).
   We use the wdt_cfg_t overload to get stop_control = DISABLE. WDT.cpp:67 is the
   OTHER overload, begin(uint32_t), setting ENABLE -- which is the reason this one
   exists, not evidence for DISABLE. r_wdt_api.h:113-114 defines both values,
   and that overload NEVER assigns _timeout (WDT.cpp:32-46), so getTimeout() would
   return 0 on a running dog. hal_wdt_granted() computes this number instead.
   The counter is a DOWN-counter at PCLKB/8192 = 2929.7 Hz = 2.93 counts/ms, which is
   what hal_wdt_alive() measures across an UNFED window (§2.5). ---- */
#define PB_WDT_GRANTED_MS     5592
#define PB_WDT_PROBE_MS         40     /* the ONE unfed window in the program. 40/5592 = 0.7%. */
#define PB_WDT_PROBE_MIN_COUNTS 58     /* half of 40 * 2929.7/1000 = 117: no false negative on
                                          jitter, no false positive on a frozen counter */

/* ---- the network. MODEM_TIMEOUT default is 10000 (Modem.h:12), nearly twice the
   WDT window. The worst net_poll() pass is CONNECT = 2 AT commands (_BEGINCLIENT +
   _CLIENTCONNECT), which holds ONLY because (a) sock_close() always ran in a PRIOR
   pass and left _sock == -1 (WiFiClient.cpp:31,217), (b) every error exit routes
   through the SOCK_CLOSE state instead of closing inline, and (c) sock_read() is a
   bare client.read() -- no available(), no connected(). See §3's per-pass table.
   2 * 1200 = 2400 < 5592, and 2400 + SLACK = 4400 is what setup() asserts. ---- */
#define PB_NET_STEP_MS        1200
#define PB_NET_SLACK_MS       2000     /* setup() asserts granted >= 2*STEP + SLACK */
#define PB_NET_DEADLINE_MS    5000     /* RECV: also the closed-socket detector, since the FSM
                                          never calls connected() (2 ATs, WiFiClient.cpp:224-238) */
#define PB_NET_BACKOFF_MS     { 2000, 4000, 8000, 16000, 30000 }
#define PB_RETRY_DEADLINE_MS 30000     /* << RETRY_WINDOW_S = 300 (butler.py:86). Measured on
                                          g_t_ms (RAW millis), never on the salted g_t_wire. */

/* ---- the dose. Protocol ceilings MATCH butler.py: MAX_DOSE_ML 1000 (butler.py:89),
   MAX_CAP_S 60 (:90). The RIG ceiling is smaller, per DECISIONS #7's "a reservoir
   small enough that a full dump is a mop-up" -- and butler does NOT know about it,
   which is a going-live precondition (§4.6) and an owed backend change (§16.5.8). ---- */
#define PB_DOSE_MAX_ML        1000     /* == MAX_DOSE_ML: protocol parity */
#define PB_DOSE_RIG_MAX_ML     250     /* what dose_run() actually enforces */
#define PB_DOSE_CAP_MS_MAX   60000     /* == MAX_CAP_S * 1000 */
#define PB_DOSE_MIN_GAP_MS   10000     /* every caller. See §15.3 for why not 60 s. */
#define PB_BOOT_GAP_MS       10000     /* DECISIONS #5 "minimum gap since boot" */
#define PB_BOOT_HOME_MS      15000
#define PB_POS_RECHECK_MS     1000     /* live expander read inside the dose loop */
#define PB_COAST_MS           2000     /* impeller spin-down is not a leak */
/* cap_for(ml) = min(60, ml//FLOW_FLOOR_ML_S + 5) with FLOW_FLOOR_ML_S = 20 (butler.py:96,261-266)
   is a GUESS. At a real 30 ml/s, cap_for(500) authorises 1.8x the requested water, so
   the cap is not a bound exactly when the meter — the thing it stands in for — has failed.
   Once 7b measures the rate, this clamps the cap to 2x the requested millilitres. It is
   ALSO the constant -DPB_DOSE_BY_TIME=1 uses; there is no second ml/s constant (§6). */
#define PB_ML_PER_S_MEASURED     0     /* 7b fills this in; 0 == clamp disabled, status says so */
#define PB_CAP_SLACK_NUM         2
#define PB_CAP_SLACK_DEN         1
/* Delivered-vs-elapsed plausibility on the DOSE_OK path (§2.8): a dose that reaches its
   target in less than 1/4 of the time the measured rate says it needs is noise, not a fast
   pump. Armed only when PB_ML_PER_S_MEASURED > 0. */
#define PB_PLAUS_NUM             4
#define PB_PLAUS_DEN             1

/* ---- flow. GUESSES until bring-up 7b. YF-S401 nominal ~5880 pulses/L; the
   -0207 floor is 0.2 L/min and the -3507 ceiling 6 L/min = 588 pulses/s.
   `cal` sets pulses_per_l at runtime so 7b needs no reflash; 7b's numbers get
   COMMITTED here. ---- */
#define PB_PULSES_PER_L_DEFAULT 5880
#define PB_PULSES_PER_L_MIN     1000   /* `cal` and dose_run() both enforce this range */
#define PB_PULSES_PER_L_MAX    20000
#define PB_PRIME_MS_DEFAULT     3000
#define PB_PRIME_LONG_MS       15000   /* `prime` EXTENDS the window; it never removes it */
#define PB_PRIME_CAP_MS        20000   /* and caps the whole dose regardless of the typed ms */
#define PB_STALL_MS_DEFAULT     1500
#define PB_PRIME_MIN_PULSES        5
#define PB_FLOW_MIN_GAP_US       500   /* ISR reject; honest: only bites above 2 kHz */
#define PB_FLOW_MAX_HZ          1200   /* 2x the meter's 588/s ceiling -> DOSE_ABORT_NOISE */
#define PB_FLOW_IDLE_MAX_HZ        2   /* non-zero with the pump off -> DOSE_REFUSED_NOISE */
/* pulses_flow_rate()'s window, and it has to be STATED or the rate rules are unfalsifiable:
   at the ISR's own 2 kHz ceiling a 250 ml target (1250 pulses at cfg=5000) is reached in
   ~625 ms, so any estimator slower than that loses the race. 100 ms wins it 6x over. */
#define PB_FLOW_RATE_WINDOW_MS   100
#define PB_HANG_MS              3000   /* `pump <ms> hang`: run the dose this long, THEN stop
                                          feeding (bring-up 7c). PB_BRINGUP only. */

/* ---- the cart. Pulses-per-gate DOES NOT EXIST YET: today's Manifold is purely
   time-based, so there is nothing to port. Bring-up 6 measures it. Until then
   cart_goto() and cart_pos_known() are COMPILED OUT to hard false (§2.15). ---- */
#define PB_PULSES_PER_GATE       0     /* 6 fills this in; 0 == goto compiles to `return false` */
#define PB_PULSES_HOME_TO_1      0
#define PB_MOVE_CAP_MS       45000
#define PB_STALL_WINDOW_MS    2500
#define PB_SERVO_CAP_MS      10000
#define PB_SCREW_MIN_GAP_US   2000

/* ---- the float ---- */
#define PB_FLOAT_OK_SAMPLES      3
#define PB_FLOAT_SAMPLE_MS      20
/* After this many CONSECUTIVE DOSE_REFUSED_FLOAT results, the report forces float=0 and
   err=float regardless of the report-time debounce, and water_rules goes dark at
   butler.py:680-681. Cleared by any granted dose. Without it a float flapping at the
   waterline can grant in the report and refuse in the dose -- separate samples, minutes
   apart -- and the acked refusal sets the pot's cooldown and pages HIGH, forever (§2.10). */
#define PB_FLOAT_FLAP_LIMIT      3

/* ---- I2C. TwoWire's transfer timeout is a fixed 1000 ms (Wire.cpp:194, private,
   no setter); TwoWire::flush() (Wire.cpp:833) spins forever and is never called. ---- */
#define PB_I2C_FAIL_LIMIT        3
#define PB_I2C_BACKOFF_MS     5000
#define PB_I2C_RECOVER_CLOCKS    9     /* a FIXED count, never "until SDA releases" */

/* ---- going live. See §4.6. Ships DEFINED; flipping it turns on backend watering. ---- */
#ifndef PB_REPORT_POS_UNKNOWN
#  define PB_REPORT_POS_UNKNOWN  1
#endif

/* ---- buffers. Main stack is 1024 B (bsp_cfg.h:26) — every one of these is
   FILE-STATIC, never a stack local.

   PB_BODY_WORST_FIXED is the report body's worst case with `c=` and its value EXCLUDED,
   summed term by term at the maximum width the grammar permits, with every diagnostic
   clamped to six digits (§4.1):

       t=4294967295                        13
       six wired channels, chN=16383     6*10 =  60   (14-bit ADC: 5 digits)
       ten diagnostics, chNNN=999999    10*13 = 130   (clamped; unclamped it is 10*16 = 160)
       float=1                              8
       pos=unknown                         12
       ack=4294967295                      15
       flow_ml=1000                        13   (bounded by PB_DOSE_MAX_ML)
       err=resetmid                        13   (longest token is 8 chars)
                                          ---
                                          264, rounded up to 288

   The old PB_BODY_CAP of 288 therefore had NO margin at all: any PB_CONTROLLER longer
   than six characters overflowed, and the failure mode is err=txcap with the report
   DROPPED -- a silent reporting blackout, not a 400. netfsm.cpp static_asserts
   sizeof(PB_CONTROLLER) + 2 + PB_BODY_WORST_FIXED <= PB_BODY_CAP, and that
   sizeof(PB_CONTROLLER) > 1 (an empty c= is a permanent 400, butler.py:252-253). ---- */
#define PB_BODY_WORST_FIXED    288
#define PB_BODY_CAP            384
#define PB_DIAG_CLAMP       999999     /* every chN diagnostic is min(v, this) on the way out:
                                          chN must be < MAX_RAW = 2**31 (butler.py:88,251), and
                                          a storming D2 pushes ch205 past 2**31 in ~12.4 days */
#define PB_TX_CAP              768     /* PB_HDR_FIXED + HOST_NAME + BUTLER_TOKEN + PB_BODY_CAP */
#define PB_RX_CAP              256
#define PB_HDR_FIXED           128     /* the fixed part of the request line + headers */
#define PB_LINE_CAP             96
#define PB_STACK_MARGIN       2048     /* the break must stay this far below __StackLimit.
                                          _sbrk is UNCHECKED (§12), so this is the only bound
                                          that exists; crossing it latches err=heap. */

/* ---- .noinit ---- */
#define PB_NOINIT_MAGIC   0x50423031u  /* "PB01" */
```

Baud moves to **115200**. At 9600 an 80-character line of the `hall` stream is 83 ms of blocking — a
stall source in its own right. `AGENTS.md`, the `Makefile` and the bring-up notes move together
(§16.2, §16.3).

---

## 8. Sim mode

**A separate binary, entered by flashing it.** There is no serial command, no jumper and no EEPROM
byte that turns it on in the bench build.

```ini
[env:uno_r4_wifi_sim]
extends = env:uno_r4_wifi_bringup
build_flags = ${env:uno_r4_wifi_bringup.build_flags} -DPB_SIM=1 -DPB_CONTROLLER='"bench1sim"'
build_src_filter = +<*> -<hal_uno.cpp>          ; the pump driver is NOT compiled
lib_ignore = Network                            ; no WiFiS3 at all
```

Everything above the two seams is **byte-identical** to the bringup build. `hal_sim.cpp` models a tank
with a float line, a pump that emits flow pulses after a prime delay at a settable ml/s, a screw that
accrues pulses while the servo runs and passes a home region, six channels, the expander, a settable
WDT counter and a settable UART.

**Fault injection, each mapped to a bring-up step or a finding:**

```
sim float 0|1        D5 reads HIGH -> `pump` refused (5a); dropping it mid-dose is 5b
sim flow <ml_s>      0 exercises the prime abort; a mid-dose stop exercises the stall abort (7b)
sim flow storm       10 kHz on D2 -> DOSE_REFUSED_NOISE / DOSE_ABORT_NOISE
sim i2c fail|ok      every expander read fails -> position unknown -> pump refused, pos=unknown
sim mux stuck        every channel returns the canary's value -> err=stuck
sim stall on|off     the screw stops pulsing while driven -> goto aborts, position lost
sim leak on          pulses with D6 off -> ch205 rises and err=leak
sim wdt stop         the counter FREEZES -> hal_wdt_alive() false -> every dose refused
sim wdt slow <hz>    the counter moves, but too slowly -> delta below PB_WDT_PROBE_MIN_COUNTS
sim noinit clobber   scramble the .noinit struct -> the checksum fails -> reads as a cold boot
sim ch <0-5> <raw>   plant a raw count
sim resp "<body>"    inject a canned backend response: the ack offbeat, on a desk
sim reset warm|cold  re-enter setup() with .noinit kept or cleared
```

### How sim is prevented from ever pumping with 12 V present

The guarantee is **structural, and checkable**, not a runtime flag:

1. `src/hal_uno.cpp` is the only file that defines `PB_PUMP_OWNER`, and therefore the only file for
   which `pins.h` defines `PIN_PUMP_EN` at all. It is excluded from the sim env by
   `build_src_filter`. **In a sim binary no translation unit can even name the pin** — D6 is never
   made an output, stays an input from reset, and R1 holds the relay's OFF level in hardware even
   with 12 V on COM. `make check` verifies the exclusion by grepping the file set each env compiles,
   not a linker map (a map cannot prove this: `PIN_PUMP_EN` is a macro that leaves no symbol).
2. `lib_ignore = Network` — **the sim binary has no network stack**. It cannot mint a phantom
   `bench1sim` row in `controllers`, which matters because the silence rule iterates every controller
   with `last_seen > 0` (`butler.py:1137`) and **has no notion of retirement**: one afternoon of desk
   work would otherwise page the phone hourly, forever, until someone hand-edited the SQLite file.
   `sim resp` exercises the report cycle deterministically instead.
3. Loud: `SIM ` prefixes every serial line, `*** SIM: D6 NOT DRIVEN ***` holds OLED row 0 and LCD
   row 0 permanently, LED_BUILTIN double-blinks, and `status` prints `build=sim`.
4. `make sim` prints "SIM BUILD — the 12 V brick must be unplugged" before it uploads and writes
   `firmware-SIM.bin`.

**The reverse hazard** — the real build on the bench when you meant the sim — is covered by `dry on`,
which is strictly more refusing and therefore cannot fail dangerously.

**What this costs, plainly:** the binary the sim exercises is not the binary that meets 12 V. One file
differs — `hal_uno.cpp` versus `hal_sim.cpp` — and it is the ~230 lines no host test covers. That file
is kept deliberately stupid (every function one to six lines, no arithmetic, no state machine, a
review rule capping `if (` in it), and bring-up 1–4c is what actually proves it.

---

## 9. Tests

### `env:native`, Unity, `pio test -e native` — deterministic, `sim_advance()` drives the clock

**`test/test_dose/test_dose.cpp`**
```
test_boot_configures_d6_with_one_pfs_write_carrying_direction_and_level   (call-trace order)
test_pinmode_is_never_called_on_the_pump_pin
test_every_pump_write_restates_the_direction_as_well_as_the_level         (§2.1's repair property)
test_wdt_alive_is_false_only_when_the_counter_is_frozen                   (unfed probe, §2.5)
test_wdt_alive_does_not_feed_inside_its_probe_window                      (the fake asserts it)
test_wdt_alive_is_true_on_a_counter_that_moves_at_the_real_2929_hz
test_dose_refused_when_the_watchdog_counter_is_not_moving
test_dose_refused_when_the_dry_latch_is_set
test_dose_refused_when_the_contradiction_latch_is_set
test_dose_refused_inside_the_boot_gap
test_dose_refused_inside_the_minimum_gap_since_the_last_dose
test_dose_refused_when_the_float_reads_not_ok
test_dose_refused_when_a_single_float_sample_is_bad
test_dose_refused_when_position_is_unknown
test_dose_refused_when_the_cart_is_at_another_outlet
test_dose_refused_when_i2c_is_unhealthy
test_dose_refused_when_ml_exceeds_the_rig_ceiling
test_dose_refused_when_the_idle_pulse_rate_is_nonzero
test_cal_rejects_zero_and_absurd_values
test_metered_dose_with_a_zero_target_is_refused_not_run_to_cap
test_console_pump_does_not_require_a_known_position           (bring-up 4a/5a/5b)
test_dose_stops_at_the_millilitre_target
test_dose_stops_at_the_cap_when_flow_never_reaches_target
test_dose_cap_is_clamped_to_sixty_seconds
test_cap_is_clamped_to_twice_the_requested_millilitres        (once PB_ML_PER_S_MEASURED > 0)
test_prime_abort_fires_when_nothing_flows_in_the_prime_window
test_prime_flag_still_aborts_when_nothing_ever_flows          (replaces the design's inverted case)
test_prime_flag_caps_the_dose_at_the_prime_cap
test_stall_abort_is_armed_on_time_not_on_pulses
test_dose_aborts_when_the_pulse_rate_exceeds_the_meter_rating
test_a_storm_that_begins_AT_PUMP_ON_aborts_before_the_target_is_reached   (the idle guard
                                                                          cannot catch this one)
test_the_rate_rules_are_evaluated_above_the_target_rule                   (ordering, §2.8)
test_a_dose_that_reaches_target_implausibly_fast_is_noise_not_ok          (PB_ML_PER_S_MEASURED > 0)
test_target_pulses_match_the_calibration_within_one_pulse                 (cfg = 1000, 1999,
                                                                          5880, 20000 — the
                                                                          truncation bug)
test_five_spurious_edges_at_start_do_not_disable_the_abort
test_dose_stops_within_one_iteration_when_the_float_drops     (bring-up 5b, in software)
test_dose_aborts_when_the_expander_read_fails_mid_dose
test_stop_typed_mid_dose_stops_it_within_one_iteration        (through the real serial fake)
test_dry_on_typed_mid_dose_stops_it
test_bytes_buffered_during_a_dose_are_discarded_not_executed
test_pump_is_off_on_every_exit_path                           (all 18 results)
test_pump_on_time_never_exceeds_the_cap                       (all paths)
test_watchdog_is_fed_on_every_iteration_of_the_dose_loop
test_idle_safety_tick_rewrites_the_off_level
test_refusal_reports_zero_millilitres_not_the_previous_dose
test_dose_cap_holds_across_a_millis_rollover                  (clock starts at 0xFFFFF000)
test_ml_from_pulses_rounds_down_and_does_not_overflow
test_i2c_recovery_never_runs_while_the_pump_is_asserted
test_recovery_is_a_fixed_nine_clocks_with_sda_held_low
```

**`test/test_contra/test_contra.cpp`** — the new latch, on its own
```
test_latch_sets_when_the_float_said_ok_and_no_pulse_ever_arrived
test_latch_does_not_set_when_the_float_dropped_mid_dose        (the sensors AGREE)
test_latch_does_not_set_when_flow_started_and_then_stalled     (got > 0)
test_latch_does_not_set_when_the_dose_was_stopped_before_the_prime_window
test_latch_uses_the_doses_own_prime_window_not_the_configured_default
test_latch_does_not_set_for_a_console_prime_dose               (bring-up 7a on a dry line)
test_latch_does_not_set_for_a_dose_that_was_refused
test_latch_refuses_every_subsequent_dose_including_a_console_one
test_latch_does_not_refuse_homing                              (parking is still wanted)
test_boot_self_home_runs_under_both_latches                    (§2.11)
test_latch_reports_err_contra_and_ch207_and_float_zero
test_latch_survives_a_warm_reset_and_not_a_cold_one
test_latch_clears_only_on_the_literal_two_token_command
test_latch_is_not_cleared_by_dry_off_or_by_a_successful_home
```

**`test/test_report/test_report.cpp`**
```
test_report_carries_c_t_and_the_valid_channels
test_report_always_carries_at_least_one_diagnostic_channel     (never a blackout)
test_report_omits_a_channel_whose_read_failed_rather_than_sending_zero
test_report_omits_the_wired_channels_and_says_stuck_when_the_canary_matches
test_report_float_is_the_debounced_tank_verdict_anded_with_not_contra   (over a flapping sim float)
test_repeated_float_refusals_drive_float_to_zero_on_the_wire   (PB_FLOAT_FLAP_LIMIT)
test_a_granted_dose_clears_the_float_refusal_counter
test_report_float_is_only_ever_zero_or_one
test_report_pos_is_unknown_while_the_going_live_flag_is_set
test_report_pos_is_unknown_while_the_dry_latch_is_set
test_report_pos_is_unknown_when_the_gate_pitch_is_uncalibrated
test_report_omits_flow_ml_when_there_is_no_ack
test_report_never_emits_ack_without_flow_ml                    (the other direction; NULL flow_ml
                                                                charges the pot the full ml)
test_report_never_repeats_a_key                                (sweep where a channel index is
                                                                both wired and a diagnostic)
test_a_saturated_diagnostic_counter_stays_inside_max_raw       (PB_DIAG_CLAMP)
test_ch204_is_zero_before_d5_has_ever_changed_not_a_sentinel
test_err_recv_never_reaches_the_wire
test_no_report_is_built_between_receiving_a_command_and_executing_it
test_report_never_emits_ack_zero
test_report_ack_id_survives_above_sixty_five_thousand
test_report_t_is_unsigned_at_and_above_two_to_the_thirty_one   (0x7FFFFFFF, 0x80000000, 0xFFFFFFFF)
test_report_t_differs_across_two_boots_fifteen_seconds_apart   (the salt; the dedup aliasing bug)
test_report_err_token_never_contains_whitespace                (over EVERY producer of g_last_err:
                                                                every dose_result_t, the receipt
                                                                ack `recv`, the goto failure `goto`,
                                                                `resetmid`, `txcap`, `stuck`,
                                                                `heap`, `leak`, `adc` — not
                                                                dose_result_t alone)
test_report_body_is_byte_identical_on_the_retry
test_report_content_length_matches_the_bytes_actually_written
test_report_refuses_to_send_on_truncation_and_says_txcap
test_report_fits_the_buffer_at_maximum_field_widths            (t=0xFFFFFFFF, six wired channels
                                                                at 16383, ten diagnostics at
                                                                PB_DIAG_CLAMP, ack=0xFFFFFFFF,
                                                                flow_ml=1000, err=resetmid,
                                                                pos=unknown — §7's sum, executed)
test_report_matches_the_fake_device_shape                      (golden string vs build_report)
test_response_parses_next_only
test_response_parses_a_water_command
test_response_parses_a_stop_command
test_response_ignores_unknown_keys
test_response_rejects_command_id_zero
test_response_rejects_a_repeated_or_lower_command_id           (the replay guard)
test_response_rejects_water_without_ml_or_without_cap_s
test_response_rejects_ml_zero
test_response_truncated_body_yields_no_command
test_response_is_never_parsed_from_a_four_hundred_body         (canned 400 carrying cmd=1 water=3)
test_stale_bytes_in_the_rx_buffer_cannot_become_a_command
test_response_next_out_of_range_keeps_the_previous_interval
```

**`test/test_net/test_netfsm.cpp`**
```
test_http_post_carries_host_token_and_content_length
test_socket_is_closed_on_success_error_timeout_and_a_failed_open
test_connect_is_never_issued_without_a_close_in_a_prior_pass
test_no_pass_issues_more_than_two_at_commands                  (FSM shape only — link_fake cannot
                                                                see WiFiClient's internal writes;
                                                                the driver half is on-device)
test_every_error_exit_transitions_to_sock_close_rather_than_closing_inline
test_sock_read_calls_neither_available_nor_connected
test_an_exchange_that_produced_no_bytes_is_retried_exactly_once
test_a_retry_is_abandoned_rather_than_sent_outside_the_dedup_window   (run with a NON-ZERO boot
                                                                salt: a single-stamp
                                                                implementation must fail this)
test_a_response_that_produced_any_bytes_is_never_retried
test_a_truncated_reply_is_never_retried                        (bytes arrived)
test_a_four_hundred_is_never_retried
test_a_five_hundred_is_not_retried
test_a_five_oh_three_is_retried_once
test_a_modem_timeout_poisons_the_link_and_counts_a_desync
test_a_second_link_reset_still_produces_a_working_at_round_trip (the `beginned` bug)
test_link_drop_returns_to_joining_with_exponential_backoff
test_poll_is_a_noop_while_the_pump_is_asserted
test_command_is_surfaced_only_once_per_round_trip
test_command_is_not_executed_in_the_pass_that_received_it
test_an_ack_is_set_the_moment_a_command_is_received
test_a_failed_goto_still_acks
test_a_stop_command_is_acked
test_every_terminal_path_in_exec_pending_sets_an_ack
test_pending_ack_rides_the_next_report_after_every_discard_path
test_refused_dose_acks_with_flow_ml_zero_and_an_err_token
```

**`test/test_cart/test_cart.cpp`**
```
test_position_is_unknown_after_boot_until_homed
test_goto_refuses_when_pulses_per_gate_is_zero
test_pos_is_never_ok_before_calibration
test_goto_counts_pulses_not_milliseconds
test_home_zeroes_the_count_only_when_the_hall_asserts
test_home_from_outlet_five_actually_reaches_home              (regression: reset()'s 80 s error)
test_home_that_times_out_leaves_position_unknown
test_stall_aborts_within_the_stall_window_and_loses_position
test_an_i2c_error_on_the_home_hall_is_unknown_not_not_home
test_goto_rejects_an_outlet_outside_one_to_five
test_servo_is_stopped_on_every_exit_path
test_move_deadline_holds_across_a_millis_rollover
test_the_cart_is_parked_off_every_outlet_after_every_command_including_a_failed_goto
test_the_cart_is_parked_after_a_stop_command_and_after_an_out_of_range_outlet
```

**`test/test_sensors/test_sensors.cpp`**
```
test_select_holds_p4_high_so_the_home_hall_stays_readable
test_read_discards_the_first_conversion_and_keeps_the_second
test_an_i2c_error_is_reported_as_error_not_as_zero
test_sweep_feeds_the_watchdog_between_channels
test_sweep_reads_the_open_canary_channel_every_time
test_a_stuck_mux_is_reported_as_an_error_not_as_readings
test_three_consecutive_failures_back_off_and_mark_the_bus_unhealthy
test_leak_does_not_latch_from_coast_down_pulses_after_a_dose
```

**`test/test_cli/test_cli.cpp`**
```
test_parses_every_bench_command
test_bringup_commands_are_absent_from_the_bench_build         (compiled twice)
test_pump_without_an_argument_is_refused
test_pump_ms_is_clamped_to_the_hard_cap
test_pump_hang_requires_the_literal_third_token
test_clear_requires_both_literal_tokens
test_goto_rejects_zero_and_six
test_an_overlong_line_is_dropped_whole_not_truncated_into_a_command
test_status_reports_the_watchdog_grant_liveness_and_the_pump_active_level
test_dose_summary_line_carries_outlet_ms_pulses_ml_and_mls
test_no_float_formatting_appears_in_any_printed_line
```

~1,400 lines of test, ~150 cases.

### Mechanical invariants — `make check`, no board required

```
exactly 1  file containing  PB_PUMP_OWNER
exactly 0  hits for         pinMode(PIN_PUMP_EN
exactly 2  hits for         R_IOPORT_PinCfg.*PIN_PUMP_EN  (hal_boot_pump_off, hal_pump_write — §2.1)
exactly 0  hits for         R_IOPORT_PinWrite.*PIN_PUMP_EN|digitalWrite.*PIN_PUMP_EN
exactly 0  hits for         pinMode(PIN_FLOW|pinMode(PIN_HALL_SCREW   outside hal_arm_pulse_pins_
exactly 1  hit  for         WDT.refresh                  (inside hal_wdt_feed)
exactly 1  hit  for         hal_wdt_feed(                in safety.cpp (safety_tick)
exactly 0  hits for         hal_wdt_feed(                outside safety.cpp, hal_uno.cpp, hal_sim.cpp
                                                         (the only other callers are hal_wdt_alive's
                                                          two, which bracket the unfed probe — §2.5)
exactly 0  hits for         WDT.getTimeout               (it lies under the cfg overload)
exactly 0  hits for         \bdelay(                     outside hal_uno.cpp
exactly 0  hits for         Wire.flush
exactly 0  hits for         lcd\.print|lcd\.println      (no hook between characters — §5)
exactly 0  hits for         WiFi.ping                    (it resets modem.timeout to 10 s)
exactly 0  hits for         %[0-9.]*[fgeFGE]\b           in any format string  (newlib float printf)
exactly 0  hits for         %[0-9.]*[di]\b               in report.cpp and netfsm.cpp format strings
                                                         (every t=/ack=/chN= site is %lu with an
                                                          explicit (unsigned long) cast — see below)
exactly 0  hits for         WiFiS3|link\.h|Network\.h    in safety.cpp and lib/Manifold
exactly 0  hits for         safety\.h|dose_run|hal_pump_write   in netfsm.cpp, lib/Network/, ui.cpp
exactly 1  hit  for         dose_run(                    in cli.cpp
exactly 0  hits for         for\s*\(\s*;\s*;\s*\)|while\s*\(\s*(true|1)\s*\)   outside safety.cpp
exactly 0  hits for         String|std::map|std::string|\bnew\b|malloc   outside lib/Network, lib/Screen
exactly 0  hits for         Arduino\.h                   outside src/hal_uno.cpp, lib/Network, lib/Screen
exactly 0  hits for         hang|" prime"|"cal "         in the bench env's preprocessed src/cli.cpp
exactly 2  files containing PB_BRINGUP                   (cli.cpp, main.cpp)
identical  sha              safety.o and hal_uno.o between uno_r4_wifi and uno_r4_wifi_bringup
```

**The `%d` grep is load-bearing, not belt-and-braces.** `t = hal_boot_salt() + hal_millis()` with a
salt that is the boot counter times a large odd stride is **above 2^31 on ordinary boots**, so a
single `%d` against a `uint32_t` prints a leading `-`, `_int_in` rejects it, and **every report 400s
from the first one** — not from day 25. `-Wall -Wextra` does not diagnose this; `-Wformat-signedness`
does, and §10 puts it in `build_src_flags` (warning, not error, because the flag would also apply to
any library later pulled into `src/`). The grep is the enforcement; the flag is the early warning.

The `hang`/`prime`/`cal` grep is scoped to `src/cli.cpp` because those are **console affordances** and
`cli.cpp` is where a console affordance can exist; `long_prime` is deliberately absent from it (§6
explains why a whole-tree grep for it can never pass), and the object-hash line is the stronger check
that replaces it. The `for(;;)`/`while(true)` grep pins the program's only intentional unbounded loop
to the one function that owns D6.

### NOT tested on the host, and stated so nobody thinks green means safe

`hal_uno.cpp`'s ~230 lines (pin numbers, the `R_IOPORT_PinCfg` boot write, the two ISRs and their
debounce, the `R_ICU->IRQCR` filter write and the `IELSR` scan, `analogReadResolution(14)`,
`WDT.begin(cfg)`, the stack paint, every Servo/Wire/WiFiS3 call), `lib/Screen`, `lib/Network`, **the
`delay(1000)` inside `LiquidCrystal_I2C::init_priv()`**, **`adcConvert`'s unbounded
`ADC_STATE_SCAN_IN_PROGRESS` spin (`analog.cpp:486-489`)**, ADC settling on a 10 k source, I2C
pull-up adequacy, relay polarity, servo torque, whether the buck holds through a stall while the
ESP32 transmits, whether `setConnectionTimeout`'s unit is milliseconds **and whether
`_CLIENTCONNECT` still resolves a hostname** (§3 change 4), **whether `.noinit` survives the
bootloader across a warm reset** (§2.3 — bring-up 7c), and **whether the watchdog actually resets the
chip** — that last is bring-up 7c and cannot be simulated. The host tests prove the LOGIC; bring-up
0–7d proves the WIRING. Neither substitutes for the other.

### On-device runs

These run in a **fourth env**, `[env:uno_r4_wifi_test]` (§10). They cannot run in
`uno_r4_wifi_bringup`: that env has no `test_build_src = yes`, so `src/report.cpp` and friends never
link and every suite fails on unresolved symbols, and it inherits
`build_src_filter = +<*> -<hal_sim.cpp> -<link_fake.cpp>` from `[env:uno_r4_wifi]`, which removes the
very fake `test/support/harness.h` is a fixture over. The test env extends `bringup` with
`test_build_src = yes`, `test_framework = unity`, and a filter that keeps `hal_uno.cpp` (the point is
the real board) while adding `hal_sim.cpp` back for the suites that need a fixture.

- `pio test -e uno_r4_wifi_test -f test_report -f test_cart`, once per protocol change: catches
  `int` width in the `k=v` formatting, unaligned struct access, and `millis()` wrap arithmetic.
  **Only `test_report` and `test_cart` run on the device**; `test_dose`, `test_contra`, `test_net`,
  `test_sensors` and `test_cli` are host-only, because they drive fault injectors `hal_uno.cpp` does
  not have.
- `test_wifi_begin_returns_within_two_seconds` — a platform bump that breaks the `setTimeout(0)`
  trick becomes loud rather than a mysterious 10 s stall.
- `test_sock_open_from_a_stale_socket_completes_within_the_wdt_window` — assert the **wall clock** of
  a connect entered after a deliberately abandoned socket, which is the case the design's budget
  missed.
- `test_a_recv_pass_against_a_slow_responder_completes_within_the_wdt_window` — the other half of
  §3's budget. `link_fake.cpp` cannot see `WiFiClient`'s internal `modem.write` calls, so a RECV pass
  that quietly grew an `available()` or a `connected()` would pass every host test and blow the
  budget on the bench.
- `test_wdt_alive_returns_true_on_real_silicon` — the probe of §2.5 against the actual counter, once,
  because the sim's counter is a fake by construction.
- A device test that pulls the AP mid-connect and asserts (a) no reset, and (b) the next three
  exchanges still parse. **This is the one that catches a `link_reset()` that does not clear
  `beginned`.**

---

## 10. `platformio.ini`, in full

```ini
; NOTE: the global [env] section carries NO `platform` and NO `framework`. PlatformIO
; inherits [env] into EVERY environment (project/config.py walk_options), LoadProjectOptions
; copies `framework` into PIOFRAMEWORK (builder/tools/pioproject.py), and BuildFrameworks
; then does `if "BOARD" not in env: stderr.write("Please specify `board` ... to use with
; 'arduino' framework"); env.Exit(1)` (builder/tools/piobuild.py). [env:native] has no board,
; so a global `framework = arduino` makes `pio test -e native` ABORT BEFORE COMPILING
; ANYTHING -- i.e. it would kill this spec's own first gate. Verified empirically on
; PlatformIO Core 6.1.19: with framework in [env], `pio test -e native` errors; with it in
; [env:uno_r4_wifi], the same test passes.
;
; Nor does [env] carry -std=gnu++17: the renesas-ra builder already supplies -std=gnu++17 for
; C++ and -std=gnu11 for C (builder/frameworks/arduino.py:100,123), and a -std=gnu++ in
; build_flags lands in CCFLAGS and warns once per core .c file on every build (§1).

[env]
build_flags = -Wall -Wextra

[env:uno_r4_wifi]                                  ; THE UNATTENDED BINARY
platform = renesas-ra@1.6.0
framework = arduino
board = uno_r4_wifi
build_flags =
    ${env.build_flags}
    -DPB_RELAY_ACTIVE_HIGH        ; SET BY BRING-UP 4a. No default exists; pins.h #errors.
    -DPB_REPORT_POS_UNKNOWN=1     ; SHIPS DEFINED. Flipping it turns on watering — §4.6.
;   -DPB_DOSE_BY_TIME=1           ; 7b's stated fallback (§6). Uncomment ONLY after
;                                 ; PB_ML_PER_S_MEASURED is committed non-zero; config.h #errors
;                                 ; if it is not.
build_src_flags = -Wformat-signedness   ; %d against a uint32_t t= is a first-report 400 (§9)
build_src_filter = +<*> -<hal_sim.cpp> -<link_fake.cpp>
monitor_speed = 115200
lib_deps =
    arduino-libraries/Servo@^1.2.2
    arduino-libraries/Arduino_Sensorkit@^1.4.0
    marcoschwartz/LiquidCrystal_I2C@^1.1.4

[env:uno_r4_wifi_bringup]                          ; BRING-UP 0-7d ONLY. Never left running.
extends = env:uno_r4_wifi
build_flags = ${env:uno_r4_wifi.build_flags} -DPB_BRINGUP=1

[env:uno_r4_wifi_test]                             ; §9's on-device suites. Never left running.
extends = env:uno_r4_wifi_bringup
test_framework = unity
test_build_src = yes                               ; or src/report.cpp never links into a test
build_src_filter = +<*> -<link_fake.cpp>           ; keep hal_uno.cpp (the real board) AND
                                                   ; hal_sim.cpp (harness.h is a fixture over it)

[env:uno_r4_wifi_sim]
extends = env:uno_r4_wifi_bringup
build_flags = ${env:uno_r4_wifi_bringup.build_flags} -DPB_SIM=1 -DPB_CONTROLLER='"bench1sim"'
build_src_filter = +<*> -<hal_uno.cpp>
lib_ignore = Network

[env:native]
platform = native
test_framework = unity
build_flags = ${env.build_flags} -std=gnu++17 -DPB_NATIVE=1 -DPB_SIM=1 -DPB_BRINGUP=1 -DPB_CONTROLLER='"test1"' -I include
test_build_src = yes
build_src_filter = +<*> -<main.cpp> -<hal_uno.cpp>
lib_ignore = Network, Screen, Servo, Arduino_Sensorkit, LiquidCrystal_I2C

; --- 2026-09-03 correction. Two flags and four one-flag variants that the block above
; --- did not print. Six suites in this document are specified to compile TWICE, once
; --- with a flag and once without; without a named environment for each, none of those
; --- cases has anywhere to be compiled in, and `pio test -e native` cannot even build,
; --- because pins.h #errors when no relay polarity is defined. Every variant is one flag
; --- on top of env:native and nothing else, so there is one place to change the rest.
[env:native_bench]                                 ; the bench-vs-bringup cases
extends = env:native
build_flags = ${env:native.build_flags} -UPB_BRINGUP

[env:native_cal]                                   ; the calibrated arm of the cart #if
extends = env:native
build_flags = ${env:native.build_flags} -DPB_PULSES_PER_GATE=1450

[env:native_measured]                              ; the two cap-clamp cases
extends = env:native
build_flags = ${env:native.build_flags} -DPB_ML_PER_S_MEASURED=30

[env:native_nosimcli]                              ; `sim ...` is not a command here
extends = env:native
build_flags = ${env:native.build_flags} -UPB_SIM_CLI
```

**`[env:native]`'s flags, corrected 2026-09-03.** The printed line above is missing two that
it cannot build without. `-DPB_RELAY_ACTIVE_HIGH` is required because §2.2 gives `pins.h` no
default polarity and an unconditional `#error`, and `safety.cpp` and `sensors.cpp` both include
it — the flag is inert on the host, since `PB_PUMP_OWNER` is never defined off the board, so
`PIN_PUMP_EN` and the PFS macros do not exist there. `-DPB_SIM_CLI=1` is required because the
console suites drive the fake's serial console. `native_nosimcli` undefines `PB_SIM_CLI` and
**not** `PB_SIM`: `PB_SIM` also gates every `hal_*` body in `hal_sim.cpp`, so `-UPB_SIM` would
leave the host suite linking against no HAL at all.

Notes an implementer needs:

- **`platform` and `framework` belong in `[env:uno_r4_wifi]`, never in `[env]`** — see the comment
  block above; `uno_r4_wifi_bringup`, `uno_r4_wifi_test` and `uno_r4_wifi_sim` inherit them through
  `extends`. If you would rather keep them global, the only other working form is an explicit empty
  `framework =` inside `[env:native]`.
- `test_build_src = yes` is required in **both** `[env:native]` and `[env:uno_r4_wifi_test]`, or none
  of `src/report.cpp`, `src/safety.cpp`, `src/netfsm.cpp` links into a test and every suite fails on
  unresolved symbols.
- `hal_uno.cpp` and `hal_sim.cpp` both compile in `uno_r4_wifi_test`, so every `hal_*` symbol must be
  namespaced by the filter, not by the linker: `hal_sim.cpp` is `#if PB_SIM`-gated internally and
  contributes only its fault injectors in that env. If that proves fiddly, the accepted fallback is
  two device test envs, one per HAL — say so in `status`, and keep §9's list of which suite runs
  where accurate.
- `src/main.cpp` must be filtered out on native (`setup`/`loop`, no `main()`), and `hal_uno.cpp`
  because it includes `<Arduino.h>`.
- `lib_ignore` on native must name `Network` and `Screen` **and** the three external libraries, or
  the LDF will try to compile `WiFiS3.h` / `LiquidCrystal_I2C.h` for the host.
- `test_cli` compiles its bench-vs-bringup case **twice**, once with `-UPB_BRINGUP`.
- Shared test fixtures go in `test/support/harness.h`, which is a **header** — `test/support/` must
  not become a suite directory with no runner.
- The project `include/` directory is on the CPPPATH for `lib/` builds too, which is why
  `lib/Manifold/src/cart.cpp` can `#include "hal.h"`. **Prove `pio test -e native --without-testing`
  links before a single module is written**, exactly as the preamble says.

---

## 11. File tree and staging

```
plant_butler/
├── platformio.ini                              85   five envs, in full (§10)
├── Makefile                                    55   + test, sim, bringup, check, calib, compiledb
├── tools/check.sh                              70   the greps of §9
├── AGENTS.md                                (edit)  envs, seams, the greps, 115200 baud (§16.3)
├── README.md                                (edit)  String -> char[] TODO closed
├── docs/superpowers/specs/2026-09-03-bench-sketch-design.md   this file
├── include/
│   ├── pins.h                                  90   the wiring contract; PB_PUMP_OWNER gate; #error on polarity
│   ├── config.h                               110   every tunable with its citation (§7)
│   ├── hal.h                                   85   seam 1: 32 free functions, Arduino-free
│   ├── link.h                                  40   seam 2: 10 primitives
│   ├── noinit.h                                35   the warm-reset struct + checksum (§2.3)
│   ├── safety.h                                55
│   ├── pulses.h                                35
│   ├── sensors.h                               45
│   ├── report.h                                55
│   ├── netfsm.h                                45
│   ├── cli.h                                   20
│   ├── ui.h                                    25
│   ├── sim.h                                   50   (sim + native only)
│   ├── secrets.h            (gitignored)       16   + BUTLER_TOKEN, PB_CONTROLLER
│   └── secrets.h.example    (checked in)       16
├── src/
│   ├── main.cpp                               150   setup order, the 5-line loop, exec_pending()
│   ├── hal_uno.cpp                            230   DEVICE ONLY. The only file with <Arduino.h>, a pin or an ISR
│   ├── hal_sim.cpp                            270   sim + native: the fake rig and its fault injectors
│   ├── link_fake.cpp                          100   sim + native: canned responses, scripted failures
│   ├── safety.cpp                             240   safety_tick, safety_wait, float, dry, CONTRA, dose_run  <- READ FIRST
│   ├── pulses.cpp                              90   D2/D3 ISRs, gap reject, rate, snapshots, ml, leak watch
│   ├── sensors.cpp                            175   PCF8575 + mux + canary + home hall + I2C health + recovery
│   ├── report.cpp                             190   report_build + response_parse + the replay guard
│   ├── netfsm.cpp                             260   the report FSM + HTTP framing, above seam 2
│   ├── cli.cpp                                320   bench commands; bring-up commands under #if PB_BRINGUP
│   └── ui.cpp                                 140   pure ui_render/ui_render_lcd + the coarsened painter
├── lib/
│   ├── Manifold/include/cart.h                 60   reworked: pulses, no Screen*, no test()
│   ├── Manifold/src/cart.cpp                  205   home/goto/jog/bus_check; one servo start, one stop
│   ├── Network/include/Network.h               10   reworked: now just `#include "link.h"`
│   ├── Network/src/link_wifi.cpp              185   reworked: socket closed on EVERY path; link_reset()
│   ├── Screen/include/Screen.h                 45   kept; + probe(), present()
│   └── Screen/src/Screen.cpp                  100   kept; a missing panel is a no-op, never a hang
└── test/
    ├── support/harness.h                       70   Unity fixture over hal_sim
    ├── test_dose/test_dose.cpp                330
    ├── test_contra/test_contra.cpp            110
    ├── test_report/test_report.cpp            300
    ├── test_net/test_netfsm.cpp               220
    ├── test_cart/test_cart.cpp                200
    ├── test_sensors/test_sensors.cpp          110
    └── test_cli/test_cli.cpp                  150
```

**Bench binary ≈ 2,300 lines** (the tree less `hal_sim.cpp`, `link_fake.cpp`, `sim.h`, and the
`#if PB_BRINGUP` blocks). Tests ≈ 1,400. Against today's 416.

**Deleted, not replaced:** `Manifold::test()` and its minutes of blocking delays;
`Manifold::reset()`'s one-gate-width guess (~80 s wrong from gate 5); the `Screen*` the manifold
logged through; `NetworkClient`'s `std::map`/`String` API; the `while(true)` on `WL_NO_MODULE`; the
unbounded `while (WiFi.status() != WL_CONNECTED) delay(300)`; `getServerReply()`'s
`while (client.connected() && !client.available()) delay(1)` and its unbounded `String server_reply`;
`main.cpp`'s `std::map<std::string,float>` and the `sprintf` into a `char[16]` that needs 17. The
README's `String -> char[]` TODO closes by construction: `hal.h` exposes only `const char *`.

### Staging, so the appetite is respected

The pitch buys ~1.5 person-weeks. The bring-up order stages it:

| drop | what | unblocks |
| --- | --- | --- |
| **1** | `pins.h`, `config.h`, `hal.h`, `noinit.h`, `hal_uno.cpp` (including §2.1's boot write and §2.14's pulse-pin config), `pulses`, `sensors` (with the canary), `cli` bench subset, `Screen`, `ui` | bring-up **0–3**; two of the pitch's four unknowns |
| **2** | `safety.cpp` **whole** — including the contradiction latch, the two time-armed flow rules, the cooldown, the plausibility ceiling and the measured cap clamp — plus `cart`, and `cli`'s `#if PB_BRINGUP` block | bring-up **4a–7d**; the pitch's ml/s and flow verdict |
| **3** | `report`, `netfsm`, `link_wifi`, the ack cycle, the diagnostic channels | the 48-hour unattended run |

**Drop 2 is the safety spine and must not be diluted.** If the appetite runs short, the things that
give are drop 3's diagnostic channels and drop 1's UI coarsening — never a guard, never the latch,
never a `make check` grep.

---

## 12. RAM budget — measured, not estimated

Measured on today's build. `arm-none-eabi-size` prints `bss 14980`, which is **wrong to read as
static usage**: it folds in `.heap` (0x2000) and `.stack_dummy` (0x400), both NOBITS linker
reservations. From `firmware.map`:

| section | today | note |
| --- | --- | --- |
| `.data` | 204 B (0xcc) | |
| `.bss` | **5480 B (0x1568)** | the real static figure |
| `.heap` | 8192 B | `BSP_CFG_HEAP_BYTES`; sbrk actually runs `__HeapBase` 0x20001650 … `__HeapLimit` 0x20007b00 ≈ **25,776 B** |
| `.stack_dummy` | **1024 B** | `BSP_CFG_STACK_MAIN_BYTES`. Fixed. Shared with both ISRs. |

Largest `.bss` contributors today (`-DNO_USB` is defined for this board, so `Serial` is `_UART1_`,
the hardware UART the ESP32 bridges to the host's USB CDC, and `Serial2` — the modem's port — is
`_UART3_`): `_UART1_` (`Serial`, the console) 1196, `Wire` 1096, `Wire1` **1096, pure waste**
(`WIRE_HOWMANY 2`, both in one TU, unavoidable without patching the core), `ra_servos` 288, `SPI` 204,
`Oled` (u8x8, tile-based, no framebuffer) 80.

**What this design adds:** `_UART3_` (`Serial2` → ESP32, pulled in by `ModemClass(&Serial2)`,
`Modem.cpp:417`) ~1196; WiFiS3 statics ~300; tx buffer 768; rx buffer 256; report body 384; console
line + argv 112; OLED shadow 8×17 + LCD 2×17 = 170; channel store + validity + timestamps 40;
`.noinit` 44; cart + pulses + safety + sensors state ~200. **~1.9 KB of our own.** Totals ~8.9 KB
`.bss` + 204 B `.data`, ~1.05 KB pinned heap, on 32 KB.

**The binding constraint is the 1 KB main stack, and NOTHING bounds the heap at all.** `fsp.ld:260-261`
puts `__HeapLimit` and `__StackLimit` at the *same address*, with no MPU guard region and no MSPLIM on
a Cortex-M4 — but the sharper point is that **`_sbrk` never reads either symbol**. The linked `_sbrk`
is libnosys's unchecked 28-byte version (`firmware.map:257-258, 3610-3611`); disassembled it is
`ldr r2; ldr r1; ldr r3,[r2]; cmp #0; it eq; moveq r3,r1; add r0,r3; str r0,[r2]; mov r0,r3; bx lr` —
no limit compare, no `-1` return — and `__HeapLimit` is referenced by nothing in the image. So the
25,776 B figure above is a **distance, not a bound**: `malloc` will never fail from exhaustion, it
will hand out addresses inside and above the 1 KB stack. And a stack overflow does not fault either —
it silently corrupts the top of the heap, which is exactly where `WiFiClient::_read()`'s per-read
`std::string` for up to 1023 bytes lives (`WiFiClient.cpp:117-142`), i.e. the bytes that become a
water command.

That makes `ch200`/`ch201`/`ch202` more valuable, not less — they are the only visibility that
exists — and it makes one more thing mandatory:

0. **`hal_begin()` and every report check the break against the stack**, because nothing else will.
   `hal_heap_break()` must stay below `hal_stack_limit() - PB_STACK_MARGIN` (2048 B). If it does not,
   latch `err=heap`, **disable the network** (the network stack is the largest allocator in the
   program, so continuing is how the corruption reaches a water command) and say so in `status`. The
   headroom prints in `status` alongside `arena` and `ordblks`.

Four more consequences, all acted on:

1. **`%f`/`%g` are banned and grepped for.** newlib's float formatting is the deepest stack consumer
   in the program, and the design planned to link it for one cosmetic `mls=29.1`. Integer tenths and
   `%lu.%lu` instead.
2. **The stack is painted at boot** from `__StackLimit` upward with 0xA5, and `hal_stack_hwm()`
   reports `__StackTop - <lowest unpainted word>`. It rides out as `ch202`.
3. **The instrument is `arena` and `ordblks`, not `fordblks`.** `mallinfo().fordblks` is free bytes
   *inside the current break*: it typically rises then plateaus, it does not fall on fragmentation,
   and it is blind to stack damage. `arena` (`ch200`) shows break growth; `ordblks` (`ch201`) shows
   the free-chunk count, which is the actual fragmentation signal. The design's "a monotonic fall
   across the 48-hour run is the fragmentation signal" described a number that does not behave that
   way.
4. **Every buffer above is file-static; none is a stack local.** The deepest call chain is
   `loop → net_poll → sock_write → modem.write`, four frames.

---

## 13. Bring-up 0 → 7e

Run bring-up on the **`bringup`** binary. `status` must say `build=bringup` for steps 0–7d and
`build=bench` for the unattended run.

**The order below is not runnable without `dry off`, and that is not a detail.** Several steps
reset the board mid-dose on purpose, and §2.3 latches dry on exactly that — so a step that ends in
a reset leaves the next one refusing before it starts. Left implicit, 4c has no click to interrupt
and 7c can never reach the hang that proves the watchdog. `dry off` between the halves is how you
carry on, and `status` says `dry=1` whenever you have forgotten. `cad/wiring/README.md` carries the
same correction.

| step | what the wiring package asks | what this design provides | notes |
| --- | --- | --- | --- |
| **0** | continuity before power | — | **owed doc change:** add "confirm `status` says `build=bringup`, `dry=0`, `contra=0` and the expected pump level before 12 V goes onto COM; confirm `build=bench` before the unattended run" |
| **1** | `i2c` sees 0x20 (0x38 if DHT20) | `i2c` scans in one bounded pass | USB power. **Also expect 0x27 (LCD) and 0x3C (OLED)** — an owed `nets.py` change (§16.1). |
| **2** | `mux all` reads 16 channels; the wired one moves when wetted | `mux all` — select, ≥1 ms, read twice, keep the second; the canary channel is read too | the README's recipe verbatim |
| **3** | screw (D3) and float (D5) follow a magnet; home changes through P4 | `hall` streams all three at 5 Hz; an I2C error prints `home unknown`; `status` prints `irq_armed=yes yes` | proves both pull-ups and the expander as an input |
| **4a** | relay **dry**, no 12 V on COM: `pump 2000` clicks; COM-NO closed while it holds | `pump 2000` — `need_pos = false`, `by_time = true`. The float must read OK. | `status` first, to read the compiled polarity. **Additionally: COM-NO must stay OPEN across a power cycle AND across a `hang`-forced watchdog reset** — that is what proves §2.1's boot write, and the old recipe would have failed it on an active-LOW module. That watchdog reset latches dry: **`dry off` before 4b.** |
| **4b** | still dry: RESET mid-click → contacts open | nothing to do — D6 reverts to an input, R1 holds OFF | proves R1. This RESET latches dry as well: **`dry off` before 4c**, or 4c has no click to interrupt. |
| **4c** | still dry: pull the D6 jumper mid-click → open. **Only now** wire 12 V onto COM. | nothing to do | proves no hidden path to the coil |
| **5a** | float below the line → `pump 2000` refused | `DOSE_REFUSED_FLOAT`, printed by name; proven on the host first | the debounce means 3 consecutive OK samples are needed to grant |
| **5b** | unplug the float hall mid-dose → stops | `DOSE_ABORT_FLOAT` within one loop iteration. **Confirm `contra=0` afterwards** — the float dropping is agreement, not contradiction. | R2 lifts the open line to "not OK" |
| **6** | barrel jack from the 12 V brick. `servo`, `home`, `goto`, then repeat with WiFi connected and the cart stalled | `dry on` first — `home`/`goto` run, the pump is refused unconditionally. **This step produces `PB_PULSES_PER_GATE` and `PB_PULSES_HOME_TO_1`.** Commit them. | until 6 lands, `cart_goto` is compiled to `return false` and `pos` is never `ok`. **`dry off` before 7a** — 6 is the one step that latches dry deliberately rather than by accident. |
| **7a** | bucket only: prime with `pump`, then `pump 2000` runs | `pump 20000 prime` — the prime window extends to 15 s and the dose is capped at 20 s. The no-flow abort is **still live**. A console `prime` dose is **exempt from the contradiction latch** (§2.7): "float OK, no flow" is what priming a dry line *is*. | if the line will not prime inside 20 s, run it again; do not reach for a flag that removes the abort, because there isn't one. **If it does latch anyway** (a plain `pump`, no `prime`, on a dry line will), `status` says `contra=1`: type `clear contra` and prime again. |
| **7b** | pump a weighed 500 ml counting pulses; record the lowest flow that still pulses; set the no-flow timeout | `calib` runs `pump 10000 prime` and prints `dose outlet=… ms=… pulses=… ml=… mls=…`. `cal <n>` retunes without a reflash (1000..20000). **Commit `PB_PULSES_PER_L_DEFAULT`, `PB_PRIME_MS_DEFAULT`, `PB_STALL_MS_DEFAULT` and `PB_ML_PER_S_MEASURED`.** | this line **is** the pitch's ml/s deliverable and its flow-meter verdict. If the meter does not pulse at the rig's real flow, `-DPB_DOSE_BY_TIME=1` is the stated fallback. `PB_ML_PER_S_MEASURED` also arms the cap clamp — until it is committed, `status` says `cap=UNCLAMPED`. |
| **7c** | RESET mid-dose → stops. Then force a hang → the watchdog resets the board and drops the pump inside its window. | `pump 3000 hang` asserts D6, runs `PB_HANG_MS` (3 s), then spins without feeding. `status` has already printed `granted=5592ms alive=yes delta=~117`, so the pass criterion is a number. **After the reset, confirm `status` says `dry=1` and `last=resetmid`** — that is §2.3's warm-reset carry working. | **the honest spill: 5592 ms × the measured flow rate.** At 30 ml/s that is ~170 ml. Fill it in from 7b. **`dry off` between the two halves**: the RESET latches dry, and a latched board refuses the `hang` dose before it can starve the dog. |
| **7c′** *(new)* | — | **`.noinit` survival, measured rather than asserted.** Before the hang, `noinit pattern` writes a known 32-bit pattern into a spare word of the struct; after the watchdog reset, `status` prints the struct raw. **Pass = the pattern and the checksum both survive.** | §2.3 explains why this cannot be taken on trust: `__noinit_start` is at 0x200000cc, the bottom of SRAM, and the board runs a bootloader before the sketch whose SRAM use is not documented in the installed package. **If it fails, every `.noinit` guarantee in this spec is void** — the dry latch, the contradiction latch, `err=resetmid` and the replay high-water mark all become cold-boot-only — and §16.5.3 (the backend's durable half) stops being a follow-up and becomes a blocker for going live. |
| **7d** | measure pump start and dead-head current; fix F1 | `pump 5000 prime` with a clamp meter | pure hardware |
| **7e** *(new)* | flash the **`bench`** binary. `status` says `build=bench`; `pump`, `cal` and `hang` are not commands. Then the 48-hour run. | — | the unattended binary is a different binary and this is where that is checked. **Expect exactly one HIGH `pos:` push — "bench1 lost track of its manifold position: watering is on hold" — about two minutes after first boot.** That is `PB_REPORT_POS_UNKNOWN` doing its job (§4.6), not a fault. It raises once and **stands for the whole run**, and while it stands the `pos:` rule is deaf, so a genuine position loss during the 48 hours will not page. Do not clear it by hand; it clears when the flag is flipped and a `pos=ok` report arrives. |

---

## 14. What I refused to build, and what it costs

- **An RTOS or a cooperative scheduler.** ~350 lines to buy a live console during a dose. The console
  is already responsive except during a dose or a move, `stop` and `dry on` are matched byte-wise
  inside the dose loop, and the safety comes from one function that checks twelve things before
  writing one pin — not from an architecture. Cost: the screens freeze for up to 60 s during a dose,
  stated on the LCD.
- **An ISR-fed watchdog.** A dog fed from an ISR is a dog that cannot bite. `WiFi.setTimeout(0)`,
  `modem.timeout(1200)` and a guaranteed-`_sock == -1` connect bring the worst blocking call to
  2.4 s under a 5592 ms window. If bring-up ever measures a step over 4 s, the fallback is written
  down (an `FspTimer` refresher whose constructor asserts the pump is off) and deliberately unbuilt.
- **A virtual HAL.** `build_src_filter` gives the same seam with zero indirection.
- **A runtime `sim` flag.** One `if (sim_)` in the code path that asserts D6 is exactly the branch
  that must not exist. Cost: on the bench binary you cannot fake the float — you move the magnet and
  unplug the hall, which is bring-up 5a and 5b anyway.
- **Persisting cart position or calibration.** A reboot must mean position unknown. Calibration lives
  in `config.h` so a commit is the audit trail (DECISIONS #6 keeps calibration in the backend). The
  `.noinit` block persists only things that fail **more** refusing: `dry`, `contra`,
  `dose_in_flight`, and the replay high-water mark.
- **A queue of unsent reports.** Stale moisture is worse than no moisture, `butler` stamps arrival
  time, and a growing buffer on this heap is the fragmentation risk the rest of this avoids.
- **An ntfy client.** DECISIONS #2 and `butler.py`'s alert ticker own it.
- **A hardware interlock in firmware's clothing.** Nothing here equals a 74HC00. The watchdog, the
  two caps, the two flow rules, the plausibility ceiling and the contradiction latch **bound** the
  damage; they do not close THE GAP. pitch-28a903 closes it.

---

## 15. Findings declined, and why

Seven findings — or, in three cases, one sub-clause of a finding — are deliberately not applied.
Everything else raised against this document, by the two adversaries and by the three verifiers that
read the spec afterwards, is folded into §§0–14 above.

### 15.1 A hardware time base for the dose cap — declined, on evidence

**The claim:** `millis()` is `agt_time_ms`, incremented in an AGT0 underflow callback, so an edge
storm on D2 delays the callback, `millis()` runs slow, every bound in `dose_run()` stretches
proportionally, and a 60 s cap becomes 600 s of real pumping while `safety_tick()` cheerfully feeds
the watchdog. Proposed fix: latch `R_AGT0->AGT` or a dedicated `FspTimer` at pump-on and bound the
dose against that instead.

**Why declined:** the premise does not hold on this core. The AGT underflow interrupt is registered
at IPL **8** (`cores/arduino/time.cpp`, `agt_timer.setup_overflow_irq(8)`); external pin interrupts
are registered at IPL **12** (`cores/arduino/IRQManager.cpp:13`, `EXTERNAL_PIN_PRIORITY`). On
Cortex-M a *lower* numeric priority preempts, so the AGT tick **preempts** the pin ISR rather than
queueing behind it, and neither ISR masks interrupts globally. `agt_time_ms` cannot lose a tick to a
D2 edge storm. Adding a second time base would be ~50 lines in the one file with no host coverage,
buying protection against a mechanism that is not there.

**What was folded instead:** everything else in that finding — `INPUT_PULLUP` on D2 before
`attachInterrupt` (§2.14), the ICU digital filter, a physical 10 k pull-up as an owed `nets.py`
change (§16.1), the `PB_FLOW_MAX_HZ` in-dose abort and the `PB_FLOW_IDLE_MAX_HZ` pre-dose refusal
(§2.14). Those close the finding's actually-dangerous half — a storming D2 reaching a millilitre
target in milliseconds and acking `flow_ml=500` for water that never moved — **but only in the
corrected form of §2.8**, and the first draft of this section overstated the case. Three things had
to be true and only one was:

1. the rate rules must be tested **above** the target rule in the dose loop, or `got >= target`
   breaks out with `DOSE_OK` before either of them runs;
2. the rate estimator's window must be **stated and bounded** — `PB_FLOW_RATE_WINDOW_MS = 100`
   (§7) — because at the ISR's 2 kHz ceiling a 250 ml target is reached in ~625 ms and any slower
   estimate loses;
3. `PB_FLOW_IDLE_MAX_HZ` catches only a storm that is **already running**, while the finding's own
   scenario — a floating D2 beside the 12 V pump leg — is a storm that starts **with** the pump. So
   §9's storm test begins its storm at pump-on, and a delivered-vs-elapsed plausibility test guards
   the `DOSE_OK` path once `PB_ML_PER_S_MEASURED` is committed.

### 15.2 `t=` as a `uint64_t` — declined, replaced

**The claim:** `t = hal_millis()` is a `uint32_t`; if it is ever formatted `%d` (int is 32-bit here),
every value past 2^31 prints negative, `_int_in` rejects a leading `-`, and from day 25 onward every
report 400s permanently. Proposed fix: a `uint64_t` boot-monotonic counter accumulating `millis()`
wraps, printed `%llu`.

**Why declined:** `_int_in(value, "t", 0, 2**63)` (`butler.py:226`) accepts anything a `uint32_t`
printed with `%lu` can produce, so the 400 is a formatting bug, not a width bug. `%llu` on
newlib-nano additionally requires `_WANT_IO_LONG_LONG`, which this toolchain does not enable by
default, so the "fix" risks printing a literal `llu` into the wire protocol. And `readings.t` is
never ordered by (`butler.py:716,:1079`) — it is used only as half of the `(controller, t)` dedup key
— so **monotonicity is not required**.

**But the formatting bug is bigger than the finding said, and the guard against it did not exist
when this paragraph was first written.** The salt puts `t` **above 2^31 on ordinary boots**, so a
single `%d` is not a day-25 hazard — it 400s **every report from the first one**, permanently, with
the console looking healthy. `-Wall -Wextra` does not diagnose `%d` against `unsigned int`. So §9 now
carries the grep this decline claims (zero `%d`/`%i` in `report.cpp`/`netfsm.cpp` format strings;
every `t=`/`ack=`/`chN=` site is `%lu` with an explicit `(unsigned long)` cast) and §10 adds
`-Wformat-signedness` to `build_src_flags`. **The grep is load-bearing.**

**What was folded instead:** the two real bugs the finding uncovered. `t = hal_boot_salt() +
hal_millis()`, a `uint32_t` printed `%lu`, where `hal_boot_salt()` derives from the `.noinit` boot
counter times a large odd stride, kept in `g_t_wire`; and a **second**, unsalted `g_t_ms` that the
retry deadline is measured against (§4.4 — one variable would have made every retry abandon itself).
The salt closes the genuinely nasty case: in a **watchdog** reset loop the board boots, reports at
`t ≈ 15000`, resets, boots and reports at `t ≈ 15000` again — identical `(controller, t)` inside the
300 s window, so `butler` silently discards the second report as a retry and never runs
`water_rules` on it.

**Which reset the salt covers, precisely.** It covers a **watchdog reset and the RESET button** —
the resets that leave SRAM intact, so the `.noinit` boot counter advances. It does **not** cover a
**brown-out or a power-cycle loop**: those clear SRAM (§2.3), so the counter restarts, the salt
repeats, and two boots' reports can still collide on `(controller, t)` inside the 300 s window and be
silently swallowed as retries. That is exactly the failure the wiring README's 3–5× inrush warning
predicts. The residual risk is a **silently discarded report**, not water on the floor, and the bench
notes should say to look for gaps in `readings` after any power event. The 49.7-day wrap can only
alias a value used 49.7 days earlier, far outside the dedup window. Tests at `0x7FFFFFFF`,
`0x80000000`, `0xFFFFFFFF`, across a wrap, and across two boots fifteen seconds apart.

### 15.3 A 60-second minimum gap between doses — declined, shortened to 10 s

**The claim:** three impatient `pump 60000` lines typed during a dose that appears frozen execute as
three consecutive 60 s doses the instant the first ends — 180 seconds of pumping, every individual
dose inside its cap. Proposed fix: `PB_DOSE_MIN_GAP_MS` of 60 s for every caller.

**Why the 60 s figure is declined:** a 60 s global gap obstructs legitimate behaviour on both sides.
Bring-up 7b is iterative by nature — `calib`, weigh, `cal`, `calib` again — and a minute between
attempts turns a twenty-minute calibration into an hour. On the backend side, two different pots
watered on consecutive reports is a normal outcome of `water_rules`, and a firmware gap longer than
the report interval would refuse the second one for no safety reason.

**What was folded:** `PB_DOSE_MIN_GAP_MS = 10000`, enforced above the assert for **every** caller with
`DOSE_REFUSED_COOLDOWN`, acked honestly with `flow_ml=0 err=cooldown` — plus, and this is the part
that actually kills the scenario, `hal_serial_drain()` at the end of every dose, which discards the
buffered impatience rather than executing it, and prints `discarded N buffered bytes`. Between-dose
policy belongs to `butler.py`'s per-pot cooldown (DECISIONS #5: the backend decides, the firmware
protects); the firmware's job here is to make sure a frozen console cannot become a queue.

### 15.4 Rewriting `link_wifi.cpp` to drive `modem.write()` directly — declined

**The claim:** stop calling `WiFiClient` altogether; issue exactly one AT command per `net_poll()`
pass from `link_wifi.cpp`, which already owns seam 2. That would bound every pass at one round trip
and remove `WiFiClient::_read()`'s per-read 1 KB `std::string`.

**Why declined:** it is the right shape and the wrong moment. Reimplementing socket lifecycle,
`_CLIENTSEND`'s `write_nowait` + `passthrough` framing, and the sized-read path is several hundred
lines of code that **cannot be host-tested** — it lands squarely in the part of the tree §9 already
marks as unproven, and it would be the largest single unproven addition in the project. That trades
one risk for a bigger one inside a 1.5-week appetite.

**What was folded instead**, which gets most of the benefit for a tenth of the code (§3):
`sock_close()` on **every** exit including a failed open, so `_sock == -1` and `getSocket()`
short-circuits — turning the 5-round-trip stale-socket recovery the finding correctly identified into
a 2-round-trip connect; a dedicated `SOCK_CLOSE` state that **every error exit routes through**, so a
close is never in the same pass as another AT command (without that the failed-connect pass is 3 ATs
= 3600 ms and 3600 + `PB_NET_SLACK_MS` overruns the grant); `sock_read()` reduced to a bare
`client.read()` so a RECV pass is **one** AT and never calls `connected()`, which costs two;
`setConnectionTimeout()` so the ESP32 bounds its own connect inside our window, with the two
bring-up questions §3 now asks about it; `modem.begin()` paid once in `setup()` so `link_join()` is
2 ATs and not 3; treating any modem timeout as link-poisoned with a `link_reset()` **that clears
`beginned`** and a `ch206` counter; a WDT assertion re-derived from the real worst step
(2 × `PB_NET_STEP_MS`) rather than from a guessed 3, with a per-pass AT-count table so the number is
checkable; and skipping `ui_poll()`/`sensors_sweep()` in any pass where a modem command ran. If the
48-hour run still shows resets during a router reboot, the direct-`modem` driver is the next thing to
build, and it gets its own pitch with its own device tests.

### 15.5 Enforcing the dead-float rule in firmware — declined by decision

`cad/wiring/README.md` states "a float input that never changes state across a refill is presumed
dead: refuse, do not assume OK", and one finding asks for that to be enforced on the board rather
than deferred to a key nobody reads. **The human has settled this**: the firmware reports
`ch204 = <seconds since D5 last changed>` and **never refuses on staleness**; the backend raises the
alert. The reason is that the rule is written in terms of *a refill*, which is a human act the board
cannot observe — a board that refused on elapsed time alone would refuse a correctly-full tank that
nobody had topped up in a fortnight. The half of the finding that *was* folded is the important one:
`ch204` is now a stored channel rather than an ignored key, so the number actually reaches the
backend, where the refill log (§16.5) makes the README's rule implementable for the first time.

### 15.6 Renaming `_UART1_`/`_UART3_` in §12 — declined, the finding is wrong

**The claim:** §12 names the wrong UART objects. `Arduino.h:105-111` gives `Serial = SerialUSB`,
`Serial1 = _UART1_`, `Serial2 = _UART2_`, so `_UART1_` is the D0/D1 hardware UART, not "USB CDC", and
`ModemClass(&Serial2)` pulls in `_UART2_`, not `_UART3_`.

**Why declined:** that is the `#ifndef NO_USB` branch, and **`NO_USB` is defined for this board** —
`platforms/renesas-ra/boards/uno_r4_wifi.json:10` (`"-DNO_USB"`) and
`framework-arduinorenesas-uno/boards.txt:122`. The `#else` branch at `Arduino.h:112-118` therefore
applies: `Serial = _UART1_`, `Serial1 = _UART2_`, **`Serial2 = _UART3_`**. The design's object names
were right. The byte counts were right too (`.bss._UART1_` is `0x4ac` = 1196 at 0x200001a0 in
`firmware.map:3985`).

**What was folded anyway:** the *label*. "`_UART1_` (USB CDC)" is loose — there is no USB CDC
peripheral in play; `_UART1_` is `Serial`, a hardware UART that the on-board ESP32-S3 bridges to the
host's USB. §12 now says that, and §0's facts table carries the `NO_USB` row so the next reader does
not have to re-derive it.

### 15.7 Making `hal_irq_armed()` a per-channel ICU enable-bit read — declined, replaced

**The claim:** `hal_irq_armed(pin)` as specified reads "the ICU channel's enable bit", and there is no
such bit reachable from a pin number without `Interrupts.cpp`'s file-static `IrqPool`.

**Why the shape of the claim is accepted and its conclusion is not:** the criticism is correct —
`R_ICU_ExternalIrqEnable` goes to `R_BSP_IrqEnable` on the *allocated vector*, and the pool that owns
that vector is unreachable. But the function is worth keeping, because "the meter's interrupt is
silently detached" is a failure with no other symptom (§2.14 item 3). It is reachable by a different
route: the vector can be **found** rather than remembered. §2.14 now specifies the `IELSR` scan —
match `(R_ICU->IELSR[i] & 0xFF)` against `ELC_EVENT_ICU_IRQ0 + ch` over `i` in `[0, 32)`, then
`NVIC_GetEnableIRQ((IRQn_Type)i)` — using only public symbols. Dropping the function and reporting
only the filter state was the suggested alternative and is the accepted **fallback** if the scan will
not build; `status` prints which one shipped.

### 15.8 A backend-side suppression of the standing `pos:` alert — declined for this work

**The claim:** `PB_REPORT_POS_UNKNOWN=1` raises a HIGH `pos:` page that stands for the whole bench
programme, and while it stands a genuine position loss cannot page. Suppress `pos:` in butler while a
controller has never reported `pos=ok`.

**Why declined here:** it is a **backend** change, and this pitch buys firmware. Suppressing it in the
firmware is not possible — `pos=unknown` is the whole mechanism keeping `water_rules` dark, and the
alert is the backend's honest reaction to it. So the cost is **accepted and documented** rather than
engineered away: §4.6 and §13's 7e both say to expect exactly one such page two minutes after first
boot, that it stands, and that the `pos:` rule is deaf while it does. §16.5.10 records the backend
change for whoever wants it; if the 48-hour run genuinely needs a live `pos:` alert, that item has to
land first.

---

## 16. Owed changes to other repos

Nothing in this list is firmware work. Each item names the file and what it should say.

### 16.1 `cad/wiring/` — `nets.py` first, then regenerate `README.md`

`README.md` is generated by `gen_readme.py` from `nets.py`, so every change below is a `nets.py`
change followed by `make`.

1. **The D6 boot recipe is wrong for this silicon and must be corrected.** `RELAY_NOTES` currently
   says: *"In the sketch, set the level BEFORE the direction: `digitalWrite(D6, <OFF level>);
   pinMode(D6, OUTPUT)`. The other order glitches the pin to the ON level at boot and kicks the
   pump."* On the RA4M1 with this core, `pinMode(pin, OUTPUT)` is a whole-PFS-register write that
   latches PODR = 0 and drives the pin **LOW**, discarding the preceding `digitalWrite`
   (`cores/arduino/digital.cpp:12-14` → `bsp_io.h:391-395`; `IOPORT_CFG_PORT_OUTPUT_HIGH = 0x1` is a
   bit `pinMode` never sets, `r_ioport_api.h:186`). On an **active-LOW** module that recipe asserts
   the pump at boot, every boot, watchdog resets included. Replace it with: *"One PFS write carries
   direction and level together: `R_IOPORT_PinCfg(NULL, g_pin_cfg[D6].pin,
   IOPORT_CFG_PORT_DIRECTION_OUTPUT | (OFF level is HIGH ? IOPORT_CFG_PORT_OUTPUT_HIGH : 0))`. Never
   call `pinMode` on D6 — on this core it would drive the pin LOW and discard the level you just
   set."* Also fix the summary line near the top of the pin table ("D6 is set LOW before it becomes
   an OUTPUT"), which is only correct for an active-HIGH module.
2. **Add a 10 k pull-up on D2 (FLOW).** The pin table gives D2 a 1 k series resistor and no pull —
   the only one of D2/D3/D5 without one. An unplugged, cut or dead meter leaves it floating beside a
   servo lead and a 12 V pump leg. Add it as R4 (or renumber), with the note "an unplugged meter must
   read a firm level, not oscillate: a floating counted-pulse input is indistinguishable from flow".
3. **Add the two screens to `nets.py`.** They are not in it today — the only near-mention is the I2C
   address table's `0x21-0x27 / more PCF8575 / later manifolds` row. Add:
   - two I2C rows in the wire list: `UNO A4/A5 → OLED (SSD1306, Grove/Sensor Kit) 0x3C` and
     `UNO A4/A5 → LCD1602 I2C backpack (PCF8574) 0x27`, both on the `I2C` net with the existing
     `_I2C_PULL` note, plus their 5V_BOARD feeds.
   - two rows in `I2C_ADDRESSES`: `0x3C / SSD1306 OLED 128x64 (u8x8) / Sensor Kit` and
     `0x27 / LCD1602 I2C backpack / PCF8574`.
   - **fix the collision this creates**: the `0x21-0x27 / more PCF8575` row must become
     `0x21-0x26`, because 0x27 is now the LCD.
   - one power-budget row: `5V_BOARD / two I2C screens (OLED + LCD backlight) / ~60 mA / the LCD
     backlight is most of it`. Re-total: running ~470 mA, servo stall ~870 mA, against the ~1.05 A
     ceiling — still inside it, and worth saying so explicitly rather than leaving the reader to
     re-add the column.
   - **bring-up step 1's expectation changes**: `i2c` must see **0x20, 0x27 and 0x3C** (and 0x38 if
     the kit has the DHT20). Today the step says only 0x20, so a working rig would read as a fail.
   - a note that both screens share A4/A5 with the mux select lines and the home hall, that neither
     is painted while the pump is asserted, and that any library added to this bus must be grepped
     for `Wire.flush()` because `TwoWire::flush()` (`Wire.cpp:833`) spins without a bound.
   This also needs a dated `DECISIONS.md` note, because #6 says the two screens are dropped (§16.4).
4. **Bring-up 0** gains: "confirm `status` says `build=bringup`, `dry=0`, `contra=0` and the expected
   pump level before 12 V goes onto COM; confirm `build=bench` before the unattended run".
5. **Bring-up 4a** gains a second criterion: COM-NO stays OPEN across a **power cycle** and across a
   `hang`-forced watchdog reset, not only across a `pump 2000`. That is what proves item 1.
6. **Bring-up 5b** gains: "confirm `status` says `contra=0` afterwards — the float dropping mid-dose
   is the two sensors agreeing, not contradicting".
7. **Bring-up 7a** becomes `pump <ms> prime`, with the note that `prime` **extends** the no-flow
   window and caps the dose at 20 s; it does not remove any abort.
8. **Bring-up 7c** gains: "after the reset, `status` must say `dry=1` and `last=resetmid`".
9. **A new bring-up 7e**: flash the `bench` binary, confirm `status` says `build=bench` and that
   `pump`, `cal` and `hang` are not commands, then start the 48-hour run.
10. **The Bench command set table** gains `clear contra`, marks `pump`/`cal`/`servo`/`home`/`goto` as
    bring-up-binary-only, and changes the `status` row from "whether the IWDT is enabled" to "whether
    the WDT is enabled, its granted window and whether its counter is moving".
11. **The `what stops the pump` table** rows that say "IWDT" should say "WDT" with a pointer to
    §2.5 — see §16.4. And a new row: "float says OK, meter sees nothing → the contradiction latch
    refuses every dose until `clear contra`".
12. The power budget's pump row says "measure in 7f"; the bring-up table ends at 7d. Fix one or the
    other.

### 16.2 `Makefile` (firmware repo)

Monitor speed 9600 → **115200**. Add `test`, `sim`, `bringup`, `check`, `calib` targets. `make sim`
and `make bringup` print a warning line before uploading.

### 16.3 `AGENTS.md` (firmware repo)

- Monitor speed 9600 → **115200**.
- The four envs, what each is for, and **which one is left running** (`uno_r4_wifi`, not
  `uno_r4_wifi_bringup`).
- The two seams (`hal.h`, `link.h`) and the rule that `hal_uno.cpp` is the only file with
  `<Arduino.h>`, a pin, an ISR or a D6 write.
- `tools/check.sh` and what each grep protects.
- The cycle-1 line "A4 becomes channel 5" is **dead** — the bench wiring supersedes it (A4/A5 are
  I2C for the expander and both screens; the five channels arrive through the mux on A0).
- A pointer to this spec as the thing to read before touching `src/safety.cpp`.

### 16.4 `DECISIONS.md` — two dated amendments owed

Per the file's own rule ("a decision that changes gets a new dated entry, not an edit"):

1. **#10 says "the RA4M1 IWDT enabled" and that word is false.** This work ships the **WDT** —
   register-start, PCLKB-clocked, granted window 5592 ms — because the IWDT auto-starts from OFS0
   option bytes the Arduino core does not expose and that can lock the board out of uploads. The
   amendment should say so, say that `status` prints "(WDT, not IWDT)" so nobody reads #10 and
   believes something the code does not do, and say that moving to the true IWDT is a separate pitch
   **whose first deliverable is a documented DFU recovery path**. Nothing in this work writes
   `.option_setting_ofs0`.
2. **#6 says "the two I2C screens are dropped (they were debugging aids; A4 becomes channel 5)".**
   The bench rig keeps both, and this spec keeps both. The amendment should record that the bench
   keeps them, that A4/A5 are I2C (so "A4 becomes channel 5" is superseded by the mux), and what
   keeping them costs — both screens share the bus that carries the mux select lines and the home
   hall, so neither is painted while the pump is asserted and the UI cadence is coarsened to hold bus
   traffic down (§5).

### 16.5 `backend/` and `app/` — none of it firmware work

1. **The refill log.** The human taps "refilled" in the app; the backend records it against the
   controller. This is what finally makes the wiring README's rule — *a float input that never
   changes state across a refill is presumed dead: refuse, do not assume OK* — implementable, because
   the refill is a human act the board cannot observe. The firmware's whole contribution is
   `ch204 = <seconds since D5 last changed>` (§2.10). Backend: a `refills` table and a `POST /refill`
   endpoint. App: a button.

   **Amend pitch-d155fe ("Trust the tank") first.** Its no-go says "the board's whole contribution is
   **`fstale=`** and the contradiction report". This spec emits **`ch204`** instead, deliberately: an
   unknown key reaches no storage and no alert rule (`parse_report`, `butler.py:204-257`), so a
   firmware that shipped `fstale=` would satisfy the pitch's letter and deliver nothing at all, while
   `ch204` lands in `readings` and is graphable with zero backend change. Whichever key the amendment
   settles on, the value must be a **bare non-negative ASCII integer** — `_int_in` rejects a leading
   `-` and any non-digit, so `-1`, `unknown` or `never` in a `chN` 400s the whole report.
2. **The `ch204` staleness alert.** With the refill log in place, raise when `ch204` spans a recorded
   refill without D5 having changed. The firmware never refuses on staleness (§15.5); this alert is
   the whole of the rule's enforcement.
3. **The contradiction latch's durable half. Required, not optional, and possibly a blocker.** The
   firmware latch lives in `.noinit`: it survives a watchdog reset and does **not** survive a power
   cycle (§2.7) — **and whether it survives even a watchdog reset is unproven until bring-up 7c′
   passes** (§2.3). If 7c′ fails, the firmware latch is cold-boot-only and this item is a hard
   precondition for flipping `PB_REPORT_POS_UNKNOWN`. So the backend must own durability.
   On `err=contra`, `ch207=1`, or `float=` dropping to 0 from a controller that was reporting 1:
   stop queuing water commands for that controller and **keep refusing until a human confirms in the
   app** — not until the next report says something nicer. The app needs a "the tank has been checked,
   resume" action, which should also be what tells the operator to type `clear contra` on the board.
   Until this lands, a power cycle after a latch silently rearms the rig.
4. **Store `err=`. This is a stated precondition of the contradiction latch's notification story, not
   a nice-to-have.** It is an unknown key today and `parse_report` discards it
   (`butler.py:204-257`), so the only channel by which the board can say *why* it refused reaches
   nobody. Concretely: a latch surfaces as `float=0`, which raises the fixed text *"the reservoir on
   X is empty or at the waterline"* (`:1246-1249`) — **even when the cause was a kinked hose with a
   full tank**, which §2.7 latches on deliberately. Until this lands (or a `ch207` alert rule does),
   the phone gives the operator the wrong first thing to check. A `TEXT` column on `status` and a
   line in the report view would make every refusal legible. The numeric diagnostics already ride as
   `ch200..ch209` and need no backend change at all.
5. **Controller retirement.** The silence rule iterates `controllers WHERE last_seen > 0`
   (`butler.py:1137`) and there is no notion of a retired controller, so one afternoon of desk work
   with a sim board — or one bench controller renamed — pages the phone hourly, forever, until
   someone hand-edits the SQLite file. Add a `retired` column, a way to set it, and make the silence
   rule skip retired controllers. This is why §8 gives the sim binary no network stack at all: it is
   a workaround for a missing backend feature, not a design goal.
6. **`FLOW_FLOOR_ML_S = 20` (`butler.py:96`) is a guess** labelled "bench-rig-tunable", and it sizes
   `cap_for()`. Bring-up 7b measures the real ml/s. Update it, and note that at a real 30 ml/s the
   current value authorises 1.8× the requested water — which is why the firmware now clamps the cap
   to 2× the requested millilitres itself (§2.8, `PB_ML_PER_S_MEASURED`).
7. **A lost response destroys a command.** When a report lands and its *response* is lost, the
   backend has already moved a command `queued → sent` (`:870-873`); the board's retry then hits the
   unconditional expire (`:837-841`) and kills a command it never saw — costing a HIGH "never
   acknowledged" page for a dose that never existed, and charging the pot the full `ml` against its
   daily cap because `flow_ml` is NULL (`:745-751`). The firmware narrows the window (§4.4: never
   retry once any response bytes arrived) but cannot close it. Backend options: charge only
   `flow_ml` for expired commands, or re-hand a `stop` but never a `water`.
8. **Teach butler the rig's dose ceiling.** `dose_run()` refuses `ml > PB_DOSE_RIG_MAX_ML` (250), but
   butler accepts `dose_ml` and `ml=` up to `MAX_DOSE_ML` (1000, `:89`) with no knowledge of the rig.
   A pot configured `dose_ml=300` — or one manual `water=N ml=300` — is queued, refused, acked with
   `flow_ml=0`, charged 0 ml, given a full cooldown and paged HIGH, once per cooldown period,
   **forever, and never watered**. Either a per-controller `max_dose_ml` validated at **config**
   time (rejected when the pot is saved, not when the dose is attempted) and enforced at `/command`
   and in `water_rules`, or have the board advertise its ceiling as a diagnostic channel the backend
   clamps against. Until then, §4.6's sixth going-live precondition is the manual stand-in.
9. **`AUTOINCREMENT` on `commands.id`.** The board's replay guard (§4.3) assumes ids never go
   backwards, which today is true only because `schema.sql` says rows are never deleted —
   `INTEGER PRIMARY KEY` is a rowid alias and a prune or a restore can reuse an id. `AUTOINCREMENT`
   makes it a property of the schema rather than of a comment. Recovery on the board today is a cold
   boot, and `status` now prints `cmd_high_water=` so the state is at least visible.
10. **Suppress `pos:` while a controller has never reported `pos=ok`.** `PB_REPORT_POS_UNKNOWN=1`
    raises one HIGH `pos:` page two minutes after first boot that then stands for the whole bench
    programme, and while it stands the `pos:` rule is deaf to a genuine position loss (§4.6, §15.8).
    A `pos_seen`-aware raise condition — or simply not raising until a controller has ever been
    `pos=ok` — makes the flag free. **If the 48-hour run needs a live `pos:` alert, this lands
    first.**
