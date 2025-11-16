# Part 1 & Part 2 Design Guide (Comprehensive)

This document explains how the program in `Proj/main.asm` is structured and how the platform services (Part 1) and configuration layer (Part 2) work together. It also clarifies why, on real hardware, you see the LED bar light up and a spinner in the top‑left of the LCD.

Sections
- High‑level architecture
- Reset and initialisation sequence (and the final success beacon)
- Platform services (Part 1)
- User/configuration layer (Part 2) and state machines
- Data layout and end‑to‑end flow
- LCD and keypad drivers (design and gotchas)
- Timing and the heartbeat spinner
- Build/verification tips

---

## High‑level architecture

The firmware follows a co‑operative loop:

1) Sample inputs (keypad + push buttons)
2) Run the state machine (decide what to do next)
3) Drive outputs (render LCD + indicators)

Timer0 overflow is the only periodic interrupt and is used for timekeeping; everything else runs in the foreground (`MainLoop`).

Subsystems
- Reset/Init: prepares CPU/peripherals and finishes with a success beacon (all LEDs on briefly).
- Platform services (Part 1): stack/register init, ports, LCD driver, keypad scanner, timers, and a buffer‑to‑LCD streamer.
- Application layer (Part 2): user configuration and a top‑level state machine. The handlers are stubs now, but the loop runs and the heartbeat remains visible.

---

## Reset and initialisation sequence

Reset enters at vector 0. The very first operation sets the stack pointer inline before any `rcall`/`ret`. On AVR, calling a subroutine before SP is initialised corrupts memory. This was the root cause of earlier “silent hangs” and is now fixed.

Initialisation order in `RESET` (see `Proj/main.asm:246`):
- Set `SPH/SPL <- RAMEND` inline (no RCALL)
- `InitRegisters` — clear caller‑saved registers
- `InitIOPorts` — LCD (PORTF/PORTA), LED bar (PORTC), keypad (PORTL), PB0/PB1
- `InitLCDDriver` — lab‑compatible LCD power‑on sequence/macros
- `InitKeypad` — prepare row/column lines
- `InitTimers` — Timer0 overflow for timekeeping
- `InitStateMachine` — clear SRAM state and set defaults
- `sei` — enable interrupts
- `Beacon9` — light all LEDs briefly as an “init success” indicator
- `MainLoop` — enter foreground loop

Why the final beacon matters
- If the LCD is miswired or contrast is wrong, the LED “all‑on” after init is a strong signal that init completed and the loop is running.

---

## Platform services (Part 1)

### Stack and registers
- `InitStack` also exists as a subroutine, but in `RESET` we set SP inline before any RCALL. After that, RCALL/RET is safe.
- `InitRegisters` clears working aliases (`temp0..temp13`, `workA..workG`) to known values.

### I/O ports
- LCD data bus: `PORTF`, control: `PORTA` (RS=PA7, E=PA6, RW=PA5). `DDRF`/`DDRA` set to outputs.
- LED bar: `PORTC` with `DDRC=0xFF`. Cleared initially; later used for beacons and “alive” indication.
- Keypad: `PORTL`; columns (PL7–PL4) outputs, rows (PL3–PL0) inputs with pull‑ups.
- PB0/PB1: inputs with pull‑ups on `PORTB`.

### LCD driver
- Macros `lcd_write_com`, `lcd_write_data`, `lcd_wait_busy` are copied in spirit from the labs: full‑port writes to `PORTA` (RS/RW), and explicit setup/hold delays around the `E` strobe.
- Power‑on init strictly follows the lab sequence: 15 ms → Function Set ×4 → Display Off → Clear → Entry Set → Display On.
- Busy‑flag polling flips `DDRF` to input, reads `PINF` D7, then restores it to output.
- `LcdWriteBuffer` streams the two SRAM line buffers (`LCDLine0/1`) to DDRAM every loop.

Gotchas (and how the code addresses them)
- SP must be set before any RCALL/RET — we do this inline at reset.
- `ldi/subi/sbci/ser` only accept `r16..r31`. Immediates are loaded into high regs (`workA..workC`) when needed and then moved.
- Busy polling requires RW to be wired; this is true on the lab board. For RW‑tied‑low displays, replace `lcd_wait_busy` with small fixed delays.

### Keypad
- `ScanKeypad` drives one column low at a time, lets signals settle, and reads the row nibble. The first zero row bit is the pressed row.
- `DecodeKey` maps `(row,col)` to ASCII (‘1’..‘9’, ‘*’, ‘0’, ‘#’) and ignores the letter column for now.
- `LatchKeyEvent` edge‑detects transitions and records a single byte in `KeypadSnapshot` for the application layer.

### Timers
- Timer0 overflow ISR increments three counters: `SystemTick`, `ScrollTimer`, `PlaybackTimer`.
- The heartbeat spinner uses `ScrollTimer` as its time base.

### Output
- `DriveOutputs` writes the current spinner character into `LCDLine0[0]`, calls `LcdWriteBuffer`, and turns `PORTC` fully on as an “alive” indicator. Feel free to later make this LED behaviour reflect states instead.

---

## User/configuration layer (Part 2) and state machines

Top‑level state: `DroneState` in SRAM with these planned values:
- `STATE_IDLE` — wait for reset/start
- `STATE_CONFIG` — edit `(x,y)` and `visibility`
- `STATE_PATH_GEN` — compute observation points into `ObservationPath`
- `STATE_SCROLL_PATH` — preview observation list scrolling on line 0
- `STATE_PLAYBACK` — step through the path (fixed altitude/speed in Part 2)
- `STATE_DONE` — final display

`RunStateMachine` (dispatcher) is a stub; add the handlers incrementally. The service loop and heartbeat continue to run while you build the UI.

---

## Data layout and end‑to‑end flow

Important SRAM symbols (see `.dseg` in `Proj/main.asm`):

Platform services
- `SystemTick`, `ScrollTimer`, `PlaybackTimer` — timekeeping counters (Timer0 ISR)
- `KeypadSnapshot`, `KeypadHold`, `ButtonSnapshot` — input latches

User configuration
- `AccidentX`, `AccidentY`, `Visibility`, `InputCursor`, `ConfigFlags` — edited in `STATE_CONFIG`

Terrain + path
- `MountainMatrix[49]`, `CoverageMask[49]` — map + coverage
- `ObservationPath[MAX_OBS_POINTS*3]`, `PathLength`, `PathIndex` — planned path as x,y,z triples

LCD buffers
- `LCDLine0[16]`, `LCDLine1[16]` — double buffer for the LCD

Heartbeat data flow
1) Timer0 ISR increments `ScrollTimer`.
2) `MainLoop` calls `DriveOutputs`.
3) `DriveOutputs` reads bit 7 of `ScrollTimer` and stores '-' or '|' into `LCDLine0[0]`.
4) `LcdWriteBuffer` streams both lines to the LCD controller.
5) LED bar (`PORTC`) is set on, so you see a static “alive” light.

This is why you see both the spinner and the LED bar lit while the program runs.

---

## LCD and keypad drivers (design and gotchas)

LCD
- Exact lab‑style timing around `E` is essential. Our macros implement the same setup/hold windows and busy flag polling.
- If init looks flaky, double‑check PORTA/PORTF wiring and contrast before suspecting code.

Keypad
- Ensure `DDRL` matches the lab: PL7–PL4 outputs, PL3–PL0 inputs with pull‑ups. The column drive mask and row mask (`INIT_COL_MASK`, `ROWMASK`) are used in the scan loop.

---

## Timing and the heartbeat spinner

Timer0 runs with `clk/64` prescaler; the spinner uses bit 7 of `ScrollTimer`. Change the bit for faster/slower toggling if needed (e.g., bit 6 for faster).

---

## Build and verification tips

- Target device must be `ATmega2560` for both build and programming.
- After reset, if you see the final all‑LEDs beacon then the LCD spinner should appear; if not, adjust the LCD contrast.
- If “nothing” runs, confirm SP is set before the first RCALL in `RESET`.
- If PF/PC pins don’t respond, ensure `DisableJTAG` executed (two writes to `MCUCR`).

---

## Next steps

- Fill `RunStateMachine` and `Handle*State` to implement configuration, planning, and playback.
- Implement `BuildMountainModel`, `GenerateSearchPath`, and format the path for the scroll preview.
- Replace the “LED always on” with state‑based LED behaviour.

---

## Part 2 Implementation Plan (with Stage Beacons)

We will implement Part 2 in small, verifiable stages. Each stage adds a visible indicator (a stage beacon) so it’s obvious on hardware that the new function is wired correctly before progressing.

Stage beacon convention
- LED: briefly light one specific bit on `PORTC` for the stage number (PC0..PC7).
- LCD: optionally render a short stage tag at the end of line 1 (columns 13–15), e.g. `S1`, `S2`, etc.
- Keep the existing heartbeat and the final init success beacon; stage beacons are brief, one‑shot cues when entering the stage or completing its action.

Stages
1) S1 — Key echo baseline
   - Show last key pressed at `LCDLine1[0]`.
   - On first detected keypress, flash PC0 once and print `S1` at line1[13..14].

2) S2 — Config screen scaffold
   - Enter `STATE_CONFIG` automatically after init.
   - Render two lines: `loc:(x,y)` and `visib: d` using temporary defaults.
   - On first render in `STATE_CONFIG`, flash PC1 and print `S2`.

3) S3 — Cursor + editing flow
   - Use digits to edit `(x,y,d)`; use `*` and `#` to move cursor/confirm fields.
   - Clamp values to bounds (x,y in 0..6; d in 0..9).
   - Flash PC2 when a field is successfully confirmed; print `S3` once when cursor logic activates.

4) S4 — Generate path (skeleton)
   - Implement `ResetCoverageMap` and a stub `GenerateSearchPath` that produces a trivial but valid `ObservationPath` (e.g., 1–2 points).
   - Flash PC3 after `GenerateSearchPath` returns non‑empty; print `S4`.

5) S5 — Format + scroll preview
   - Implement `PreparePathScrollData` and `UpdateLCDForScroll`.
   - Clear both LCD lines in `UpdateLCDForScroll` and then re‑stamp the stage tag on line 0 (cols 14–15). This prevents the S3 header `loc(x,y)` from lingering when transitioning into S4/S5.
   - Drive `STATE_SCROLL_PATH` preview using `AdvanceScrollWindow`.
   - Flash PC4 on first entry into scroll; print `S5`.

Implementation notes
- `ScrollBuffer[64]` holds the formatted preview string: `xx,yy,zz / xx,yy,zz / ...`.
- `PreparePathScrollData` builds the buffer from `ObservationPath`, stores `ScrollTextLen` and resets `ScrollHead`.
- `AdvanceScrollWindow` steps `ScrollHead` when bit 4 of `ScrollTimer` toggles (smooth, timer‑paced scroll).
- `UpdateLCDForScroll` copies a 16‑char window starting at `ScrollHead` to `LCDLine1`, wrapping at `ScrollTextLen`; line 0 only shows the stage tag (`S4`).

6) S6 — Playback stepper
   - Implement `BeginPlaybackRun`, `AdvancePlaybackStep`, `UpdateLCDForPlayback`.
   - Use fixed altitude/speed (61/2) as per Part 2 scope.
   - Flash PC5 when playback starts; print `S6`.

7) S7 — Done state
   - Implement `HandleDoneState` to show final point + accident flag.
   - Flash PC6 upon entering done; print `S7`.

8) S8 — Button bindings + polish
   - PB0: generate path; PB1: start playback; improve error handling.
   - Flash PC7 after both bindings fire successfully; print `S8`.

Notes
- Stage beacons are intentionally brief; they should not interfere with the heartbeat or the LCD text.
- As stages complete, we can remove the stage tags or keep them for service diagnostics.

---

## TODO Checklist (Part 1 and Part 2)

Part 1 — Platform Services
- [x] Reset vector to `RESET` and inline stack pointer init (no RCALL before SP set)
- [x] General register init (`InitRegisters`)
- [x] I/O ports init (`InitIOPorts`) for LCD (PORTF/PORTA), LED bar (PORTC), keypad (PORTL), PB0/PB1
- [x] Disable JTAG (two writes to `MCUCR` with `JTD` set)
- [x] LCD driver macros (`lcd_write_com`, `lcd_write_data`, `lcd_wait_busy`)
- [x] LCD power‑on init (`InitLCDDriver`) using lab timing
- [x] LCD buffer streamer (`LcdWriteBuffer`)
- [x] Keypad scan (`ScanKeypad`) and decode (`DecodeKey`)
- [x] Key event latch (`LatchKeyEvent`)
- [x] Timer0 overflow setup + ISR increments counters
- [x] Foreground cooperative loop (`MainLoop`)
- [x] Output driver: heartbeat spinner + push LCD buffers
- [x] Final init success beacon (all LEDs briefly after `sei`)
- [x] Bug fix: no `rcall` before SP (root cause found and fixed)

Nice‑to‑haves in Part 1 (optional)
- [ ] Add “no‑busy” LCD mode (fixed delays) for RW‑tied‑low displays
- [ ] Debounce PB0/PB1 using ticks
- [ ] Replace “LED always on” with state‑driven LED policy

Part 2 — Application State + UI

State machine scaffold (in progress)
- [x] Implement `RunStateMachine` dispatcher
- [x] Implement `HandleIdleState`
- [x] Implement `HandleConfigState`
- [ ] Implement `HandlePathGenState`
- [ ] Implement `HandleScrollState`
- [ ] Implement `HandlePlaybackState`
- [ ] Implement `HandleDoneState`
### Stage Beacons (completion indicators)
Each stage beacon is an explicit, visible indicator that a discrete task is implemented and verified on hardware. The beacon is the `Sx` stamp rendered at the end of LCD line 0 (by `UpdateLCDForConfig`), plus an optional LED blip during bring‑up. These are not subtasks of a single handler; they gate progress through Part 2.

- [x] S1 — Input pipeline and LCD path validated
  - Task: keypad scan → decode → event latch → LCD write‑out works end‑to‑end
  - Visual: `S1` tag; (earlier) PC0 blip during bring‑up

- [x] S2 — CONFIG screen scaffold
  - Task: enter `STATE_CONFIG`; render `loc(__, __)` and `vis: __`
  - Visual: `S2` tag (we later removed the dedicated stamping in favor of central stamping)

- [x] S3 — Two‑digit editing + confirm/clear + echo
  - Task: two‑digit live editing per field (`XEditVal/Cnt`, `YEditVal/Cnt`, `VEditVal/Cnt`); `#` confirms & clamps (X,Y: 0..14, vis: 0..15); `*` clears; echo two‑digit `x y d`
  - Visual: `S3` tag

- [x] S4 — Path generation skeleton
  - Task: `ResetCoverageMap` + minimal `GenerateSearchPath` producing a non‑empty `ObservationPath` (two points)
  - Visual: `S4` tag

- [ ] S5 — Scroll preview
  - Task: `PreparePathScrollData` + `UpdateLCDForScroll` + `AdvanceScrollWindow`
  - Visual: `S5` tag

- [ ] S6 — Playback stepper
  - Task: `BeginPlaybackRun` + `AdvancePlaybackStep` + `UpdateLCDForPlayback` at fixed 61/2
  - Visual: `S6` tag

- [ ] S7 — Done state
  - Task: `HandleDoneState` shows final point + accident flag
  - Visual: `S7` tag

- [ ] S8 — Buttons + polish
  - Task: PB0→path generation, PB1→playback; error handling and UI polish
  - Visual: `S8` tag
- [ ] S4 Generate path (skeleton) — minimal `ObservationPath`; flash PC3; tag `S4`
- [ ] S5 Scroll preview — format and scroll list; flash PC4; tag `S5`
- [ ] S6 Playback stepper — fixed 61/2; flash PC5; tag `S6`
- [ ] S7 Done state — final display; flash PC6; tag `S7`
- [ ] S8 Buttons + polish — PB0/PB1 bound; flash PC7; tag `S8`

Config entry and rendering
- [x] Cursor‑based editing for `(AccidentX, AccidentY, Visibility)` via keypad
- [x] Two‑digit live editing with per‑field buffers (`XEditVal/Cnt`, `YEditVal/Cnt`, `VEditVal/Cnt`)
- [x] Key mapping: digits append; `#` confirm & advance; `*` clear current field
- [x] Bounds/validation (X,Y clamp to 0..14; visibility clamp to 0..15)
- [x] `UpdateLCDForConfig`: render two‑slot fields: `loc(__, __)` + `vis: __`; show edit vs confirmed values
- [x] Echo confirmed values (two digits per field) on line 1 with spacing

Path generation and formatting
- [ ] `BuildMountainModel` (map heights)
- [ ] `ResetCoverageMap` (clear coverage & indices)
- [ ] `GenerateSearchPath` loop using visibility metric
- [ ] `FindNextObservation` (evaluate best next point)
- [ ] `UpdateCoverageForPoint` to mark visible cells
- [ ] `StoreObservationPoint` (push x,y,z into buffer)
- [ ] `PreparePathScrollData` (format “x,y,z / …”)
- [ ] `UpdateLCDForScroll` to copy formatted list into `LCDLine0`
- [ ] `AdvanceScrollWindow` (timer‑driven)

Playback and final status
- [ ] `BeginScrollPreview` (enter scroll state after PB0)
- [ ] `BeginPlaybackRun` (seed timers/index on PB1)
- [ ] `AdvancePlaybackStep` (tick‑driven stepping)
- [ ] `UpdateLCDForPlayback` (highlight current point; line2 “P 61 2”)
- [ ] `HandleDoneState` (final display with accident flag)

Formatting helpers
- [ ] `FORMAT_TWO_DIGIT` macro (00..99 or space/zero policy)
- [ ] `WRITE_COORD_TRIPLE` macro (“x,y,z”)

Input bindings (buttons)
- [ ] PB0 → Generate path (to `STATE_PATH_GEN`)
- [ ] PB1 → Start playback (to `STATE_PLAYBACK`)

Helpful debug crumbs (optional while building Part 2)
- [ ] Key echo on LCD line 1 (copy `KeypadSnapshot` → `LCDLine1[0]`)
- [ ] LED policy by state: off in idle, blink in scroll, on in playback
Constraints and ranges
- Grid size: up to 15×15 (MAP_SIZE=15, MAP_CELLS=MAP_SIZE*MAP_SIZE)
- Coordinate ranges: X,Y in 0..14 (15 cells per axis)
- Visibility range: 0..15
  - Live editing supports one or two digits per field; confirm commits to SRAM
