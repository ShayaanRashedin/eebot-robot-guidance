# Build and operating guide

## Build

1. Open `finalproject.mcp` in CodeWarrior for HCS12.
2. Confirm the board and derivative configuration. The project is configured for **MC9S12C32**; `Sources/derivative.inc` includes `mc9s12c32.inc` from the installed CodeWarrior library.
3. Clean and rebuild `Sources/main.asm`. Check the assembler listing and the board's memory map before loading.
4. Load the program using the lab board's debugger or monitor configuration.

The firmware uses absolute assembly: RAM begins at `$3800`, code at `$4000`, and the initial stack pointer is `$4000`. The timer-overflow vector is `$FFDE`; the reset vector is `$FFFE`. Confirm these addresses against the target board configuration.

For a new CodeWarrior project, select HCS12 absolute assembly and add `main.asm` and `derivative.inc`. The vendor include directory must be available. Compile only one copy of the main program and its vectors. For the lab's electronic submission, the source can be copied to the required name `guidance.asm`.

This repository contains source and project configuration, not a prebuilt firmware image.

## Calibration

The timing constants assume a **24 MHz bus**. The firmware does not configure the PLL. With timer prescaling of 16, one overflow tick is approximately 43.69 ms. Recalculate the intervals if the board uses a different clock.

Before a maze run, confirm motor direction and enable signals with the wheels raised, then measure the guider readings on tape and on the surrounding surface.

| Setting | Purpose |
| --- | --- |
| `CAL_*`, `*_THRESH` | Sensor reference levels and steering thresholds |
| `SETTLE_50US` | Multiplexer settling interval; 100 corresponds to approximately 5 ms |
| `PROBE_TICKS` | Advance from side-branch detection to the turning centre |
| `TURN_MIN`, `UTURN_MIN` | Minimum turning intervals before line acquisition |
| `TURN_LIMIT` | Maximum interval for acquiring a line during a turn |
| `REV_TICKS`, `ALIGN_TICKS` | Reverse and alignment intervals |
| `JUNC_TICKS` | Minimum separation before detecting another junction |
| `LOST_TICKS`, `HOME_TICKS` | Lost-line and blank-start-area stop intervals |

The pattern detectors classify tape as `reading <= CAL - THRESH`. The differential line detector steers left below its centre band and right above it. Verify this polarity on the actual robot. The battery display uses `ADC × 39 + 600` millivolts; compare its reading with a meter when calibrating.

## Operation

1. Place the robot at the beginning of the tape.
2. Press either bumper, then release both to start.
3. The robot follows the tape and records junction decisions. The front bumper triggers dead-end recovery during forward exploration.
4. At the destination, press the rear bumper while the robot is driving forward to start return traversal.
5. A second rear press during forward return motion stops the robot. Reset or reload to begin a new run.

The front bumper is active-low AN2 (`$04`); the rear bumper is active-low AN3 (`$08`). Navigation bumper actions are sampled in forward mode, so hold the destination press long enough for that state to sample it.

Turns must leave the incoming line and acquire two consecutive bow-on-tape samples after the minimum turn interval. A timeout produces a latched stop.

## Return endpoint

After the final recorded junction is retraced, continuous loss of all four pattern detectors for `HOME_TICKS` is treated as arrival at the blank starting area. If tape continues through the starting point, use a rear-bumper stop or implement a distinct endpoint marker. The sensor inputs alone do not identify an arbitrary position along uninterrupted tape.

Route storage is cleared on reset. The navigation model assumes a branching maze with at most two outgoing choices at each intersection; it does not recognize revisited intersections in cyclic maps.

## LCD feedback

The first row displays battery voltage, navigation mode, state, and an alive indicator. The second row alternates infrared readings with route and fault information.

- Mode: `0` exploration, `1` failed-branch backtracking, `2` return traversal.
- Heading: `0` north, `1` east, `2` south, `3` west in a relative coordinate system.
- `N`: stored route depth, shown in hexadecimal.
- `B`: pressed-bumper mask (`0` none, `1` front, `2` rear, `3` both).

| Fault code | Meaning |
| --- | --- |
| 0 | No fault; also used for normal or manual stops |
| 1 | Invalid state |
| 2 | Route capacity, missing exit, missing recovery path, or unsupported junction |
| 3 | Turn acquisition timeout |
| 4 | Tape lost beyond the permitted interval |
| 5 | ADC conversion timeout |
