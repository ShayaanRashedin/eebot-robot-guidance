# EEBOT Robot Guidance System

**HCS12 Assembly · Embedded Systems · Robotics · Finite-State Machines**

A group project for **COE538: Microprocessor Systems at Toronto Metropolitan University**. Our team developed a guidance system for the eebot mobile robot, connecting infrared sensing, motor control, and an LCD interface through assembly-language firmware.

The challenge brings software into the physical world: the robot must follow a tape path, make decisions at intersections, recover from dead ends, and navigate back using recorded route information.

## Our team

- **Shayaan Rahsedin**
- **Hamidullah Mohmmad Ali**
- **Daniel Chylek**

This repository presents our shared course project. The firmware and documentation show the embedded-systems concepts we applied and the engineering skills we developed as a team.

## What the firmware does

- **Follows a tape path:** reads five infrared channels and adjusts the motors using calibrated sensor thresholds.
- **Makes junction decisions:** detects side branches and selects an available direction using a consistent exploration order.
- **Remembers the route:** stores arrival, departure, and alternative headings for up to 32 junctions or corners.
- **Recovers from dead ends:** responds to the front bumper by reversing, turning around, and returning toward an alternative branch.
- **Retraces the learned path:** uses a rear-bumper signal at the destination to begin reverse traversal of the recorded route.
- **Displays live feedback:** shows battery voltage, navigation state, sensor readings, and fault information on a 20×2 LCD.
- **Handles faults:** stops on conditions such as lost-line and turn timeouts, ADC timeout, or invalid route state.

## Skills demonstrated

| Area | Application in the project |
| --- | --- |
| Assembly programming | HCS12 instructions, registers, addressing modes, stack operations, and subroutines |
| Embedded architecture | A finite-state machine separates startup, forward motion, turning, recovery, alignment, and stopping |
| Hardware interfaces | Direct register control of GPIO, motor direction and enable signals, bumpers, and LCD communication |
| Sensor processing | Multiplexer selection, ADC conversion, settling delays, and unsigned threshold comparisons |
| Interrupts and timing | A timer-overflow interrupt supports elapsed-time navigation and periodic display updates |
| Data structures | Fixed-size route arrays record decisions and support backtracking without dynamic allocation |
| Hardware debugging | Live sensor/state displays help diagnose calibration, timing, and steering behavior |

## How it is organized

Each main-loop iteration reads the guider sensors, classifies the readings, dispatches the active robot state, and refreshes the display when due. Motor helpers centralize hardware control, while route routines track junction decisions separately from line following.

| Module | Main routines |
| --- | --- |
| State machine | `STATE_DISPATCH`, `DO_START`, `DO_FWD`, `DO_TURN_L`, `DO_TURN_R`, `DO_REV_RECOV` |
| Sensing | `IR_READ_ALL`, `IR_SELECT_ONE`, `IR_CLASSIFY`, `ADC_READ` |
| Navigation | `ROUTE_DECIDE`, `ROUTE_RETRY`, `ROUTE_RETURN`, `JUNCTION_UNLOCK` |
| Motor control | `MOTOR_FWD`, `MOTOR_REV`, `MOTOR_LEFT`, `MOTOR_RIGHT`, `MOTOR_STOP` |
| Timing and display | `TOF_ISR_HANDLER`, `HUD_REFRESH`, `HUD_SHOW_IR`, `LCD_*` |

## Engineering lessons

This project connected instruction-level programming with physical feedback. Sensor polarity determines which direction the robot steers; sampling and settling time affect the reliability of its readings; and state transitions must distinguish a new junction from one the robot is still crossing. Keeping sensor, motor, navigation, and display routines separate makes those interactions easier to reason about and debug.

## Explore the code

- [Assembly firmware](Sources/main.asm)
- [Build, calibration, and operating guide](docs/SETUP.md)
- [CodeWarrior project](finalproject.mcp)

**Platform:** eebot robot, HCS12 / MC9S12C32 target configuration, CodeWarrior for HCS12, infrared guider, bumper switches, and character LCD.

Route memory lasts for one run. Navigation targets branching mazes rather than cyclic maps, and automatic stopping at the starting area assumes a blank end to the tape. Board-specific calibration and timing details are documented in the setup guide.
