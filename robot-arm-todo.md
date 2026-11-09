# Robot Arm — Full TODO List

_Created 2026-11-07. Companion to `robot-arm-handoff.md`, which has the decisions, BOM and reasoning. This file has the work._

**How to use this file**
- Tick boxes as you go. Each module ends with a **Done when** line; don't tick the module until that's true.
- `#N` numbers match the module list in the handoff file. New modules added here are #40+.
- Tags: **[G431/C]** joint firmware · **[F411/C]** power node · **[Linux/C++]** · **[Linux/Py]** · **[PC/Py]** · **[PC/C]** host tests · **[HW]** hardware or mechanical · **[DOC]** writing.
- I write all code; Claude reviews. CubeMX init and launch-file scaffolding are boilerplate Claude may write.
- Hour estimates assume about 35 h/week. Times are rough and usually run over.

---

## M0 — Setup and prep (in parallel with finishing the quad)

### Measurements and scoping
- [ ] **[HW]** Do the apartment measurement checklist (doors, table, counter, glass, route); photos of everything
- [ ] Send the numbers to Claude: mast height, reach, lift vs. no lift, per-joint torque, final motors and gear ratios
- [ ] Decide whether any door on the route is out of scope (strong closer, knob that's too stiff)

### Parts (order early; AliExpress takes 2–4 weeks)
- [ ] FOC learning kit: B-G431B-ESC1, MT6701 board, diametric magnet
- [ ] Phase 1 kit: one 8308-class motor, second ESC1, MT6701, bearings and dowel pins, CANable clone
- [ ] Check what I already own: 3D printer (or makerspace access), current-limited bench supply, scope or logic analyzer, luggage scale
- [ ] Phase 2–5 parts: order only when the previous phase works

### Dev environment
- [ ] **[PC]** WSL2 + Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic (steps in the handoff file); paste `glxinfo -B` to Claude
- [ ] **[PC]** STM32CubeIDE or CubeMX + arm-none-eabi-gcc + a debugger (OpenOCD or STM32CubeProgrammer) working on the B-G431B-ESC1
- [ ] **[PC]** usbipd-win so WSL can see the CANable and ST-Link

### Repository
- [ ] Create a public monorepo (no Claude co-author lines in commits):
  ```
  robot-arm/
  ├── firmware/joint/        ← G431 joint + wheel firmware (C)
  ├── firmware/power_node/   ← F411 power node (C)
  ├── shared/can_protocol/   ← #22, header used by firmware AND Linux
  ├── ros2_ws/src/           ← ROS 2 packages (C++/Python)
  ├── tools/                 ← PC Python scripts (tuning, tests)
  ├── tests/host/            ← host unit tests for firmware logic (C)
  ├── hardware/              ← CAD, gearbox models, wiring diagrams, PCB
  └── docs/                  ← design docs, FMEA, measurements, checkpoints
  ```
- [ ] README with the goal, a block diagram and the milestone list
- [ ] **[DOC]** `docs/CHECKPOINTS.md`: dated log with photos and measured numbers only
- [ ] CI (GitHub Actions): build the firmware and run host unit tests on every push

---

## M1 — Learn FOC on the spare quad motor (60–100 h)

Bench: ESC1 + quad motor clamped down + MT6701 under a magnet on the shaft. **No propeller. Bench supply at 8–12 V with a 1–2 A limit. Clamp voltage and current in code too.**

- [ ] Record the motor model, KV and pole count; set safe voltage and current limits
- [ ] **[HW]** Print the motor clamp and encoder bracket; glue the magnet centered on the shaft
- [ ] **[G431/C]** CubeMX project for the ESC1: clocks, TIM1, ADC, SPI/I2C or the MT6701's SSI interface, UART, LEDs
- [ ] **[G431/C]** Open-loop SVPWM: spin at a fixed electrical speed with low voltage. **Done when:** smooth rotation, current stays under the limit
- [ ] **[G431/C]** Read the encoder; print angle and speed. **Done when:** angle matches a hand turn, no jumps at wrap-around
- [ ] **[G431/C]** Align electrical angle to encoder (drive d-axis, read offset). **Done when:** offset is repeatable across restarts
- [ ] **[G431/C]** Phase-current read synced to PWM; offset calibration at boot. **Done when:** currents look sinusoidal and sum to about 0
- [ ] **[G431/C]** Clarke + Park; print i_d, i_q while spinning open-loop
- [ ] **[G431/C]** d/q current PI loop. **Done when:** i_q follows a step command, i_d stays near 0
- [ ] **[G431/C]** Velocity loop around the current loop
- [ ] **[G431/C]** Position loop (expect it to be weak on a high-KV motor; that's fine)
- [ ] **[DOC]** Checkpoint: plots of the current step response and velocity tracking, plus what went wrong and why

---

## M2 — Phase 1: one joint actuator, done properly (200–300 h)

**This is the core of the portfolio.** Hardware: 8308 motor, ESC1, MT6701, printed 9:1 cycloidal gearbox.

### Mechanical
- [ ] **[HW]** Design a 9:1 cycloidal reducer around the 8308 (output bearing, encoder mount, motor mount)
- [ ] **[HW]** Print, assemble, measure backlash at the output (dial indicator or encoder vs. a fixed reference)
- [ ] **[HW]** Test stand: joint clamped down, a 30 cm bar on the output for torque tests

### Shared protocol
- [ ] **#22 [C] CAN protocol definition** (`shared/can_protocol/`)
  - [ ] Message ID scheme: node ID + message type; priorities (faults and e-stop highest)
  - [ ] Messages: command (mode + setpoint), telemetry (position, velocity, current, temperature, voltage, status), heartbeat, fault report, parameter get/set, calibration request, version
  - [ ] Units and scaling (e.g. position in 0.01°), byte order, struct packing
  - [ ] Protocol version field; reject mismatched versions
  - [ ] Pack/unpack functions + host unit tests (#37)
  - **Done when:** the same header compiles into firmware and a PC test, and round-trip tests pass

### Joint firmware: `firmware/joint/` [G431/C]
- [ ] **#1** CubeMX init: 170 MHz clock, TIM1, ADC1/ADC2 injected, SPI/SSI, FDCAN1, UART, IWDG, GPIO
- [ ] **#2 PWM driver**
  - [ ] Center-aligned complementary PWM at 20 kHz, dead-time set for the ESC1's MOSFETs
  - [ ] TIM1 triggers the injected ADC at the PWM center
  - [ ] Duty-cycle API + clamping
  - **Done when:** scope shows correct dead-time and the ADC trigger in the right place
- [ ] **#3 Current sensing**
  - [ ] Injected conversions for phase currents + bus voltage
  - [ ] Offset calibration at boot with PWM off
  - [ ] Scaling to amps using the ESC1's shunt and op-amp gain
  - **Done when:** measured current matches the bench supply reading within a few %
- [ ] **#4 Encoder driver**
  - [ ] Read the MT6701 (SSI or SPI), CRC/parity check, error counting
  - [ ] Multi-turn counting; output-side angle via gear ratio
  - [ ] Velocity estimate (filtered difference or PLL)
  - **Done when:** no glitches over 10 minutes of spinning; velocity is smooth at low speed
- [ ] **#5 Motor calibration**
  - [ ] Pole-pair detection, direction check, electrical offset
  - [ ] Optional: phase resistance and inductance measurement
  - [ ] Store in flash (#13); skip calibration on boot when valid
  - **Done when:** calibration is repeatable and survives a power cycle
- [ ] **#6 FOC math**
  - [ ] Clarke, Park, inverse Park, SVPWM in a separate C file with no HAL calls
  - [ ] Fast sin/cos (lookup table or CORDIC; the G4 has a CORDIC peripheral)
  - [ ] Host unit tests (#37) against known values
  - **Done when:** tests pass on the PC and the code runs within the interrupt time budget
- [ ] **#7 Current loop**
  - [ ] d and q PI controllers with anti-windup
  - [ ] Voltage-vector limit (circle limit) based on bus voltage
  - [ ] Gains from measured R and L (bandwidth target about 1–2 kHz)
  - **Done when:** i_q step rise time and overshoot are measured and plotted; runs every PWM cycle
- [ ] **#8 Velocity + position loops**
  - [ ] Velocity PI at 1–5 kHz, position P or PD around it
  - [ ] Trapezoidal motion profile (max velocity and acceleration)
  - [ ] Gear ratio handling, joint limits, soft stops
  - [ ] Backlash measurement + optional compensation
  - **Done when:** position step response is measured; tracks a trajectory within a stated error
- [ ] **#9 Torque / compliance mode**
  - [ ] Torque command → i_q via the torque constant × gear ratio
  - [ ] Simple impedance mode: stiffness + damping around a setpoint
  - **Done when:** the joint feels like a spring when pushed by hand, and holds a set torque
- [ ] **#10 Fault and safety manager**
  - [ ] Faults: overcurrent (software + comparator if available), over/undervoltage, overtemperature (NTC), encoder error, CAN command timeout, position limit
  - [ ] Each fault has a defined response: disable gate driver (safe-torque-off), or brake and hold
  - [ ] Latched faults need an explicit clear command
  - [ ] IWDG fed only when the control loop is healthy
  - **Done when:** every fault is triggered on purpose (#38) and gives the documented response
- [ ] **#11 State machine**: IDLE → CALIBRATING → ARMED → RUNNING → FAULT; only legal transitions; state in telemetry
- [ ] **#12 CAN driver + protocol use**
  - [ ] FDCAN at 1 Mbit/s classic CAN; filters for this node's ID
  - [ ] Handle commands, parameter get/set, heartbeat; send telemetry at a set rate
  - [ ] Command-timeout feeds the fault manager
  - **Done when:** the PC can arm, command and read the joint entirely over CAN
- [ ] **#13 Parameter storage**: gains, offsets, limits, node ID in flash with CRC and version; defaults if invalid
- [ ] **#14 Debug telemetry**: high-rate stream (i_d, i_q, angle, setpoints) over UART for tuning plots
- [ ] **#15 Scheduler layout**: current loop in the ADC interrupt, slow loops in a timer interrupt, CAN and housekeeping in the main loop; document priorities
  - **Done when:** scope-measured interrupt time, CPU load and jitter are written down

### PC tools for Phase 1
- [ ] **#36 [PC/Py]** Tuning and plotting tool: read #14, plot step responses and current ripple, save CSVs
- [ ] **#37 [PC/C]** Host unit tests for FOC math, PI controllers, CAN pack/unpack, state machine (Unity or GoogleTest); run in CI
- [ ] **#38 [PC/Py]** Python test scripts over CAN: automated step-response test, fault-injection test, timeout test
- [ ] **#35 [PC/C++ or Py]** Fake joint on `vcan` that speaks #22 (needed for Phase 2, start here)

### Validation and portfolio
- [ ] **Torque test:** measure real continuous and peak torque with the luggage scale on the 30 cm bar. Update the BOM before buying more joints
- [ ] Thermal test: ESC1 temperature at continuous current
- [ ] **[DOC]** Design doc: architecture, loop rates, why bare-metal, CAN protocol summary
- [ ] **[DOC]** First FMEA: failure modes, effects, detection, safe state
- [ ] Video: step response, compliance by hand, fault injection (unplug CAN, joint goes safe)

---

## (Optional) Custom joint PCB (80–150 h + fab time)

- [ ] **[HW]** Schematic in KiCad: STM32G431, gate driver, MOSFETs, 3 shunts + amplifiers, CAN transceiver, encoder connector, NTC, SWD, power input protection
- [ ] **[HW]** Layout with attention to power loops and current-sense routing; Claude reviews before ordering
- [ ] **[HW]** Order from JLCPCB (assembled)
- [ ] **[G431/C]** Port the firmware via a board-config layer (pins, gains, shunt values)
- [ ] **[DOC]** Bring-up log: what failed and how it was found. Claim exactly "designed, fabricated and brought up"

---

## M3 — Phase 2: 2–3 DOF arm + ROS 2 (150–250 h)

### Hardware
- [ ] **[HW]** Build 2 more joints (shoulder yaw J1 and elbow J3); same firmware, different config
- [ ] **[HW]** CAN bus wiring with termination; power distribution with fuses
- [ ] **[HW]** Mast + shoulder mount at about door-handle height; gravity-spring mount point

### Firmware
- [ ] **[G431/C]** Per-joint config (node ID, gear ratio, limits) loaded from flash; set over CAN
- [ ] **[G431/C]** CAN bus load check at the full telemetry rate for all nodes

### Linux side
- [ ] **#25 [Linux]** URDF/xacro for the 3-joint arm (real link lengths, masses, joint limits)
- [ ] **#24 [Linux/C++] FK/IK library** (Eigen, its own package, no ROS dependency in the core)
  - [ ] Forward kinematics (DH or product of exponentials)
  - [ ] Jacobian
  - [ ] Numerical IK (damped least squares) with joint limits
  - [ ] Unit tests against the Robotics Toolbox for Python
  - **Done when:** FK/IK match the reference to <1 mm and solve in under 1 ms
- [ ] **#23 [Linux/C++] ros2_control hardware plugin**
  - [ ] SocketCAN read/write using #22
  - [ ] Position, velocity and effort command interfaces; state interfaces
  - [ ] Unit conversion; joint faults become ROS errors
  - [ ] Test against #35 on `vcan` first, then on real joints
  - **Done when:** `joint_trajectory_controller` moves the real arm, and a fake-joint fault stops it
- [ ] **#26 [Linux]** Controller YAML: `joint_trajectory_controller`, `joint_state_broadcaster`
- [ ] **#34 [PC]** Gazebo world + mock hardware; same launch files switch between mock, sim and real
- [ ] Test my FK/IK in RViz with `joint_state_publisher_gui`

### Validation
- [ ] Repeatability test: send the same pose 20 times, measure spread at the tip
- [ ] Video: sim and real arm side by side running the same trajectory
- [ ] **[DOC]** Checkpoint

---

## M4 — Phase 3: full 5-DOF arm + gripper + MoveIt (150–250 h)

### Hardware
- [ ] **[HW]** Wrist joints J4 (pitch) and J5 (roll) with 4108 motors and 5:1 reducers
- [ ] **[HW]** Gravity spring or counterweight on J2; tune it so the arm nearly floats
- [ ] **[HW]** Gripper: STS3215 + printed fingers with rubber pads; thin fingertips for pens
- [ ] **[HW]** USB-to-half-duplex serial adapter for the servo bus

### Code
- [ ] **#40 [Linux/C++] Gripper driver**: Feetech serial protocol (position, load, temperature), exposed through ros2_control as a gripper joint with a force (load) limit
- [ ] **#24** Extend FK/IK to 5-DOF: handle unreachable orientations (prioritize position + gripper direction)
- [ ] **#25** Full 5-DOF URDF + gripper
- [ ] **#27 [Linux]** MoveIt config via the Setup Assistant; tune planning; optional: my IK as a MoveIt kinematics plugin (C++)
- [ ] **#28 [Linux/C++] Safety supervisor node**
  - [ ] Watches all joint heartbeats and fault states
  - [ ] Software e-stop topic → all joints to safe state
  - [ ] Workspace limits (don't hit the base or the floor)
  - **Done when:** killing any joint's CAN link stops the whole arm safely
- [ ] **[G431/C]** Gravity feedforward: per-joint torque from the arm's pose (computed on Linux, sent with commands)

### Validation
- [ ] Pick-and-place an opaque cup with MoveIt (hard-coded positions are fine here)
- [ ] Payload test: lift 0.5 kg at full reach; check J2 and J4 temperatures
- [ ] Video: milestone 2 (arm picks a cup from a table and places it)
- [ ] **[DOC]** Updated FMEA + checkpoint

---

## M5 — Phase 4: mobile base (120–200 h)

### Hardware
- [ ] **[HW]** Strip a used hoverboard: hub motors, Hall sensors, wheels
- [ ] **[HW]** Chassis + casters; battery mounted low; mast mounted rigidly
- [ ] **[HW]** Power: 24 V pack + BMS, main fuse, e-stop button, contactor, 24→5 V buck
- [ ] **[HW]** Lidar (LD19) mounted with a clear 360° view

### Firmware
- [ ] **#17 [G431/C]** Hall-sensor support in the joint firmware (sensor type from config); Hall-based angle + interpolation
- [ ] **#18 [G431/C]** Wheel velocity mode + speed and acceleration limits
- [ ] **#19 [F411/C] Power node**: battery voltage and current monitor, e-stop input, contactor control, low-battery warning
- [ ] **#20 [F411/C]** Power node link: CAN using #22, or micro-ROS over USB (the one micro-ROS spot)
- [ ] **#21 [G431/C + F411/C]** Base command timeout → wheels stop; e-stop cuts the contactor

### Linux side
- [ ] **#29 [Linux]** `diff_drive_controller` through my ros2_control plugin (wheels are just two more CAN joints)
- [ ] Lidar driver; SLAM (slam_toolbox) to map the apartment
- [ ] Nav2 config: robot footprint including the arm, speed limits, inflation for narrow doorways
- [ ] Arm "stow" pose for driving

### Validation
- [ ] Drive from the table to the kitchen autonomously 10 times; count failures
- [ ] Threshold test: crosses every threshold on the route
- [ ] Traction test: pull force before the wheels slip (luggage scale)
- [ ] Video: milestone 3

---

## M6 — Phase 5: vision + fetch demo (150–250 h)

### Hardware
- [ ] **[HW]** Wrist mount for the NoIR camera, VL53L5CX and 850 nm IR LED

### Code
- [ ] **#41 [Linux/C or C++] VL53L5CX driver/node** (I2C; ST's ultra-lite driver as a base) → publishes the 8×8 range grid
- [ ] **#30 [Linux/Py or C++] Vision node**
  - [ ] Camera driver (v4l2 / libcamera) → ROS image topic
  - [ ] Cup detection (OpenCV first, then my own small CNN if needed)
  - [ ] Combine image direction with ToF range → 3D target position
- [ ] **#31 [Linux/Py] Hand-eye calibration** with a ChArUco board → camera-to-wrist transform; save to a file
- [ ] **#42 [Linux/C++] Visual-servoing approach**: keep the target centered, move in, stop on contact (joint current threshold)
- [ ] **#32 [Linux/C++ or Py] Fetch task logic** (state machine or behavior tree):
  drive to table → find cup → grasp → verify grasp (gripper load) → stow → drive to kitchen → place → return
- [ ] Failure handling: grasp failed → retry once → give up safely and report

### Validation
- [ ] 10 fetch attempts with the door open; record the success rate
- [ ] **Video: milestone 4** (fetch with the door already open). **This is the main target.** Label what's autonomous.
- [ ] **[DOC]** Checkpoint + post publicly

---

## M7 — Glass and door opening (150–300 h)

### Glass
- [ ] **#43 [Linux/Py] Transparent-object detection**: a small CNN trained on my own photos of glasses (depth sees through glass)
- [ ] Gentle grasp: gripper force limit + slip check through the servo load reading
- [ ] Video: milestone 5

### Door
- [ ] **[Linux/C++]** Handle detection: plane fit to the door + find the protrusion at handle height (ToF/camera); lever vs. knob
- [ ] **#44 [Linux/C++] Door-opening behavior**
  - [ ] Approach and grasp the handle
  - [ ] Unlatch: push the lever down (or twist the knob with J5) using joint torque mode
  - [ ] Swing: compliant arm (impedance mode #9) while the base drives along the door's arc
  - [ ] Detect "latched / unlatched / stuck" from joint torques; abort safely
- [ ] **[G431/C]** Tune impedance mode for contact with the door
- [ ] Test on a demo cabinet or light door first, then the real door
- [ ] Optional: do the door in complete darkness using the IR LED
- [ ] Video: milestone 6

---

## M8 — Final demo and polish (40–80 h)

- [ ] **#45 [PC/Py] Telemetry overlay tool**: turn logged joint currents, torques and states into overlays on the video
- [ ] Final video: door → glass → kitchen, with overlays and honest labels (autonomous, teleoperated, speed)
- [ ] Fault-injection clip: unplug CAN mid-motion, robot goes safe
- [ ] **[DOC]** Final README: block diagram, measured numbers, demo videos, lessons learned
- [ ] **[DOC]** Final FMEA and design docs

---

## Engineering-quality extras (do along the way)

- [ ] **[DOC]** FMEA kept current every phase
- [ ] Fault-injection tests automated in #38 and run before every release
- [ ] Scope-measured timing written down: interrupt latency, CPU load, jitter
- [ ] CI with hardware in the loop: one real joint on the bench runs #38 on every push (self-hosted runner)
- [ ] Protocol versioning enforced in firmware and the Linux plugin
- [ ] **#16 [G431/C] CAN bootloader**: update all joints without opening the robot
- [ ] Design docs for every major decision and its trade-offs

---

## File manifest: every file in the repo

Suggested names; rename freely, but keep one job per file. **Who** column: **Me** = I write it (Claude reviews) · **Gen** = generated by a tool, I only tune it · **BP** = boilerplate Claude may write · **Vendor** = third-party code I use, not write.

### `shared/can_protocol/`: one source of truth for the CAN protocol (#22, M2)

| File | Language | Runs on | Who | Purpose |
|---|---|---|---|---|
| `arm.dbc` | DBC (CAN database) | – | Me | **Source of truth**: every message, signal, scaling and unit. DBC is the industry standard, and Python reads it directly with `cantools`, so firmware, Linux and tests can't drift apart |
| `can_protocol.h` | C | G431, F411, Linux, PC tests | Me (or Gen from the DBC) | IDs, structs, version constant |
| `can_protocol.c` | C | same | Me | Pack/unpack functions |
| `tools/gen_can_header.py` | Python | PC | Me (optional) | Generates `can_protocol.h` from `arm.dbc` |

### `firmware/joint/`: G431 joint + wheel firmware, C (M1–M2, M5)

| File | Module | Who | Purpose |
|---|---|---|---|
| `Core/Src/main.c`, `stm32g4xx_it.c`, `*.ioc` | #1 | Gen (CubeMX) / BP | Clocks and peripheral init; calls `app_main()` |
| `app/app_main.c/.h` | #15 | Me | Startup order, main loop (CAN, housekeeping) |
| `app/isr.c` | #15 | Me | ADC interrupt → current loop; timer interrupt → slow loops |
| `app/board_config.h` | – | Me | Pins, shunt gain, dead-time, limits per board (ESC1 vs. custom PCB) |
| `app/pwm.c/.h` | #2 | Me | 3-phase center-aligned PWM, dead-time, ADC trigger |
| `app/current_sense.c/.h` | #3 | Me | Injected ADC, offsets, scaling to amps, bus voltage |
| `app/encoder_mt6701.c/.h` | #4 | Me | MT6701 read, CRC, multi-turn, velocity estimate |
| `app/hall.c/.h` | #17 | Me | Hall-sensor angle + interpolation for the hoverboard wheels |
| `app/calibration.c/.h` | #5 | Me | Pole pairs, direction, electrical offset, R/L |
| `app/foc_math.c/.h` | #6 | Me | Clarke, Park, inverse Park, SVPWM, sin/cos. **No HAL calls**, so it's host-testable |
| `app/pi_controller.c/.h` | #7, #8 | Me | Reusable PI with anti-windup and limits. **No HAL calls** |
| `app/current_loop.c/.h` | #7 | Me | d/q current control + voltage-circle limit |
| `app/trajectory.c/.h` | #8 | Me | Trapezoidal motion profile. **No HAL calls** |
| `app/motion_control.c/.h` | #8, #18 | Me | Velocity + position loops, gear ratio, limits, backlash, wheel mode |
| `app/torque_mode.c/.h` | #9 | Me | Torque command → i_q, impedance (stiffness + damping) |
| `app/faults.c/.h` | #10 | Me | Fault detection, responses, latching, watchdog feeding |
| `app/state_machine.c/.h` | #11 | Me | IDLE → CALIBRATING → ARMED → RUNNING → FAULT |
| `app/can_driver.c/.h` | #12 | Me | FDCAN setup, filters, TX/RX queues |
| `app/can_handler.c/.h` | #12 | Me | Dispatch received messages; schedule telemetry; command timeout |
| `app/params.c/.h` | #13 | Me | Flash storage of parameters with CRC and version |
| `app/telemetry_uart.c/.h` | #14 | Me | High-rate debug stream for tuning plots |
| `bootloader/can_bootloader.c` | #16 | Me | Receive firmware over CAN, verify, jump to app |
| `bootloader/flash.c/.h` | #16 | Me | Erase and write flash pages |
| `bootloader/bootloader.ld`, `app.ld` | #16 | Me | Split flash between bootloader and app |
| `Makefile` or `CMakeLists.txt` | – | BP | Build, flash, size report |

### `firmware/power_node/`: F411 Blackpill, C (M5)

| File | Module | Who | Purpose |
|---|---|---|---|
| `Core/Src/main.c`, `*.ioc` | – | Gen / BP | Init |
| `app/battery_monitor.c/.h` | #19 | Me | Pack voltage and current via ADC, low-battery levels |
| `app/estop.c/.h` | #19, #21 | Me | E-stop input (debounced), broadcast e-stop on CAN |
| `app/contactor.c/.h` | #19 | Me | Main power on/off, pre-charge if needed |
| `app/power_state.c/.h` | #19 | Me | OFF → PRECHARGE → ON → FAULT state machine |
| `app/can_link.c/.h` | #20 | Me | CAN using `can_protocol.h` (or `micro_ros_link.c` for micro-ROS over USB) |

### `tests/host/`: firmware logic tested on the PC, C (#37, M2 onward)

| File | Language | Who | Tests |
|---|---|---|---|
| `CMakeLists.txt` | CMake | BP | Builds the no-HAL firmware files + Unity |
| `test_foc_math.c` | C | Me | Transforms and SVPWM against known values |
| `test_pi_controller.c` | C | Me | Anti-windup, limits, step behavior |
| `test_trajectory.c` | C | Me | Profile hits limits and the end point |
| `test_can_protocol.c` | C | Me | Pack/unpack round trips, version mismatch |
| `test_state_machine.c` | C | Me | Only legal transitions allowed |
| `test_faults.c` | C | Me | Each fault → correct response, latching |

### `ros2_ws/src/`: Linux board, ROS 2 Jazzy (M3–M7)

| Package / file | Language | Module | Who | Purpose |
|---|---|---|---|---|
| **`arm_kinematics/`** (plain C++ library, no ROS in the core) | | #24 | | |
| `include/arm_kinematics/kinematics.hpp` | C++ | #24 | Me | Public API |
| `src/forward_kinematics.cpp` | C++ | #24 | Me | FK (DH or product of exponentials) |
| `src/jacobian.cpp` | C++ | #24 | Me | Jacobian |
| `src/inverse_kinematics.cpp` | C++ | #24 | Me | Damped least-squares IK, 5-DOF handling, joint limits |
| `test/test_kinematics.cpp` | C++ (GoogleTest) | #24 | Me | Compare with Robotics Toolbox results |
| `CMakeLists.txt`, `package.xml` | CMake/XML | – | BP | |
| **`arm_hardware/`** (ros2_control plugin) | | #23, #40 | | |
| `include/arm_hardware/can_arm_system.hpp` | C++ | #23 | Me | Plugin class |
| `src/can_arm_system.cpp` | C++ | #23 | Me | on_init / read / write / activate; faults → errors |
| `src/socketcan.cpp/.hpp` | C++ | #23 | Me | Raw SocketCAN wrapper |
| `src/feetech_bus.cpp/.hpp` | C++ | #40 | Me | STS3215 serial protocol |
| `src/gripper_interface.cpp` | C++ | #40 | Me | Gripper as a ros2_control joint with a force limit |
| `arm_hardware.xml` | XML | – | BP | pluginlib export |
| `test/test_can_arm_system.cpp` | C++ | #23 | Me | Against the fake joint on `vcan` |
| **`arm_description/`** | | #25, #26 | | |
| `urdf/arm.urdf.xacro` | xacro | #25 | Me | 5-DOF arm + gripper |
| `urdf/base.urdf.xacro` | xacro | #25 | Me | Hoverboard base, mast, lidar, camera |
| `urdf/robot.ros2_control.xacro` | xacro | #25 | Me | Selects mock / Gazebo / real hardware |
| `config/controllers.yaml` | YAML | #26 | Me | joint_trajectory, gripper, diff_drive, joint_state_broadcaster |
| `launch/view_robot.launch.py` | Python | – | BP | RViz + sliders |
| **`arm_moveit_config/`** | | #27 | | |
| (whole package) | YAML/XML | #27 | Gen (Setup Assistant) | Then I tune `kinematics.yaml`, `joint_limits.yaml`, SRDF |
| `src/my_ik_plugin.cpp` (optional) | C++ | #27 | Me | My IK as a MoveIt kinematics plugin |
| **`arm_gazebo/`** | | #34 | | |
| `worlds/apartment.sdf` | SDF | #34 | Me | Rough apartment: table, counter, door |
| `launch/sim.launch.py` | Python | #34 | BP | |
| **`arm_safety/`** | | #28 | | |
| `src/safety_supervisor.cpp` | C++ | #28 | Me | Heartbeats, e-stop, workspace limits |
| **`arm_interfaces/`** | | – | | |
| `msg/JointFault.msg`, `msg/PowerStatus.msg`, `msg/GraspResult.msg` | ROS IDL | – | Me | Custom messages |
| **`tof_vl53l5cx/`** | | #41 | | |
| `src/vl53l5cx_node.cpp` | C++ | #41 | Me | I2C + publishes the 8×8 range grid |
| `vendor/VL53L5CX_ULD/` | C | #41 | Vendor (ST) | Sensor driver; check its license |
| **`arm_vision/`** | | #30, #31, #43 | | |
| `arm_vision/cup_detector_node.py` | Python | #30 | Me | OpenCV (later CNN) cup detection |
| `arm_vision/target_estimator_node.py` | Python | #30 | Me | Image direction + ToF range → 3D target |
| `arm_vision/glass_detector_node.py` | Python | #43 | Me | CNN for transparent glasses |
| `scripts/hand_eye_calibration.py` | Python | #31 | Me | ChArUco → camera-to-wrist transform |
| `scripts/train_glass_cnn.py` | Python | #43 | Me | Train on my own photos |
| (camera driver) | – | – | Vendor | Use the existing `v4l2_camera` or `camera_ros` package |
| **`arm_manipulation/`** | | #42, #44 | | |
| `src/visual_servo.cpp` | C++ | #42 | Me | Centered approach, stop on contact |
| `src/gravity_feedforward.cpp` | C++ | – | Me | Per-joint gravity torque from the current pose |
| `src/handle_detector.cpp` | C++ | #44 | Me | Door plane fit + handle protrusion |
| `src/door_opening.cpp` | C++ | #44 | Me | Unlatch, compliant swing, stuck detection |
| **`arm_tasks/`** | | #32 | | |
| `src/fetch_task.cpp` | C++ (BehaviorTree.CPP) | #32 | Me | Fetch sequence + failure handling |
| `trees/fetch.xml` | XML | #32 | Me | Behavior tree |
| **`arm_bringup/`** | | #29, #33 | | |
| `launch/bringup.launch.py` | Python | #33 | Me | One launch with `mode:=mock|sim|real` |
| `config/nav2_params.yaml` | YAML | #29 | Me | Footprint, speeds, inflation |
| `config/slam.yaml` | YAML | #29 | Me | slam_toolbox settings |
| `systemd/robot.service` | systemd unit | #33 | Me | Start on boot |

### `tools/`: PC scripts, Python (M2 onward)

| File | Module | Who | Purpose |
|---|---|---|---|
| `tune_plot.py` | #36 | Me | Read UART telemetry; plot step responses, ripple; save CSV |
| `fake_joint.py` | #35 | Me | Pretends to be a joint on `vcan`, using `arm.dbc` via cantools |
| `can_tests/conftest.py` | #38 | Me | pytest setup: open CAN bus, arm/disarm joints |
| `can_tests/test_step_response.py` | #38 | Me | Automated step test with pass/fail limits |
| `can_tests/test_fault_injection.py` | #38 | Me | Trigger each fault, check the response |
| `can_tests/test_timeout.py` | #38 | Me | Stop sending commands → joint goes safe |
| `torque_test_log.py` | – | Me | Log current vs. luggage-scale readings for the torque chart |
| `telemetry_overlay.py` | #45 | Me | Logged data → video overlays |

### Repo-level files

| File | Language | Who | Purpose |
|---|---|---|---|
| `.github/workflows/ci.yml` | YAML | Me | Build firmware, run host tests, build ROS packages |
| `README.md` | Markdown | Me | Goal, block diagram, videos, numbers |
| `docs/design.md` | Markdown | Me | Architecture and trade-offs |
| `docs/can_protocol.md` | Markdown | Me | Human-readable version of `arm.dbc` |
| `docs/fmea.md` | Markdown | Me | Failure modes, effects, detection, safe state |
| `docs/timing.md` | Markdown | Me | Measured interrupt time, CPU load, jitter |
| `docs/measurements.md` | Markdown | Me | Apartment measurements, torque and thermal results |
| `docs/CHECKPOINTS.md` | Markdown | Me | Dated progress log |
| `hardware/` | Onshape exports, KiCad | Me | Not code: CAD, gearboxes, wiring diagram, PCB |

**Counts (rough):**
- About **75 files I write**: about 35 C, about 20 C++, about 15 Python, plus URDF/YAML/DBC/docs.
- Plus generated or boilerplate files I only tune.

## Side track: Rust practice (`rust-practice/`)

**Rust practice only. Doesn't block the quad or the arm.** Write my version first, then compare with BananaHand. Don't copy.

- [ ] **1 [PC/Rust]** Packet library: CRC-16, sequence IDs (`crates/protocol/src/packet.rs`)
- [ ] **2 [PC/Rust]** COBS encoder/decoder + tests on corrupted frames (`crates/protocol/src/cobs.rs`)
- [ ] **3 [PC/Rust]** PID with anti-windup + test against a simulated motor (`crates/control/src/pid.rs`)
- [ ] **4 [F446/Rust]** Pin map and constants (`firmware/src/board.rs`)
- [ ] **5 [F446/Rust + PC/Py]** Async UART commands + telemetry (`firmware/src/comms.rs`, `tools/serial_link.py`)
- [ ] **6 [F446/Rust]** Shared sensor data between tasks (`firmware/src/shared.rs`)
- [ ] **7 [F446/Rust]** Fixed-rate ADC sampling of a potentiometer (`firmware/src/adc.rs`)
- [ ] **8 [PC + F446/Rust]** `Actuator` trait + mock + PWM version, LED first (`crates/control/src/actuator.rs`, `firmware/src/actuator.rs`)
- [ ] **9 [F446/Rust]** Control task + command-timeout fail-safe (`firmware/src/control_task.rs`)
