# Mobile 5-DOF Robot Arm ("Dum-E") — Session Handoff

_Handoff written 2026-10-27, updated 2026-11-07 (switched to 5-DOF on a mobile base, door-capable BOM). Paste this into a new Claude session on the PC to continue._

---

**Task list:** `robot-arm-todo.md` (same folder) is the detailed checklist of every module, task and "done when" criterion, milestone by milestone.

Personal notes are kept outside this repo, in an untracked `private-notes.md`.

## How to work with me (Claude: read this first)

- **Reviewer, not author.** For anything new to me (FOC, CAN protocol, FK/IK, safety logic, ros2_control plugin), I write the code and Claude reviews and teaches. Claude may only take over writing a kind of code once I've shown I can rewrite and defend it myself. Boilerplate (HAL/CubeMX init, launch file scaffolding) is fine for Claude to write.
- I learn visually, so use diagrams and block diagrams.
- Lessons should be hands-on with at most one comprehension question. I'll paste output back.
- Always link the official docs.
- Never overclaim: only finished and working phases count as done.

---

## Goal

Build a firmware-first mobile manipulator, with the motor control, CAN network, safety logic and kinematics all written from scratch.

Languages: C for joint firmware, modern C++ for high-level code (FK/IK, ros2_control plugin), Python for tools. Rust/Embassy is a later port only.

## Project order

Mini HIL rig → quadcopter (C + FreeRTOS, Radxa companion) → **this arm**.

## Phases (each is a standalone project)

1. **One BLDC joint actuator:** my own FOC current, velocity and position loops, magnetic encoder, CAN commands and telemetry, fault handling and safe-torque-off. **This is the most valuable phase** and carries most of the firmware value on its own.
2. **2–3 DOF arm:** multi-node CAN, my own FK/IK in C++.
3. **Full 5-DOF arm + ROS 2:** MoveIt on top is fine. (Decided 2026-11-07: 5-DOF, not 6. The mobile base supplies the missing freedom.)
4. **Mobile base:** odometry, Nav2.
5. **Vision-guided pick:** reuse my Kinova and CNN experience. See "Vision and door opening" below.

**Priority:** Phase 1 done to a high standard is the core project. Phases 2–5 build toward the final demo (below), but if time runs short, a finished Phase 1 still matters most.

Optional but recommended: a **custom joint-controller PCB** in KiCad (STM32G4, gate driver, current sensing, CAN). It closes my PCB gap.

## Design decisions so far

- **Cheap and firmware-first, but strong enough for the final demo** (open a door, fetch a glass). 3D-printed links and cycloidal/planetary gearboxes, BLDC motors sized per joint (see BOM), 24 V. Reach and mount height are set after the apartment measurements. _(Earlier plan was a weak 100–300 g arm with small gimbal motors; that can't open doors, so it was replaced.)_
- **5-DOF arm on a mobile base:**
  - J1 shoulder yaw, J2 shoulder pitch, J3 elbow, J4 wrist pitch, J5 wrist roll; gripper separate.
  - Wrist roll is what lets it turn lever handles and knobs, so 4-DOF is too few.
  - 6-DOF adds a joint for little gain on a mobile base: same firmware, more cost and wrist weight. Design the wrist so a 6th joint could be added later.
  - IK note: 5-DOF means handling poses the arm can't reach exactly (prioritize position plus gripper direction). That's a good IK problem.
  - Possible alternative after measuring: a Stretch-style vertical lift on the mast plus a shorter arm, which keeps torques low.
- **Don't cut:**
  - real BLDC motors with phase-current sensing on every joint (no hobby servos, which hide the motor control)
  - a decent encoder (AS5047P or MT6701, not the AS5600)
  - CAN on every joint
  - a current-limited bench power supply
- **Driver board to start:** B-G431B-ESC1 (about $20, has current sensing and CAN). I write my own FOC on it, then possibly move to the custom PCB.
- **Architecture:**
  ```
  Linux board (Radxa reused / Pi 5): ROS 2 + MoveIt + Nav2 + vision
      └─ ros2_control hardware interface (my C++), talks CAN
           │ CAN bus (my own protocol)
     STM32 nodes, same FOC firmware: 5 arm joints + 2 hoverboard wheel motors
     (FOC 10–20 kHz, encoder/Hall, safety). No ROS on the MCUs.
     + small STM32 power node: battery monitor, e-stop, main contactor
  ```
  - Full ROS 2 needs Linux. An STM32 can only run micro-ROS, which needs an agent on Linux.
  - Use micro-ROS in at most one spot (base or sensor board), just to have used it.
- **Backlash and flex** in the printed gearboxes are a feature for the portfolio: measure them, tune around them, and write up what I learned.

## BOM and cost estimate: 5-DOF arm + mobile base (updated 2026-11-07)

Rough USD from typical AliExpress/hobby prices, **not current quotes**. CAD is about ×1.4 plus shipping and duties. Motor sizes are **preliminary**: finalize after the apartment measurements and a torque calculation.

**Preliminary torque sizing.** Assumes about 55 cm reach, about 300 g glass, about 300 g gripper.
- Shoulder pitch holds roughly 9–12 N·m against gravity, more during door pulls. A gravity-compensation spring cuts that a lot.
- The base, not the arm, does most of the door pulling.

### Torque check: can it open doors and lift glasses, mugs, pens? (2026-11-07)

**Assumptions** (typical figures; "8308"/"4108" vary by brand, so check the datasheet of the exact motor):
- 8308: about 0.7 N·m continuous, 1.5 N·m peak. With 9:1 cycloidal at about 80% efficiency: **about 5 N·m continuous, 11 N·m peak** at the joint.
- 4108: about 0.2 / 0.4 N·m. With 5:1: **about 1 / 2 N·m**.
- About 55 cm reach, about 300 g gripper, about 500 g of wrist motors and gearboxes.

| Joint | Load (rough) | Capacity (cont / peak) | Verdict |
|---|---|---|---|
| J2 shoulder pitch, arm extended with a glass | about 9–12 N·m | 5 / 11 N·m | ❌ alone. ✅ with a **gravity spring** taking 60–70% (leaves about 3–4 N·m) |
| J3 elbow | about 3–4 N·m | 5 / 11 N·m | ✅ |
| J4 wrist pitch, gripper + glass (0.6 kg at 10 cm) | about 0.6 N·m | 1 / 2 N·m | ✅ |
| J4 with a full mug (about 0.5 kg) | about 0.9–1 N·m | 1 / 2 N·m | ⚠️ marginal for long holds |
| J5 wrist roll, twisting a knob | about 0.5–1.5 N·m | 1 / 2 N·m | ✅ most knobs; levers are easier |
| Gripper (STS3215) | firm, non-slip grip | roughly 50–60 N at the fingertips | ✅ with rubber pads; force limit for glass |

**By task:**
- **Glasses, empty mugs, pens, phones, remotes: yes,** under about 0.5 kg at full reach.
- **Pens are a precision problem.** Printed-gearbox backlash (about 0.1–0.5° per joint) adds up to about 1 cm at the gripper, so use the wrist camera for the last few cm (visual servoing) and design thin fingertips.
- **Pushing a lever down:** easy, gravity helps.
- **Pulling the door open (20–50 N): the base does the pulling,** and the arm just holds the handle.
  - Mount the shoulder at about handle height (about 0.9–1 m), so pulls barely load it.
  - Keep the base heavy: at 10–15 kg on rubber tires it can pull about 50–75 N before slipping. Mount the battery low, or add ballast.
- **Doors with strong closers** (apartment fire doors, often 60+ N) are probably out of scope. Measure them.

**Conditions this BOM depends on:**
1. A gravity spring or counterweight on the shoulder (required).
2. Shoulder at about handle height on the mast.
3. Payload under about 0.5 kg at full reach.
4. No strong-closer doors; a heavy, low base.
5. Check ESC1 heat at the 8308 joints' working current.

**Rejected alternative:** a 15–20:1 shoulder instead of a spring. It's harder to back-drive, which spoils current-based force sensing, and doors need that sensing.

**Phase 1 validation:** clamp the first 8308 joint, attach a 30 cm bar, and measure real continuous and peak torque with the luggage scale. Replace these estimates with measured numbers **before buying the other joints**. It's also a good portfolio chart.

### Arm

| Item | Qty | Each | Total |
|---|---|---|---|
| J2 shoulder pitch, J3 elbow: 8308-class BLDC + 8–10:1 printed cycloidal | 2 | $50–65 | $100–130 |
| J1 shoulder yaw: 5010/5208-class BLDC + 6:1 printed reducer | 1 | $25–35 | $25–35 |
| J4 wrist pitch, J5 wrist roll: 4108-class BLDC + 5:1 printed reducer | 2 | $20–30 | $40–60 |
| B-G431B-ESC1 driver (current sensing + CAN) | 5 | $18–25 | $90–125 |
| MT6701 magnetic encoder board (cheaper than AS5047P, good enough) | 5 | $3–6 | $15–30 |
| Bearings, dowel pins, bolts for gearboxes and joints | – | – | $60–100 |
| Gripper: STS3215 serial-bus servo + printed fingers. A servo is fine here: the gripper isn't the firmware showcase | 1 | $15–20 | $15–25 |
| Shoulder gravity-compensation spring | 1 | – | $10 |
| Mast: 2040 aluminum extrusion + brackets | – | – | $30–50 |
| **Arm subtotal** | | | **$385–565** |

### Mobile base

| Item | Cost |
|---|---|
| **Used hoverboard** (Facebook Marketplace) for its two hub motors with Hall sensors and 6.5" wheels. These handle thresholds and can carry the robot. | $20–50 |
| 2 more B-G431B-ESC1 to run the hub motors with **my own FOC** (same firmware as the joints, plus Hall sensor support) at 24 V, so slower than a hoverboard, which suits indoors | $40–50 |
| 24 V (6S) Li-ion pack with BMS. The hoverboard's 36 V pack is above the ESC1's voltage rating, so don't reuse it for the motors | $60–100 |
| Chassis (plywood or extrusion), 2 casters | $30–50 |
| Power: e-stop button, fuse, main switch/contactor, 24→5 V buck for the Linux board | $30–50 |
| Small STM32 power node (Blackpill) | $5–10 |
| **Base subtotal** | **$185–310** |

### Compute, sensors, wiring

| Item | Cost |
|---|---|
| Linux board: **start at $0** (Radxa from the quad, with MoveIt, Nav2 and heavy vision on the PC over Wi-Fi). Final: Raspberry Pi 5 8 GB, because the Radxa Zero 3W is too weak to run MoveIt, Nav2 and OpenCV together | $0–80 |
| 2D lidar: LD19/LD06 (cheaper than RPLidar) | $40–70 |
| Wrist NoIR camera + VL53L5CX ToF + 850 nm IR LED | $30–50 |
| USB-CAN adapter (CANable clone) | $15–30 |
| Wiring: XT30 power connectors, JST-GH, twisted-pair CAN, termination | $30–40 |
| **Subtotal** | **$115–270** |

### Consumables and spares

| Item | Cost |
|---|---|
| 3–4 kg PETG/PLA+ filament | $60–80 |
| Spares: 1–2 extra ESC1, burned-MOSFET buffer | $40–80 |
| Luggage scale for measurements | $10 |
| **Subtotal** | **$110–170** |

### Total

| | USD | CAD (×1.4) |
|---|---|---|
| **Full robot** | **about $800–1,300** | **about $1,100–1,850** |
| Phase 1 only: one 8308 joint + ESC1 + MT6701 + bearings + CANable | about $110–150 | about $155–210 |
| FOC learning on the spare quad motor: ESC1 + MT6701 + magnet | about $25–35 | about $35–50 |

**Not included:**
- 3D printer: about $250–300 if I don't have one; the Waterloo makerspace may be free.
- Current-limited bench power supply: $60–150 if I don't have one.
- Custom joint PCB (optional): +$100–300.
- Glass detection upgrade: a used RealSense, about $100–150.
- Stronger motors if the measurements show door forces above estimate.

### Ways to save cost (already applied above)

1. A used hoverboard for the base: about $30 gets wheels, hub motors and Hall sensors. It also gives 2 extra FOC nodes on the same firmware, which strengthens the portfolio.
2. MT6701 instead of AS5047P encoders.
3. A servo gripper.
4. A gravity spring at the shoulder, so a smaller shoulder motor is enough.
5. Reuse the Radxa and run heavy planning on the PC at first; buy the Pi 5 last.
6. LD19 lidar instead of RPLidar.
7. CANable clone.
8. Buy in phases: Phase 1 costs about $120, then buy joints as each phase starts.
9. Check used marketplaces for 8308 motors and 24 V tool or e-bike batteries.
10. Print at the makerspace if I don't own a printer.

Things that could still change cost:
- doors needing more force than estimated
- choosing a vertical lift instead of a longer arm (adds a lead screw or belt, about $40–80, but shrinks the shoulder and elbow motors)
- the ESC1's continuous current limit on the 8308 joints (check thermals during Phase 1)

## Learning FOC early: spare quad motor

Using my spare quad motor to learn FOC is fine. It needs:
- a B-G431B-ESC1 (my quad ESC can't do it: it runs BLHeli/AM32 and doesn't sense per-phase current)
- a diametric magnet on the shaft, with an AS5047P or MT6701 encoder mounted 1–2 mm below on a printed bracket
- a bench power supply at about 8–12 V with a 1–2 A limit
- voltage and current limits in software too, and no propeller fitted

High KV means very low resistance and inductance. That's good for learning the current and velocity loops, but the motor is weak at position holding (the gimbal motors handle that).

Order:
1. Open-loop space-vector PWM.
2. Encoder and electrical-angle alignment.
3. Phase currents, then the Clarke and Park transforms.
4. d/q current PI loop.
5. Velocity loop.
6. Position loop.

**Open question:** which quad motor and KV is the spare? That sets the starting voltage and current limits.

## Reference project: BananaHand

https://github.com/bananaHand-project/bananaHand

- What it is: an open-source 16-DOF humanoid hand, about $900 BOM.
  - STM32G474 (control) + STM32C071 (force sensing)
  - Rust/Embassy, H-bridge drive, position and force PID at 200 Hz
  - UART with COBS framing; CAN still on their TODO list
  - KiCad PCBs, ROS and Foxglove
- Software side: **ROS 2 Humble on a host computer, no ROS on the MCUs.** Python nodes: serial bridge (plus a Kinova Kortex version), custom messages (`HandState.msg`, `GraspRecommendation.msg`), webcam + MediaPipe hand-tracking teleop, MuJoCo and Foxglove visualization, grasp classification from object scans. These roles are inferred from package names; I haven't read the node code.
- Where my arm differs: a `ros2_control` hardware plugin in C++ over CAN instead of a custom serial bridge (so MoveIt and controllers work in both sim and real), and ROS 2 Jazzy instead of Humble.
- Their firmware README lists known gaps: no CAN/RS485, no sequence IDs or ACK/NACK, no host heartbeat or command-timeout fail-safe, protocol not shared between firmware and the ROS bridge, no calibration procedure, no current-based fault protection.
- What to learn from it: system structure, the two-MCU split, COBS framing, Foxglove telemetry, how the repo is organized.
- What it doesn't teach: FOC. For FOC, use ODrive, SimpleFOC and moteus as references only.
- **No license was visible**, so I can learn from it but not copy code or CAD.
- Possible side quest: they want contributors and CAN is on their TODO list. A merged CAN pull request would be a useful contribution.
- For my goal (robotics firmware), my plan is the stronger project once it's built, because of FOC, CAN, safety and solo ownership.

## Every module I have to write

★ = highest firmware value. Core modules: #2–8, 10–12, 22, 23, 24.

**Joint node: STM32G431 (B-G431B-ESC1, later custom PCB), C (C++ OK).** One firmware image for all 7 motor nodes (5 arm joints + 2 hoverboard wheels); node ID, gear ratio, sensor type and limits come from config.
1. Peripheral init (CubeMX boilerplate)
2. ★ Center-aligned 3-phase PWM with dead-time, triggering the ADC
3. ★ Phase-current sensing: injected ADC synced to PWM, offset calibration
4. ★ Encoder driver (AS5047P/MT6701): parity/CRC, turn counting, velocity estimate
5. ★ Motor calibration: electrical offset, pole pairs, direction (optionally R and L); saved to flash
6. ★ FOC math: Clarke, Park, inverse Park, SVPWM
7. ★ d/q current PI loop with anti-windup, 10–20 kHz in the ADC interrupt
8. ★ Velocity + position loops at 1–5 kHz, trapezoidal profile, gear ratio, backlash compensation
9. Torque / compliance mode
10. ★ Fault and safety manager: overcurrent, over/undervoltage, overtemperature, encoder fault, CAN timeout → safe-torque-off; IWDG
11. ★ State machine: IDLE → CALIBRATING → ARMED → RUNNING → FAULT
12. ★ FDCAN driver + protocol: commands, telemetry, heartbeat, parameters, faults
13. Parameter storage in flash with CRC
14. Debug telemetry streaming over UART/SWO for tuning plots
15. Scheduler layout: bare-metal, current loop in ADC interrupt, slower loops in a timer interrupt
16. CAN bootloader (optional, phase 3+)

**Mobile base: wheel nodes reuse the joint firmware; a separate STM32 power node (Blackpill), C.**
17. Hall-sensor support in the joint firmware (hoverboard hub motors) · 18. Wheel velocity mode and limits in the joint firmware · 19. Power node: battery voltage/current monitor, e-stop input, main contactor control · 20. Power node link to Linux: CAN, or micro-ROS over USB (the one micro-ROS spot) · 21. Base command timeout → stop wheels

**Shared C header + library:** 22. ★ CAN protocol definition (IDs, payloads, units, version), used by both firmware and Linux.

**Linux board: ROS 2 Jazzy, C++ (Python where noted).**
23. ★ ros2_control hardware plugin over SocketCAN · 24. ★ My own FK/IK library (Eigen) · 25. URDF/xacro · 26. joint_trajectory_controller config · 27. MoveIt config (optional IK plugin) · 28. Safety supervisor node · 29. diff_drive / odometry + Nav2 config · 30. Vision node · 31. Hand-eye calibration (Python) · 32. Pick-and-place task logic · 33. Launch files + systemd service

**PC (dev/sim).**
34. Gazebo / mock hardware setup · 35. ★ Fake joint on `vcan` · 36. Tuning and plotting tool (Python) · 37. Host unit tests for FOC math and CAN encoding · 38. Python test scripts over CAN (step response, fault injection) · 39. Foxglove dashboards

Used, not written: MoveIt, Nav2, Gazebo, sensor drivers, Eigen, FreeRTOS, STM32 HAL.

## Rust practice project: `rust-practice/`

Side project to practice Rust/Embassy on the Nucleo F446RE, using BananaHand's firmware as a reference. **This is Rust practice, not arm firmware** (the arm stays C/C++), so it must not delay the quad. Rule: write my version first, then compare with BananaHand's. Read only, don't copy (no license found).

Scaffold created 2026-11-04 by Claude (boilerplate only): firmware builds for the F446RE, `cargo test` passes, nothing flashed or committed yet. Embassy now uses the `platform-cortex-m` executor feature, not `arch-cortex-m`.

| # | Module | File | BananaHand reference |
|---|---|---|---|
| 1 | Packet library: CRC-16, sequence IDs | `crates/protocol/src/packet.rs` | `common/src/force_data_packet.rs` |
| 2 | COBS encoder/decoder | `crates/protocol/src/cobs.rs` | `primary/src/protocol.rs` |
| 3 | PID with anti-windup | `crates/control/src/pid.rs` | `primary/src/pid.rs` |
| 4 | Pin map, constants | `firmware/src/board.rs` | `board_map.rs`, `config.rs` |
| 5 | Async UART commands + telemetry | `firmware/src/comms.rs`, `tools/serial_link.py` | `command_reader.rs`, `telemetry_sender.rs` |
| 6 | Shared sensor data between tasks | `firmware/src/shared.rs` | `shared.rs` |
| 7 | Fixed-rate ADC sampling | `firmware/src/adc.rs` | `position_reader.rs` |
| 8 | `Actuator` trait + mock / PWM version | `crates/control/src/actuator.rs`, `firmware/src/actuator.rs` | `pq12p_actuator/src/lib.rs` |
| 9 | Control task + command-timeout fail-safe | `firmware/src/control_task.rs` | `motor_control.rs` |

- Commands: `cargo test` in `rust-practice/` (modules 1–3), `cargo run --release` in `rust-practice/firmware/`.
- Hardware for module 8: the trait is hardware-neutral (signed effort -1.0 to 1.0). Start with an LED on PWM. Later: N20/TT DC gearmotor + TB6612FNG or DRV8833 + potentiometer, about $15–25. The quad motor won't work with an H-bridge.
- The arm's Phase 1 BOM is a separate decision.

## Vision and door opening

- **Architecture:** sensor → USB → Linux board running ROS 2 + OpenCV. A depth camera doesn't replace the Linux board or OpenCV; it's just a different sensor plugged into it.
- **Chosen setup (budget):** a wrist-mounted IR-sensitive camera (NoIR IMX219 or a USB camera without an IR-cut filter) + VL53L5CX 8×8 ToF + 850 nm IR LED, about $30–50, with OpenCV on the Linux board. Same pattern as the quadcopter's camera pipeline.
- Ways to get depth from one camera: visual servoing until FOC current detects contact (start here), using two wrist positions as a stereo pair via my own FK, a line laser, a ToF sensor, or ArUco tags with `solvePnP`. Skip monocular depth networks.
- **Upgrade path if needed:** a used RealSense D435 (about $100–150) or OAK-D Pro (about $300–400). Least code, works in the dark with an IR projector. Check minimum range for wrist use.
- **Don't build my own two-camera stereo for the arm:** it needs sync, calibration, CPU-heavy matching and an IR projector for blank door surfaces, and the OV9281s are wired as DVP for the FPGA, not for a Linux board.
- **How commercial arms do it:** an off-the-shelf depth camera, usually on the wrist (Kinova Gen3 vision module, Spot Arm gripper camera, Stretch, add-on RealSense on Franka/UR), plus torque sensing (Kinova and Franka have joint torque sensors, UR a wrist F/T sensor). Lidar is for navigation only. Optimus and Figure use cameras plus neural networks. My plan uses FOC motor current as the torque estimate (proprioceptive actuation, like MIT Mini Cheetah), which suits low gear ratios.
- **Opening doors (in darkness as a bonus) is now part of the final demo,** and the BOM above is sized for it:
  - 2D lidar to reach the door, active depth/IR camera to find the handle (a plane with a 5–8 cm protrusion at about 0.9–1.1 m), force control to open.
  - Real doors need about 1–3 N·m at the wrist and 20–50 N of pull. That's why the BOM uses 8308-class shoulder and elbow motors, wrist roll and a heavy base.
  - Middle step: a demo door or cabinet with a light lever handle.
  - Ties to Vision AI 2.0's IR stereo work as a story.
- **Glass is transparent,** so depth cameras and ToF often see straight through it. Detect it in the camera image (e.g. my own CNN), confirm with contact sensing, and grip gently. Use an opaque plastic cup first.

## Final demo goal (decided 2026-11-07)

**The robot opens a door, picks a glass off a table and brings it to the kitchen counter.** A 10-second story anyone understands.

**Rules for the video:**
- **Make the firmware visible:** overlay live joint current and torque, show the arm giving way as the door swings, unplug a CAN cable mid-motion and show it going safely limp. The impressive parts (navigation, vision) are robotics software; the overlays are what show firmware skill.
- **Label honestly:** "autonomous", "teleoperated", "2× speed".
- **Phase 1 stays the core project.** This demo comes on top of it, not instead of it.

**Milestones, each with its own video:**
1. Actuator: FOC, step response, fault injection. Still the most valuable milestone.
2. Arm picks up an opaque cup from a table and places it elsewhere.
3. Base drives from the table to the kitchen using Nav2.
4. **Fetch demo with the door already open:** plastic cup, table to kitchen counter. Already very impressive; a good place to stop if time runs out.
5. A real glass instead of the cup: detecting a transparent object, gentle grip.
6. Opening the door: the final boss.

Estimated extra time over the base plan: about 6–12 months.

## Apartment measurements to take (needed before the BOM is final)

Tools: tape measure, luggage scale. Take a photo of everything.

**Every door on the route:**
- [ ] Lever or knob (a knob is much harder)
- [ ] Floor to handle center height
- [ ] Lever length, and rotation needed to unlatch (degrees)
- [ ] Unlatch force: luggage scale at the lever tip, pulling down
- [ ] Opening force: scale at the handle; starting force and force to keep it moving; is there a closer or spring?
- [ ] Push or pull from the robot's side, and which side the hinge is on
- [ ] Door width, handle distance from the door edge, free floor space beside the door
- [ ] Threshold height

**Table:** height, depth (edge to where the glass sits), can the base fit under the edge?

**Kitchen counter:** height, depth, where the glass will be placed.

**Glass:** weight, diameter, height. Also weigh the plastic stand-in cup.

**Route:** narrowest passage, tightest turn, rug edges and thresholds, floor type.

**After measuring:** send the numbers and photos to Claude to work out mast height, reach, lift vs. no lift, per-joint torque, then finalize motors, gear ratios and the BOM.

---

## NEXT: set up simulation on the PC (RTX 4060)

Simulate before assembling. The ros2_control **hardware plugin** is the only thing that changes between simulation and the real robot (mock → Gazebo → my CAN plugin). MoveIt, controllers and launch files stay the same.

### Environment: WSL2 + Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic

Reasons:
- A normal VM gives poor GPU access, so Gazebo falls back to software rendering.
- Dual-boot would take Windows offline.
- Fallback if WSL2 stutters or USB passthrough becomes a pain: dual-boot.

1. Install Ubuntu in WSL2 (PowerShell):
   ```powershell
   wsl --install -d Ubuntu-24.04
   ```
2. Keep the **Windows** NVIDIA driver updated. **Do not install an NVIDIA driver inside Ubuntu.**
3. Check that Ubuntu is using the GPU:
   ```bash
   sudo apt install mesa-utils && glxinfo -B
   ```
   - Expected: `D3D12 (NVIDIA GeForce RTX 4060)`.
   - If it shows `llvmpipe`, graphics are in software.
   - If it shows an integrated GPU, run `export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA` and check again.
4. Install ROS 2 Jazzy: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
5. Install Gazebo, the ROS bridge and MoveIt:
   ```bash
   sudo apt install ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-moveit
   ```
6. Smoke test: `gz sim shapes.sdf` should run smoothly.
7. Later: pass the CANable and ST-Link into WSL with usbipd-win: https://github.com/dorssel/usbipd-win

### Simulation roadmap

1. **URDF/xacro model** of the arm, hand-written first. Later export from CAD with onshape-to-robot.
   - https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/URDF-Main.html
   - https://onshape-to-robot.readthedocs.io/
2. **RViz + `joint_state_publisher_gui`:** check kinematics and joint limits, and test my own FK/IK.
3. **ros2_control with mock hardware:** https://control.ros.org/jazzy/doc/ros2_control/hardware_interface/doc/mock_components_userdoc.html
4. **Gazebo physics** via gz_ros2_control (gravity, payload, collisions, base, camera): https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html
5. **MoveIt Setup Assistant:** https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html
6. **Nav2** for the base: https://docs.nav2.org/
7. **Firmware-side testing without hardware.** Write my ros2_control CAN plugin against Linux `vcan` and a fake joint program that speaks my CAN protocol.
   - https://docs.kernel.org/networking/can.html
   - Later: one real actuator plus simulated joints, a hardware-in-the-loop setup like the mini HIL project.
8. Portfolio: record a "simulation" video and a "real hardware" video.

**First thing to do on the PC:** steps 1–6 of the environment setup. Paste the `glxinfo -B` output and the result of `gz sim shapes.sdf` back to Claude.
