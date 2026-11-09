# robo_arm

Mobile 5-DOF robot arm: open a door, pick a glass off a table, bring it to the kitchen.

- Plan and BOM: `robot-arm-handoff.md`
- Task list and file manifest: `robot-arm-todo.md`

| Folder | Contents |
|---|---|
| `shared/can_protocol/` | CAN protocol (arm.dbc + C header), used by firmware, Linux and tests |
| `firmware/joint/` | STM32G431 joint + wheel firmware (C) |
| `firmware/power_node/` | STM32F411 power node (C) |
| `tests/host/` | Host unit tests for HAL-free firmware code |
| `ros2_ws/src/` | ROS 2 Jazzy packages (C++/Python) |
| `tools/` | PC Python tools: tuning, CAN tests, fake joint |
| `docs/` | Design, FMEA, timing, measurements, checkpoints |
| `hardware/` | CAD, gearboxes, wiring, PCB |
| `rust-practice/` | Separate Rust/Embassy practice project (F446RE) |

TODO: block diagram, demo videos, measured numbers.
