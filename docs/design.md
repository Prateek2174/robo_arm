# design

Architecture and trade-offs.

Faults

The motor shuts off first. Disabling the gates must not depend on the state machine, so the fault path turns off PWM before it records the state. Later, TIM1’s hardware break input can do this with no software at all.

Latching lives in faults.c. It holds which faults are active, and the state machine only learns whether any remain. A clear request is rejected while a cause is still active.

Need rest position
    so the robot knows what position to return to

    Linux (sees the whole arm)       Joint firmware (sees one joint)
──────────────────────────       ───────────────────────────────
normal shutdown:                 if commands stop or something goes wrong:
  plan trajectory to rest pose     damp instead of going limp
  wait until it gets there         (gates on, motor resists motion,
  then send DISARM                  so the arm sinks slowly)