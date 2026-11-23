// Startup order and main loop (CAN, housekeeping).
// Module: #15   Milestone: M1-M2
// TODO: implement (see robot-arm-todo.md for the 'done when' criteria).

#include "app_main.h"

static void app_force_safe_outputs(void);  // step 1: MOE=0, gates off
static void app_init(void);                // steps 2-8 from the lesson, in order
static void app_loop_once(void);           // one pass: can_handler, telemetry framing, params save if IDLE, IWDG