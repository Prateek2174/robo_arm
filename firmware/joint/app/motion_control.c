// Velocity + position loops, gear ratio, limits, backlash, wheel mode.
// Module: #8 #18   Milestone: M2
// TODO: implement (see robot-arm-todo.md for the 'done when' criteria).

#include "motion_control.h"
#include "pi_controller.h"

static const pi_controller_config_t V_PI_CONFIG = {
    .Kp = 1.0f,
    .Ki = 0.1f,
    .u_min = -10.0f,
    .u_max = 10.0f,
    .Ts = 0.01f,
};

static const pi_controller_config_t P_PI_CONFIG = {
    .Kp = 1.0f,
    .Ki = 0.0f,
    .u_min = -10.0f,
    .u_max = 10.0f,
    .Ts = 0.01f,
};

static pi_controller_t v_pi_controller;
static pi_controller_t p_pi_controller;