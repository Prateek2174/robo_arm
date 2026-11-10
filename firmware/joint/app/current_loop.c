// d/q current control + voltage-circle limit.
// Module: #7   Milestone: M1-M2
// TODO: implement (see robot-arm-todo.md for the 'done when' criteria).

#include "current_loop.h"
#include "pi_controller.h"

static const pi_controller_config_t D_PI_CONFIG = {
    .Kp = 1.0f,
    .Ki = 0.1f,
    .u_min = -10.0f,
    .u_max = 10.0f,
    .Ts = 0.01f,
};

static const pi_controller_config_t Q_PI_CONFIG = {
    .Kp = 1.0f,
    .Ki = 0.1f,
    .u_min = -10.0f,
    .u_max = 10.0f,
    .Ts = 0.01f,
};

static pi_controller_t d_pi_controller;
static pi_controller_t q_pi_controller;