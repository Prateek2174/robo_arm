// Reusable PI controller with anti-windup and limits. NO HAL CALLS.
// Module: #7 #8   Milestone: M1-M2
// TODO: implement (see robot-arm-todo.md for the 'done when' criteria).

//TODO: Decide whether I updates before or after computing u. The test numbers below assume before. 
//      Updating after just delays the integrator by one sample, so either is fine if you document it.

#include "pi_controller.h"

static const pi_controller_config_t default_pi_controller_config = {
    .Kp = 1.0f,
    .Ki = 0.1f,
    .u_min = -10.0f,
    .u_max = 10.0f,
    .Ts = 0.01f,
};

void pi_controller_reset(pi_controller_t *controller){

}

void pi_controller_init(pi_controller_t *controller, const pi_controller_config_t *config){

}

float pi_controller_update(pi_controller_t *controller, float error){   //return u as a float, in output units (volts or amps)
    
}

void pi_controller_set_limits(pi_controller_t *controller, float u_min, float u_max){
    
}