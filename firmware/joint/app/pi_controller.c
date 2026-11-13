// Reusable PI controller with anti-windup and limits. NO HAL CALLS.
// Module: #7 #8   Milestone: M1-M2
// TODO: implement (see robot-arm-todo.md for the 'done when' criteria).

//TODO: Decide whether I updates before or after computing u. The test numbers below assume before. 
//      Updating after just delays the integrator by one sample, so either is fine if you document it.

#include "pi_controller.h"

void pi_controller_reset(pi_controller_t *controller)
{
    controller->I = 0.0f;
    controller->prev_error = 0.0f;
}

void pi_controller_init(pi_controller_t *controller, const pi_controller_config_t *config)
{
    controller->Kp = config->Kp;
    controller->Ki = config->Ki;
    controller->u_min = config->u_min;
    controller->u_max = config->u_max;
    controller->Ts = config->Ts;
    pi_controller_reset(controller);
}

float pi_controller_update(pi_controller_t *controller, float error)
{   //return u as a float, in output units (volts or amps)    
    float proportional = controller->Kp * error;
    controller->I += 0.5f * controller->Ki * controller->Ts * (error + controller->prev_error); //trapezoidal integration

    //Anti-windup wih dynamic integrator clamping

    float max_limit_integrator;
    float min_limit_integrator;

    if (controller->u_max > proportional){
        max_limit_integrator = controller->u_max - proportional;
    } else {
        max_limit_integrator = 0.0f;
    }

    if (controller->u_min < proportional){
        min_limit_integrator = controller->u_min - proportional;
    } else {
        min_limit_integrator = 0.0f;
    }

    //Clamp integrator
    if (controller->I > max_limit_integrator){
        controller->I = max_limit_integrator;
    } else if (controller->I < min_limit_integrator){
        controller->I = min_limit_integrator;
    }

    //Compute output
    float u = proportional + controller->I;

    //Apply output limits
    if (u > controller->u_max){
        u = controller->u_max;
    } else if (u < controller->u_min){
        u = controller->u_min;
    }

    controller->prev_error = error;
    return u;
}

void pi_controller_set_limits(pi_controller_t *controller, float u_min, float u_max)
{
    controller->u_min = u_min;
    controller->u_max = u_max;
}