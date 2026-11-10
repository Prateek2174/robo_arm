// PI controller API.
// Module: #7 #8   Milestone: M1-M2

#ifndef PI_CONTROLLER_H
#define PI_CONTROLLER_H

typedef struct {
    float Kp;
    float Ki;
    float I; //integral term

    float u_min;
    float u_max;

    float Ts; //sample time in seconds

    float prev_error; //for trapezoidal integration
} pi_controller_t;

typedef struct {
    float Kp;
    float Ki;

    float u_min;
    float u_max;

    float Ts; //sample time in seconds
} pi_controller_config_t;

void pi_controller_reset(pi_controller_t *controller);
void pi_controller_init(pi_controller_t *controller, const pi_controller_config_t *config);
float pi_controller_update(pi_controller_t *controller, float error); //return u as a float, in output units (volts or amps)
void pi_controller_set_limits(pi_controller_t *controller, float u_min, float u_max);

#endif // PI_CONTROLLER_H
