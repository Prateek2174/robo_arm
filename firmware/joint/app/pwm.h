// PWM driver API.
// Module: #2   Milestone: M1-M2
// TODO: implement (see robot-arm-todo.md for the 'done when' criteria).

#ifndef PWM_H
#define PWM_H

#include <stdint.h>

typedef struct {
    uint16_t arr; // CCR range: 0...arr
    float duty_min; 
    float duty_max;
} pwm_config_t;

typedef struct {
    uint16_t a;
    uint16_t b;
    uint16_t c;
} pwm_ccr_t;

float pwm_clamp_duty(const pwm_config_t *cfg, float duty); // TODO: need to decide how to handle NaN
uint16_t pwm_duty_to_ccr(const pwm_config_t *cfg, float duty); // clamp ---> scale to 0...arr


//------------------------Hardware Required Functions------------------------//

void pwm_init(const pwm_config_t *cfg); //
void pwm_start(void); // start CH1, CH2, CH3 and CH1N, CH2N, CH3N ---------- MOE still off
void pwm_enable_outputs(void); // MOE = 1
void pwm_disable_outputs(void); // MOE = 0

void pwm_set_duty_cycle(const pwm_ccr_t *duty); // every 20 kHz ADC ISR: CLAMP ---> CONVERT ---> WRITE CCR1, CCR2, CCR3

#endif // PWM_H
