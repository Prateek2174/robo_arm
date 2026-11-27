// Motion control API.
// Module: #8 #18   Milestone: M2
// TODO: implement (see robot-arm-todo.md for the 'done when' criteria).

#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include "trajectory.h"
#include "pi_controller.h"

//omega_ref and tau_accel are written together at 1 kHz and read at 5 kHz. 
//If the 5 kHz ISR preempts between the two writes, it sees a new ω_ref with the old τ_accel.

// converts movement commands into torque for the motor

// 1. takes a command (100 times/s)
// 2. plans a smooth path to reach desired location ---> (trajectory.c)
// 3. position (control) loop (1000 times/s):
//      - checks where the joint should be and where it is
// 4. velocity (control) loop (5000 times/s):
//      - compares target speed with current speed ---> decides required torque

//GEARING:
//      - also needs to convert encoder (turns 9 time / 1 turn of joint) 

//LIMIT JOINT RANGE FOR SAFETY:
//      - slow down early dont make it immediately stop at the limit

typedef struct {
    float gear_ratio;   // N = 9 for J2/J3, 1 for wheels
    int8_t gear_dir;    // +1 / -1 ---> output direction vs motor direction
    float pos_min;
    float pos_max;
    float vel_max;      // rad/s
    float accel_max;    // rad/s^2
    float torque_max;   // Nm
    float backlash_rad; // b
    float inertia;     // J, tau_ff = J*a
    pi_controller_config_t vel_pi;
    float pos_kp;
    bool is_wheel;      // to choose between wheel/velocity mode
} motion_control_config_t;

typedef enum {
    MOTION_CONTROL_IDLE     = 0,
    MOTION_CONTROL_POSITION = 1,
    MOTION_CONTROL_VELOCITY = 2,
    MOTION_CONTROL_WHEEL    = 3
} motion_control_mode_t;

typedef struct {
    float theta;    // output rad
    float omega;    // output rad/s
} motion_control_feedback_t;

typedef struct {
    motion_control_mode_t mode;
    trajectory_plan_t traj_plan;
    uint32_t traj_tick;
    pi_controller_t vel_pi;
    motion_control_config_t cfg;
    float omega_ref; // computed in control loop
    float omega_cmd; // from CAN as a command
    float omega_applied; // omega_cmd ---> slew ---> omega_applied (limits the change)
    float tau_out;
    float tau_gravity;
    float backlash_offset;
    bool tau_saturated;
    bool homed;
    float theta_homing_offset;
    float tau_accel; // torque used to accelerate
    int8_t prev_dir;
    bool traj_active;

} motion_control_t;

void motion_control_init(motion_control_t *mc, const motion_control_config_t *cfg);
void motion_control_reset(motion_control_t *mc); // on FAULT / exiting RUNNING ---> 0 integrator and drop trajectory

//----------- GEAR: MOTOR ---> OUTPUT -----------//

motion_control_feedback_t motion_control_feedback_from_motor(const motion_control_config_t *cfg, int32_t motor_turns, float motor_theta_in_turn, float motor_omega);

void motion_control_set_homed(motion_control_t *mc, float theta_at_home, float theta_now);
bool motion_control_is_homed(const motion_control_t *mc);

bool motion_control_cmd_position(motion_control_t *mc, float theta_f, float theta_i); // if not homed / outside limits --> false
void motion_control_cmd_velocity(motion_control_t *mc, float cmd); // velocity or wheel mode
void motion_control_set_torque_ff(motion_control_t *mc, float tau_gravity); // ff = feedforward
void motion_control_stop(motion_control_t *mc);
bool motion_control_trajectory_done(const motion_control_t *mc);

//----------- CONTROL LOOPS -----------//

void motion_control_position_update(motion_control_t *mc, const motion_control_feedback_t *feedback); // 1 kHz: sample traj, compute omega_ref
float motion_control_velocity_update(motion_control_t *mc, const motion_control_feedback_t *feedback); // 5 kHz: returns tau_out (Nm) for torque_mode

//----------- HELPERS -----------//

float motion_control_soft_limit_velocity(const motion_control_config_t *cfg, float theta, float omega_ref); // sqrt(2*a*d) cap
float motion_control_backlash_offset(motion_control_t *mc, float omega_ref);
float motion_control_slew(float current, float target, float max_step);

#endif // MOTION_CONTROL_H
