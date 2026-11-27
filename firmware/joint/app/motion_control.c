// Velocity + position loops, gear ratio, limits, backlash, wheel mode.
// Module: #8 #18   Milestone: M2
// TODO: implement (see robot-arm-todo.md for the 'done when' criteria).

#include "motion_control.h"
#include <math.h>

//========================================================
// HELPERS
//========================================================

static float motion_control_clamp(float x, float low_bound, float high_bound)
{
    // in case i make a mistake:
    if (low_bound > high_bound)
    {
        float temp = low_bound;
        low_bound = high_bound;
        high_bound = temp;
    }

    if (x < low_bound) return low_bound;
    if (x > high_bound) return high_bound;
    return x;
}

float motion_control_soft_limit_velocity(const motion_control_config_t *cfg, float theta, float omega_ref) // sqrt(2*a*d) cap
{
    // slow the joint early using a soft limit so that it can always stop before the hard limit

    // If moving positive: d = pos_max − θ. If moving negative: d = θ − pos_min.

    if (cfg->gear_dir == 1) // moving positive
    {
        
    }
    else if (cfg->gear_dir == -1) // moving negative
    {
        /* code */
    }
    
    

    // If d ≤ 0 (at or past the limit), allow no velocity toward it. Return 0 if ω_ref points that way. Moving away from the limit is still allowed.
    // Otherwise the allowed speed is √(2·a_max·d). Clamp ω_ref's magnitude to it, keeping its sign.
}

float motion_control_backlash_offset(motion_control_t *mc, float omega_ref)
{
    // add a small correction when the joint reverses, to take up the gear slop.

    // Work out the commanded direction from ω_ref. Treat it as unchanged unless |ω_ref| exceeds a small hysteresis threshold, so noise near zero doesn't flip it.
    // Remember the last direction in the struct.
    // The target offset is +b/2 for positive and −b/2 for negative.
    // Move the stored offset toward that target gradually (you can reuse motion_slew), so it doesn't kick the joint.
    // Return the current offset. If backlash_rad is 0, always return 0.

    if (mc->omega_ref > )
    {
        /* code */
    }
    else if (mc->omega_ref < )
    {
        /* code */
    }
    else
    {

    }
    
    
}

float motion_control_slew(float current, float target, float max_step)
{
    // move a value toward a target but by no more than max_step per call -- wheel accel limit
    float diff = target - current;

    if (diff < -1.0f*max_step)
    {
        diff = -1.0f*max_step;
    }
    else if (diff > max_step)
    {
        diff = max_step;
    }
    
    return current + diff;
}

motion_control_feedback_t motion_control_feedback_from_motor(const motion_control_config_t *cfg, int32_t motor_turns, float motor_theta_in_turn, float motor_omega)
{
    // converts encoder measurements (motor shaft) ---> joint units (output shaft)

    motion_control_feedback_t result;
    float motor_theta = (float)motor_turns * (motor_theta_in_turn / cfg->gear_ratio);
    result.theta = motor_theta / cfg->gear_ratio * cfg->gear_dir;
    result.omega = motor_omega / cfg->gear_ratio * cfg->gear_dir;
    return result;
}

//========================================================
// CORE
//========================================================

void motion_control_init(motion_control_t *mc, const motion_control_config_t *cfg)
{
    // store the config, init the velocity PI from cfg->vel_pi with its limits set to ±torque_max, 
    // set mode to IDLE, set homed to false, then call motion_reset
    mc->cfg = *cfg;
    mc->homed = false;
    mc->traj_active = false;
    mc->tau_saturated = false;
    mc->mode = MOTION_CONTROL_IDLE;
    trajectory_plan_t traj_plan;
    mc->traj_tick = 0;
    pi_controller_init(&mc->vel_pi, &cfg->vel_pi);
    motion_control_config_t cfg;
    mc->omega_ref = 0.0f;
    mc->omega_cmd = 0.0f; 
    mc->omega_applied = 0.0f;
    mc->tau_out = 0.0f;
    mc->tau_gravity = 0.0f;
    mc->backlash_offset = 0.0f;
    mc->theta_homing_offset = 0.0f;
    mc->a_ff = 0.0f;
    mc->prev_dir = mc->cfg.gear_dir;
}

void motion_control_reset(motion_control_t *mc) // on FAULT / exiting RUNNING ---> 0 integrator and drop trajectory
{
    // go back to a safe "doing nothing" state. Call it on FAULT or when leaving RUNNING.
    mc->traj_active = false;
    mc->omega_ref = 0.0f;
    mc->tau_out = 0.0f;
    mc->omega_applied = 0.0f;
    mc->backlash_offset = 0.0f;
    mc->mode = MOTION_CONTROL_IDLE;
}

void motion_control_set_homed(motion_control_t *mc, float theta_at_home, float theta_now)
{
    // save the offset between the raw output angle and the true joint angle, and set homed = true. 
    mc->theta_homing_offset = theta_at_home - theta_now;
    mc->homed = true;

    //joint_zero_offset = θ_at_home − θ_out_raw
}

bool motion_control_is_homed(const motion_control_t *mc)
{
    // let the state machine and the CAN handler check homing before allowing position mode
    return mc->homed;
    // returns the flag
}

bool motion_control_cmd_position(motion_control_t *mc, float theta_f, float theta_i)
{
    // start a smooth move to a new angle:
    // clamp the target theta, then plan the traj and if the plan is successful then
    // reset time tick, activated traj_active and move to POSITION mode
    
    if (mc->homed && isfinite(theta_f)) // only if homed and target theta is finite
    {
        // clamp theta_f
        if (theta_f < mc->cfg.pos_min)
        {
            theta_f = mc->cfg.pos_min;
        }
        else if (theta_f > mc->cfg.pos_max)
        {
            theta_f = mc->cfg.pos_max;
        }

        trajectory_limits_t limits;
        limits.a_max = mc->cfg.accel_max;
        limits.v_max = mc->cfg.vel_max;
        
        bool plan_success;
        plan_success = trajectory_plan(&mc->traj_plan, theta_i, theta_f, &limits);

        if (!plan_success)
        {
            return plan_success;
        }

        mc->traj_tick = 0;
        mc->traj_active = true;
        mc->mode = MOTION_CONTROL_POSITION;
        return true;
    }
    else
    {
        return false;
    }
} 

void motion_control_cmd_velocity(motion_control_t *mc, float cmd) // velocity or wheel mode
{
    // set a speed target in velocity or wheel mode

    // ω_cmd = clamp(ω_cmd, −vel_max, +vel_max)

    // and set the mode. --- wdym by set the mode --- should i change the mode?
    // Don't apply it directly. In wheel mode the 5 kHz loop slews toward it

    if (mc->cfg.is_wheel)
    {
        if (cmd > mc->cfg.vel_max)
        {
            mc->omega_cmd = mc->cfg.vel_max;
        }
        else if (cmd < -1.0f*mc->cfg.vel_max)
        {
            mc->omega_cmd = -1.0f*mc->cfg.vel_max;
        }
        else
        {
            mc->omega_cmd = cmd;
        }

        if (mc->cfg.is_wheel)
        {
            mc->mode = MOTION_CONTROL_WHEEL;
        }
        else
        {
            mc->mode = MOTION_CONTROL_VELOCITY;
        }
    }
}

void motion_control_set_torque_ff(motion_control_t *mc, float tau_gravity) // ff = feedforward
{
    // store the host's gravity torque for the velocity loop to add

    if (isfinite(tau_gravity))
    {
        if (tau_gravity < -1.0f*mc->cfg.torque_max)
        {
            tau_gravity = -1.0f*mc->cfg.torque_max;
        }
        else if (tau_gravity > mc->cfg.torque_max)
        {
            tau_gravity = mc->cfg.torque_max;
        }
        
        mc->tau_gravity = tau_gravity;
    }
}

void motion_control_stop(motion_control_t *mc)
{
    // command-timeout response for wheels: coast down smoothly instead of cutting power

    // set the commanded velocity to 0. The slew in the velocity loop ramps it down

    mc->omega_cmd = 0.0f;
}

bool motion_control_trajectory_done(const motion_control_t *mc)
{
    // tell the CAN handler and the host that the move has finished
    // return true when no trajectory is active, or when the tick time has reached the plan's duration.

    return (!mc->traj_active || mc->traj_tick * mc->vel_pi.Ts >= mc->traj_plan.t_f);
}

//========================================================
// CONTROL LOOPS
//========================================================

void motion_control_position_update(motion_control_t *mc, const motion_control_feedback_t *feedback) // 1 kHz: compute omega_ref, tau_accel
{
    // decide how fast the joint should be moving right now by telling velocity loop -----> omega_ref (vel) and tau_accel (accel)

    // When the trajectory has finished, keep holding the final θ_ref with zero feedforward.

    if (mc->mode == MOTION_CONTROL_POSITION)
    {
        float t;
        t = mc->traj_tick * mc->vel_pi.Ts;
        float theta_ref;
        float omega_ff;
        float a_ff;
        // get current traj data
        trajectory_instant_t current;
        current = trajectory_sample(&mc->traj_plan, t);
        theta_ref = current.d;
        omega_ff = current.v;
        a_ff = current.a;
        
        mc->traj_tick++;
        theta_ref += motion_control_backlash_offset(mc, mc->omega_ref);

        // get position error
        float pos_err = theta_ref - feedback->theta;
        mc->omega_ref = omega_ff + mc->cfg.pos_kp * pos_err;

        // use soft limit on omega_ref then clamp it between the max/min
        mc->omega_ref = motion_control_soft_limit_velocity(&mc->cfg, feedback->theta, mc->omega_ref);
        mc->omega_ref = motion_control_clamp(mc->omega_ref, -1.0f * mc->cfg.vel_max, mc->cfg.vel_max);
        mc->tau_accel = mc->cfg.inertia * a_ff;
    }
}

float motion_control_velocity_update(motion_control_t *mc, const motion_control_feedback_t *feedback) // 5 kHz: returns tau_out (Nm) for torque_mode
{
    // decide how hard to push right now

    // If IDLE, return 0.
    // Pick the velocity target:
    // POSITION: ω_ref from the position loop.
    // VELOCITY: the commanded value.
    // WHEEL: slew the applied value toward the commanded value (max step = accel_max · 200 µs), then use the applied value.
    // Velocity error: e = target − fb->ω.
    // τ = PI(e) + J·a_ff + τ_gravity.
    // Clamp τ to ±torque_max, and record whether it saturated, for telemetry.
    // Return τ. torque_mode turns it into i_q.

    if (mc->mode == MOTION_CONTROL_IDLE)
    {
        return 0;
    }


    
}