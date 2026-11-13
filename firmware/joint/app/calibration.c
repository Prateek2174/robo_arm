// Pole pairs, direction, electrical offset, optional R/L measurement.
// Module: #5   Milestone: M1-M2
// TODO: implement (see robot-arm-todo.md for the 'done when' criteria).

#include "calibration.h"
#include "math.h"

#define PI_F 3.14159265f
#define R_THRESH 0.95

void calibration_travel_reset(calibration_travel_t *travel)
{ 
    travel->has_last = 0;
    travel->theta_m_last = 0;
    travel->travel_m = 0;
}

void calibration_travel_add(calibration_travel_t *travel, float theta_m)
{ 
    travel->travel_m += calibration_angle_diff(travel->theta_m_last, theta_m);
    travel->theta_m_last = theta_m;

    if (!travel->has_last){
        travel->has_last = true;
    }
}

calibration_result_t calibration_detect_direction(float travel_m, float min_travel_m, int8_t *dir)
{    //6a. s = sign(T_m)
    //compute direction but reject if there is no motion

}

calibration_result_t calibration_estimate_pole_pairs(float travel_e, float travel_m, float tolerance, uint8_t *pole_pairs)
{ 
    //6b. p̂ = T_e / |T_m|, then round and check
}

void calibration_circular_reset(calibration_circular_t *cir)
{ 
    cir->n = 0;
    cir->sum_cos = 0;
    cir->sum_sin = 0;
}

void calibration_circular_add(calibration_circular_t *cir, float angle)
{
    cir->n++;
    cir->sum_cos += cosf(angle);
    cir->sum_sin += sinf(angle);
}

bool calibration_circular_mean(const calibration_circular_t *cir, float *mean, float *r)
{
    *mean = calibration_wrap_2pi(atan2f(cir->sum_sin, cir->sum_cos));
    *r = sqrtf( cir->sum_cos*cir->sum_cos + cir->sum_sin*cir->sum_sin) / cir->n;

    if (*r >= R_THRESH)
    {
        return true;
    }

    return false;
    
}

float calibration_wrap_2pi(float x)
{
    //ensure output is between [0, 2pi)
    return x - 2 * PI_F * (x/2*PI_F);
}

float calibration_angle_diff(float start, float end)
{ //5. Δ(a, b) and summed travel

}

float calibration_offset_sample(float theta_m, float theta_cmd, uint8_t pole_pairs, int8_t dir)
{    //3. θoff = W(s·p·θm − θ_cmd)
    //at rest: theta_e = theta_cmd
    theta_cmd = dir * pole_pairs - calibration_wrap_2pi(dir*pole_pairs*theta_m - theta_cmd);
}

bool calibration_is_valid(const calibration_data_t *data, uint8_t expected_pole_pairs)
{
    return data->pole_pairs == expected_pole_pairs;
}

void calibration_start(calibration_t *cal, const calibration_config_t *cfg)
{

    //resets the context and stores the config
    cal->cfg = cfg;

}

calibration_output_t calibration_step(calibration_t *cal, float theta_m)
{    //computes which sweeps to run
    //within OFFSET_BACK and OFFSET_FWD phases
    //7. Forward/backward cancels lag

}

calibration_state_t calibration_state(const calibration_t *cal)
{
    //Tells the state machine whether calibration is running, finished or failed
    calibration_state_t out;
    out = cal->phase;
    return out;
}

calibration_result_t calibration_result(const calibration_t *cal, calibration_data_t *d_out)
{
    //Copies out calibration_data_t for params to save
    d_out->direction =
    d_out->pole_pairs =
    d_out->theta_cmd = cal->cmd_elec_angle;
}