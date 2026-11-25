// Pole pairs, direction, electrical offset, optional R/L measurement.
// Module: #5   Milestone: M1-M2
// TODO: implement (see robot-arm-todo.md for the 'done when' criteria).

#include "calibration.h"
#include <math.h>

#define PI_F 3.14159265f

//========================================================
// HELPERS
//========================================================

float calibration_wrap_2pi(float x)
{
    //ensure output is between [0, 2pi)
    float theta = x - 2.0f * PI_F * floorf(x/(2.0f*PI_F));

    if (theta >= 2.0f * PI_F)
    {
        return 0.0f;
    }
    
    return theta;
}

float calibration_angle_diff(float start, float end)
{ 
    return calibration_wrap_2pi(end - start + PI_F) - PI_F;
}

float calibration_offset_sample(float theta_m, float theta_cmd, uint8_t pole_pairs, int8_t dir)
{   
    // θoffset = W(s·p·θm − θ_cmd)
    //at rest: theta_e = theta_cmd

    theta_cmd = calibration_wrap_2pi((float)dir * (float)pole_pairs * theta_m - theta_cmd);
    return theta_cmd;
}

bool calibration_is_valid(const calibration_data_t *data, uint8_t expected_pole_pairs)
{
    // need to go through all the checks to ensure the pole_pairs are valid
    
    bool is_valid = false;
    
    if (data->pole_pairs == expected_pole_pairs || expected_pole_pairs == 0)
    {
        if (data->pole_pairs >= 1 && data->pole_pairs <= 50)
        {
            if (data->dir == 1 || data->dir == -1)
            {
                if (isfinite(data->theta_offset))
                {
                    if (data->theta_offset >= 0 && data->theta_offset < 2.0f*PI_F)
                    {
                        is_valid = true;
                    }
                }
            }
        }
    }

    return is_valid;
}

//========================================================
// CORE
//========================================================

void calibration_travel_reset(calibration_travel_t *travel)
{ 
    travel->has_last = 0;
    travel->theta_m_last = 0;
    travel->travel_m = 0;
}

void calibration_travel_add(calibration_travel_t *travel, float theta_m)
{ 
    if (travel->has_last) // skip add for the first 
    {
        travel->travel_m += calibration_angle_diff(travel->theta_m_last, theta_m);
    }

    travel->theta_m_last = theta_m;    

    if (!travel->has_last)
    {
        travel->has_last = true;
    }
}

calibration_result_t calibration_detect_direction(float travel_m, float min_travel_m, int8_t *dir)
{   
    calibration_result_t result;

    if (fabsf(travel_m) < min_travel_m)
    {
        result = CALIBRATION_ERR_NO_MOTION;
    }
    else
    {
        *dir = travel_m > 0.0f ? 1 : -1;
        result = CALIBRATION_OK;
    }

    return result;
}

calibration_result_t calibration_estimate_pole_pairs(float travel_e, float travel_m, float tolerance, uint8_t *pole_pairs)
{ 
    calibration_result_t result;
    //check if travel_m is near 0

    if (fabsf(travel_m) <= 0.001f) // so there isnt a divide by zero
    {
        result = CALIBRATION_ERR_NO_MOTION;
        return result;
    }

    float pp_estimate;
    pp_estimate = travel_e / fabsf(travel_m);
    float pp_temp = roundf(pp_estimate);

    if (fabsf(pp_estimate - pp_temp) > tolerance)
    {
        result = CALIBRATION_ERR_POLE_PAIRS;
    }
    else if (pp_temp < 1 || pp_temp > 50)
    {
        result = CALIBRATION_ERR_POLE_PAIRS;
    }
    else
    {
        result = CALIBRATION_OK;
        *pole_pairs = (uint8_t)pp_temp;
    }

    return result;
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
    if (cir->n == 0)
    {
        return false;
    }

    *mean = calibration_wrap_2pi(atan2f(cir->sum_sin, cir->sum_cos));
    *r = sqrtf(cir->sum_cos*cir->sum_cos + cir->sum_sin*cir->sum_sin) / (float)cir->n;

    return true;
}

//========================================================
// STATE SEQUENCE FUNCTIONS
//========================================================

void calibration_start(calibration_t *cal, const calibration_config_t *cfg)
{
    // check steps_per_ece_turn so it doesnt /0

    if (cfg->steps_per_ece_turn == 0)
    {
        cal->error = CALIBRATION_ERR_BAD_DATA;
        cal->phase = CALIBRATION_STATE_FAILED;
    }
    else
    {
        // resets the context and stores the config
        calibration_travel_reset(&cal->travel);
        calibration_circular_reset(&cal->offsets);
        cal->cfg = *cfg;
        cal->step = 2*PI_F / (float)cal->cfg.steps_per_ece_turn;
        cal->phase = CALIBRATION_STATE_ALIGN; // reset state
        cal->error = CALIBRATION_OK;
        cal->step_count = 0;
        cal->theta_cmd = 0;
        cal->curr_pole_pairs = 0;
        cal->curr_dir = 0;
        cal->theta_offset = 0;
    }
}

calibration_output_t calibration_step(calibration_t *cal, float theta_m)
{  
    calibration_output_t out;
    out.v_d = cal->cfg.v_d;
    out.theta_cmd = cal->theta_cmd;
    out.drive = false; 

    calibration_result_t result;

    if (cal->phase == CALIBRATION_STATE_ALIGN) // ALIGN: Holds θ_cmd = 0 at V_d and waits
    {

        if (cal->step_count < cal->cfg.align_steps) // keep counting
        {
            cal->step_count++;
            out.theta_cmd = 0;
            out.drive = true;
            return out;
        }
        else // align is over
        {
            cal->phase = CALIBRATION_STATE_DETECT; // move on to next state
            cal->step_count = 0;
            calibration_travel_reset(&cal->travel);
            out.drive = true;
            return out;
        }
    }
    else if (cal->phase == CALIBRATION_STATE_DETECT) //DETECT: Sweeps forward, then works out direction and pole pairs
    {
        calibration_travel_add(&cal->travel, theta_m); // record the change in rotor

        // check to see if the sweep finished by checking step count

        if (cal->step_count >= (cal->cfg.detect_turns * cal->cfg.steps_per_ece_turn)) // if the sweep is over get dir and p
        {
            // get direction and check for any error

            result = calibration_detect_direction(cal->travel.travel_m, cal->cfg.min_travel_m, &cal->curr_dir);
            
            if (result != CALIBRATION_OK)
            {
                cal->error = result;
                cal->phase = CALIBRATION_STATE_FAILED;
                return out;
            }

            // get the estimated pole_pairs and ensure no error happened

            float travel_e = cal->step_count * cal->step;
            result = calibration_estimate_pole_pairs(travel_e, cal->travel.travel_m, cal->cfg.pole_pairs_tolerance, &cal->curr_pole_pairs);

            if (result != CALIBRATION_OK)
            {
                cal->error = result;
                cal->phase = CALIBRATION_STATE_FAILED;
                return out;
            }

            // are the pole_pairs received as expected?

            if (cal->cfg.expected_pole_pairs != 0 && cal->curr_pole_pairs != cal->cfg.expected_pole_pairs)
            {
                cal->error = CALIBRATION_ERR_POLE_PAIRS;
                cal->phase = CALIBRATION_STATE_FAILED;
                return out;    
            }
            
            cal->phase = CALIBRATION_STATE_OFFSET_BACK;
            out.drive = true;
            cal->step_count = 0;
            calibration_circular_reset(&cal->offsets);
        }
        else // continue the sweep
        {
            cal->theta_cmd += cal->step;
            out.theta_cmd = cal->theta_cmd;
            cal->step_count++;
            out.drive = true;
            return out;
        }
    }
    else if (cal->phase == CALIBRATION_STATE_OFFSET_BACK)
    {
        float sample;
        sample = calibration_offset_sample(theta_m, cal->theta_cmd, cal->curr_pole_pairs, cal->curr_dir);
        calibration_circular_add(&cal->offsets, sample);

        if (cal->step_count >= (cal->cfg.offset_turns * cal->cfg.steps_per_ece_turn)) // sweep is over
        {
            cal->step_count = 0;
            cal->phase = CALIBRATION_STATE_OFFSET_FWD;
            out.drive = true;
        }
        else // continue the sweep
        {
            cal->theta_cmd -= cal->step;
            out.theta_cmd = cal->theta_cmd;
            cal->step_count++;
            out.drive = true;
            return out;
        }
    }
    else if (cal->phase == CALIBRATION_STATE_OFFSET_FWD)
    {
        float sample;
        sample = calibration_offset_sample(theta_m, cal->theta_cmd, cal->curr_pole_pairs, cal->curr_dir);
        calibration_circular_add(&cal->offsets, sample);

        if (cal->step_count >= (cal->cfg.offset_turns * cal->cfg.steps_per_ece_turn)) // DONE: end of sweeps --> combines the offsets and checks them
        {
            float mean;
            float r;
            bool check;
            check = calibration_circular_mean(&cal->offsets, &mean, &r);

            if (!check || r < cal->cfg.r_min) // make sure the radius > min and successful
            {
                cal->error = CALIBRATION_ERR_SPREAD;
                cal->phase = CALIBRATION_STATE_FAILED;
                return out;
            }
            
            cal->theta_offset = mean;

            // create a temp calibration_data_t to test the collected data to ma0ke sure its valid

            calibration_data_t data;

            data.dir = cal->curr_dir;
            data.pole_pairs = cal->curr_pole_pairs;
            data.theta_offset = cal->theta_offset;
            data.offset_ref = CALIBRATION_REF_ABSOLUTE;

            check = calibration_is_valid(&data, cal->cfg.expected_pole_pairs);

            if (!check) 
            {
                cal->error = CALIBRATION_ERR_BAD_DATA;
                cal->phase = CALIBRATION_STATE_FAILED; 
                return out;
            }

            cal->phase = CALIBRATION_STATE_DONE;
            out.drive = false;
        }
        else // continue the sweep
        {
            cal->theta_cmd += cal->step;
            out.theta_cmd = cal->theta_cmd;
            cal->step_count++;
            out.drive = true;
            return out;
        }
    }
    return out;
}

calibration_state_t calibration_state(const calibration_t *cal)
{
    // Tells the state machine whether calibration is running, finished or failed
    calibration_state_t out;
    out = cal->phase;
    return out;
}

calibration_result_t calibration_result(const calibration_t *cal, calibration_data_t *d_out)
{
    // If the phase is FAILED, return cal->error, the reason it failed, and don't touch d_out.

    if (cal->phase == CALIBRATION_STATE_FAILED)
    {
        return cal->error;
    }
    else if (cal->phase == CALIBRATION_STATE_DONE) // done
    {
        d_out->dir = cal->curr_dir;
        d_out->pole_pairs = cal->curr_pole_pairs;
        d_out->theta_offset = cal->theta_offset;
        d_out->offset_ref = CALIBRATION_REF_ABSOLUTE;
        return CALIBRATION_OK;
    }
    else // still running -- not done
    {
        return CALIBRATION_ERR_NOT_DONE;
    }
}