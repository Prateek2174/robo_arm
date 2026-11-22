// Hall-sensor angle + interpolation for hoverboard wheel motors.
// Module: #17   Milestone: M5
// TODO: implement (see robot-arm-todo.md for the 'done when' criteria).

// TODO: use the hall_code_to_sector helper function

#include "hall.h"
#include <math.h>

#define PI_F 3.14159265f

//========================================================
// HELPERS
//========================================================

static int8_t hall_code_to_sector(const hall_config_t *cfg, uint8_t code)
{
    code = code & 0x07; //mask the code to 3 bits

    if (code == 0b111 || code == 0b000)
    {
        return -1;
    }
    
    return cfg->sector_lut[code];
}

static float hall_wrap_2pi(float x)
{
    return x - 2.0f * PI_F * floorf(x/(2.0f*PI_F));
}

static int8_t hall_sector_delta(int8_t curr_s, int8_t prev_s)
{
    //wrap the delta between [-2, 3]
    int8_t result = curr_s - prev_s;

    if (result > 3)
    {
        result -= 6;
    }
    else if (result < -2)
    {
        result += 6;
    }
    
    return result;

    // +1: forward one sector.
    // −1: backward one sector.
    //  0: the same sector.
    // ±2 or 3: a skipped sector
}

static float hall_get_sector_center(hall_t *hall, int8_t sector)
{
    return hall_wrap_2pi(sector * PI_F / 3.0f + PI_F / 6.0f + hall->cfg.theta_offset_e);
}

static float hall_get_period_mean(hall_t *hall)
{
    float total = 0.0f;

    for (int8_t i = 0; i < hall->period_count; i++)
    {
        total += hall->sector_period[i];
    }
    
    return total/hall->period_count;
}

static float hall_clamp_angle(float angle, float low_bound, float high_bound)
{
    // in case i make a mistake:
    if (low_bound > high_bound)
    {
        float temp = low_bound;
        low_bound = high_bound;
        high_bound = temp;
    }

    if (angle < low_bound) return low_bound;
    if (angle > high_bound) return high_bound;
    return angle;
}

//========================================================
// CORE
//========================================================

bool hall_init(hall_t *hall, const hall_config_t *cfg)
{
    // first check the config struct to make sure values make sense

    if (cfg->omega_interpolate_off >= cfg->omega_interpolate_on || cfg->pole_pairs <= 0 || cfg->tick_s <= 0)
    {
        return false;
        // the bounds dont make sense
        // cant have -ve or 0 pole_pairs
        // tick_s (time) must be +ve non zero
    }

    if (cfg->dir_sign != 1 && cfg->dir_sign != -1)
    {
        return false;
    }

    hall->cfg.dir_sign = cfg->dir_sign;
    hall->cfg.omega_interpolate_off = cfg->omega_interpolate_off;
    hall->cfg.omega_interpolate_on = cfg->omega_interpolate_on;
    hall->cfg.pole_pairs = cfg->pole_pairs;

    uint8_t lut[6] = {0};

    // sector_lut[hall_code] = sector

    for (uint8_t i = 0; i < 8; i++)
    {
        if (i == 0 || i == 7)
        {
            hall->cfg.sector_lut[i] = -1;
        }
        else
        {
            hall->cfg.sector_lut[i] = cfg->sector_lut[i];

            if (cfg->sector_lut[i] < 0 || cfg->sector_lut[i] > 5)
            {
                return false;
            }
            
            lut[cfg->sector_lut[i]]++;
        }
    }

    // check to make sure each value 0-5 occurs only ONCE in the lut

    for (uint8_t i = 0; i < 6; i++)
    {
        if (lut[i] != 1)
        {
            return false;
        }
    }

    hall->cfg.theta_offset_e = cfg->theta_offset_e;
    hall->cfg.tick_s = cfg->tick_s;
    hall->cfg.timeout_s = cfg->timeout_s;

    hall->sector = -1; // -1 == unknown sector
    hall->prev_sector = -1; 
    hall->dir = 0;            

    for (uint8_t i = 0; i < 6; i++)
    {
        hall->sector_period[i] = 0;     
    }
    
    hall->period_index = 0;
    hall->period_count = 0;
    hall->ignore_next_dt = true; 
    hall->omega_e = 0;              
    hall->theta_edge = 0;           
    hall->interpolation_active = false;
    hall->edge_count = 0;
    hall->error_count = 0; 

    return true;
}

hall_status_t hall_set_initial_sector(hall_t *hall, uint8_t code) // call once at boot
{
    hall_status_t status;
    
    // mask & decode the code ---> if invalid return appropriate status

    code = code & 0b111;

    if (code == 0b000 || code == 0b111)
    {
        status = HALL_ERR_INVALID_CODE;
        hall->error_count++;
        return status;
    }

    // code -> sector lut -> set sector, prev_sector, theta_edge
    
    int8_t s_initial = hall->cfg.sector_lut[code];
    hall->sector = s_initial;
    hall->prev_sector = s_initial;
    hall->theta_edge = hall_get_sector_center(hall, s_initial);

    hall->dir = 0;
    hall->omega_e = 0;
    hall->interpolation_active = false;
    hall->ignore_next_dt = true;

    status = HALL_OK;
    return status;
} 

hall_status_t hall_on_edge(hall_t *hall, uint8_t code, uint32_t dt_ticks) // called 6 times per ece turn 
{
    // dt_ticks = time since prev edge
    hall_status_t status;

    // decode:
    code = code & 0b111;

    if (code == 0b000 || code == 0b111)
    {
        status = HALL_ERR_INVALID_CODE;
        hall->error_count++;
        return status;
    }

    int8_t s = hall->cfg.sector_lut[code];

    // check to ensure the sector isnt unknown == -1

    if (hall->sector == -1)
    {
        hall->prev_sector = s;
        hall->sector = s;
        hall->theta_edge = hall_get_sector_center(hall, s);
        hall->dir = 0;
        hall->omega_e = 0;
        hall->interpolation_active = false;
        hall->ignore_next_dt = true;
        status = HALL_OK;
        return status;
    }
    
    // get change in sector and perform some checks

    int8_t delta = hall_sector_delta(s, hall->sector);

    if (delta == 0) // maybe just noise so ignore and return OK
    {
        status = HALL_OK;
        return status;
    }
    else if (delta >= 2 || delta <= -2) // a skipped sector has happened
    {
        hall->error_count++;

        hall->theta_edge = hall_get_sector_center(hall, s);

        // clear period buffer
        hall->period_count = 0;
        hall->period_index = 0;
        
        hall->interpolation_active = false;
        hall->ignore_next_dt = true;

        hall->prev_sector = hall->sector;
        hall->sector = s;
        hall->dir = 0;
        status = HALL_ERR_SKIPPED_SECTOR;
        return status;
    }
    else if (delta == 1 || delta == -1) // normal edge 
    {
        status = HALL_OK;
    }
    
    // if direction of wheels have changed:

    if (hall->dir != 0 && hall->dir != delta)
    {
        // clear period buffer
        hall->period_count = 0;
        hall->period_index = 0;

        hall->omega_e = 0;
        hall->interpolation_active = false;
        hall->ignore_next_dt = true;
    }
    
    // angle at the edge

    if (delta == 1) //fwd
    {
        hall->theta_edge = hall_wrap_2pi(s * PI_F / 3.0f + hall->cfg.theta_offset_e);
    }
    else if (delta == -1) //bwd
    {
        hall->theta_edge = hall_wrap_2pi((s + 1) * PI_F / 3.0f + hall->cfg.theta_offset_e);
    }

    // speed

    if (!hall->ignore_next_dt)
    {
        hall->sector_period[hall->period_index] = (float)dt_ticks * hall->cfg.tick_s;
        
        if (hall->period_count < 6)
        {
            hall->period_count++;
        }

        hall->period_index = (uint8_t)((hall->period_index + 1) % 6); // so the index stays in the array bounds
        hall->omega_e = delta * PI_F / 3.0f / hall_get_period_mean(hall); // average the 6 periods (1 ece turn) so errors cancel out
    }
    
    hall->ignore_next_dt = false; // clear it 

    // update struct with new sector
    hall->prev_sector = hall->sector;
    hall->sector = s;
    hall->edge_count += delta;
    hall->dir = delta;
    
    // interpolation check:

    if (!hall->interpolation_active && fabsf(hall->omega_e) > hall->cfg.omega_interpolate_on)
    {
        hall->interpolation_active = true; // turn on
    }
    else if (hall->interpolation_active && fabsf(hall->omega_e) < hall->cfg.omega_interpolate_off)
    {
        hall->interpolation_active = false; // turn off
    }
    
    return status;
} 

float hall_get_theta_e(hall_t *hall, uint32_t ticks_since_edge) // use in 20 kHz current loop
{
// If it's on, compute adv = omega_e · ticks · tick_s, clamp it to [0, π/3] going forward or [−π/3, 0] going backward, 
// then return wrap(theta_edge + adv).

    if (hall->sector == -1) // unkown sector
    {
        return 0;
    }
    
    if (hall->interpolation_active == false)
    {
        return hall_get_sector_center(hall, hall->sector);
    }
    
    // calc the change in angle using omega_e
    float delta = hall->omega_e * (float)ticks_since_edge * hall->cfg.tick_s;
    
    if (hall->dir == 1) // fwd
    {
        // clamp it to [0, π/3]
        delta = hall_clamp_angle(delta, 0.0f, PI_F / 3.0f);
    }
    else if (hall->dir == -1) // bwd
    {
        // clamp it to [−π/3, 0]
        delta = hall_clamp_angle(delta, -1 * PI_F / 3.0f, 0.0f);
    }

    return hall_wrap_2pi(hall->theta_edge + delta); 
}

float hall_get_omega_e(const hall_t *hall) // to get ece speed in rad/s with direction/sign
{
    return hall->cfg.dir_sign * hall->omega_e;
}

void hall_on_timeout(hall_t *hall) // called when TIM4 counter OVERFLOWS == no edge has arrived for a full timer period
{
    hall->omega_e = 0.0f;
    hall->interpolation_active = false;

    //clear period buffer
    hall->period_count = 0;
    hall->period_index = 0;

    hall->ignore_next_dt = true;
}

int32_t hall_get_edge_count(const hall_t *hall) // for ODOMETRY
{
    return hall->edge_count * hall->cfg.dir_sign;
}

//========================================================
// HARDWARE
//========================================================

void hall_hardware_init(void)
{

}

uint8_t hall_hardware_read_code(void) // read PB6/PB7/PB8 ----> 3-bit code
{

}