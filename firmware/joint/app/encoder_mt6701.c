// MT6701 read, CRC, multi-turn counting, velocity estimate.
// Module: #4   Milestone: M1-M2

#include "encoder_mt6701.h"
#include <math.h>

#define PI_F 3.14159265f


uint8_t encoder_crc6(uint32_t data_18)
{
    // mask the input to 18 bits
    data_18 = data_18 & 0x3FFFF;
    uint8_t crc = 0x00;
    uint8_t temp = 0x00;
    
    // XOR the input bit with the register’s top bit.
    // Shift the register left by one.
    // 6 bits (& 0x3F) after each shift.
    // If the result of step 1 was 1, XOR in 0b000011

    for (int8_t i = 17; i >= 0; i--)
    {
        temp = ((data_18 >> i) & 1) ^ (crc >> 5);
        crc = crc << 1;
        crc &= 0x3F;

        if (temp == 1)
        {
            crc ^= 0x03;
        }
        
    }
    
    return crc;
}

void encoder_unpack(uint32_t frame_24, encoder_frame_t *d_out) //frame -> struct
{
    d_out->data = (frame_24 >> 10) & 0x3FFF;    // Bit 0-13: 14-bit Angle Data D[13:0]
    d_out->mg = (frame_24 >> 6) & 0x0F;         // Bit 14-17: 4-bit Magnetic Field Status Mg[3:0]
    d_out->crc = frame_24 & 0x3F;               // Bit 18-23: 6-bit CRC Code CRC[5:0] 
}

bool encoder_frame_valid(const encoder_frame_t *frame) //crc matches
{
    // create the 18 bit variable with data and mg
    uint32_t data_18 = frame->data << 4 | frame->mg;
    return encoder_crc6(data_18) == frame->crc;
}

int32_t encoder_wrap(uint16_t current, uint16_t prev, uint32_t cpr) //cpr == count_per_revolution
{
    // mt6701 only knows where the shaft is within 1 turn (0 t0 16383) but we need to keep track of total angle
    // over many turns. --> so we keep a running count: 
    int32_t data = current - prev;
    int32_t cnt_per_rev = cpr; 

    if (data >= cnt_per_rev/2)
    {
        data -= cnt_per_rev;
    } 
    else if (data < -1*cnt_per_rev/2)
    {
        data += cnt_per_rev;
    }
    
    return data;
}

void encoder_init(encoder_t *encoder, const encoder_config_t *cfg)
{
    encoder->cfg.cnt_per_rev = cfg->cnt_per_rev;
    encoder->cfg.gear_ratio = cfg->gear_ratio;
    encoder->cfg.max_bad_in_row = cfg->max_bad_in_row;
    encoder->cfg.pll_bandwidth = cfg->pll_bandwidth;
    encoder->cfg.Ts = cfg->Ts;

    encoder->prev_data = 0;
    encoder->count = 0;
    encoder->pos_estimate = 0.0f;
    encoder->vel_estimate = 0.0f;
    encoder->Kp = 2.0f * cfg->pll_bandwidth;
    encoder->Ki = encoder->Kp * encoder->Kp / 4.0f;
    encoder->crc_err = 0;
    encoder->field_err = 0;
    encoder->bad_in_row = 0;
    encoder->ready = false; // first good frame has happened
}

encoder_status_t encoder_update(encoder_t *encoder, uint32_t frame_24)
{
    encoder_frame_t frame;
    encoder_status_t result;

    // unpack the frame 
    encoder_unpack(frame_24, &frame);

    // check the crc
    bool valid;
    valid = encoder_frame_valid(&frame);

    if (!valid)
    {
        result = ENCODER_ERR_CRC;
        encoder->bad_in_row++;
        encoder->crc_err++;
        return result;
    }
    
    // check field status in Mg of the frame

    if ((frame.mg & ENCODER_MG_FIELD_Msk) == ENCODER_MG_FIELD_STRONG)
    {
        encoder->field_err++;
        encoder->bad_in_row++;
        result = ENCODER_ERR_FIELD_STRONG;
        return result;
    }
    else if ((frame.mg & ENCODER_MG_FIELD_Msk) == ENCODER_MG_FIELD_WEAK)
    {
        encoder->field_err++;
        encoder->bad_in_row++;
        result = ENCODER_ERR_FIELD_WEAK;
        return result;
    }
    else if ((frame.mg & ENCODER_MG_OVERSPEED_Msk) == 0x8)
    {
        encoder->field_err++;
        encoder->bad_in_row++;
        result = ENCODER_ERR_OVERSPEED;
        return result;
    }
    
    if (encoder->ready == false) //first good frame has arrived
    {
        encoder->prev_data = frame.data;
        encoder->count = frame.data;
        encoder->pos_estimate = encoder->count; //current pos is just the first movement
        encoder->vel_estimate = 0.0f;
        encoder->ready = true;
        encoder->bad_in_row = 0;
        result = ENCODER_OK;
        return result;
    }
    else //following frames
    {
        encoder->count += encoder_wrap(frame.data, encoder->prev_data, encoder->cfg.cnt_per_rev);
        encoder->prev_data = frame.data;
        encoder->bad_in_row = 0;

        //run pll --> for guess of pos and vel
        encoder->pos_estimate += encoder->vel_estimate * encoder->cfg.Ts; //where it should be based on est
        float err = encoder->count - encoder->pos_estimate; //how far off the guess is from current measurement

        //now adjust pos and vel est closer to reality
        encoder->pos_estimate += encoder->cfg.Ts * encoder->Kp * err;
        encoder->vel_estimate += encoder->cfg.Ts * encoder->Ki * err;

        if (fabsf(encoder->vel_estimate) < 0.5f * encoder->cfg.Ts * encoder->Ki) // to account for wobbling between counts so noise
        {
            encoder->vel_estimate = 0.0f;
        }

        result = ENCODER_OK;
        return result;
    }

}

void encoder_set_cnt(encoder_t *encoder, int64_t cnt)
{
    encoder->count = cnt;
    encoder->pos_estimate = cnt;
} 

uint16_t encoder_get_data(const encoder_t *encoder)
{
    return encoder->prev_data;
}

float encoder_get_motor_angle(const encoder_t *encoder)
{
    float angle = encoder->count * 2 * PI_F / encoder->cfg.cnt_per_rev;
    return angle;
}

float encoder_get_motor_velocity(const encoder_t *encoder)
{
    float v = encoder->vel_estimate * 2 * PI_F / encoder->cfg.cnt_per_rev;
    return v;
}

float encoder_get_output_angle(const encoder_t *encoder)
{
    float angle = encoder->count * 2 * PI_F / encoder->cfg.cnt_per_rev / encoder->cfg.gear_ratio;
    return angle;
}

float encoder_get_output_velocity(const encoder_t *encoder)
{
    float v = encoder->vel_estimate * 2 * PI_F / encoder->cfg.cnt_per_rev / encoder->cfg.gear_ratio;
    return v;
}

bool encoder_is_healthy(const encoder_t *encoder)
{
    return encoder->ready && (encoder->bad_in_row < encoder->cfg.max_bad_in_row);
}