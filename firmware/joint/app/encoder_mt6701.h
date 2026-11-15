// Encoder API.
// Module: #4   Milestone: M1-M2
// TODO: implement (see robot-arm-todo.md for the 'done when' criteria).

#ifndef ENCODER_MT6701_H
#define ENCODER_MT6701_H

#include <stdint.h>
#include <stdbool.h>

//After chip power-on, the ABZ output is blocked for 50ms to guarantee proper output

//Using SSI interface

//a data transfer starts when CSN is pulled to logic
// ‘Low’. The MT6701 transfers data on the falling edge of CLK, and the data transfer finally
// stops when CSN is pulled to logic ‘High’

//Data Output from MT6701, it is driven at the
//rising edge of CLK and should be captured at the falling edge of CLK

// TIMING

// Min clk period = 64 ns
// At least 100 ns from CSN going low to the first clock.
// At least 0.5 × TCLK from the last clock edge to CSN going high.

// SSI READ ANGLE:

//ANGLE:          angle_deg = D * 360/16384


// Mg[3:0] Magnetic Field Status Truth Table

// Mg[1:0] Status

// 0       Normal
// 1       Magnetic Field is too Strong
// 2       Magnetic Field is too Weak

//Mg[3:2] = 0 --> Normal

//Mg[2]: push button.
//Mg[3]: overspeed.

// CRC6

// Polynomial x⁶ + x + 1, so the 6-bit pattern is 0b000011.

// XOR the input bit with the register’s top bit.
// Shift the register left by one.
// 6 bits (& 0x3F) after each shift.
// If the result of step 1 was 1, XOR in 0b000011
// After 18 bits, the register must equal C5..C0. Write crc6() and parse_frame() as pure functions so tests/host can test them.

typedef struct {
    uint16_t data;  // Bit 18-23: 14-bit Angle Data D[13:0]
    uint8_t  mg;    // Bit 14-17: 4-bit Magnetic Field Status Mg[3:0]
    uint8_t  crc;   // Bit 0-13: 6-bit CRC Code CRC[5:0] 
} encoder_frame_t;

typedef enum {
    ENCODER_OK                  = 0,
    ENCODER_ERR_CRC             = 1,
    ENCODER_ERR_FIELD_WEAK      = 2,
    ENCODER_ERR_FIELD_STRONG    = 3,
    ENCODER_ERR_OVERSPEED       = 4, // or track loss in other words
} encoder_status_t;

typedef struct {
    uint32_t cnt_per_rev; // 16384
    float gear_ratio;
    float pll_bandwidth; // rad/s
    float Ts; //update period, s (50e-6 at 20 kHz)
    uint16_t max_bad_in_row; //how many bad frames itll tolerate before -> unhealthy
} encoder_config_t;

typedef struct {
    encoder_config_t cfg;
    uint16_t prev_data;
    int64_t count;
    float pos_estimate; // PLL state
    float vel_estimate; // PLL state
    float Kp;
    float Ki;
    uint32_t crc_err;
    uint32_t field_err;
    uint16_t bad_in_row;
    bool ready; // first good frame has happened
} encoder_t;

uint8_t encoder_crc6(uint32_t data_18);
void    encoder_unpack(uint32_t frame_24, encoder_frame_t *d_out); //frame -> struct
bool    encoder_frame_valid(const encoder_frame_t *frame); //crc matches
int32_t encoder_wrap(uint16_t current, uint16_t prev, uint32_t cpr);

void                encoder_init(encoder_t *encoder, const encoder_config_t *cfg);
encoder_status_t    encoder_update(encoder_t *encoder, uint32_t frame_24); // call per read
void                encoder_set_cnt(encoder_t *encoder, int64_t cnt); 

uint16_t encoder_get_data(const encoder_t *encoder); // to get theta_e
float encoder_get_motor_angle(const encoder_t *encoder);
float encoder_get_motor_velocity(const encoder_t *encoder);
float encoder_get_output_angle(const encoder_t *encoder);
float encoder_get_output_velocity(const encoder_t *encoder); // rad/s
bool encoder_is_healthy(const encoder_t *encoder);

#endif // ENCODER_MT6701_H