// Hall sensor API.
// Module: #17   Milestone: M5
// TODO: implement (see robot-arm-todo.md for the 'done when' criteria).

#ifndef HALL_H
#define HALL_H

#include <stdint.h>
#include <stdbool.h>

// 3 hall switches spaced 120 degrees apart in ece angle
// 6 sectors

// ORDER FOR MOTOR

// which code is sector 0 and what means -> fwd

// Calibration (#5) measures it: drive a slow open-loop electrical angle and log the hall code at each angle. 
// Store the result as a 6-entry lookup table in params (#13).

// typically -> 15 pole pairs --> 1 wheel turn == 15 ece turns == 90 hall edges

// Sector 

// 0 -> no change
// 1 -> fwd
// 2/3 --> skipped edge (glitch or too fast)
// -1 -> bwd

// going fwd -> theta_e = s * 60
// going bwd -> theta_e = (s+1) * 60

// keep error counter for skipped sectors and fault if it grows

// Angle with no info --> use sector centre: s * 60 + 30 (error is +- 30)

// Interpolating/Extracting the angle between edges:

// θe(t) = θ_edge + dir · ω_e · (t − t_edge),    clamped to the current sector
// ω_e   = 60° / Δt_last_sector   (optionally averaged over the last 6 edges)

// 3 switches -> [hall] -> theta_e (to Park), omega_e (to motion_control), edge count (odometry), faults

typedef struct {
    int8_t sector_lut[8]; // hall code -> sector 0...5, -1 = invalid (000/111) sector_lut[hall_code] = sector
    int8_t dir_sign;      // +1/-1 so it matches the wheel's fwd
    int8_t pole_pairs;    // likely 15 ----> check during testing ---- the ece turns in 1 wheel turn
    float tick_s;                   // timer tick period in seconds
    float omega_interpolate_on;     // rad/s ece --> interpolate above this value
    float omega_interpolate_off;    // lower threshold (hysteresis) 
    float timeout_s;                // reset omega = 0 if the timeout is hit
    float theta_offset_e;
} hall_config_t;

typedef enum {
    HALL_OK,
    HALL_ERR_INVALID_CODE,
    HALL_ERR_SKIPPED_SECTOR
} hall_status_t;

typedef struct {
    hall_config_t cfg;
    int8_t sector;
    int8_t prev_sector;
    int8_t dir;                 // +1, -1, 0 = unknown
    float sector_period[6];     // prev 6 sector times (1 ece turn) --> to average
    uint8_t period_index; // the next index the next sample goes in
    uint8_t period_count; // number of slots with valid data
    bool ignore_next_dt; 
    float omega_e;              // rad/s ece
    float theta_edge;           // angle at the last edge
    bool interpolation_active;
    int32_t edge_count; // for odometry
    uint32_t error_count; 
} hall_t;

bool hall_init(hall_t *hall, const hall_config_t *cfg);
hall_status_t hall_set_initial_sector(hall_t *hall, uint8_t code); // at startup we need to tell the module which sector it starts in
hall_status_t hall_on_edge(hall_t *hall, uint8_t code, uint32_t dt_ticks); // called 6 times per ece turn
float hall_get_theta_e(hall_t *hall, uint32_t ticks_since_edge); // use in 20 kHz current loop
float hall_get_omega_e(const hall_t *hall);
void hall_on_timeout(hall_t *hall); 
int32_t hall_get_edge_count(const hall_t *hall);

void hall_hardware_init(void);
uint8_t hall_hardware_read_code(void); // read PB6/PB7/PB8 ----> 3-bit code

#endif // HALL_H
