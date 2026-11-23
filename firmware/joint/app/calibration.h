// Calibration API.
// Module: #5   Milestone: M1-M2
// TODO: implement (see robot-arm-todo.md for the 'done when' criteria).

#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdint.h>
#include <stdbool.h>

typedef struct { // OUTPUT STRUCT
    uint8_t pole_pairs;       // 1-50 and equal to motors value when configured
    int8_t  dir;              // -1 or +1
    float   theta_offset;
    float   theta_offset_ref; // what the stored θ_off is measured from
} calibration_data_t;

typedef struct {
    float    v_d;                   // volt on d-axis during drive phase ---------- v_d = R * calibration curent
    uint16_t steps_per_ece_turn; 
    uint16_t align_steps;           // to hold theta_cmd = 0 while rotor settles
    uint8_t  detect_turns;          // N, ece turns in DETECT state ------------- travel_e = N * 2pi
    uint8_t  offset_turns; 
    float    min_travel_m;          // smallest mechanical travel that counts as movement [rad]
    float    pole_pairs_tolerance; 
    float    r_min;                 // smallest radius allowed from circular_mean
    uint8_t  expected_pole_pairs;   // pole_pairs from the datasheet
} calibration_config_t;

typedef struct { // [CALIBRATION] -------> [FOC_MATH]
    float v_d;
    bool drive;
    float theta_cmd;
} calibration_output_t;

typedef enum {
    CALIBRATION_STATE_IDLE          = 0,
    CALIBRATION_STATE_ALIGN         = 1,
    CALIBRATION_STATE_DETECT        = 2,
    CALIBRATION_STATE_OFFSET_BACK   = 3,
    CALIBRATION_STATE_OFFSET_FWD    = 4,
    CALIBRATION_STATE_DONE          = 5,
    CALIBRATION_STATE_FAILED        = 6,
} calibration_state_t;

typedef enum {
    CALIBRATION_OK              = 0,
    CALIBRATION_ERR_NO_MOTION   = 1,
    CALIBRATION_ERR_POLE_PAIRS  = 2,
    CALIBRATION_ERR_SPREAD      = 3,
    CALIBRATION_ERR_BAD_DATA    = 4,
    CALIBRATION_ERR_NOT_DONE    = 5,
} calibration_result_t;

typedef struct {
    float    sum_sin;
    float    sum_cos;
    uint32_t n;
} calibration_circular_t;

typedef struct {
    float theta_m_last;
    float travel_m;
    bool  has_last;
} calibration_travel_t;

typedef struct { // TEMP STRUCT -------> OUTPUT STRUCT
    uint16_t step_count;    //steps taken in current state/phase --- 4 turns x 64 steps = 256
    float    step;
    float    theta_cmd; //current location of sweep
    float    theta_cmd_prev;
    uint8_t  curr_pole_pairs;
    int8_t   curr_dir;
    float    theta_offset;

    calibration_state_t    phase;
    calibration_travel_t   travel;
    calibration_circular_t offsets; //runing sin/cos sums
    calibration_result_t   error;   //if it failed
    calibration_config_t   cfg;
} calibration_t;

void calibration_travel_reset(calibration_travel_t *travel);
void calibration_travel_add(calibration_travel_t *travel, float theta_m);

calibration_result_t calibration_detect_direction(float travel_m, float min_travel_m, int8_t *dir);
calibration_result_t calibration_estimate_pole_pairs(float travel_e, float travel_m, float tolerance, uint8_t *pole_pairs);

void calibration_circular_reset(calibration_circular_t *cir);
void calibration_circular_add(calibration_circular_t *cir, float angle);
bool calibration_circular_mean(const calibration_circular_t *cir, float *mean, float *r);

float calibration_wrap_2pi(float x);
float calibration_angle_diff(float start, float end);
float calibration_offset_sample(float theta_m, float theta_cmd, uint8_t pole_pairs, int8_t dir);

bool calibration_is_valid(const calibration_data_t *data, uint8_t expected_pole_pairs);

void calibration_start(calibration_t *cal, const calibration_config_t *cfg);
calibration_output_t calibration_step(calibration_t *cal, float theta_m); //call this at a fixed rate/freq
calibration_state_t calibration_state(const calibration_t *cal);
calibration_result_t calibration_result(const calibration_t *cal, calibration_data_t *d_out);

#endif // CALIBRATION_H