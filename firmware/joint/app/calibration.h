// Calibration API.
// Module: #5   Milestone: M1-M2
// TODO: implement (see robot-arm-todo.md for the 'done when' criteria).

#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint8_t pole_pairs; // 1-50 and equal to motors value when configured
    int8_t direction;   // -1 or +1
    float theta_m;      //mechanical rotor angle 
    float theta_e;      //electrical angle -- magnets d axis relative to phase A
    float theta_offset;
    float theta_cmd;
} calibration_data_t;

typedef struct {
    //undecided-----------------------------
} calibration_config_t;

typedef struct {
    uint8_t phase;
    uint8_t step_count; //steps taken in current state/phase
    float cmd_elec_angle; //current location of sweep
    calibration_travel_t travel;
    calibration_circular_t offsets; //runing sin/cos sums
    calibration_result_t error; //if it failed
    calibration_config_t *cfg;
} calibration_t;

typedef struct {
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
} calibration_result_t;

typedef struct {
    float sum_sin;
    float sum_cos;
    uint32_t n;
} calibration_circular_t;

typedef struct {
    float theta_m_last;
    float travel_m;
    bool has_last;
} calibration_travel_t;

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
