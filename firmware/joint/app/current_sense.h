// Current sensing API.
// Module: #3   Milestone: M1-M2
// TODO: implement (see robot-arm-todo.md for the 'done when' criteria).

#ifndef CURRENT_SENSE_H
#define CURRENT_SENSE_H

#include <stdint.h>
#include <stdbool.h>
#include "foc_math.h"

typedef enum {
    CURRENT_SENSE_PHASE_A = 0,
    CURRENT_SENSE_PHASE_B = 1,
    CURRENT_SENSE_PHASE_C = 2
} current_sense_phase_t;

typedef struct {
    uint32_t sum[3];
    uint32_t count[3];
} current_sense_offset_acc_t;

void current_sense_offset_reset(current_sense_offset_acc_t *acc);
void current_sense_offset_add(current_sense_offset_acc_t *acc, current_sense_phase_t phase, uint32_t raw);
bool current_sense_offset_finish(const current_sense_offset_acc_t *acc, uint32_t min_samples, float offset_out[3]); //false = too few samples / outside window

float current_sense_cnt_to_amps(uint16_t raw, float offset_counts); //I = -(raw - offset) * k
float current_sense_cnt_to_vbus(uint16_t raw);

#endif // CURRENT_SENSE_H