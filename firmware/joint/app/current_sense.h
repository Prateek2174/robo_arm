// Current sensing API.
// Module: #3   Milestone: M1-M2
// TODO: implement (see robot-arm-todo.md for the 'done when' criteria).

#ifndef CURRENT_SENSE_H
#define CURRENT_SENSE_H

#include <stdint.h>
#include <stdbool.h>
#include "foc_math.h"

//Start, add a raw sample, finish → offsets plus a pass/fail validity check
current_sense_offset_accum();
//raw + offset → amps, sign included
current_sense_cnt_to_amps();
//3 duties → which 2 phases to sample
current_sense_pick_phases();
//2 measured currents + which pair → abc_t for clarke()
current_sense_reconstruct();
//raw → bus volts
current_sense_cnt_to_volts();

#endif // CURRENT_SENSE_H
