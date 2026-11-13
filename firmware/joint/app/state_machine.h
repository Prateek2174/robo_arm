// State machine API.
// Module: #11   Milestone: M2
// TODO: implement (see robot-arm-todo.md for the 'done when' criteria).

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdbool.h>
#include <stdatomic.h>

//any state can go to fault, even fault itself
//FAULT only exists to IDLE
//rejected request is ignored and returns error so it can be reported over CAN

typedef enum {
    STATE_IDLE          = 0,
    STATE_CALIBRATING   = 1, //CALIBRATION -> IDLE 
    STATE_ARMED         = 2, //gates enabled and current loop running at 0 current
    STATE_RUNNING       = 3,
    STATE_FAULT         = 4,
} joint_state_t;

typedef enum {
    EVENT_NONE               = 0,
    EVENT_ARM                = 1, //IDLE -> ARMED       from a CAN command
    EVENT_DISARM             = 2, //ARMED -> IDLE       from a CAN command
    EVENT_CALIBRATE          = 3, //IDLE -> CALIBRATING from a CAN command
    EVENT_CLEAR_FAULT        = 4, //FAULT -> IDLE
    EVENT_CALIBRATION_DONE   = 5, //CALIBRATING -> IDLE
    EVENT_CALIBRATION_FAILED = 6, //CALIBRATING -> FAULT
    EVENT_START              = 7, //ARMED -> RUNNING    from a CAN command
    EVENT_STOP               = 8, //RUNNING -> ARMED    from a CAN command
} joint_event_t;

//if commands stop or something goes wrong -> damp instead of going limp so the arm falls slowly

//ALLOWABLE TRANSITIONS
//RUNNING -> ARMED -> IDLE -----linux only sends those AFTER moving te arm to rest pose

void state_machine_init(void);
joint_state_t state_machine_get_state(void);
bool state_machine_request_event(joint_event_t event);
void state_machine_enter_fault(void);
void state_machine_state_name(joint_state_t state, char *out_name, size_t out_size);

#endif // STATE_MACHINE_H