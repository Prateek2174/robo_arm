// IDLE -> CALIBRATING -> ARMED -> RUNNING -> FAULT.
// Module: #11   Milestone: M2
// TODO: implement (see robot-arm-todo.md for the 'done when' criteria).

#include "state_machine.h"

//This file only tracks the state, it never uses PWM or HAL

//state machine learns if any faults are still active via faults.c
//a clear request is ignored if any faults are still active, and the state machine stays in FAULT until all faults are cleared

void state_machine_init(void){


}

joint_state_t state_machine_get_state(void){


}

bool state_machine_request_event(joint_event_t event){


}

void state_machine_enter_fault(void){


}

void state_machine_state_name(joint_state_t state, char *out_name, size_t out_size){

    
}