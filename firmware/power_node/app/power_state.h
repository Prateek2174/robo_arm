// Power state API.
// Module: #19   Milestone: M5
// TODO: implement (see robot-arm-todo.md for the 'done when' criteria).

#ifndef POWER_STATE_H
#define POWER_STATE_H

typedef enum {
    STATE_OFF,
    STATE_PRECHARGE,
    STATE_ON,
    STATE_FAULT
} power_state_t;

#endif // POWER_STATE_H
