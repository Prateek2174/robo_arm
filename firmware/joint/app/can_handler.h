// CAN handler API.
// Module: #12   Milestone: M2
// TODO: implement (see robot-arm-todo.md for the 'done when' criteria).

#ifndef CAN_HANDLER_H
#define CAN_HANDLER_H

#include <stdint.h>

typedef struct {
    //node ID
    //timeout in ms
    uint32_t timeout_ms;
    //fast periods

    //slow_periods
} can_handler_config_t;

typedef struct {

} can_handler_t;

can_handler_init();
can_handler_on_frame();
can_handler_tick(uint32_t now_ms);

#endif // CAN_HANDLER_H
