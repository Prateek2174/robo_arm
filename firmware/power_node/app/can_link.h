// CAN link API.
// Module: #20   Milestone: M5
// TODO: implement (see robot-arm-todo.md for the 'done when' criteria).

#ifndef CAN_LINK_H
#define CAN_LINK_H

#include <stdint.h>

typedef struct {
    uint8_t node_id; // power node's fixed ID in the shared map
    uint32_t heartbeat_period_ms; // 100 ms (10 Hz)
    uint32_t status_period_ms; // 100 ms (10 Hz)
    uint32_t host_timeout_ms;
} can_link_config_t;

typedef struct {
    // power state
    // flags
    // pack voltage
    // pack current
} can_link_status_t;

void can_link_init(const can_link_config_t *cfg);


#endif // CAN_LINK_H
