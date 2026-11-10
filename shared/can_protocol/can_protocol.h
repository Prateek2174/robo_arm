// Message IDs, payload structs, protocol version constant.
// Module: #22   Milestone: M2
// TODO: implement (see robot-arm-todo.md for the 'done when' criteria).

/*
CAN Protocol

I chose to split the 11-bit CAN ID into a 4-bit message type and a 7-bit node ID:

Message Type    Message	            Sender → receiver	    Rate	    Purpose

0x0             E-stop	            power node / PC → all	on event	everyone goes to safe-torque-off
0x1             Fault report	    joint → PC	            on event	which fault, plus a value
0x2             Command	            PC → joint	            100 Hz	    setpoints
0x3             Fast telemetry	    joint → PC	            100 Hz	    what ros2_control reads
0x4             State request	    PC → joint	            on event	arm, disarm, calibrate, clear fault
0x5             Heartbeat	        every node	            10 Hz	    “I’m alive” plus version
0x6             Slow status	        joint → PC	            10 Hz	    bus voltage, temperature
0x7             Parameter get/set	PC ↔ joint	            on demand	gains, limits, node ID

*/

#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

#define CAN_PROTOCOL_VERSION 1

typedef enum {
    CAN_MSG_TYPE_ESTOP             = 0x00,
    CAN_MSG_TYPE_FAULT_REPORT      = 0x01,
    CAN_MSG_TYPE_COMMAND           = 0x02,
    CAN_MSG_TYPE_FAST_TELEMETRY    = 0x03,
    CAN_MSG_TYPE_STATE_REQUEST     = 0x04,
    CAN_MSG_TYPE_HEARTBEAT         = 0x05,
    CAN_MSG_TYPE_SLOW_STATUS       = 0x06,
    CAN_MSG_TYPE_PARAMETER_GET_SET = 0x07,
} can_msg_id_t;

typedef struct {
    can_msg_id_t msg_id;
    uint8_t node_id; //7-bit node ID, 0-127
    uint8_t payload[8];
} can_t;

void can_protocol_init(void);
void can_pack(can_t *can_msg, const void *payload, uint8_t payload_size, uint8_t *out_buffer, uint8_t *out_size);
bool can_unpack(const uint8_t *buffer, uint8_t size, can_t *out_can_msg, void *out_payload, uint8_t *out_payload_size);


#endif // CAN_PROTOCOL_H
