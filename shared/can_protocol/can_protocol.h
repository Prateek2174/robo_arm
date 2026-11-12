// Message IDs, payload structs, protocol version constant.
// Module: #22   Milestone: M2
// TODO: implement (see robot-arm-todo.md for the 'done when' criteria).

//TODO: the power node also sends a heartbeat, but it has a different state machine (OFF → PRECHARGE → ON → FAULT). 
//      The PC knows which node is the power node from the ID, so you could give it its own state enum.

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

#define CAN_HEARTBEAT_LEN         4
#define CAN_COMMAND_LEN           7 
#define CAN_FAULT_REPORT_LEN      4
#define CAN_FAST_TELEMETRY_LEN    8
#define CAN_STATE_REQUEST_LEN     1
#define CAN_SLOW_STATUS_LEN       8
#define CAN_PARAMETER_GET_SET_LEN 8

typedef enum {
    CAN_MSG_TYPE_ESTOP             = 0x00,
    CAN_MSG_TYPE_FAULT_REPORT      = 0x01,
    CAN_MSG_TYPE_COMMAND           = 0x02,
    CAN_MSG_TYPE_FAST_TELEMETRY    = 0x03,
    CAN_MSG_TYPE_STATE_REQUEST     = 0x04,
    CAN_MSG_TYPE_HEARTBEAT         = 0x05,
    CAN_MSG_TYPE_SLOW_STATUS       = 0x06,
    CAN_MSG_TYPE_PARAMETER_GET_SET = 0x07,
} can_msg_type_t;

//Each message type will get its own struct for the payload, and a function to pack/unpack it for simplicity. 
// The CAN message struct will be the same for all messages, just with different payloads.

//========================================================
// Heartbeat message
//========================================================

typedef enum {
    CAN_JOINT_STATE_IDLE        = 0,
    CAN_JOINT_STATE_CALIBRATING = 1,
    CAN_JOINT_STATE_ARMED       = 2,
    CAN_JOINT_STATE_RUNNING     = 3,
    CAN_JOINT_STATE_FAULT       = 4,
} can_joint_state_t;

typedef struct {
    uint8_t state; //to see the joints state IDLE, CALIBRATING, ARMED, RUNNING, FAULT
    uint8_t protocol_version; //version check happens at startup just once
    uint8_t fault_active; //to see if a fault is active, 1 = yes, 0 = no
    uint8_t counter; //to detect missed messages, incremented by the sender and checked by the receiver
} can_heartbeat_t;

//========================================================
// State request message
//========================================================

typedef enum {
    CAN_STATE_REQUEST_NONE = 0,
    CAN_STATE_REQUEST_ARM = 1,
    CAN_STATE_REQUEST_DISARM = 2,
    CAN_STATE_REQUEST_CALIBRATE = 3,
    CAN_STATE_REQUEST_CLEAR_FAULT = 4,
} can_state_request_t;

typedef struct {
    uint8_t request; //can_state_request_t
} can_state_request_msg_t;

//========================================================
// Command message
//========================================================

typedef enum {
    CAN_COMMAND_MODE_POSITION = 0,
    CAN_COMMAND_MODE_VELOCITY = 1,
    CAN_COMMAND_MODE_TORQUE   = 2,
} can_command_mode_t;

typedef struct {
    uint8_t mode; //can_command_mode_t
    int16_t position; //position setpoint in encoder counts
    int16_t velocity; //velocity setpoint in encoder counts per second
    int16_t torque; //torque setpoint in mNm
} can_command_t;

//========================================================
// Fast telemetry message
//========================================================


//========================================================
// Slow status message
//========================================================

//========================================================
// E-stop message
//========================================================


//========================================================
// Parameter get/set message
//========================================================

void can_pack_heartbeat(can_heartbeat_t *heartbeat, uint8_t *out_buffer, uint8_t *out_size);
// void can_pack(can_t *can_msg, const void *payload, uint8_t payload_size, uint8_t *out_buffer, uint8_t *out_size);
// bool can_unpack(const uint8_t *buffer, uint8_t size, can_t *out_can_msg, void *out_payload, uint8_t *out_payload_size);


#endif // CAN_PROTOCOL_H
