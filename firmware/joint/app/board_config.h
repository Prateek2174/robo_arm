// Pins, shunt gain, dead-time, limits per board (ESC1 vs custom PCB).
// Module: -   Milestone: M1-M2
// TODO: implement (see robot-arm-todo.md for the 'done when' criteria).

#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

//------NAME-----------------VALUE-------------------USED IN------//

#define BOARD_NAME           "B-G431B-ESC1"          // app_main, telemetry
#define SYSCLK_HZ            170000000               // pwm
#define SHUNT_OHM            0.003f                  // current_sense

#define CSA_GAIN             9.14f                   // current_sense
#define CSA_INVERTING        1                       // current_sense
#define CSA_OFFSET_V                                 // 

#define ADC_VREF             3.3f                    // current_sense, VBUS, NTC
#define ADC_TOTAL_STEPS      4096.0f                 //
#define VBUS_DIV_RATIO       10.39f                  // current_sense, faults

#define PWM_FREQ_HZ          20000                   // pwm, isr (Ts = 1/f)
#define PWM_MAX_DUTY                                 // 

#define DEADTIME_NS                                  // 
#define I_PHASE_ABS_MAX_A    40.0f                   // faults
#define VBUS_ABS_MAX_V       26.0f                   // faults
#define VBUS_ABS_MIN_V       18.0f                   // 
#define ENCODER_INTERFACE                            // encoder_mt6701, calibration
#define DEBUG_PIN_PORT                               // isr
#define DEBUG_PIN                                    // isr
#define CAN_TERMINATE_PIN                            // can_driver
#define TEMP_ABS_MAX_C                               // faults
#define CAN_SHDN_PIN                                 //
#define CAN_BITRATE                                  // 
#define STATUS_LED_PIN                               // faults

#define NTC_R25_OHM                                  //
#define NTC_BETA                                     // 
#define NTC_PULLUP_OHM                               //

#endif // BOARD_CONFIG_H
