#ifndef CANBUS_H
#define CANBUS_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/twai.h"


// =======================================================
// DASH DATA STRUCTURE
// =======================================================
// Shared decoded ECU state.  protocol_loader maps JSON signal names into these
// fields, canbus_task updates them from incoming frames, and main.c consumes
// them when SENSOR_SOURCE_CAN is selected.

typedef struct{
    float rpm;
    float speed;
    float coolant_temp;
    float air_temp;
    float battery_voltage;
    float oil_pressure;
    float air_fuel_ratio;
    float boost;
    float fuel_comp;
} can_dash_data_t;


// Global decoded data.  It is volatile because it is written by the CAN receive
// task and read by the application mapping/UI tasks.
extern volatile can_dash_data_t can_data;



// =======================================================
// API
// =======================================================

void canbus_init(void);

// FreeRTOS receive task.  Call canbus_init() before creating this task.
void canbus_task(void *arg);

// Decode one standard 11-bit CAN frame using the detected protocol.
void process_can_frame(uint32_t id, uint8_t *data);

#endif
