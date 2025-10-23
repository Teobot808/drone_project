// mavlink_functions.h - Header file for MAVLink communication functions

#ifndef MAVLINK_FUNCTIONS_H
#define MAVLINK_FUNCTIONS_H

#include <Arduino.h>
#include "MAVLINKmsg.h"

// Initialize MAVLink communication
void mavlink_init(HardwareSerial &serial, long baud_rate = MAVLINK_BAUD);

// Process incoming MAVLink messages
void mavlink_receive();

// Send heartbeat to Pixhawk
void mavlink_send_heartbeat();

// Check if Pixhawk is connected
bool mavlink_is_connected();

// Print telemetry data to serial monitor
void mavlink_print_telemetry();

// Get specific telemetry data
void mavlink_get_attitude(float* roll, float* pitch, float* yaw);
void mavlink_get_position(double* lat, double* lon, float* alt);
float mavlink_get_battery_voltage();
int mavlink_get_battery_percent();

// Get full telemetry structure (advanced)
const TelemetryData* mavlink_get_telemetry();

// Get communication statistics
void mavlink_get_stats(uint32_t* total_msgs, uint32_t* heartbeats);

#endif // MAVLINK_FUNCTIONS_H