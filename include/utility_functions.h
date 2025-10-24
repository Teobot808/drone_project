// utility_functions.h
#ifndef UTILITY_FUNCTIONS_H
#define UTILITY_FUNCTIONS_H

#include <Arduino.h>
#include "mavlink.h"  // or mavlink/common/mavlink.h if needed

// Forward declarations for global variables (if used)
extern HardwareSerial PixhawkSerial;

// Declare your function here
void mavlink_send_tof_distance(uint8_t sensor_id, uint16_t distance_mm, uint8_t signal_quality);

#endif // UTILITY_FUNCTIONS_H
