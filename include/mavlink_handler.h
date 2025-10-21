#ifndef MAVLINK_HANDLER_H
#define MAVLINK_HANDLER_H

#include <Arduino.h>
#include <mavlink.h>

// Structure to store all relevant telemetry data
struct TelemetryData {
    // System status
    uint8_t system_mode;
    uint8_t system_state;
    float battery_voltage;
    float battery_current;
    float battery_remaining;

    // Attitude
    float roll;
    float pitch;
    float yaw;

    // Global Position
    int32_t lat;
    int32_t lon;
    int32_t alt;

    // GPS
    uint8_t fix_type;
    uint8_t satellites_visible;

    // IMU
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float mag_x, mag_y, mag_z;

    // HUD / flight data
    float airspeed;
    float groundspeed;
    int16_t heading;
    uint16_t throttle;
    float climb_rate;
};

extern TelemetryData telemetryData;

void handleMavlinkMessage(mavlink_message_t* msg);
void printEssentialTelemetry();

#endif
