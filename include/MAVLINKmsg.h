// MAVLINKmsg.h - MAVLink wrapper using official ArduPilot library
// This file provides a clean interface to MAVLink functionality

#ifndef MAVLINK_MSG_H
#define MAVLINK_MSG_H

#include <Arduino.h>

// Include the official MAVLink library (okalachev/MAVLink)
#include <mavlink.h>

// UART pins for Pixhawk communication
#define MAVLINK_RX_PIN 16  // ESP32 RX <- Pixhawk TX
#define MAVLINK_TX_PIN 17  // ESP32 TX -> Pixhawk RX
#define MAVLINK_BAUD 57600

// MAVLink system IDs
#define ESP32_SYSTEM_ID 2
#define ESP32_COMPONENT_ID MAV_COMP_ID_ONBOARD_COMPUTER

// Telemetry data structure for easy access
struct TelemetryData {
    // Heartbeat
    struct {
        uint8_t type;
        uint8_t autopilot;
        uint8_t base_mode;
        uint8_t system_status;
        uint32_t custom_mode;
        bool valid;
    } heartbeat;
    
    // Attitude (roll, pitch, yaw)
    struct {
        float roll;       // radians
        float pitch;      // radians
        float yaw;        // radians
        float rollspeed;  // rad/s
        float pitchspeed; // rad/s
        float yawspeed;   // rad/s
        uint32_t time_boot_ms;
        bool valid;
    } attitude;
    
    // GPS Position
    struct {
        double lat;       // degrees
        double lon;       // degrees
        float alt;        // meters MSL
        float relative_alt; // meters AGL
        int16_t vx;       // cm/s
        int16_t vy;       // cm/s
        int16_t vz;       // cm/s
        uint16_t hdg;     // degrees * 100
        bool valid;
    } gps_position;
    
    // VFR HUD (velocity, heading, altitude)
    struct {
        float airspeed;     // m/s
        float groundspeed;  // m/s
        float alt;          // m MSL
        float climb;        // m/s
        int16_t heading;    // degrees
        uint16_t throttle;  // percent
        bool valid;
    } vfr_hud;
    
    // System Status
    struct {
        float voltage_battery;  // Volts
        float current_battery;  // Amps
        int8_t battery_remaining; // percent
        uint16_t load;          // CPU load %
        bool valid;
    } sys_status;
    
    // RC Channels
    struct {
        uint16_t chan1_raw;
        uint16_t chan2_raw;
        uint16_t chan3_raw;
        uint16_t chan4_raw;
        uint16_t chan5_raw;
        uint16_t chan6_raw;
        uint16_t chan7_raw;
        uint16_t chan8_raw;
        uint8_t rssi;
        bool valid;
    } rc_channels;
};

#endif // MAVLINK_MSG_H