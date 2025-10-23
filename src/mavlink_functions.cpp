// mavlink_functions.cpp - MAVLink communication using official library

#include "mavlink_functions.h"
#include "MAVLINKmsg.h"

// Global variables
static HardwareSerial* mavlink_serial = nullptr;
static bool pixhawk_connected = false;
static unsigned long last_heartbeat_time = 0;
static uint8_t packet_sequence = 0;

// Telemetry data storage
static TelemetryData telemetry;

// MAVLink communication channel
static mavlink_message_t msg;
static mavlink_status_t status;

// Statistics
static uint32_t messages_received = 0;
static uint32_t heartbeats_received = 0;

// Initialize MAVLink communication
void mavlink_init(HardwareSerial &serial, long baud_rate) {
    mavlink_serial = &serial;
    mavlink_serial->begin(baud_rate, SERIAL_8N1, MAVLINK_RX_PIN, MAVLINK_TX_PIN);
    
    // Initialize telemetry structure
    memset(&telemetry, 0, sizeof(TelemetryData));
    
    Serial.println("[MAVLink] Initialized on UART2");
    Serial.printf("[MAVLink] RX Pin: %d, TX Pin: %d, Baud: %ld\n", 
                  MAVLINK_RX_PIN, MAVLINK_TX_PIN, baud_rate);
}

// Process incoming MAVLink messages
void mavlink_receive() {
    if (!mavlink_serial) return;
    
    while (mavlink_serial->available() > 0) {
        uint8_t c = mavlink_serial->read();
        
        // Try to parse a MAVLink message
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
            messages_received++;
            
            // Update connection status
            pixhawk_connected = true;
            last_heartbeat_time = millis();
            
            // Process message based on ID
            switch (msg.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT: {
                    mavlink_heartbeat_t hb;
                    mavlink_msg_heartbeat_decode(&msg, &hb);
                    
                    telemetry.heartbeat.type = hb.type;
                    telemetry.heartbeat.autopilot = hb.autopilot;
                    telemetry.heartbeat.base_mode = hb.base_mode;
                    telemetry.heartbeat.system_status = hb.system_status;
                    telemetry.heartbeat.custom_mode = hb.custom_mode;
                    telemetry.heartbeat.valid = true;
                    
                    heartbeats_received++;
                    break;
                }
                
                case MAVLINK_MSG_ID_ATTITUDE: {
                    mavlink_attitude_t att;
                    mavlink_msg_attitude_decode(&msg, &att);
                    
                    telemetry.attitude.roll = att.roll;
                    telemetry.attitude.pitch = att.pitch;
                    telemetry.attitude.yaw = att.yaw;
                    telemetry.attitude.rollspeed = att.rollspeed;
                    telemetry.attitude.pitchspeed = att.pitchspeed;
                    telemetry.attitude.yawspeed = att.yawspeed;
                    telemetry.attitude.time_boot_ms = att.time_boot_ms;
                    telemetry.attitude.valid = true;
                    break;
                }
                
                case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
                    mavlink_global_position_int_t pos;
                    mavlink_msg_global_position_int_decode(&msg, &pos);
                    
                    telemetry.gps_position.lat = pos.lat / 1e7;
                    telemetry.gps_position.lon = pos.lon / 1e7;
                    telemetry.gps_position.alt = pos.alt / 1000.0;
                    telemetry.gps_position.relative_alt = pos.relative_alt / 1000.0;
                    telemetry.gps_position.vx = pos.vx;
                    telemetry.gps_position.vy = pos.vy;
                    telemetry.gps_position.vz = pos.vz;
                    telemetry.gps_position.hdg = pos.hdg;
                    telemetry.gps_position.valid = true;
                    break;
                }
                
                case MAVLINK_MSG_ID_VFR_HUD: {
                    mavlink_vfr_hud_t hud;
                    mavlink_msg_vfr_hud_decode(&msg, &hud);
                    
                    telemetry.vfr_hud.airspeed = hud.airspeed;
                    telemetry.vfr_hud.groundspeed = hud.groundspeed;
                    telemetry.vfr_hud.alt = hud.alt;
                    telemetry.vfr_hud.climb = hud.climb;
                    telemetry.vfr_hud.heading = hud.heading;
                    telemetry.vfr_hud.throttle = hud.throttle;
                    telemetry.vfr_hud.valid = true;
                    break;
                }
                
                case MAVLINK_MSG_ID_SYS_STATUS: {
                    mavlink_sys_status_t sys;
                    mavlink_msg_sys_status_decode(&msg, &sys);
                    
                    telemetry.sys_status.voltage_battery = sys.voltage_battery / 1000.0;
                    telemetry.sys_status.current_battery = sys.current_battery / 100.0;
                    telemetry.sys_status.battery_remaining = sys.battery_remaining;
                    telemetry.sys_status.load = sys.load / 10;
                    telemetry.sys_status.valid = true;
                    break;
                }
                
                case MAVLINK_MSG_ID_RC_CHANNELS: {
                    mavlink_rc_channels_t rc;
                    mavlink_msg_rc_channels_decode(&msg, &rc);
                    
                    telemetry.rc_channels.chan1_raw = rc.chan1_raw;
                    telemetry.rc_channels.chan2_raw = rc.chan2_raw;
                    telemetry.rc_channels.chan3_raw = rc.chan3_raw;
                    telemetry.rc_channels.chan4_raw = rc.chan4_raw;
                    telemetry.rc_channels.chan5_raw = rc.chan5_raw;
                    telemetry.rc_channels.chan6_raw = rc.chan6_raw;
                    telemetry.rc_channels.chan7_raw = rc.chan7_raw;
                    telemetry.rc_channels.chan8_raw = rc.chan8_raw;
                    telemetry.rc_channels.rssi = rc.rssi;
                    telemetry.rc_channels.valid = true;
                    break;
                }
            }
        }
    }
    
    // Check connection timeout (no data for 3 seconds)
    if (pixhawk_connected && (millis() - last_heartbeat_time > 3000)) {
        pixhawk_connected = false;
    }
}

// Send heartbeat to Pixhawk
void mavlink_send_heartbeat() {
    if (!mavlink_serial) return;
    
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    
    // Pack heartbeat message
    mavlink_msg_heartbeat_pack(
        ESP32_SYSTEM_ID,
        ESP32_COMPONENT_ID,
        &msg,
        MAV_TYPE_ONBOARD_CONTROLLER,
        MAV_AUTOPILOT_INVALID,
        0,  // base_mode
        0,  // custom_mode
        MAV_STATE_ACTIVE
    );
    
    // Send message
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    mavlink_serial->write(buf, len);
}

// Check if Pixhawk is connected
bool mavlink_is_connected() {
    return pixhawk_connected;
}

// Print telemetry data to serial
void mavlink_print_telemetry() {
    if (!pixhawk_connected) {
        Serial.println("\n========================================");
        Serial.println("   PIXHAWK NOT CONNECTED");
        Serial.println("   Please connect Pixhawk to ESP32");
        Serial.println("========================================\n");
        return;
    }
    
    Serial.println("\n======== DRONE TELEMETRY ========");
    Serial.printf("Messages: %u | Heartbeats: %u\n", messages_received, heartbeats_received);
    
    // Heartbeat / System Status
    if (telemetry.heartbeat.valid) {
        Serial.printf("\nSystem:\n");
        Serial.printf("  Type: %d | Autopilot: %d\n", 
                      telemetry.heartbeat.type, telemetry.heartbeat.autopilot);
        Serial.printf("  Mode: 0x%02X | Status: %d\n", 
                      telemetry.heartbeat.base_mode, telemetry.heartbeat.system_status);
    }
    
    // Attitude
    if (telemetry.attitude.valid) {
        Serial.printf("\nAttitude:\n");
        Serial.printf("  Roll:  %7.2f° (%6.3f rad/s)\n", 
                      telemetry.attitude.roll * 57.2958, telemetry.attitude.rollspeed);
        Serial.printf("  Pitch: %7.2f° (%6.3f rad/s)\n", 
                      telemetry.attitude.pitch * 57.2958, telemetry.attitude.pitchspeed);
        Serial.printf("  Yaw:   %7.2f° (%6.3f rad/s)\n", 
                      telemetry.attitude.yaw * 57.2958, telemetry.attitude.yawspeed);
    }
    
    // GPS Position
    if (telemetry.gps_position.valid) {
        Serial.printf("\nGPS Position:\n");
        Serial.printf("  Lat: %11.7f°\n", telemetry.gps_position.lat);
        Serial.printf("  Lon: %11.7f°\n", telemetry.gps_position.lon);
        Serial.printf("  Alt: %7.2f m (MSL) | %7.2f m (AGL)\n", 
                      telemetry.gps_position.alt, telemetry.gps_position.relative_alt);
        Serial.printf("  Heading: %6.2f°\n", telemetry.gps_position.hdg / 100.0);
    }
    
    // Velocity and Flight Data
    if (telemetry.vfr_hud.valid) {
        Serial.printf("\nFlight Data:\n");
        Serial.printf("  Ground Speed: %6.2f m/s\n", telemetry.vfr_hud.groundspeed);
        Serial.printf("  Air Speed:    %6.2f m/s\n", telemetry.vfr_hud.airspeed);
        Serial.printf("  Climb Rate:   %6.2f m/s\n", telemetry.vfr_hud.climb);
        Serial.printf("  Heading:      %6d°\n", telemetry.vfr_hud.heading);
        Serial.printf("  Throttle:     %6d%%\n", telemetry.vfr_hud.throttle);
    }
    
    // Battery
    if (telemetry.sys_status.valid) {
        Serial.printf("\nBattery:\n");
        Serial.printf("  Voltage:  %5.2f V\n", telemetry.sys_status.voltage_battery);
        Serial.printf("  Current:  %6.2f A\n", telemetry.sys_status.current_battery);
        Serial.printf("  Remaining: %4d%%\n", telemetry.sys_status.battery_remaining);
        Serial.printf("  CPU Load:  %4d%%\n", telemetry.sys_status.load);
    }
    
    Serial.println("=================================\n");
}

// Get latest telemetry data (for other modules)
void mavlink_get_attitude(float* roll, float* pitch, float* yaw) {
    if (roll) *roll = telemetry.attitude.roll;
    if (pitch) *pitch = telemetry.attitude.pitch;
    if (yaw) *yaw = telemetry.attitude.yaw;
}

void mavlink_get_position(double* lat, double* lon, float* alt) {
    if (lat) *lat = telemetry.gps_position.lat;
    if (lon) *lon = telemetry.gps_position.lon;
    if (alt) *alt = telemetry.gps_position.alt;
}

float mavlink_get_battery_voltage() {
    return telemetry.sys_status.voltage_battery;
}

int mavlink_get_battery_percent() {
    return telemetry.sys_status.battery_remaining;
}

// Get full telemetry structure (advanced use)
const TelemetryData* mavlink_get_telemetry() {
    return &telemetry;
}

// Get statistics
void mavlink_get_stats(uint32_t* total_msgs, uint32_t* heartbeats) {
    if (total_msgs) *total_msgs = messages_received;
    if (heartbeats) *heartbeats = heartbeats_received;
}