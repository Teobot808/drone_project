#include <Arduino.h>
#include "mavlink_functions.h"
//#include <config.h>
#include <tasks.cpp>


// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================

// Function to safely stop all tasks (for emergency)
void trigger_emergency_stop() {
    emergency_stop = true;
    Serial.println("\n[EMERGENCY] EMERGENCY STOP TRIGGERED!");
    
    // Could add code here to:
    // - Send emergency stop command to Pixhawk
    // - Cut power to non-essential systems
    // - Log critical data
}

// Get system uptime
unsigned long get_system_uptime() {
    return millis() / 1000;  // Convert to seconds
}

// Print detailed system diagnostics
void print_system_diagnostics() {
    Serial.println("\n========== SYSTEM DIAGNOSTICS ==========");
    Serial.printf("Uptime: %lu seconds\n", get_system_uptime());
    Serial.printf("CPU Frequency: %d MHz\n", ESP.getCpuFreqMHz());
    Serial.printf("Free Heap: %d / %d bytes\n", 
                  ESP.getFreeHeap(), ESP.getHeapSize());
    Serial.printf("Min Free Heap Ever: %d bytes\n", ESP.getMinFreeHeap());
    Serial.printf("PSRAM: %s\n", psramFound() ? "Found" : "Not Found");
    
    // Task information
    Serial.println("\nTask Status:");
    Serial.printf("  MAVLink:        %s\n", 
                  Task_MAVLink_Handle ? "Running" : "Stopped");
    Serial.printf("  Communication:  %s\n", 
                  Task_Communication_Handle ? "Running" : "Stopped");
    Serial.printf("  Sensors:        %s\n", 
                  Task_Sensors_Handle ? "Running" : "Stopped");
    Serial.printf("  Computation:    %s\n", 
                  Task_Computation_Handle ? "Running" : "Stopped");
    
    Serial.println("========================================\n");
}

void init(){
// Initialize serial for debugging
    Serial.begin(115200);
    delay(1000);  // Wait for serial to initialize
    
    Serial.println("\n\n");
    Serial.println("========================================");
    Serial.println("  ESP32 DRONE BRAIN - INITIALIZING");
    Serial.println("========================================");
    
    // Print ESP32 info
    Serial.printf("ESP32 Chip: %s\n", ESP.getChipModel());
    Serial.printf("Chip Revision: %d\n", ESP.getChipRevision());
    Serial.printf("CPU Frequency: %d MHz\n", ESP.getCpuFreqMHz());
    Serial.printf("Flash Size: %d MB\n", ESP.getFlashChipSize() / (1024 * 1024));
    Serial.printf("Free Heap: %d bytes\n", ESP.getFreeHeap());
    Serial.println();
    
    // Initialize MAVLink communication with Pixhawk
    Serial.println("[INIT] Initializing MAVLink...");
    mavlink_init(PixhawkSerial, MAVLINK_BAUD);
    delay(500);
    
    // TODO: Initialize SPI for slave microcontrollers
    Serial.println("[INIT] Initializing SPI communication...");
    // SPI.begin();
    
    // TODO: Initialize sensors
    Serial.println("[INIT] Initializing sensors...");
    // Initialize your sensors here
    
    Serial.println("\n[INIT] Creating FreeRTOS tasks...");

}

extern HardwareSerial PixhawkSerial;  // The UART connected to Pixhawk

void mavlink_send_tof_distance(uint8_t sensor_id, uint16_t distance_mm, uint8_t signal_quality) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_distance_sensor_t sensor_msg;
    sensor_msg.time_boot_ms = millis(); // or TOF_0.system_time if synced
    sensor_msg.min_distance = 200;       // minimum measurable distance [mm]
    sensor_msg.max_distance = 50000;     // maximum measurable distance [mm]
    sensor_msg.current_distance = distance_mm;
    sensor_msg.type = MAV_DISTANCE_SENSOR_LASER; // laser TOF
    sensor_msg.id = sensor_id;
    sensor_msg.orientation = MAV_SENSOR_ROTATION_PITCH_270; // adjust orientation if needed
    sensor_msg.covariance = 0;          // if unknown
    sensor_msg.signal_quality = signal_quality; // optional

    mavlink_msg_distance_sensor_encode(1, 200, &msg, &sensor_msg); // sysid=1, compid=200
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    PixhawkSerial.write(buf, len);
}