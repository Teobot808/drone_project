// esp32_main.cpp - Main entry point for ESP32 Drone Brain
// Handles dual-core task management and coordinates all subsystems

#include <Arduino.h>
#include "mavlink_functions.h"
//#include <config.h>
//#include <tasks.cpp>
#include <utility_functions.cpp>




// =============================================================================
// INITIALIZATION
// =============================================================================

void setup() {
    
    
    init();
    
    // Create Core 0 tasks (Protocol & Communication)
    xTaskCreatePinnedToCore(
        Task_MAVLink,           // Task function
        "MAVLink",              // Task name
        4096,                   // Stack size (bytes)
        NULL,                   // Parameters
        PRIORITY_MAVLINK,       // Priority
        &Task_MAVLink_Handle,   // Task handle
        CORE_PROTOCOL           // Core ID
    );
    
    xTaskCreatePinnedToCore(
        Task_Communication,
        "Communication",
        2048,
        NULL,
        PRIORITY_COMMUNICATION,
        &Task_Communication_Handle,
        CORE_PROTOCOL
    );
    
    // Create Core 1 tasks (Computation & Sensors)
    xTaskCreatePinnedToCore(
        Task_Sensors,
        "Sensors",
        4096,
        NULL,
        PRIORITY_SENSORS,
        &Task_Sensors_Handle,
        CORE_COMPUTE
    );
    
    xTaskCreatePinnedToCore(
        Task_Computation,
        "Computation",
        8192,                   // Larger stack for heavy computation
        NULL,
        PRIORITY_COMPUTATION,
        &Task_Computation_Handle,
        CORE_COMPUTE
    );
    
    Serial.println("[INIT] All tasks created successfully!");
    Serial.println("\n========================================");
    Serial.println("  SYSTEM READY - WAITING FOR PIXHAWK");
    Serial.println("========================================\n");
    
    system_initialized = true;
}

// =============================================================================
// MAIN LOOP (Runs on Core 1 by default in Arduino)
// =============================================================================

void loop() {
    // Main loop can be used for low-priority monitoring tasks
    // Most work is done in FreeRTOS tasks
    
    // Monitor system health
    static unsigned long last_health_check = 0;
    if (millis() - last_health_check > 5000) {  // Every 5 seconds
        // Print task status and memory info
        Serial.println("\n[SYSTEM] Health Check:");
        Serial.printf("  Free Heap: %d bytes\n", ESP.getFreeHeap());
        Serial.printf("  Min Free Heap: %d bytes\n", ESP.getMinFreeHeap());
        Serial.printf("  Pixhawk Connected: %s\n", 
                      mavlink_is_connected() ? "YES" : "NO");
        
        // Check for stack overflows or task failures
        if (Task_MAVLink_Handle != NULL) {
            UBaseType_t stack_watermark = uxTaskGetStackHighWaterMark(Task_MAVLink_Handle);
            Serial.printf("  MAVLink Task Stack Free: %d bytes\n", stack_watermark * 4);
        }
        
        last_health_check = millis();
    }
    
    // Emergency stop check
    if (emergency_stop) {
        Serial.println("[EMERGENCY] SYSTEM HALTED");
        // Implement emergency procedures here
        while (true) {
            delay(1000);
        }
    }
    
    delay(100);  // Don't hog the CPU
}

