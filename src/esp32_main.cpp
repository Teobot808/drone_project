// esp32_main.cpp - Main entry point for ESP32 Drone Brain
// Handles dual-core task management and coordinates all subsystems

#include <Arduino.h>
#include "mavlink_functions.h"

// FreeRTOS task handles
TaskHandle_t Task_MAVLink_Handle;
TaskHandle_t Task_Sensors_Handle;
TaskHandle_t Task_Computation_Handle;
TaskHandle_t Task_Communication_Handle;

// Task priorities (higher number = higher priority)
#define PRIORITY_MAVLINK 3      // High priority for flight-critical data
#define PRIORITY_SENSORS 2      // Medium-high for sensor reading
#define PRIORITY_COMPUTATION 1  // Medium for processing
#define PRIORITY_COMMUNICATION 1 // Medium for SPI communication

// Core assignment
#define CORE_PROTOCOL 0  // Core 0: Arduino loop, MAVLink, Communications
#define CORE_COMPUTE 1   // Core 1: Heavy computation, sensor processing

// Task timing
#define MAVLINK_UPDATE_RATE 50       // 50ms = 20Hz
#define TELEMETRY_PRINT_RATE 1000    // 1000ms = 1Hz
#define HEARTBEAT_RATE 1000          // 1000ms = 1Hz

// Global status flags
volatile bool system_initialized = false;
volatile bool emergency_stop = false;

// Serial for Pixhawk (UART2)
HardwareSerial PixhawkSerial(2);

// =============================================================================
// CORE 0 TASKS (Protocol & Communication)
// =============================================================================

// Task 1: MAVLink Communication with Pixhawk
void Task_MAVLink(void* parameter) {
    unsigned long last_heartbeat = 0;
    unsigned long last_telemetry_print = 0;
    
    Serial.println("[Task_MAVLink] Started on Core 0");
    
    while (true) {
        unsigned long current_time = millis();
        
        // Receive and parse MAVLink messages
        mavlink_receive();
        
        // In Task_MAVLink, add after mavlink_receive():
        if (PixhawkSerial.available()) {
            Serial.printf("[DEBUG] Bytes available: %d\n", PixhawkSerial.available());
}

        // Send heartbeat at 1Hz
        if (current_time - last_heartbeat >= HEARTBEAT_RATE) {
            mavlink_send_heartbeat();
            last_heartbeat = current_time;
        }
        
        // Print telemetry at 1Hz
        if (current_time - last_telemetry_print >= TELEMETRY_PRINT_RATE) {
            mavlink_print_telemetry();
            last_telemetry_print = current_time;
        }
        
        // Task delay (gives time to other tasks)
        vTaskDelay(pdMS_TO_TICKS(MAVLINK_UPDATE_RATE));
    }
}

// Task 2: SPI Communication with other microcontrollers
void Task_Communication(void* parameter) {
    Serial.println("[Task_Communication] Started on Core 0");
    
    // TODO: Initialize SPI for communication with slave microcontrollers
    // SPI.begin(SCK, MISO, MOSI, SS);
    
    while (true) {
        // TODO: Send/receive data to/from slave microcontrollers
        // - LED control data
        // - Servo position commands
        // - Sensor data from slaves
        
        vTaskDelay(pdMS_TO_TICKS(50));  // 20Hz update rate
    }
}

// =============================================================================
// CORE 1 TASKS (Computation & Sensors)
// =============================================================================

// Task 3: Sensor Reading and Processing
void Task_Sensors(void* parameter) {
    Serial.println("[Task_Sensors] Started on Core 1");
    
    // TODO: Initialize sensors not connected to Pixhawk
    // Examples:
    // - Additional IMU/Gyroscope
    // - Distance sensors (ultrasonic, lidar)
    // - Camera modules
    // - Environmental sensors
    
    while (true) {
        // TODO: Read sensor data
        // TODO: Apply filtering/calibration
        // TODO: Send to Pixhawk if needed (via MAVLink)
        
        vTaskDelay(pdMS_TO_TICKS(20));  // 50Hz sensor reading
    }
}

// Task 4: Heavy Computation
void Task_Computation(void* parameter) {
    Serial.println("[Task_Computation] Started on Core 1");
    
    while (true) {
        // TODO: Perform computationally intensive tasks:
        // - Computer vision processing
        // - Path planning algorithms
        // - Object detection/tracking
        // - AI/ML inference
        // - Navigation calculations
        
        // Example: Get current position and attitude for processing
        if (mavlink_is_connected()) {
            float roll, pitch, yaw;
            mavlink_get_attitude(&roll, &pitch, &yaw);
            
            // Use attitude data for computations...
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));  // 10Hz computation rate
    }
}

// =============================================================================
// INITIALIZATION
// =============================================================================

void setup() {
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