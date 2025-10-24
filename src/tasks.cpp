#include <Arduino.h>
#include "mavlink_functions.h"
#include <config.h>
#include "TOF_Sense.cpp"
#include <utility_functions.h>

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
        mavlink_send_tof_distance(TOF_0.id, TOF_0.dis, TOF_0.signal_strength);
        vTaskDelay(pdMS_TO_TICKS(100));  // 20Hz update rate
    }
}

// =============================================================================
// CORE 1 TASKS (Computation & Sensors)
// =============================================================================

// Task 3: Sensor Reading and Processing
void Task_Sensors(void* parameter) {
    Serial.println("[Task_Sensors] Started on Core 1");
      TOF_UART.begin(921600, SERIAL_8N1, TOF_RX_PIN, TOF_TX_PIN);//Initialize the TOF serial port baud rate to 921600 and specify the pin 初始化TOF串口波特率到921600,并指定引脚

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
          TOF_Active_Decoding();//Query and decode TOF data 查询获取TOF数据，并进行解码



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