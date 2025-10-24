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