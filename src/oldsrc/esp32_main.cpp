#include <Arduino.h>
#include "mavlink_handler.h"

HardwareSerial FC_Serial(2);  // UART2 for Pixhawk
#define RX_PIN 16
#define TX_PIN 17
#define BAUD_RATE 57600

void setup() {
    Serial.begin(115200);
    FC_Serial.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
    Serial.println("ESP32 MAVLink Telemetry Test Started");
}

void loop() {
    mavlink_message_t msg;
    mavlink_status_t status;

    while (FC_Serial.available() > 0) {
        uint8_t c = FC_Serial.read();
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
            handleMavlinkMessage(&msg);
        }
    }

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 1000) {  // print every 1 second
        printEssentialTelemetry();
        lastPrint = millis();
    }
}
