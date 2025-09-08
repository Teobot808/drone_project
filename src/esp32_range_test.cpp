#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// MAC address of the receiver ESP32 (change for sender!)
uint8_t receiverMAC[] = {0x24, 0x6F, 0x28, 0xAB, 0xCD, 0xEF};

// Data structure for messages
typedef struct struct_message {
  uint32_t counter;
} struct_message;

struct_message outgoing;
struct_message incoming;

// Role: set true for sender, false for receiver
#define IS_SENDER true

// Callback for data sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Callback for data received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incoming, incomingData, sizeof(incoming));
  int32_t rssi = WiFi.RSSI();  // last packet RSSI
  Serial.printf("Received packet #%lu | RSSI: %d dBm\n", incoming.counter, rssi);
}

void setup() {
  Serial.begin(115200);

  // Set device in STA mode
  WiFi.mode(WIFI_STA);

  // Enable Long Range mode
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

#if IS_SENDER
  // Register peer
  esp_now_peer_info_t peerInfo{};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

#else
  // Receiver
  esp_now_register_recv_cb(OnDataRecv);
#endif
}

void loop() {
#if IS_SENDER
  outgoing.counter++;
  esp_err_t result = esp_now_send(receiverMAC, (uint8_t *) &outgoing, sizeof(outgoing));
  if (result == ESP_OK) {
    Serial.printf("Sent packet #%lu\n", outgoing.counter);
  } else {
    Serial.println("Error sending the data");
  }
  delay(1000);
#endif
}
