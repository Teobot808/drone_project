#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>

// Replace with your sender MAC address
const uint8_t senderMAC[] = {0x84, 0x1F, 0xE8, 0x26, 0xD5, 0x6C};

// ESP-NOW data structure
typedef struct struct_message {
  uint32_t counter;
} struct_message;

// Promiscuous callback
void wifi_sniffer_packet(void* buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_DATA && type != WIFI_PKT_MGMT) return;

  wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *)buf;

  // Extract MAC address of sender (source address in WiFi header)
  uint8_t *mac = ppkt->payload + 10; // ESP-NOW sender MAC offset

  // Only process packets from our sender
  if (memcmp(mac, senderMAC, 6) != 0) return;

  int8_t rssi = ppkt->rx_ctrl.rssi;

  // Interpret payload as struct_message (skip ESP-NOW header)
  struct_message *msg = (struct_message*)(ppkt->payload + 24);

  // Print info
  Serial.printf("Packet #%lu | RSSI: %d dBm | From MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                msg->counter, rssi,
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Set WiFi to station mode and disconnect from any AP
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Set ESP32 WiFi channel (must match sender)
  uint8_t channel = 1; // replace with sender channel
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);

  // Enable long range mode
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);

  // Enable promiscuous mode
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(&wifi_sniffer_packet);

  Serial.println("ESP32 Promiscuous Receiver ready!");
}

void loop() {
  // Nothing needed, packets handled in callback
}
