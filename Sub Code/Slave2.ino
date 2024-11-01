#include <esp_now.h>
#include <WiFi.h>

#define CHANNEL 1

esp_now_peer_info_t master;  // Global copy of master

uint8_t command[2];


uint8_t masterAddress[] = {0xC8, 0xF0, 0x9E, 0xF6, 0xFB, 0x34};  // MAC address of the master

void setup() {
  Serial.begin(115200);
  // Initialize WiFi and ESP-NOW
  WiFi.mode(WIFI_STA);  // Set device in station mode
  esp_now_init();
  esp_now_register_send_cb(OnDataSent);

  memcpy(master.peer_addr, masterAddress, 6);  // Set peer address
  master.channel = CHANNEL;  // Pick a channel
  master.encrypt = 0;        // No encryption
  esp_now_add_peer(&master);                   // Add master as peer

  
}

void loop() {
  command[0] = 0;
  command[1] = 5;
  esp_now_send(master.peer_addr, command, sizeof(command));
  delay(500);
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Sent: ");
  Serial.print(command[0]);
  Serial.print("\t");
  Serial.println(command[1]);
}