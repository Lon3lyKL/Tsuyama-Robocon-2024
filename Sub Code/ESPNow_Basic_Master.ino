#include <esp_now.h>
#include <WiFi.h>
#include <PS4Controller.h>

#define CHANNEL 1

esp_now_peer_info_t slave;  // Global copy of slave

uint8_t command[2]; 
uint8_t slaveAddress[] = {0xFC, 0xB4, 0x67, 0xF1, 0xDF, 0xE8}; // MAC address of the slave

void setup() {
  Serial.begin(115200);
  PS4.begin("C8:2E:18:EF:4E:E2");
  WiFi.mode(WIFI_STA);                   // Set device in STA mode
  esp_now_init();                        // Init ESPNow protocol
  esp_now_register_send_cb(OnDataSent);  // Get the status of transmitted packet
  memcpy(slave.peer_addr, slaveAddress, 6);
  slave.channel = CHANNEL;  // Pick a channel
  slave.encrypt = 0;        // No encryption
  esp_now_add_peer(&slave);
}

void loop() {
  command[0] = map(PS4.RStickX(), -128, 127, 0, 255);
  command[1] = map(PS4.RStickY(), -128, 127, 0, 255);
  
  // Send both commands to the slave
  esp_now_send(slave.peer_addr, command, sizeof(command)); // Send command array to the slave

}

// Callback when data is sent from Master to Slave
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("X-axis: ");
  Serial.print(command[0]);
  Serial.print("\tY-axis: ");
  Serial.println(command[1]);
}
