#include <esp_now.h>
#include <WiFi.h>
#include <PS4Controller.h>

#define CHANNEL 1

esp_now_peer_info_t slave;  // Global copy of slave

uint8_t command[12]; 
uint8_t slaveAddress[] = {0xFC, 0xB4, 0x67, 0xF1, 0xDF, 0xE8}; // MAC address of the slave

void setup() {
  Serial.begin(115200);
  PS4.begin();
  while (!PS4.isConnected()) {
    Serial.println("Waiting for PS4 controller to connect...");
    delay(1000);
  }
  //PS4.begin("C8:2E:18:EF:4E:E2");
  WiFi.mode(WIFI_STA);                   // Set device in STA mode
  esp_now_init();                        // Init ESPNow protocol
  esp_now_register_send_cb(OnDataSent);  // Get the status of transmitted packet
  memcpy(slave.peer_addr, slaveAddress, 6);
  slave.channel = CHANNEL;  // Pick a channel
  slave.encrypt = 0;        // No encryption
  esp_now_add_peer(&slave);
}

void loop() {
  if (PS4.isConnected()) {
    command[0] = map(PS4.RStickX(), -128, 127, 0, 255);
    command[1] = map(PS4.RStickY(), -128, 127, 0, 255);
    command[2] = map(PS4.LStickX(), -128, 127, 0, 255);
    command[3] = PS4.L2();
    command[4] = PS4.R1();
    command[5] = PS4.R2();
    command[6] = PS4.Cross();
    command[7] = PS4.Circle();
    command[8] = PS4.Triangle();
    command[9] = PS4.Square();
    command[10] = PS4.Left();
    command[11] = PS4.Right();
    
    // Send both commands to the slave
    esp_now_send(slave.peer_addr, command, sizeof(command)); // Send command array to the slave
  }
}

// Callback when data is sent from Master to Slave
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("X: ");
  Serial.print(command[0]);
  Serial.print("\tY: ");
  Serial.print(command[1]);
  Serial.print("\tX2: ");
  Serial.print(command[2]);
  Serial.print("\tL2: ");
  Serial.print(command[3]);
  Serial.print("\tR1: ");
  Serial.print(command[4]);
  Serial.print("\tR2: ");
  Serial.print(command[5]);
  Serial.print("\t✕: ");
  Serial.print(command[6]);
  Serial.print("\t◯: ");
  Serial.print(command[7]);
  Serial.print("\t△: ");
  Serial.print(command[8]);
  Serial.print("\t▢: ");
  Serial.print(command[9]);
  Serial.print("\t◀: ");
  Serial.print(command[10]);
  Serial.print("\t▶: ");
  Serial.println(command[11]);
}
