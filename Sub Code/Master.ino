#include <esp_now.h>
#include <WiFi.h>
#include <PS4Controller.h>

#define CHANNEL 1

esp_now_peer_info_t slave1;  
esp_now_peer_info_t slave2;

uint8_t command[14]; 
//uint8_t rcvdata[2];
uint8_t slaveAddress1[] = {0x14, 0x2B, 0x2F, 0xEA, 0x07, 0xF8};
uint8_t slaveAddress2[] = {0x08, 0x3A, 0xF2, 0xB4, 0x65, 0x80}; 

void setup() {
  Serial.begin(115200);
  PS4.begin();
  while (!PS4.isConnected()) {
    Serial.println("Waiting for PS4 controller to connect...");
    delay(1000);
  }
  Serial.print("Connected! Battery Level: ");
  Serial.println(PS4.Battery());
  delay(2000);

  WiFi.mode(WIFI_STA);                   
  esp_now_init();                        
  esp_now_register_send_cb(OnDataSent);  
  //esp_now_register_recv_cb(OnDataRecv); 

  memcpy(slave1.peer_addr, slaveAddress1, 6);
  slave1.channel = CHANNEL;  
  slave1.encrypt = 0;       
  esp_now_add_peer(&slave1);

  memcpy(slave2.peer_addr, slaveAddress2, 6);
  slave2.channel = CHANNEL; 
  slave2.encrypt = 0;       
  esp_now_add_peer(&slave2);
}

void loop() {
  if (PS4.isConnected()) {
    command[0] = map(PS4.RStickX(), -128, 127, 0, 255);
    command[1] = map(PS4.RStickY(), -128, 127, 0, 255);
    command[2] = map(PS4.LStickX(), -128, 127, 0, 255);
    command[3] = PS4.L1();
    command[4] = PS4.L2();
    command[5] = PS4.R1();
    command[6] = PS4.R2();
    command[7] = PS4.Cross();
    command[8] = PS4.Circle();
    command[9] = PS4.Triangle();
    command[10] = PS4.Square();
    command[11] = PS4.Left();
    command[12] = PS4.Right();
    command[13] = PS4.Touchpad();
     
    esp_now_send(slave1.peer_addr, command, sizeof(command)); // Send command array to the slave
    esp_now_send(slave2.peer_addr, command, sizeof(command));
  
    Serial.print("X: ");
    Serial.print(command[0]);
    Serial.print("\tY: ");
    Serial.print(command[1]);
    Serial.print("\tX2: ");
    Serial.print(command[2]);
    Serial.print("\tL1: ");
    Serial.print(command[3]);
    Serial.print("\tL2: ");
    Serial.print(command[4]);
    Serial.print("\tR1: ");
    Serial.print(command[5]);
    Serial.print("\tR2: ");
    Serial.print(command[6]);
    Serial.print("\t✕: ");
    Serial.print(command[7]);
    Serial.print("\t◯: ");
    Serial.print(command[8]);
    Serial.print("\t△: ");
    Serial.print(command[9]);
    Serial.print("\t▢: ");
    Serial.print(command[10]);
    Serial.print("\t◀: ");
    Serial.print(command[11]);
    Serial.print("\t▶: ");
    Serial.println(command[12]);
  }
}

// Callback when data is sent from Master to Slave
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  
}

/*void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  rcvdata[0] = data[0];
  rcvdata[1] = data[1];
}*/
