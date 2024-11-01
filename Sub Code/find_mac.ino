#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
  
  SerialBT.begin("ESP32");  // Start Bluetooth with name "ESP32"
  Serial.println("The device started, now you can pair it with other Bluetooth devices!");

  // Print ESP32's Bluetooth MAC Address using getBtAddress
  uint8_t mac[6]; // Array to store the MAC address
  esp_read_mac(mac, ESP_MAC_BT); // Read the Bluetooth MAC address into the array
  
  Serial.print("MAC Address: ");
  for (int i = 0; i < 6; i++) {
    if (i > 0) {
      Serial.print(":");
    }
    Serial.print(mac[i], HEX); // Print each byte of the MAC address in HEX format
  }
  Serial.println();
}

void loop() {
  // Put code here to execute repeatedly
}
