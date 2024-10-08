#include <esp_now.h>
#include <WiFi.h>
#include "driver/ledc.h"

#define CHANNEL 1

esp_now_peer_info_t master;  // Global copy of master

// Motor control pins for Motor A
const int dirPinA1 = 5;  // IN1 for Motor A direction control
const int dirPinA2 = 18; // IN2 for Motor A direction control
const int pwmPinA = 4;   // ENA pin (controls Motor A speed)

// Motor control pins for Motor B
const int dirPinB1 = 19; // IN1 for Motor B direction control
const int dirPinB2 = 21; // IN2 for Motor B direction control
const int pwmPinB = 22;  // ENB pin (controls Motor B speed)

uint8_t command[2];  // Array to store incoming commands (0: Y-axis, 1: Z-axis)
int motorSpeedA = 0; // Motor A speed
int motorSpeedB = 0; // Motor B speed
uint8_t masterAddress[] = {0xC8, 0x2E, 0x18, 0xEF, 0x4E, 0xE0}; // MAC address of the master

void setup() {
    Serial.begin(115200);

    // Set direction control pins as output for Motor A
    pinMode(dirPinA1, OUTPUT);
    pinMode(dirPinA2, OUTPUT);
    
    // Set direction control pins as output for Motor B
    pinMode(dirPinB1, OUTPUT);
    pinMode(dirPinB2, OUTPUT);
  
    // Set up PWM channels for speed control (Motor A and B)
    ledcSetup(0, 5000, 8);  // Channel 0 for Motor A
    ledcAttachPin(pwmPinA, 0);  // Attach Motor A's ENA to PWM channel 0
    ledcSetup(1, 5000, 8);  // Channel 1 for Motor B
    ledcAttachPin(pwmPinB, 1);  // Attach Motor B's ENB to PWM channel 1

    // Initialize WiFi and ESP-NOW
    WiFi.mode(WIFI_STA); // Set device in station mode
    esp_now_init();
    esp_now_register_recv_cb(OnDataRecv); // Register callback for data reception
  
    memcpy(master.peer_addr, masterAddress, 6); // Set peer address
    esp_now_add_peer(&master); // Add master as peer
}

void loop() {
    // Control Motor A based on motorSpeedA
    
    if (motorSpeedA > 127) {
        digitalWrite(dirPinA1, HIGH);  // Set direction to forward
        digitalWrite(dirPinA2, LOW);
    } else if (motorSpeedA < 127) {
        digitalWrite(dirPinA1, LOW);   // Set direction to reverse
        digitalWrite(dirPinA2, HIGH);
    } else {
        digitalWrite(dirPinA1, LOW);   // Stop Motor A
        digitalWrite(dirPinA2, LOW);
    }
    ledcWrite(0, abs(motorSpeedA)); // Use the absolute value for Motor A PWM duty cycle

    // Control Motor B based on motorSpeedB
    if (motorSpeedB > 127) {
        digitalWrite(dirPinB1, HIGH);  // Set direction to forward
        digitalWrite(dirPinB2, LOW);
    } else if (motorSpeedB < 127) {
        digitalWrite(dirPinB1, LOW);   // Set direction to reverse
        digitalWrite(dirPinB2, HIGH);
    } else {
        digitalWrite(dirPinB1, LOW);   // Stop Motor B
        digitalWrite(dirPinB2, LOW);
    }
    ledcWrite(1, abs(motorSpeedB)); // Use the absolute value for Motor B PWM duty cycle
  
    delay(0.2); // Small delay to avoid overwhelming the system
}

// Callback when data is received from Master
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  if (data_len == 2) {  // Ensure we receive exactly 2 bytes
    command[0] = data[0]; // Update command for Y-axis (Motor A)
    command[1] = data[1]; // Update command for Z-axis (Motor B)

    // Remap command values to motor speeds
    if (command[0] < 124) {
        motorSpeedA = map(command[0], 0, 124, -255, 0); // Reverse for Motor A
    } else if (command[0] > 126) {
        motorSpeedA = map(command[0], 126, 255, 0, 255); // Forward for Motor A
    } else {
        motorSpeedA = 0; // Neutral position for Motor A
    }

    if (command[1] < 124) {
        motorSpeedB = map(command[1], 0, 124, -255, 0); // Reverse for Motor B
    } else if (command[1] > 126) {
        motorSpeedB = map(command[1], 126, 255, 0, 255); // Forward for Motor B
    } else {
        motorSpeedB = 0; // Neutral position for Motor B
    }

  Serial.print("Motor A speed: ");
  Serial.print(motorSpeedA);
  Serial.print("\t\tMotor B speed: ");
  Serial.println(motorSpeedB);
  }
}
