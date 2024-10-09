#include <esp_now.h>
#include <WiFi.h>
#include "driver/ledc.h"

#define CHANNEL 1

esp_now_peer_info_t master;  // Global copy of master

// Motor control pins for Motor A
const int dirPinA1 = 15;  // IN1 for Motor A direction control
const int dirPinA2 = 4;   // IN2 for Motor A direction control
const int pwmPinA = 2;    // ENA pin (controls Motor A speed)

// Motor control pins for Motor B
const int dirPinB1 = 18;  // IN1 for Motor B direction control
const int dirPinB2 = 5;   // IN2 for Motor B direction control
const int pwmPinB = 19;   // ENB pin (controls Motor B speed)

// Motor control pins for Motor C
const int dirPinC1 = 12;  // IN1 for Motor C direction control
const int dirPinC2 = 14;  // IN2 for Motor C direction control
const int pwmPinC = 13;   // ENA pin (controls Motor C speed)

// Motor control pins for Motor D
const int dirPinD1 = 27;  // IN1 for Motor D direction control
const int dirPinD2 = 26;  // IN2 for Motor D direction control
const int pwmPinD = 25;   // ENB pin (controls Motor D speed)


int motorSpeedA = 0;
int motorSpeedB = 0;
int motorSpeedC = 0;
int motorSpeedD = 0;

int RStickX = 0;
int RStickY = 0;

double theta = 0;
uint8_t command[2];

uint8_t masterAddress[] = { 0xC8, 0x2E, 0x18, 0xEF, 0x4E, 0xE0 };  // MAC address of the master

void setup() {
  Serial.begin(115200);

  pinMode(dirPinA1, OUTPUT);
  pinMode(dirPinA2, OUTPUT);
  pinMode(dirPinB1, OUTPUT);
  pinMode(dirPinB2, OUTPUT);
  pinMode(dirPinC1, OUTPUT);
  pinMode(dirPinC2, OUTPUT);
  pinMode(dirPinD1, OUTPUT);
  pinMode(dirPinD2, OUTPUT);

  // Set up PWM channels for speed control (Motor A and B)
  ledcSetup(0, 5000, 8);      // Channel 0 for Motor A
  ledcAttachPin(pwmPinA, 0);  // Attach Motor A's ENA to PWM channel 0
  ledcSetup(1, 5000, 8);      // Channel 1 for Motor B
  ledcAttachPin(pwmPinB, 1);  // Attach Motor B's ENB to PWM channel 1
  ledcSetup(2, 5000, 8);      // Channel 1 for Motor C
  ledcAttachPin(pwmPinB, 2);  // Attach Motor C's ENA to PWM channel 2
  ledcSetup(3, 5000, 8);      // Channel 1 for Motor D
  ledcAttachPin(pwmPinB, 3);  // Attach Motor D's ENB to PWM channel 3

  // Initialize WiFi and ESP-NOW
  WiFi.mode(WIFI_STA);  // Set device in station mode
  esp_now_init();
  esp_now_register_recv_cb(OnDataRecv);  // Register callback for data reception

  memcpy(master.peer_addr, masterAddress, 6);  // Set peer address
  esp_now_add_peer(&master);                   // Add master as peer
}

void loop() {
  /*
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

    // Control Motor C based on motorSpeedD
    if (motorSpeedC > 127) {
        digitalWrite(dirPinC1, HIGH);  // Set direction to forward
        digitalWrite(dirPinC2, LOW);
    } else if (motorSpeedC < 127) {
        digitalWrite(dirPinC1, LOW);   // Set direction to reverse
        digitalWrite(dirPinC2, HIGH);
    } else {
        digitalWrite(dirPinC1, LOW);   // Stop Motor C
        digitalWrite(dirPinC2, LOW);
    }
    ledcWrite(2, abs(motorSpeedC)); // Use the absolute value for Motor C PWM duty cycle

    // Control Motor D based on motorSpeedD
    if (motorSpeedD > 127) {
        digitalWrite(dirPinD1, HIGH);  // Set direction to forward
        digitalWrite(dirPinD2, LOW);
    } else if (motorSpeedD < 127) {
        digitalWrite(dirPinD1, LOW);   // Set direction to reverse
        digitalWrite(dirPinD2, HIGH);
    } else {
        digitalWrite(dirPinD1, LOW);   // Stop Motor D
        digitalWrite(dirPinD2, LOW);
    }
    ledcWrite(3, abs(motorSpeedD)); // Use the absolute value for Motor D PWM duty cycle
  
    delay(0.2);*/
}

// Callback when data is received from Master
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  command[0] = data[0];  // Right Stick X
  command[1] = data[1];  // Right Stick Y
  if (118 <= command[0] && command[0] <= 138) {
    RStickX = 0;
  } else {
    RStickX = map(command[0], 0, 255, -255, 255);
  }
  if (118 <= command[1] && command[1] <= 138) {
    RStickY = 0;
  } else {
    RStickY = map(command[1], 0, 255, -255, 255);
  }

  //Theta calculation
  if (RStickX == 0 && RStickY > 0) {
    theta = 90;
  } else if (RStickX == 0 && RStickY < 0) {
    theta = -90;
  } else if (RStickX == 0 && RStickY == 0) {
    theta = 0;
  } else {
    theta = atan(static_cast<float>(RStickY) / RStickX) * 180.0 / PI;

    if (RStickX < 0 && RStickY >= 0) {
     theta += 180;  // Adjust the angle to handle negative X direction
    } else if (RStickX < 0 && RStickY < 0) {
      theta -= 180;  // Third quadrant (X negative, Y negative)
    }
  }

  Serial.print("X-axis: ");
  Serial.print(RStickX);
  Serial.print("\tY-axis: ");
  Serial.print(RStickY);
  Serial.print("\tTheta: ");
  Serial.println(theta);
}
