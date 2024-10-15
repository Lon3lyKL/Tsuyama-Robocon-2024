#include <esp_now.h>
#include <WiFi.h>
#include "driver/ledc.h"

#define CHANNEL 1

esp_now_peer_info_t master;  // Global copy of master

// Motor control pins for Motor A
const int dirPinA1 = 4;  // IN1 for Motor A direction control
const int dirPinA2 = 2;  // IN2 for Motor A direction control
const int pwmPinA = 15;  // ENA pin (controls Motor A speed)

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

int vel_X = 0;
int vel_Y = 0;
int vel_X2 = 0;
int vel_A = 0;
int vel_B = 0;
int vel_C = 0;
int vel_D = 0;

double theta = 0;
uint8_t command[3];

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

  // Set up PWM channels for speed control
  ledcSetup(0, 5000, 8);      // Channel 0 for Motor A
  ledcAttachPin(pwmPinA, 0);  // Attach Motor A's ENA to PWM channel 0
  ledcSetup(1, 5000, 8);      // Channel 1 for Motor B
  ledcAttachPin(pwmPinB, 1);  // Attach Motor B's ENB to PWM channel 1
  ledcSetup(2, 5000, 8);      // Channel 1 for Motor C
  ledcAttachPin(pwmPinC, 2);  // Attach Motor C's ENA to PWM channel 2
  ledcSetup(3, 5000, 8);      // Channel 1 for Motor D
  ledcAttachPin(pwmPinD, 3);  // Attach Motor D's ENB to PWM channel 3

  // Initialize WiFi and ESP-NOW
  WiFi.mode(WIFI_STA);  // Set device in station mode
  esp_now_init();
  esp_now_register_recv_cb(OnDataRecv);  // Register callback for data reception

  memcpy(master.peer_addr, masterAddress, 6);  // Set peer address
  esp_now_add_peer(&master);                   // Add master as peer
}

int* Stop() {
  vel_A = 0;
  vel_B = 0;
  vel_C = 0;
  vel_D = 0;
  static int vel_arr[4] = {vel_A, vel_B, vel_C, vel_D};
  // Motor A: Stop
  digitalWrite(dirPinA1, LOW);
  digitalWrite(dirPinA2, LOW);
  ledcWrite(0, 0);

  // Motor B: Stop
  digitalWrite(dirPinB1, LOW);
  digitalWrite(dirPinB2, LOW);
  ledcWrite(1, 0);

  // Motor C: Stop
  digitalWrite(dirPinC1, LOW);
  digitalWrite(dirPinC2, LOW);
  ledcWrite(2, 0);

  // Motor D: Stop
  digitalWrite(dirPinD1, LOW);
  digitalWrite(dirPinD2, LOW);
  ledcWrite(3, 0);
  return vel_arr;
}

int MotorA(int x, int y, int x2, double theta) {
  int resultant_A = y + x + x2; 
  if (-1024 <= x2 && x2 < 0) {
    digitalWrite(dirPinA1, LOW);
    digitalWrite(dirPinA2, HIGH);
  } else if (0 < x2 && x2 <= 1024) {
    digitalWrite(dirPinA1, HIGH);
    digitalWrite(dirPinA2, LOW);
  } else if (-45 < theta && theta < 135) { // Forward for -45 < theta < 135
    digitalWrite(dirPinA1, HIGH);
    digitalWrite(dirPinA2, LOW);
  } else if (-45 > theta && theta > -180 || 180 >= theta && theta > 135) { // Backward for -45 > theta > 135
    digitalWrite(dirPinA1, LOW);
    digitalWrite(dirPinA2, HIGH);
  } else { // Speed = 0 at -45 and 135
    digitalWrite(dirPinA1, LOW);
    digitalWrite(dirPinA2, LOW);
  }
  ledcWrite(0, abs(resultant_A));
  return resultant_A;
}

int MotorB(int x, int y, int x2, double theta) {
  int resultant_B = y - x - x2;
  if (-1024 <= x2 && x2 < 0) {
    digitalWrite(dirPinB1, HIGH);
    digitalWrite(dirPinB2, LOW);
  } else if (0 < x2 && x2 <= 1024) {
    digitalWrite(dirPinB1, LOW);
    digitalWrite(dirPinB2, HIGH);
  } else if (-135 > theta && theta > -180 || 180 >= theta && theta > 45) { // Forward for -135 < theta < 45
    digitalWrite(dirPinB1, HIGH);
    digitalWrite(dirPinB2, LOW);
  } else if (45 > theta && theta > -135) { // Backward for 45 > theta > -135 
    digitalWrite(dirPinB1, LOW);
    digitalWrite(dirPinB2, HIGH);
  } else { // Speed = 0 at 45 and -135
    digitalWrite(dirPinB1, LOW);
    digitalWrite(dirPinB2, LOW);
  }
  ledcWrite(1, abs(resultant_B));
  return resultant_B;
}

int MotorC(int x, int y, int x2, double theta) {
  int resultant_C = y + x - x2;
  if (-1024 <= x2 && x2 < 0) {
    digitalWrite(dirPinC1, HIGH);
    digitalWrite(dirPinC2, LOW);
  } else if (0 < x2 && x2 <= 1024) {
    digitalWrite(dirPinC1, LOW);
    digitalWrite(dirPinC2, HIGH);
  } else if (-45 < theta && theta < 135) { // Forward for -45 < theta < 135
    digitalWrite(dirPinC1, HIGH);
    digitalWrite(dirPinC2, LOW);
  } else if (-45 > theta && theta > -180 || 180 >= theta && theta > 135) { //Backward for -45 > theta > 135
    digitalWrite(dirPinC1, LOW);
    digitalWrite(dirPinC2, HIGH);
  } else { // Speed = 0 at -45 and 135
    digitalWrite(dirPinC1, LOW);
    digitalWrite(dirPinC2, LOW);
  }
  ledcWrite(2, abs(resultant_C));
  return resultant_C;
}

int MotorD(int x, int y, int x2, double theta) {
  int resultant_D = y - x + x2;
  if (-1024 <= x2 && x2 < 0) {
    digitalWrite(dirPinD1, LOW);
    digitalWrite(dirPinD2, HIGH);
  } else if (0 < x2 && x2 <= 1024) {
    digitalWrite(dirPinD1, HIGH);
    digitalWrite(dirPinD2, LOW);
  } else if (-135 > theta && theta > -180 || 180 >= theta && theta > 45) { // Forward for -135 < theta < 45
    digitalWrite(dirPinD1, HIGH);
    digitalWrite(dirPinD2, LOW);
  } else if (45 > theta && theta > -135) { // Backward for 45 > theta > -135 
    digitalWrite(dirPinD1, LOW);
    digitalWrite(dirPinD2, HIGH);
  } else { // Speed = 0 at 45 and -135
    digitalWrite(dirPinD1, LOW);
    digitalWrite(dirPinD2, LOW);
  }
  ledcWrite(3, abs(resultant_D));
  return resultant_D;
}

void loop() {
  // Velocity calculations
  if (0 <= theta && theta <= 90) { // 1st  Quadrant
    vel_X = map(theta, 0, 90, RStickX, 0);
    vel_Y = map(theta, 0, 90, 0, RStickY);
  } else if (90 < theta && theta <= 180) {  // 2nd Quadrant
    vel_X = map(theta, 90, 180, 0, RStickX);
    vel_Y = map(theta, 90, 180, RStickY, 0);
  } else if (-180 < theta && theta <= -90) {  // 3rd Quadrant
    vel_X = map(theta, -180, -90, RStickX, 0);
    vel_Y = map(theta, -180, -90, 0, RStickY);
  } else if (-90 <= theta && theta <= 0) {  // 4th Quadrant
    vel_X = map(theta, -90, 0, 0, RStickX);
    vel_Y = map(theta, -90, 0, RStickY, 0);
  }
  
  if (!(RStickX == 0 && RStickY == 0 && vel_X2 == 0)) { // if controller detects input
    vel_A = MotorA(vel_X, vel_Y, vel_X2, theta);
    vel_B = MotorB(vel_X, vel_Y, vel_X2, theta);
    vel_C = MotorC(vel_X, vel_Y, vel_X2, theta);
    vel_D = MotorD(vel_X, vel_Y, vel_X2, theta);
  } else { // Stop when controller has no input
    int *arr = Stop();
    vel_A = arr[0];
    vel_B = arr[1];
    vel_C = arr[2];
    vel_D = arr[3];
  }

  /*Serial.print("X-axis: ");
  Serial.print(RStickX);
  Serial.print("\tY-axis: ");
  Serial.print(RStickY);*/
  Serial.print("theta: ");
  Serial.print(theta);
  Serial.print("\tvel_A: ");
  Serial.print(vel_A);
  Serial.print("\tvel_B: ");
  Serial.print(vel_B);
  Serial.print("\tvel_C: ");
  Serial.print(vel_C);
  Serial.print("\tvel_D: ");
  Serial.print(vel_D);
  Serial.print("\tvel_X2: ");
  Serial.println(vel_X2);
  
}

// Callback when data is received from Master
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  command[0] = data[0];  // Right Stick X
  command[1] = data[1];  // Right Stick Y
  command[2] = data[2]; // Left Stick X
  if (118 <= command[0] && command[0] <= 138) {
    RStickX = 0;
  } else {
    RStickX = map(command[0], 0, 255, -1024, 1024);
  }

  if (118 <= command[1] && command[1] <= 138) {
    RStickY = 0;
  } else {
    RStickY = map(command[1], 0, 255, -1024, 1024);
  }

  if (118 <= command[2] && command[2] <= 138) {
    vel_X2 = 0;
  } else {
    vel_X2 = map(command[2], 0, 255, -1024, 1024);
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
  /*
  Serial.print("X-axis: ");
  Serial.print(RStickX);
  Serial.print(" Y-axis: ");
  Serial.print(RStickY);
  Serial.print(" Theta: ");
  Serial.print(theta);*/
}
