#include <PS4Controller.h>
#include <ESP32Servo.h>

Servo primingServo;
Servo flywheel;

const int pwmPinServoPrime = 32;
const int pwmPinflywheel = 15;

int flywheel_vel = 47;

void setup() {
  Serial.begin(115200);
  primingServo.attach(pwmPinServoPrime, 500, 2500);
  flywheel.attach(pwmPinflywheel, 1000, 2000);
  flywheel.write(0);
  PS4.begin();
  while (!PS4.isConnected()) {
    Serial.println("Waiting for PS4 controller to connect...");
    delay(1000);
  }
}

void loop() {
  if (PS4.isConnected()) {
    if (PS4.R1()) { 
    primingServo.writeMicroseconds(map(30, 0, 300, 500, 2500));
    delay(200);
    } else { 
      primingServo.writeMicroseconds(map(60, 0, 300, 500, 2500));
    }
    if (PS4.R2()) {
      flywheel.write(flywheel_vel);
    } else {
      flywheel.write(0);
    }
  }
}
