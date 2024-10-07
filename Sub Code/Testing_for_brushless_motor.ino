
#include <Servo.h>

Servo ESC;  //create servo object name as ESC to control the ESC


int buttonstate = 0;

void setup() {
  Serial.begin(9600);
  // attach the ESC on pin 9:
  ESC.attach(9,1000,2000);
  ESC.writeMicroseconds(0); // (pin, min pulse width, max pulse width in microseconds)
  ESC.write(0);
  delay(5000);
  pinMode(8, INPUT);
}

void loop() {
  buttonstate = digitalRead(8);
  if (buttonstate == HIGH) {
    ESC.write(180);
    Serial.println("Motor On");
  }
  else {
    ESC.write(0);
    Serial.println("Motor Off");
  }
}
