// DO NOT directly connect laptop to supply power for this servo motor, upload it individually then use external power supply

#include <Servo.h>

Servo myservo;  // create servo object to control a servo

void setup() {
  myservo.attach(9,600,2300);  // (pin, min, max)
  Serial.begin(9600);
}

void loop() {
  myservo.write(0);  // tell servo to go to a particular angle
  delay(1000);
  
  myservo.write(90);              
  delay(500); 
  
  myservo.write(135);              
  delay(500);
  
  myservo.write(180);              
  delay(1500);
  Serial.println("Ola");                    
}