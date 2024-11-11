#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <PS4Controller.h>

#define CHANNEL 1

Servo flywheel;

const int pwmPinflywheel = 13;
const int digitalPin = 12; 
const int S0 = 14;
const int S1 = 27;
const int S2 = 26;
const int S3 = 25;
const int sensorOut = 33;

volatile unsigned long lastPulseTime = 0;
volatile unsigned long pulseInterval = 0;

int flywheel_vel = 0;
float rpm = 0;
float prev_rpm = 0;
float rpm_diff = 0;

int R_Min = 28;  
int R_Max = 140;
int G_Min = 28;  
int G_Max = 140; 
int B_Min = 28;  
int B_Max = 140; 

int Red = 0;
int Green = 0;
int Blue = 0;

int redValue;
int greenValue;
int blueValue;
int Frequency;
int colour_detect= 0;

esp_now_peer_info_t master;  // Global copy of master

uint8_t command[2];

uint8_t masterAddress[] = {0x08, 0x3A, 0xF2, 0xB4, 0x65, 0x80};  // MAC address of the master

// ISR to handle Hall effect sensor pulses
void IRAM_ATTR handlePulse() {
  unsigned long currentTime = micros();
  pulseInterval = currentTime - lastPulseTime;
  lastPulseTime = currentTime;
}

int getRed() {
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  Frequency = pulseIn(sensorOut, LOW); /*Get the Red Color Frequency*/
  return Frequency;
}

int getBlue() {
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  Frequency = pulseIn(sensorOut, LOW); /*Get the Blue Color Frequency*/
  return Frequency;
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);  
  esp_now_init();
  esp_now_register_send_cb(OnDataSent);

  memcpy(master.peer_addr, masterAddress, 6);  
  master.channel = CHANNEL;  
  master.encrypt = 0;        // No encryption
  esp_now_add_peer(&master);

  pinMode(S0, OUTPUT);       
  pinMode(S1, OUTPUT);   
  pinMode(S2, OUTPUT);     
  pinMode(S3, OUTPUT);      
  pinMode(sensorOut, INPUT);      

  // Sensitivity
  digitalWrite(S0, LOW);      
  digitalWrite(S1, HIGH);              

  pinMode(digitalPin, INPUT_PULLUP);
  attachInterrupt(digitalPin, handlePulse, FALLING);  // Trigger on the falling edge
}

void loop() {
  // Calculate RPM if there was a pulse
  if (pulseInterval > 0) {
    rpm = 60000000.0 / pulseInterval;  // Convert microseconds per pulse to RPM
    rpm = rpm * 0.5;
  }
  
  rpm_diff = abs(rpm - prev_rpm);
  prev_rpm = rpm;

  Red = getRed();
  redValue = map(Red, R_Min, R_Max, 255, 0);
 
  Blue = getBlue();
  blueValue = map(Blue, B_Min, B_Max, 255, 0);   

  if (redValue > blueValue + 25) colour_detect = 1; // RED
  else if (blueValue > redValue + 15) colour_detect = 2; // BLUE
  else colour_detect = 0; // UNCERTAIN

  command[0] = rpm;
  command[1] = colour_detect;
  esp_now_send(master.peer_addr, command, sizeof(command));
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Sent: ");
  Serial.print(command[0]);
  Serial.print("\t");
  Serial.println(command[1]);
}