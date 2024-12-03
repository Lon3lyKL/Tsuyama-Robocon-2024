#include <esp_now.h>
#include <WiFi.h>
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#define CHANNEL 1

esp_now_peer_info_t master;  // Global copy of master

uint8_t command[2];

#define SENSOR_PIN 15 // GPIO36 (VP) is an analog pin
#define PIN_WS2812B 22  // The ESP32 pin GPIO16 connected to WS2812B
#define NUM_PIXELS 5    // The number of LEDs (pixels) on WS2812B LED strip

Adafruit_NeoPixel ws2812b(NUM_PIXELS, PIN_WS2812B, NEO_GRB + NEO_KHZ800);

float base_dist = 0;
float dist = 0;
int pucks = 0;
int Touchpad = 0;
int R1 = 0;
int R2 = 0;

uint8_t masterAddress[] = {0x14, 0x2B, 0x2F, 0xEB, 0xC0, 0xBC};  // MAC address of the master

void displayBinary(int number) {
  ws2812b.clear();  // Clear all LEDs

  for (int bit = 0; bit < NUM_PIXELS; bit++) {
    if (number & (1 << bit)) {  // Check if the bit is set in 'number'
      ws2812b.setPixelColor(bit, ws2812b.Color(0, 255, 0));  // Turn ON LED (green)
    } else {
      ws2812b.setPixelColor(bit, ws2812b.Color(0, 0, 0));    // Turn OFF LED (black/off)
    }
  }

  ws2812b.show();  // Update the LED strip
}

void setup() {
  Serial.begin(115200);
  ws2812b.begin();  // Initialize WS2812B strip object

  WiFi.mode(WIFI_STA); 
  //WiFi.disconnect(true);
  esp_now_init();
  esp_now_register_recv_cb(OnDataRecv);
  
  memcpy(master.peer_addr, masterAddress, 6);  
  master.channel = CHANNEL;  
  master.encrypt = 0;        // No encryption
  esp_now_add_peer(&master);
}

void loop() {
  if (Touchpad) {
    esp_now_deinit();
    WiFi.mode(WIFI_OFF);
    delay(10);

    int rawValue = analogRead(SENSOR_PIN); // Read the raw ADC value (0-4095)
    float voltage = (rawValue / 4095.0) * 5.0; // Convert to voltage (for 3.3V ADC reference)
    
    // Convert voltage to distance (approximation based on the sensor's datasheet)
    dist = 27.86 / (voltage - 0.42); // Calibration formula
    
    base_dist = 16.70;
    pucks = round((base_dist - dist) / 0.44);
    Touchpad = 0;
  }
  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_recv_cb(OnDataRecv);
  

  static bool R1Pressed = false;
  if (R1 && R2) {
    if (!R1Pressed && pucks > 0) {
        pucks--;
        R1Pressed = true;
    }
  } else R1Pressed = false;

  displayBinary(pucks);  // Display binary representation on the LED strip
  Serial.print("Base Distance: ");
  Serial.print(base_dist);
  Serial.print("\tDistance: ");
  Serial.print(dist, 2);
  Serial.print(" cm\tPucks: ");
  Serial.print(pucks);
  Serial.print("\tR1: ");
  Serial.print(R1);
  Serial.print("\tR2: ");
  Serial.print(R2);
  Serial.print("\tTouchpad: ");
  Serial.println(Touchpad);

  /*command[0] = rpm;
  command[1] = colour_detect;
  esp_now_send(master.peer_addr, command, sizeof(command));*/
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  Touchpad = data[13];
  R1 = data[5];
  R2 = data[6];
}