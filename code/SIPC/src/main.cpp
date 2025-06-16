#include <Arduino.h>

#define LED_PIN 2

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(921600);
  
}

void loop() {
  Serial.printf("Hello loop");
  delay(5000);
}