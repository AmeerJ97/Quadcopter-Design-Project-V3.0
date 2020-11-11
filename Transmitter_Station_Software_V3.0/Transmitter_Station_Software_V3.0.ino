#include "PS_3.h"
#include "radio_nrf24.h"
#include "global_struct.h"
#include <Arduino.h>

void setup() {
  Serial.begin(57600);
  PS3_c.PS3_Init();
  rfRadio_c.radio_Init();
}

float configTimer = millis();

void loop() {
  PS3_c.read_PS3();
  rfRadio_c.radio_tx();
  if (millis() - configTimer > 2000){
  PS3_c.reset_buttons();
  configTimer = millis();
  }
} 
