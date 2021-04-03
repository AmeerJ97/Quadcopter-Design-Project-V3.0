#include "led_.h"
#include <Arduino.h>
#include "global_var.h"

/*------------------- LED Global Variables-------------------*/
extern const int bluePin = A0;                       //Blue LED pin
extern const int redPin = A1;                        //Red LED pin
extern const int yellowPin = A2;                     //Yellow LED pin
extern const int greenPin = A3;                      //Green LED pin

led_class::led_class() {
}

void led_class::led_Init() {
  //Initialize LED pins as outputs
  pinMode(bluePin, OUTPUT);
  pinMode(redPin, OUTPUT);
  pinMode(yellowPin, OUTPUT);
  pinMode(greenPin, OUTPUT);

  analogWrite(yellowPin, 255);
}

led_class led_c = led_class();
