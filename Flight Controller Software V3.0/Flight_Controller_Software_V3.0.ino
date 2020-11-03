#include "global_var.h"
#include "led_.h"
#include "radio_nrf24.h"
#include "gyro.h"
#include "PID_controller.h"
#include "Motor_controller.h"
#include <Arduino.h>

uint32_t loopTimer;                     //loopTimer to ensure each main loop iteration is 4000microseconds

void setup() {
  Serial.begin(57600);
  led_c.led_Init();
  
  gyro_c.imu_Init();
  gyro_c.gyro_calibrate();

  rfRadio_c.radio_Init();

  Motor_c.motor_attach();
  Motor_c.motor_Init();
  loopTimer = micros();
}

uint32_t configTimer =  millis();  

void loop() {
  if (armMotors == true && motorFlag == true ){    //Re-arming ESC if requested
    Motor_c.motor_Init();
    armMotors = false;
  }  
  rfRadio_c.check_radio();
  
  gyro_c.read_Imu();
  
  inputVariables[0] = (inputVariables[0] * 0.7) + ((g_raw[1] / 32.8) * 0.3);
  inputVariables[1] = (inputVariables[1] * 0.7) + ((g_raw[2] / 32.8) * 0.3);
  inputVariables[2] = (inputVariables[2] * 0.7) + ((g_raw[3] / 32.8) * 0.3);

  gyro_c.the_Gyroscope();
  
  PID_c.PID_Init();
  PID_c.PID_Controller();
  
  while (micros() - loopTimer < 4000);
  loopTimer = micros();
  Motor_c.e_Driver(); 
}
