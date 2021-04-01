#include "global_var.h"
#include "led_.h"
#include "radio_nrf24.h"
#include "gyro.h"
#include "PID_controller.h"
#include "Motor_controller.h"
#include "Arduino.h"

uint32_t loopTimer;                     //loopTimer to ensure each main loop iteration is 4000microseconds

void setup() {
  Serial.begin(57600);
  led_c.led_Init();
  
  gyro_c.imu_Init();
  gyro_c.gyro_calibrate();

  rfRadio_c.radio_Init();

  Motor_c.motor_attach();
  Motor_c.port_Init();
 //Motor_c.motor_port_Init();
  
  delay(5000);
  
  interrupts();         //Ensure Interrupts are Enabled
  
  loopTimer = micros();
}


void loop() {
  
  rfRadio_c.check_radio();
  
  analogWrite(redPin, 0);
  
  gyro_c.read_Imu();
  
  inputVariables[0] = (inputVariables[0] * 0.7) + ((g_raw[2] / 65.5) * 0.3);
  inputVariables[1] = (inputVariables[1] * 0.7) + ((g_raw[1] / 65.5) * 0.3);
  inputVariables[2] = (inputVariables[2] * 0.7) + ((g_raw[3] / 65.5) * 0.3);

  gyro_c.the_Gyroscope();

  PID_c.PID_Init();
  PID_c.PID_Controller();

  Serial.print(" RadioStatus: ");Serial.println(radio_online);
  Serial.print(" AutoPilot: ");Serial.println(auto_pilot);
  Serial.print(" ESC1: ");Serial.print(mESC1);
  Serial.print(" ESC2: ");Serial.print(mESC2);
  Serial.print(" ESC3: ");Serial.print(mESC3);
  Serial.print(" ESC4: ");Serial.println(mESC4);
  Serial.print(" Set Roll: ");Serial.print(setVariables[0]);
  Serial.print(" Set Pitch: ");Serial.print(setVariables[1]);
  Serial.print(" Set Yaw: ");Serial.print(setVariables[2]);
  Serial.print(" Set Thrust: ");Serial.print(setVariables[3]);
  Serial.print(" Input roll: "); Serial.println(integralVariables[0]);
  Serial.println(" ");
  Serial.print(" In Roll: ");Serial.print(inputVariables[0]);
  Serial.print(" In Pitch: ");Serial.print(inputVariables[1]);
  Serial.print(" In Yaw: ");Serial.print(inputVariables[2]);
  Serial.println(" ");
  Serial.print(" Roll: ");Serial.print(outputVariables[0]);
  Serial.print("  Pitch: ");Serial.print(outputVariables[1]);
  Serial.print("  Yaw: ");Serial.print(outputVariables[2]);
  Serial.print("  Thrust: ");Serial.print(setVariables[3]);
//  Serial.print("  Integral: ");Serial.print(integralVariables[0]);
//  Serial.print("  Integral 2: ");Serial.print(integralVariables[1]);


  if(micros() - loopTimer > 4050)digitalWrite(yellowPin, HIGH);
  
  while (micros() - loopTimer < 4000);    //Ensuring arduino 250Hz Clock
  loopTimer = micros();
  
  
  Motor_c.e_Driver(); 
  //Motor_c.gen_Pulse(loopTimer);
  Motor_c.motor_drive();
  
//  Serial.println(" ");
//  Serial.print(" ESCT 1: ");Serial.print(esc_timer_1 / 1000000);
//  Serial.print(" ESCT 2: ");Serial.print(esc_timer_2 / 1000000);
//  Serial.print(" ESCT 3: ");Serial.print(esc_timer_3 / 1000000);  
//  Serial.print(" ESCT 4: ");Serial.print(esc_timer_4 / 1000000);
//  Serial.println(" ");
//  Serial.print(" LoopT: ");Serial.print(loopTimer);


}
