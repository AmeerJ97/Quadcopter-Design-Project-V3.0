#include "global_var.h"
#include "led_.h"
#include "radio_nrf24.h"
#include "gyro.h"
#include "PID_controller.h"
#include "Motor_controller.h"
#include "State_controller.h"
#include "Arduino.h"

uint32_t loopTimer;                     //loopTimer to ensure each main loop iteration is 4000microseconds
uint32_t timer_i;                       //integral loop timer

void setup() {
  Serial.begin(57600);
  led_c.led_Init();

  gyro_c.imu_Init();
  gyro_c.gyro_calibrate();
  PID_c.Init_PID();

  rfRadio_c.radio_Init();

  Motor_c.motor_attach();
  Motor_c.port_Init();
  //Motor_c.motor_port_Init();

  delay(5000);

  interrupts();         //Ensure Interrupts are Enabled

  analogWrite(yellowPin, 0);
  quad_state = 1;
  loopTimer = micros();
  timer_i = micros();
}


void loop() {
  if (quad_state == -1) {
    analogWrite(bluePin, 255);
    State_c.State_controller();
    Motor_c.motor_sleep();
    Serial.print("STATE: "); Serial.println(quad_state);
    analogWrite(bluePin, 0);
  }
  if (quad_state == 0) {
    State_c.State_Motor_Init();
    Serial.print("STATE: "); Serial.println(quad_state);
    loopTimer = timer_i = micros();
  }
  while (quad_state == 1) {

    if (loopTimer - timer_i > 30000000 ) {
      //PID_c.reset_Integral();
      timer_i = micros();
    }

    State_c.State_controller();

    analogWrite(redPin, 0);

    gyro_c.read_Imu();

    inputVariables[0] = (inputVariables[0] * 0.7) + ((g_raw[2] / 65.5) * 0.3);
    inputVariables[1] = (inputVariables[1] * 0.7) + ((g_raw[1] / 65.5) * 0.3);
    inputVariables[2] = (inputVariables[2] * 0.7) + ((g_raw[3] / 65.5) * 0.3);

    gyro_c.the_Gyroscope();

    PID_c.PID_Init();
    PID_c.PID_Controller();


    while (micros() - loopTimer < 4000);    //Ensuring arduino 250Hz Clock
    loopTimer = micros();


    Motor_c.e_Driver();
    //Motor_c.gen_Pulse(loopTimer);
    Motor_c.motor_drive();

    print_data();
  }

}
//int vibration_counter = 0;
//int vibration = 0;
//int vibration_tot = 0;

void print_data() {
  Serial.print("                              STATE: "); Serial.println(quad_state);
  //  Serial.println(timer_i);
  //  Serial.println(loopTimer);
  //  Serial.println(" ");
  //  Serial.print(" ESCT 1: "); Serial.print(esc_timer_1 / 1000000);
  //  Serial.print(" ESCT 2: "); Serial.print(esc_timer_2 / 1000000);
  //  Serial.print(" ESCT 3: "); Serial.print(esc_timer_3 / 1000000);
  //  Serial.print(" ESCT 4: "); Serial.print(esc_timer_4 / 1000000);
  //  Serial.println(" ");
  //  Serial.print(" LoopT: "); Serial.print(loopTimer);
  Serial.print("                              Radio Status: "); Serial.println(radio_online);
  Serial.print("                              Radio Counter:"); Serial.println(counter);
  Serial.print("                              Auto Pilot: "); Serial.println(auto_pilot);
  Serial.print(" ESC1: "); Serial.print(mESC1);
  Serial.print(" ESC2: "); Serial.print(mESC2);
  Serial.print(" ESC3: "); Serial.print(mESC3);
  Serial.print(" ESC4: "); Serial.println(mESC4);
  Serial.print(" Set Roll: "); Serial.print(setVariables[0]);
  Serial.print(" Set Pitch: "); Serial.print(setVariables[1]);
  Serial.print(" Set Yaw: "); Serial.print(setVariables[2]);
  Serial.print(" Set Thrust: "); Serial.println(setVariables[3]);
  Serial.print(" In Roll: "); Serial.print(inputVariables[0]);
  Serial.print(" In Pitch: "); Serial.print(inputVariables[1]);
  Serial.print(" In Yaw: "); Serial.println(inputVariables[2]);
  Serial.print(" Roll: "); Serial.print(outputVariables[0]);
  Serial.print("  Pitch: "); Serial.print(outputVariables[1]);
  Serial.print("  Yaw: "); Serial.print(outputVariables[2]);
  Serial.print("  Thrust: "); Serial.println(setVariables[3]);
  Serial.print("  Integral roll: "); Serial.print(integralVariables[0]);
  Serial.print("  Integral pitch: "); Serial.print(integralVariables[1]);
  Serial.print("  Integral yaw: "); Serial.println(integralVariables[2]);
  Serial.print("  Gx_angle: "); Serial.print(g_angle[1]);
  Serial.print("  Gy_angle: "); Serial.print(g_angle[2]);
  Serial.print("  Gz_angle: "); Serial.println(g_angle[3]);
  Serial.print("  Gx_raw: "); Serial.print(g_raw[1]);
  Serial.print("  Gy_raw: "); Serial.print(g_raw[2]);
  Serial.print("  Gz_raw: "); Serial.println(g_raw[3]);
  Serial.print("  Ax_angle: "); Serial.print(a_angle[1]);
  Serial.print("  Ay_angle: "); Serial.print(a_angle[2]);
  Serial.print("  Az_angle: "); Serial.println(a_angle[3]);
  Serial.print("  Ax_raw: "); Serial.print(a_raw[1]);
  Serial.print("  Ay_raw: "); Serial.print(a_raw[2]);
  Serial.print("  Az_raw: "); Serial.println(a_raw[3]);
  Serial.println(" ");
  Serial.println(" ");
  //  for (int start = 0; start <= 15 ; start++){
  //    gyro_c.read_Imu();
  //    vibration_tot += acc_vector;
  //  }
  //
  //  vibration_tot /= 16;
  //
  //  if(vibration_counter < 20){
  //    vibration_counter ++;
  //    gyro_c.read_Imu();
  //    vibration += abs(acc_vector - vibration_tot);
  //  }else{
  //    vibration_counter = 0;
  //    Serial.print("Vibration: ");Serial.println((vibration/50) - 321);
  //    vibration = 0;
  //  }
}
