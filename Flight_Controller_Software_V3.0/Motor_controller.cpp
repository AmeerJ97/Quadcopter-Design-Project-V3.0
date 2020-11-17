#include "Motor_controller.h"
#include "global_var.h"
#include "Arduino.h"

#include <Servo.h>

/*------------------- ESC Variables-------------------*/
#define SPEED_MIN (1000)
#define SPEED_MAX (2000)
int motorSpeed = 0;               //Base ESC speed for all Motors
boolean motorFlag = true;         //Turn on motors on first iteration
boolean elevate = false;          //Boolean to control motorSpeed variable
boolean armMotors = false;        //Boolean to initiate arm motors method
boolean stopMotors = false;       //Boolean to stop the quadcopter
int mESC1, mESC2, mESC3, mESC4, mOffset; //ESC motor variables

Servo oESC1,oESC2,oESC3,oESC4;    //Declaring ESC Motors as objects

Motor_class::Motor_class(){
}

void Motor_class::motor_Init(){
  analogWrite(yellowPin, 255);                //Blue LED to indicate arming sequence
  armMotors = false;                          //Turn off arming flag
  oESC1.writeMicroseconds(2000);
  oESC2.writeMicroseconds(2000);
  oESC3.writeMicroseconds(2000);
  oESC4.writeMicroseconds(2000);
  delay(5000); 
  oESC1.writeMicroseconds(1000);
  oESC2.writeMicroseconds(1000);
  oESC3.writeMicroseconds(1000);
  oESC4.writeMicroseconds(1000);
  delay(5000);
  analogWrite(yellowPin, 0);
}

void Motor_class::port_Init(){
  DDRD != B01111000;                          //Configuring ports 6,5,4,3 as outputs
}

void Motor_class::gen_Pulse(uint32_t loopTimer, int esc1, int esc2, int esc3, int esc4){
  PORTD |= B01111000;       //Set ports 3, 4, 5, 6 to high
  esc_timer_1 = mESC1 + loopTimer;
  esc_timer_2 = mESC2 + loopTimer;
  esc_timer_3 = mESC3 + loopTimer;
  esc_timer_4 = mESC4 + loopTimer;

  while(PORTD >= 16){
    esc_loop_timer = micros();
    if(esc_timer_1 <= esc_loop_timer) PORTD &= B10111111; //Set ESC1 or port 6 to low
    if(esc_timer_2 <= esc_loop_timer) PORTD &= B11011111; //Set ESC2 or port 5 to low
    if(esc_timer_3 <= esc_loop_timer) PORTD &= B11101111; //Set ESC3 or port 4 to low
    if(esc_timer_4 <= esc_loop_timer) PORTD &= B11110111; //Set ESC4 or port 3 to low
  }
  
}
void Motor_class::e_Driver(){
  analogWrite(greenPin,255);                  //Green Pin to indicate motors signal
  analogWrite(redPin,0);
  if (elevate == true){
    motorSpeed = 1450;
    mOffset = 100;
  }
  else if (elevate == false){
    motorSpeed = 1150;
    mOffset = 50;
  }

  //Deadband to counteract vibrations and other sources of error
  if(outputVariables[3] - outputVariables[0] > 20 || outputVariables[3] - outputVariables[0] < -20) outputVariables[0] = outputVariables[0];
  else if (outputVariables[3] - outputVariables[0] < 20 || outputVariables[3] - outputVariables[0] > -20)outputVariables[0] = outputVariables[3];

  if(outputVariables[4] - outputVariables[1]  > 20 || outputVariables[4] - outputVariables[1]  < -20)  outputVariables[1] = outputVariables[1];
  else if (outputVariables[4] - outputVariables[1]  < 20 || outputVariables[4] - outputVariables[1]  > - 20) outputVariables[1] = outputVariables[4];

  if(outputVariables[5] - outputVariables[2] > 10 || outputVariables[5] - outputVariables[2] < -10) outputVariables[2] = outputVariables[2];
  else if (outputVariables[5] - outputVariables[2] < 10 || outputVariables[5] - outputVariables[2] > -10)  outputVariables[2] = outputVariables[5];

//  if(pid.lastelevSet - pid.elevSet > 15  || pid.lastelevSet - pid.elevSet < -15) pid.elevSet = pid.elevSet;
//  else if (pid.lastelevSet - pid.elevSet < 15 || pid.lastelevSet - pid.elevSet > -15) pid.elevSet = pid.lastelevSet;

  //Updating last output variables
  outputVariables[3] = outputVariables[0];
  outputVariables[4] = outputVariables[1];
  outputVariables[5] = outputVariables[2];
  
  if(motorFlag == false && stopMotors == true){
   mESC1 = 0;
   mESC2 = 0; 
   mESC3 = 0;
   mESC4 = 0;
   
  } else if(motorFlag == true && stopMotors == false){
   mESC1 = motorSpeed + setVariables[3] + outputVariables[0] - outputVariables[1] - outputVariables[2];
   mESC2 = motorSpeed + setVariables[3] - outputVariables[0] - outputVariables[1] + outputVariables[2];
   mESC3 = motorSpeed + setVariables[3] - outputVariables[0] + outputVariables[1] - outputVariables[2];
   mESC4 = motorSpeed + setVariables[3] + outputVariables[0] + outputVariables[1] + outputVariables[2];
  if(mESC1 < 1050) mESC1 = 1050; if(mESC1 > 1750) mESC1 = 1750;
  if(mESC2 < 1050) mESC2 = 1050; if(mESC2 > 1750) mESC2 = 1750;
  if(mESC3 < 1050) mESC3 = 1050; if(mESC3 > 1750) mESC3 = 1750;
  if(mESC4 < 1050) mESC4 = 1050; if(mESC4 > 1750) mESC4 = 1750;  
  }

  oESC1.writeMicroseconds(mESC1);
  oESC2.writeMicroseconds(mESC2);
  oESC3.writeMicroseconds(mESC3);
  oESC4.writeMicroseconds(mESC4);
  

  analogWrite(greenPin,0);
}

void Motor_class::motor_attach(){
  oESC1.attach(6);
  oESC2.attach(5);
  oESC3.attach(4);
  oESC4.attach(3);
}

Motor_class Motor_c = Motor_class();
