#include "Motor_controller.h"
#include "global_var.h"

#include <Servo.h>

/*------------------- ESC Variables-------------------*/
#define SPEED_MIN (1000)
#define SPEED_MAX (2000)
int motorSpeed = 0;               //Base ESC speed for all Motors
boolean motorFlag = true;         //Turn on motors on first iteration
boolean elevate = false;          //Boolean to control motorSpeed variable
boolean armMotors = false;        //Boolean to initiate arm motors method
boolean stopMotors = false;       //Boolean to stop the quadcopter
float mESC1, mESC2, mESC3, mESC4, mOffset; //ESC motor variables

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

//  pid.lastoutRoll = pid.outRoll;
//  pid.lastoutPitch = pid.outPitch;
//  pid.lastoutYaw = pid.outYaw;
//  pid.lastelevSet = pid.elevSet;
//  
  
  oESC1.writeMicroseconds(mESC1);
  oESC2.writeMicroseconds(mESC2);
  oESC3.writeMicroseconds(mESC3);
  oESC4.writeMicroseconds(mESC4);
  
  Serial.print(" ESC1: ");Serial.print(mESC1);
  Serial.print(" ESC2: ");Serial.print(mESC2);
  Serial.print(" ESC3: ");Serial.print(mESC3);
  Serial.print(" ESC4: ");Serial.println(mESC4);
  
    
  
  analogWrite(greenPin,0);
}

void Motor_class::motor_attach(){
  oESC1.attach(6);
  oESC2.attach(5);
  oESC3.attach(4);
  oESC4.attach(3);
}
Motor_class Motor_c = Motor_class();
