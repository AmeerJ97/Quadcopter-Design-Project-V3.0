#include "PID_Controller.h"
#include "global_var.h"

/*------------------- PID Variables-------------------*/
//P Variables
float KP_roll = 1;
float KP_pitch = 1;
float KP_yaw = 0.5;
//I Variables
float KI_roll = 0.001;
float KI_pitch = 0.001;
float KI_yaw = 0;
//D Variables
float KD_roll = 0;
float KD_pitch = 0;
float KD_yaw = 0;


float setVariables[4] = {0,0,0,0};       //roll set, pitch set, yaw set, elevation set
float fixVariables[3] = {0,0,0};         //roll fix, pitch fix, yaw pitch 
float inputVariables[3] = {0,0,0};       //roll input, pitch input, yaw input
float integralVariables[3] = {0,0,0};    //roll integral, pitch integral, yaw integral
float outputVariables[6] = {0,0,0,0,0,0};//roll output, pitch output, yaw output, last roll output, last pitch output, last yaw output
float lastDvariables[3] = {0,0,0};       //roll last D, pitch last D, yaw last D
controllerStruct controllerData;

PID_class::PID_class(){
}

  /* PID initialization function to input controller data */
void PID_class::PID_Init(){
  /* Motor Startup condition via L2 and R2, R1 and L1 stop motors */
  if (controllerData.Lt == true && controllerData.Rt == true){
    motorFlag = true;
    stopMotors = false;
    armMotors = true;
   // Serial.println("########### MOTOR STARTUP ############");
    controllerData.Lt = false;
    controllerData.Rt = false;
    
  } else if (controllerData.Lb == true && controllerData.Rb == true){
    motorFlag = false;
    //Serial.println("########### MOTOR SHUTDOWN ############");
    controllerData.Lb = false;
    controllerData.Rb = false;
    armMotors = false;
    elevate = false;
    stopMotors = true;
   }
  if (controllerData.sButton == true){
    motorFlag = false;
    armMotors = false;
    elevate = false;
    stopMotors = true;
    controllerData.sButton = false;
  }
   
  //State control of quadcopter via PS3 buttons
  if (motorFlag == true && controllerData.xButton == true) armMotors = true;
  if (motorFlag == true && controllerData.oButton == true) elevate = true;
  if (motorFlag == true && controllerData.tButton == true) elevate = false;
//  if (motorFlag == true && controllerData.sButton == true) stopMotors = true;
//  if (motorFlag == true && controllerData.sButton == false) stopMotors = false;



  /* Pid setpoint controlled by PS3 controller */
  //Requires a deadband of 90 due to PS3 input fluctuations
  //Thus, maximum set roll input is 164.0 deg/s
  //Roll Set calculations
  setVariables[0] = 0;
  if (controllerData.Rx > 1590)setVariables[0] = (controllerData.Rx - 1590) ;
  else if (controllerData.Rx < 1410)setVariables[0] = (controllerData.Rx - 1410);
  setVariables[0] -= g_angle[1] * 15;          //Gyroscope roll correction
  setVariables[0] /= 3;


  //Pitch Set calculations
  setVariables[1] = 0;
  if (controllerData.Ry > 1590)setVariables[1] = controllerData.Ry - 1590;
  else if (controllerData.Ry < 1410)setVariables[1] =  controllerData.Ry - 1410;
  setVariables[1] -= g_angle[2]* 15;
  setVariables[1] /= 3;

  //Yaw Set calculations
  setVariables[2] = 0;
  if(controllerData.Lx > 1590)setVariables[2] = controllerData.Lx - 1590;
  else if(controllerData.Lx < 1410)setVariables[2] = controllerData.Lx - 1410;
  setVariables[2] /= 5;
  
  //Elevation set calculations
  setVariables[3] = 0;
  if(controllerData.Ly > 1590)setVariables[3] = controllerData.Ly - 1590;
  else if(controllerData.Ly < 1410)setVariables[3] = controllerData.Ly - 1410;
  setVariables[3] /= 2;

}

  /* PID Controller Function */ 
void PID_class::PID_Controller(){
   //Roll Calculations
  fixVariables[0] = inputVariables[0] - setVariables[0];                //Calculating error
  integralVariables[0] += KI_roll * fixVariables[0];                    //Integral Controller
  if (integralVariables[0] > 400) integralVariables[0] = 400;           //Upper limit on integral controller
  else if (integralVariables[0] <= -400) integralVariables[0] = -400;  //Lower limit on integral controller

  outputVariables[0] = KP_roll * fixVariables[0] +integralVariables[0] + KD_roll * (fixVariables[0] - lastDvariables[0]);
  if (outputVariables[0] > 400) outputVariables[0] = 400;
  else if (outputVariables[0] < -400) outputVariables[0] = -400;
  lastDvariables[0] = fixVariables[0];

  //Pitch Calculations
  fixVariables[1] = inputVariables[1] - setVariables[1];
  integralVariables[1] += KI_pitch * fixVariables[1];
  if (integralVariables[1] > 400) integralVariables[1] = 400;
  else if (integralVariables[1] <= -400) integralVariables[1] = -400;

  outputVariables[1] = KP_pitch * fixVariables[1]  + KD_pitch * (fixVariables[1] - lastDvariables[1]);
  if (outputVariables[1] > 400) outputVariables[1] = 400;
  else if (outputVariables[1] < -400) outputVariables[1] = -400;
  lastDvariables[1] = fixVariables[1];

  //Yaw Calculations
  fixVariables[2] = inputVariables[2] - setVariables[2];
  integralVariables[2] += KI_yaw * fixVariables[2];
  if (integralVariables[2] > 400) integralVariables[2] = 400;
  else if (integralVariables[2] <= -400) integralVariables[2] = -400;

  outputVariables[2] = KP_yaw * fixVariables[2]  + KD_yaw * (fixVariables[2] - lastDvariables[2]);
  if (outputVariables[2] > 400) outputVariables[2] = 400;
  else if (outputVariables[2] < -400) outputVariables[2] = -400;
  lastDvariables[2] = fixVariables[2];



}

PID_class PID_c = PID_class();
