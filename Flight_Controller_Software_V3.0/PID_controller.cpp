#include "PID_Controller.h"
#include "global_var.h"

/*------------------- PID Variables-------------------*/
//P Variables
float KP_roll = 0.05;
float KP_pitch = 0.05;
float KP_yaw = 0.0001;
//I Variables
float KI_roll = 0.00005;
float KI_pitch = 0.00005;
float KI_yaw = 0;
//D Variables
float KD_roll = 0.00005;
float KD_pitch = 0.00005;
float KD_yaw = 0;

int flag = 1;
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
  //State control of quadcopter via PS3 buttons
  if (controllerData.sButton == true){
    motorFlag = false;
    elevate = false;
    stopMotors = true;
    controllerData.sButton = false;
  }
  if (controllerData.xButton == true){
    motorFlag = true;
    stopMotors = false;
    controllerData.xButton = false;
  }
  if (motorFlag == true && controllerData.oButton == true) elevate = true;
  if (motorFlag == true && controllerData.tButton == true) elevate = false;

  if (counter >= 15) radio_online == false;
  counter ++;
  
  if (auto_pilot == false || radio_online == false){
    flag = 0;
  }else {
    flag = 1;
  }

  
  /* Pid setpoint controlled by PS3 controller */
  //Requires a deadband of 90 due to PS3 input fluctuations
  //Thus, maximum set roll input is 164.0 deg/s
  //Roll Set calculations
  setVariables[0] = 0;
  if (controllerData.Rx > 1512)setVariables[0] = (controllerData.Rx - 1512) ;
  else if (controllerData.Rx < 1488)setVariables[0] = (controllerData.Rx - 1488);
  setVariables[0] -= g_angle[1] * 15 * flag;          //Gyroscope roll correction
  setVariables[0] /= 3;


  //Pitch Set calculations
  setVariables[1] = 0;
  if (controllerData.Ry > 1512)setVariables[1] = 1512 - controllerData.Ry;
  else if (controllerData.Ry < 1488)setVariables[1] = 1488 - controllerData.Ry;
  setVariables[1] -= g_angle[2]* 15 * flag;
  setVariables[1] /= 3;

  //Yaw Set calculations
  setVariables[2] = 0;
  if(controllerData.Lx > 1512)setVariables[2] = controllerData.Lx - 1512;
  else if(controllerData.Lx < 1488)setVariables[2] = controllerData.Lx - 1488;
  setVariables[2] /= 5;
  
  //Elevation set calculations
  setVariables[3] = 0;
  if(controllerData.Ly > 1512)setVariables[3] = controllerData.Ly - 1512;
  else if(controllerData.Ly < 1488)setVariables[3] = controllerData.Ly - 1488;
  setVariables[3] /= 2;

  if (radio_online == false){
    setVariables[0] = setVariables[1] = setVariables[2] = setVariables[3] = 0;
  }

}

  /* PID Controller Function */ 
void PID_class::PID_Controller(){
   //Roll Calculations
  fixVariables[0] = inputVariables[0] - setVariables[0];                //Calculating error
  integralVariables[0] += KI_roll * fixVariables[0];                    //Integral Controller
  if (integralVariables[0] > 400) integralVariables[0] = 400;           //Upper limit on integral controller
  else if (integralVariables[0] <= -400) integralVariables[0] = -400;  //Lower limit on integral controller

  outputVariables[0] = KP_roll * fixVariables[0] + integralVariables[0] + KD_roll * (fixVariables[0] - lastDvariables[0]);
  if (outputVariables[0] > 400) outputVariables[0] = 400;
  else if (outputVariables[0] < -400) outputVariables[0] = -400;
  lastDvariables[0] = fixVariables[0];

  //Pitch Calculations
  fixVariables[1] = inputVariables[1] - setVariables[1];
  integralVariables[1] += KI_pitch * fixVariables[1];
  if (integralVariables[1] > 400) integralVariables[1] = 400;
  else if (integralVariables[1] <= -400) integralVariables[1] = -400;

  outputVariables[1] = KP_pitch * fixVariables[1] + integralVariables[1] + KD_pitch * (fixVariables[1] - lastDvariables[1]);
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
