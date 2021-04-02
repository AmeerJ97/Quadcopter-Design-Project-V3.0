#include "Motor_controller.h"
#include "global_var.h"
#include "Arduino.h"
#include "gyro.h"

#include <Servo.h>

/*------------------- ESC Variables-------------------*/
#define SPEED_MIN (1000)
#define SPEED_MAX (2000)
int motorSpeed = 0;               //Base ESC speed for all Motors
int pad_speed = 0;
boolean motorFlag = true;         //Turn on motors on first iteration
boolean elevate = false;          //Boolean to control motorSpeed variable
boolean auto_pilot = true;        //auto pilot mode
boolean stopMotors = false;       //Boolean to stop the quadcopter
int mESC1, mESC2, mESC3, mESC4;   //ESC motor variables
uint32_t esc_timer_1, esc_timer_2, esc_timer_3, esc_timer_4;
uint32_t esc_loop_timer;
Servo oESC1,oESC2,oESC3,oESC4;    //Declaring ESC Motors as objects

Motor_class::Motor_class(){
}

void Motor_class::motor_port_Init(){
    analogWrite(yellowPin, 255);
    PORTD |= B01111000;       //Set ports 3, 4, 5, 6 to high
    delayMicroseconds(2000);
    PORTD &= B10000111;
    delay(5000);
    PORTD |= B01111000;       //Set ports 3, 4, 5, 6 to high
    delayMicroseconds(1000);
    PORTD &= B10000111;
    delayMicroseconds(5000);
    analogWrite(yellowPin, 0);
}

void Motor_class::port_Init(){
  DDRD |= B01111000;                          //Configuring ports 6,5,4,3 as outputs
}

void Motor_class::gen_Pulse(uint32_t loop_Timer){
  analogWrite(greenPin,255);
  analogWrite(redPin,0);
  digitalWrite(yellowPin, LOW);
  loop_Timer = micros();
  PORTD |= B01111000;       //Set ports 3, 4, 5, 6 to high
  esc_timer_1 = mESC1 + loop_Timer;
  esc_timer_2 = mESC2 + loop_Timer;
  esc_timer_3 = mESC3 + loop_Timer;
  esc_timer_4 = mESC4 + loop_Timer;

  while(PORTD >= 135){
    //Serial.print(" PORTD: "); Serial.println(PORTD);
    Serial.print(" Timer: ");Serial.println(esc_loop_timer);
    esc_loop_timer = micros();
    if(esc_timer_1 <= esc_loop_timer) PORTD &= B10111111; //Set ESC1 or port 6 to low
    if(esc_timer_2 <= esc_loop_timer) PORTD &= B11011111; //Set ESC2 or port 5 to low
    if(esc_timer_3 <= esc_loop_timer) PORTD &= B11101111; //Set ESC3 or port 4 to low
    if(esc_timer_4 <= esc_loop_timer) PORTD &= B11110111; //Set ESC4 or port 3 to low
  }
 analogWrite(greenPin,0); 
}
void Motor_class::e_Driver(){
    if (elevate == true){
    motorSpeed = 1350;
  }
  else if (elevate == false){
    motorSpeed = 1175;
  }

  if (controllerData.upB == true)
  {
    pad_speed += 2;
    controllerData.upB = false;
  }
  if (controllerData.downB == true){
    pad_speed -= 2;
    controllerData.downB == false;
  }
  
  if(motorFlag == false && stopMotors == true){
   mESC1 = 1000;
   mESC2 = 1000; 
   mESC3 = 1000;
   mESC4 = 1000;
   
  } else if(motorFlag == true && stopMotors == false){
  
   mESC1 = pad_speed + motorSpeed + setVariables[3] + outputVariables[0] - outputVariables[1] - outputVariables[2];
   mESC2 = pad_speed + motorSpeed + setVariables[3] - outputVariables[0] - outputVariables[1] + outputVariables[2];
   mESC3 = pad_speed + motorSpeed + setVariables[3] - outputVariables[0] + outputVariables[1] - outputVariables[2];
   mESC4 = pad_speed + motorSpeed + setVariables[3] + outputVariables[0] + outputVariables[1] + outputVariables[2];
  
  if(mESC1 < 1050) mESC1 = 1050; if(mESC1 > 1750) mESC1 = 1750;
  if(mESC2 < 1050) mESC2 = 1050; if(mESC2 > 1750) mESC2 = 1750;
  if(mESC3 < 1050) mESC3 = 1050; if(mESC3 > 1750) mESC3 = 1750;
  if(mESC4 < 1050) mESC4 = 1050; if(mESC4 > 1750) mESC4 = 1750;  
  }

}

void Motor_class::motor_attach(){
  oESC1.attach(6);
  oESC2.attach(5);
  oESC3.attach(4);
  oESC4.attach(3);
}

void Motor_class::motor_drive(){
  if(!radio_online){
    analogWrite(redPin,0);
    mESC1 = mESC2 = mESC3 = mESC4 = 1000;
  }
  if(!stopMotors){
  analogWrite(greenPin,255);
  analogWrite(redPin,0);
  digitalWrite(yellowPin, LOW);
  oESC1.writeMicroseconds(mESC1);
  oESC2.writeMicroseconds(mESC2);
  oESC3.writeMicroseconds(mESC3);
  oESC4.writeMicroseconds(mESC4);
   analogWrite(greenPin,0); 
  }
  else{
    oESC1.writeMicroseconds(1000);
    oESC2.writeMicroseconds(1000);
    oESC3.writeMicroseconds(1000);
    oESC4.writeMicroseconds(1000);
    analogWrite(redPin, 0);analogWrite(greenPin, 0);digitalWrite(yellowPin, HIGH);
  }
  
}

Motor_class Motor_c = Motor_class();
