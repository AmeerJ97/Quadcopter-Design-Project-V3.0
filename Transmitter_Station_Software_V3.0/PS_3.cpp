#include "PS_3.h"
#include <Arduino.h>
#include "global_struct.h"

#include <PS3USB.h>
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

USB Usb;
PS3USB PS3(&Usb);

controllerStruct controllerData;

PS3_class::PS3_class(){
}

void PS3_class::PS3_Init(){
  #if !defined(_MIPSEL_)
  while(!Serial); //wait for serial port to connect 
  #endif
  if (Usb.Init() == -1){
    Serial.print(F("\r\nOSC did not start"));
    while(1); //halot
    }
    Serial.print("\r\nPS3 USB Library Started");
}

void PS3_class::reset_buttons(){
  controllerData.xButton = false;
  controllerData.oButton = false;
  controllerData.tButton = false;
  controllerData.sButton = false;
  controllerData.Lt = controllerData.Lb = controllerData.Rt = controllerData.Rb = false;
}

void PS3_class::read_PS3(){
  Usb.Task();
  if (PS3.PS3Connected || PS3.PS3NavigationConnected){        
    controllerData.Lx = PS3.getAnalogHat(LeftHatX);
    controllerData.Ly = PS3.getAnalogHat(LeftHatY);
    controllerData.Rx = PS3.getAnalogHat(RightHatX);
    controllerData.Ry = PS3.getAnalogHat(RightHatY);
    if(PS3.getButtonClick(CIRCLE))controllerData.oButton = true;
    if(PS3.getButtonClick(CROSS))controllerData.xButton = true;
    if(PS3.getButtonClick(SQUARE))controllerData.sButton = true;
    if(PS3.getButtonClick(TRIANGLE))controllerData.tButton = true;
    if(PS3.getButtonClick(L1))controllerData.Lb = true;
    if(PS3.getButtonClick(L2))controllerData.Lt = true;
    if(PS3.getButtonClick(R1))controllerData.Rb = true;
    if(PS3.getButtonClick(R2))controllerData.Rt = true; 
    
    //Mapping and shifting analog data such that 1500 is the center point
    controllerData.Lx = map(controllerData.Lx,0,255,1000,2000) ;
    controllerData.Rx = map(controllerData.Rx,0,255,1000,2000) ;
    controllerData.Ry = map(controllerData.Ry,255,0,1000,2000) ;
    controllerData.Ly = map(controllerData.Ly,255,0,1000,2000) ;
  
    
    //Deadband to avoid PS3 joystick fluctuations
    if((controllerData.Lx < 1750) && (controllerData.Lx > 1250)) controllerData.Lx = 1500;
    if((controllerData.Ly < 1750) && (controllerData.Ly > 1250)) controllerData.Ly = 1500;
    if((controllerData.Rx < 1750) && (controllerData.Rx > 1250)) controllerData.Rx = 1500;
    if((controllerData.Ry < 1750) && (controllerData.Ry > 1250)) controllerData.Ry = 1500;

//    Serial.print("Lx: ");Serial.print(controllerData.Lx);
//    Serial.print(" Ly: ");Serial.print(controllerData.Ly);
//    Serial.print(" Rx: ");Serial.print(controllerData.Rx);
//    Serial.print(" Ry: ");Serial.print(controllerData.Ry);
//    Serial.print(" S: ");Serial.print(controllerData.sButton);
//    Serial.print(" T: ");Serial.print(controllerData.tButton);
//    Serial.print(" O: ");Serial.print(controllerData.oButton);
//    Serial.print(" X: ");Serial.print(controllerData.xButton);
//    Serial.println(" "); 
  }
}

PS3_class PS3_c = PS3_class();
