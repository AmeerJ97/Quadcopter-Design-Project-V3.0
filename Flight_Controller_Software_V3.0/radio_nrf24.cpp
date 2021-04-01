#include "radio_nrf24.h"
#include <Arduino.h>
#include "global_var.h"


#include <nRF24L01.h>
#include <RF24.h>
#include <RF24_config.h>

#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>

const int pinCE = 7;                    
const int pinCSN = 8;
const int pinInt = 2;                   //Pin used for nRF24 interrupts
uint64_t channel_addr = 0xB00B1E5000LL; //Channel address
int counter;                  //Counter to track failed packets
boolean radio_online = false;
RF24 rfRadio(pinCE,pinCSN);             //Declaring nRF24 Object

radio_class::radio_class(){
}

void radio_class::radio_Interrupt(){
  analogWrite(redPin,255);                //Set red LED Pin to high to indicate data rx 
  //analogWrite(greenPin,0);
  bool tx,fail,rx;                        //Local Variables 
  rfRadio.whatHappened(tx,fail,rx);       
 //Read data on rx
 if (rx){
    rfRadio.read(&controllerData,sizeof(controllerData));
    radio_online = true;
    counter = 0;
   }

   
}

void radio_class::radio_Init(){
  rfRadio.begin();                        //Initialize nRF24
  rfRadio.setPALevel(RF24_PA_LOW);        //Setting power level to low, microstrip antenna is rx only
  rfRadio.setDataRate(RF24_1MBPS);        //Setting link speed to 1MBps
//  if(rfRadio.isChipConnected() == true){
//    Serial.println("nRF24 Chip Connected.");
//  }else{
//    Serial.println("No nRF24 Chip Found.");
//  }
  rfRadio.setAutoAck(false);                //Enable auto ack
 // rfRadio.enableAckPayload();             //Enable ack payload
  //rfRadio.setRetries(0, 2);               //Set Retries to 2 to avoid delay
  // rfRadio.setCRCLength(RF24_CRC_16);
  rfRadio.maskIRQ(1,1,0);                   //Mask all INT Triggers except for Rx
  rfRadio.openReadingPipe(1,channel_addr);  //Open channel 1 to recieve struct type data
  rfRadio.startListening();
  pinMode(pinInt, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinInt), radio_Interrupt, CHANGE); //Create interrupt for pin 9 on falling edge
}

void radio_class::check_radio(){
  if(rfRadio.failureDetected || radio_online == false) {          
    //ReInitialize Radio and set radio flag to false
       radio_Init();
   }
}
