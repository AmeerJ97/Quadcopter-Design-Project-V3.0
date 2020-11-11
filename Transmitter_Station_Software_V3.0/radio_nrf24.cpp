#include "radio_nrf24.h"
#include <Arduino.h>
#include "global_struct.h"

#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>

/*------------------- Global Variables-------------------*/
const int pinCE = 7;                       //Pin used to set nRF to standby 0 or active 1
const int pinCSN = 8;                      //Pin used to configure SPI communication (Tx/Rx)
uint64_t channel_addr = 0xB00B1E5000LL ;   //5 Channel addresses

RF24 rfRadio(pinCE, pinCSN);               //RF24 Object

radio_class::radio_class(){
}

void radio_class::radio_Init(){
 printf_begin();
 rfRadio.begin();                         //Start RF24 object
 rfRadio.setAutoAck(false);               //Disable autoACK
 //rfRadio.enableAckPayload();
 rfRadio.setPALevel(RF24_PA_HIGH);         //Setting power to high for maximum range since this is the transmitter
 //rfRadio.setDataRate(RF24_1MBPS); 
 //rfRadio.setCRCLength(RF24_CRC_16);
 if(rfRadio.isChipConnected() == true){
    Serial.println(" ");
    Serial.println("nRF24 Chip Connected.");
  }else{
    Serial.println("No nRF24 Chip Found.");
  }          
 rfRadio.openWritingPipe(channel_addr);     //Open communication channel
 rfRadio.stopListening(); 
 rfRadio.printDetails();
}

void radio_class::radio_tx(){
    Serial.print("Lx: ");Serial.print(controllerData.Lx);
    Serial.print(" Ly: ");Serial.print(controllerData.Ly);
    Serial.print(" Rx: ");Serial.print(controllerData.Rx);
    Serial.print(" Ry: ");Serial.print(controllerData.Ry);
    Serial.print(" S: ");Serial.print(controllerData.sButton);
    Serial.print(" T: ");Serial.print(controllerData.tButton);
    Serial.print(" O: ");Serial.print(controllerData.oButton);
    Serial.print(" X: ");Serial.print(controllerData.xButton);
    Serial.println(" "); 
 //Writing data to channel or pipe,
 if(!rfRadio.writeFast(&controllerData,sizeof(controllerData))){
  //Do nothing
 }
 //If packet transmission failed
 if(!rfRadio.txStandBy(10)){                    //Waits 0.01 second to check transmission status. Flushed FIFO Registers (contain data) on fail.
   //Serial.println("####TRANSMISSION FAILED####"); //Prints to serial monitor on failure
 }
}

radio_class rfRadio_c = radio_class();
