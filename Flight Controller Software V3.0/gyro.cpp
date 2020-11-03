#include "gyro.h"
#include <Arduino.h>
#include "global_var.h"

#include <Wire.h>
/*------------------- General Variables-------------------*/
const int imu_addr = 0x68;              //IMU Address for I2C communication
float rad_2_deg = 57.29577;             //Radians to degrees constant
/*------------------- GY-521 IMU Global Variables---------------*/
bool gyro_Flag = false;              //Gyroscope flags to track startup
bool gyro_Init = false;               
int calibrate;                          //Calibration variable on startup
//Gyro and accelerometer (X,Y,Z) arrays and temperature variable(s) 
float g_raw[4],a_raw[4], g_angle[4], a_angle[4],g_err[4], temperature; 
float alpha_gyro = 0.9;                 //complementary filter
float alpha_acc = 0.1;

gyro_class::gyro_class(){
}
/* IMU Initialization function */
void gyro_class::imu_Init(){
   if (gyro_Init == false){
    Wire.begin();                         //Begin the I2C link as a master
    
    //Configuring IMU Power
    Wire.beginTransmission(imu_addr);     //Start the I2C communication with the IMU
    Wire.write(0x6B);                     //Acessing Power Management register 1, 107 decimal = 6B Hex
    Wire.write(0x00);                     //Set the register as 0 to wake the IMU from sleep mode
    Wire.endTransmission();               //End transmission to IMU

    //Configuring IMU Gyroscope
    Wire.beginTransmission(imu_addr);     //Start the I2C communication with the IMU
    Wire.write(0x1B);                     //Acessing Gyroscope Configuration Register, 27 decimal = 1B
    Wire.write(0x10);                     //Set the register as 00010000 indicating readings in +/- 1000 deg/s
    Wire.endTransmission();               //End transmission to IMU

    //Configuring IMU Accelerometer
    Wire.beginTransmission(imu_addr);     //Start the I2C communication with the IMU
    Wire.write(0x1C);                     //Accessing Accelerometer Configuration Register, 28 decimal = 1C
    Wire.write(0x10);                     //Set the register as 00010000 indicating readings in +/- 8gs
    Wire.endTransmission();               //End transmission to IMU
    
    Serial.println("IMU 6050 Initialized");
    gyro_Init = true;
  }
}
  /* IMU Reading function */
void gyro_class::read_Imu(){
  //Registers 59 to 64, 65 to 66, 67 to 72 are the accelerometer, 
  //temperature, and gyroscope output registers respectively
  Wire.beginTransmission(imu_addr);       //Start the I2C communication with the IMU
  Wire.write(0x3B);                       //Acessing Acc out register, 59 = 0x3B
  Wire.endTransmission();                 //End transmission to IMU
  Wire.requestFrom(imu_addr, 14);         //Request 14 bytes from IMU starting at Acc out register
  while(Wire.available() < 14);           //Waiting for the 14 bytes to be recieved

  a_raw[1] = Wire.read()<<8 | Wire.read();  //Reading 8 bytes from high and low Acc X output Register 
  a_raw[2] = Wire.read()<<8 | Wire.read();  //Reading 8 bytes from high and low Acc Y output Register 
  a_raw[3] = Wire.read()<<8 | Wire.read();  //Reading 8 bytes from high and low Acc Z output Register 
  temperature = Wire.read()<<8 | Wire.read(); //Reading 8 bytes from high and low temperature output register
  g_raw[1] = Wire.read()<<8 | Wire.read();  //Reading 8 bytes from high and low Gyro X output Register 
  g_raw[2] = Wire.read()<<8 | Wire.read();  //Reading 8 bytes from high and low Gyro Y output Register 
  g_raw[3] = Wire.read()<<8 | Wire.read();  //Reading 8 bytes from high and low Gyro Z output Register 
  
  Wire.endTransmission();
  //Directional Correction
  //a_raw[2] *= -1;
  //g_raw[2] *= -2;
  
  //Subtracting calibrated values if calibration already preformed
  if (calibrate == 3000){
    g_raw[1] -= g_err[1];
    g_raw[2] -= g_err[2];
    g_raw[3] -= g_err[3];
  }
}

  /* Gyroscope calibration function */
void gyro_class::gyro_calibrate(){
   Serial.print(" Starting Calibration......."); 
  
  //3000 samples to calibrate
  for (int calibration = 0; calibration < 3000 ; calibration++){     
    read_Imu();
    g_err[1] += g_raw[1];
    g_err[2] += g_raw[2];
    g_err[3] += g_raw[3];    
  }
  
  calibrate = 3000;                       //Flag for read_Imu() calibration initialization
  Serial.println(" Calibration complete!");
  
  g_err[1] /= 3000;
  g_err[2] /= 3000;
  g_err[3] /= 3000;

  Serial.println(" The Calibrated values are:  ");
  Serial.print(" Gx: ");Serial.print(g_err[1]);
  Serial.print(" Gy: "); Serial.print(g_err[2]);
  Serial.print(" Gz: "); Serial.print(g_err[3]);
  Serial.println(" ");
}
  /* Gyroscope math function to convert raw deg/s input to current angle */
void gyro_class::the_Gyroscope(){
  //Discrete summation multiplied by dt equivalent to continous integration
  //where dt = 1  / ( f * 32.8) where f is the frequency of the microcontroller and 32.8 is the selected sensitivity upon gyroscope configuration
  g_angle[1] += g_raw[1] * (1/(32.8 * 250));     //Roll Angle Gx
  g_angle[2] += g_raw[2] * (1/(32.8 * 250));     //Pitch Angle  Gy
  g_angle[3] += g_raw[3] * (1/(32.8 * 250));     //Yaw Angle  Gz

  g_angle[2] -= g_angle[1] * sin(g_raw[3] * (1/(32.8 * 250)) * (1 / 57.29577)); //Pitch
  g_angle[1] += g_angle[2] * sin(g_raw[3] * (1/(32.8 * 250)) * (1 / 57.29577)); //Roll

  //Calculating accelerometer vector to determine direction of acceleration
  float acc_vector = sqrt((a_raw[1] * a_raw[1]) + (a_raw[2] * a_raw[2]) + (a_raw[3] * a_raw[3]));

  if (abs(a_raw[2]) < acc_vector){
    a_angle[2] = asin((float) a_raw[2] / acc_vector) * rad_2_deg;
  }
   if (abs(a_raw[1]) < acc_vector){
    a_angle[1] = asin((float) a_raw[1] / acc_vector) * rad_2_deg;
  }

  //Hardcoded second calibration for accelerometer
  a_angle[1] += 2;                   //Ax
  a_angle[2] += 0;                  //Ay
  a_angle[3] += 0;                   //Az

  //On first run, set gyro X&Y angle values equal to acc X&Y angle values for initial stability
  //If not, implement complementary filter 
  if (gyro_Flag == false){
    g_angle[1] = a_angle[1];
    g_angle[2] = a_angle[2];
  }else {
    g_angle[1] = g_angle[1] * alpha_gyro + a_angle[1] * alpha_acc;
    g_angle[1] = g_angle[2] * alpha_gyro + a_angle[2] * alpha_acc;
  } 
//   Serial.print("Gx_angle: ");Serial.print(g_angle[1]);
//  Serial.print("  Gy_angle: ");Serial.print(g_angle[2]);
//  Serial.print("  Gz_raw: ");Serial.print(g_raw[3]);
//  Serial.print("  Ax_angle: ");Serial.print(a_angle[1]);
//  Serial.print("  Ay_angle: ");Serial.print(a_angle[2]);
//  Serial.print("  Az_raw: ");Serial.print(a_raw[3]);
//  Serial.println(" ");
}
