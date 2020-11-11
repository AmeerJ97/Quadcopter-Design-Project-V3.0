#ifndef GUARD_V
#define GUARD_V
/*------------------- Global Variables-------------------*/
/*------------------- LED Global Variables-------------------*/
extern const int bluePin;                      //Blue LED pin
extern const int redPin;                      //Red LED pin
extern const int yellowPin;                     //Yellow LED pin
extern const int greenPin;                    //Green LED pin
/*------------------- GY-521 IMU Global Variables---------------*/
extern bool gyro_Flag;              //Gyroscope flags to track startup
extern bool gyro_Init;               
extern int calibrate;                          //Calibration variable on startup
//Gyro and accelerometer (X,Y,Z) arrays and temperature variable(s) 
extern float g_raw[4],a_raw[4], g_angle[4], a_angle[4],g_err[4], temperature; 
extern float alpha_gyro;                 //complementary filter
extern float alpha_acc;
extern float mESC1, mESC2, mESC3, mESC4, mOffset; //ESC motor variables
/*------------------- nRF24 Transceiver Global Variables-------------------*/
typedef struct{                         //Structure Data type to hold rx PS3 state variables
  int Lx;                               //Left Analog stick X data
  int Ly;                               //Left Analog stick Y data
  int Rx;                               //Right Analog stick X data
  int Ry;                               //Right Analog stick Y data
  bool xButton;                      //Cross button data
  bool oButton;                      //Circle button data
  bool tButton;                      //Triangle button Data
  bool sButton;                      //Square button data
  bool Lt;                           //Left trigger L1
  bool Lb;                           //Left bumper L2
  bool Rt;                           //Right trigger R1
  bool Rb;                           //Right bumper R2
}controllerStruct;
extern controllerStruct controllerData;
/*------------------- PID Global Variables-------------------*/
extern float setVariables[4];    //roll set, pitch set, yaw set, elevation set
extern float fixVariables[3];      //roll fix, pitch fix, yaw pitch 
extern float inputVariables[3];     //roll input, pitch input, yaw input
extern float integralVariables[3];   //roll integral, pitch integral, yaw integral
extern float outputVariables[6];//roll output, pitch output, yaw output, last roll output, last pitch output, last yaw output
extern float lastDvariables[3];      //roll last D, pitch last D, yaw last D


/*------------------- ESC Variables-------------------*/
extern int motorSpeed;               //Base ESC speed for all Motors
extern bool motorFlag;         //Turn on motors on first iteration
extern bool elevate;          //Boolean to control motorSpeed variable
extern bool armMotors;        //Boolean to initiate arm motors method
extern bool stopMotors;       //Boolean to stop the quadcopter
#endif
