#ifndef gyro
#define gyro

#if (ARDUINO >= 100)
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

class gyro_class{
  public:
    gyro_class();
    void imu_Init();
    void read_Imu();
    void gyro_calibrate();
    void the_Gyroscope();
    
};
extern gyro_class gyro_c;

#endif
