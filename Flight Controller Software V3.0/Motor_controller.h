#ifndef _MOTOR_GUARD_
#define _MOTOR_GUARD_

#if (ARDUINO >= 100)
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

class Motor_class{
  public:
    Motor_class();
    void motor_Init();
    void motor_attach();
    void e_Driver();
};

extern Motor_class Motor_c;

#endif
