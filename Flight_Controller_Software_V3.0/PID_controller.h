#ifndef _PID_GUARD_
#define _PID_GUARD_

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class PID_class {
  public:
    PID_class();
    void PID_Init();
    void PID_Controller();
    void Init_PID();
    void reset_Integral();
};

extern PID_class PID_c;

#endif
