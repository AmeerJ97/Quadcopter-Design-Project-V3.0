#ifndef p3
#define p3

#if (ARDUINO >= 100)
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

class PS3_class
{
  public:
    PS3_class();
    void PS3_Init();
    void read_PS3();
    void reset_buttons();
};

extern PS3_class PS3_c;

#endif
