#ifndef led_
#define led_

#if (ARDUINO >= 100)
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

class led_class{
  public:
    led_class();
    void led_Init();
};

extern led_class led_c;


#endif
