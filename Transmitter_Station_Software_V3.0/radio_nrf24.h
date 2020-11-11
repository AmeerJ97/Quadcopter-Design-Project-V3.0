#ifndef rf
#define rf

#if (ARDUINO >= 100)
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

class radio_class{
  public :
      radio_class();
      void radio_Init();
      void radio_tx();
};

extern radio_class rfRadio_c;

#endif
