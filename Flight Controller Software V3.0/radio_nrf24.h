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
      static void radio_Interrupt();
      void radio_Init();
      void check_radio();
};

extern radio_class rfRadio_c;

#endif
