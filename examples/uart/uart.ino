#include "uart.h"

uart bleUart = uart();

void setup(void)
{ 
  Serial.begin(115200);
  Serial.println(F("Arduino setup"));
  
  bleUart.begin();
}

void loop()
{
  /* Continually check for ACI events */
  bleUart.pollACI();
}
