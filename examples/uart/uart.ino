#include <SPI.h>
#include <avr/pgmspace.h>
#include <ble_system.h>
#include <lib_aci.h>
#include <aci_setup.h>

#include "uart/services.h"
#include "UartService.h"

UartService uart = UartService();

void setup(void)
{ 
  Serial.begin(115200);
  Serial.println(F("Arduino setup"));
  
  uart.begin();
}

void loop()
{
  /* Continually check for ACI events */
  uart.pollACI();
}