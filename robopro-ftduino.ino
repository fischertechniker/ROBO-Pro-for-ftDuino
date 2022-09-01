/*
 * ROBO Pro für den ftDuino v1.0
 * Parser für das fish.X1-Protokoll des TX Controllers
 * (TX Firmware 1.30)
 * 
 * Einschränkungen:
 * - Ultraschall-Abstandssensor nicht unterstützt
 * - keine BT-Unterstützung (work in progress)
 * - noch keine I2C-Unterstützung (work in progress)
 * - keine Synchronisation von Motoren
 * - keine Kaskadierung von ftDuinos ("Extensions") möglich
 * 
 * Bibliotheken:
 * - ftduino.h
 * - wire.h
 * 
 * Dirk Fox
 * 
 * - 02.09.2022: publication of v1.0
 * - 27.07.2022: Fork of fx1sample 0.3 by ft-ninja
 */

#include <Ftduino.h>
#include <Wire.h> 
#include "fishx1.h"

void setup()
{
  Serial.begin(TX_BAUD);  // ROBO TX Controller Serial UART (38400,8,n,1)
  ftduino.init();         // initialize ftDuino API
  Wire.begin();           // initialize I2C driver, join I2C bus as master (controller)
}

void loop()
{
  if (Serial.available() > 0) // check UART and read one symbol
    fx1Parse(Serial.read());  // parse Fish.X1 protocol
}
