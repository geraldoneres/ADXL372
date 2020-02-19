# ADXL372 Accelerometer Driver
Arduino Library for ADXL372. This code implements the SPI of the ADXL372 accelerometer via hardware and software.

## About the ADXL372 ##
The ADXL372 is a digital accelerometer that supports both SPI and I2C mode and 'range' of 200g.

More information on the ADXL372 can be found in the datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/ADXL372.pdf

## Compatibility ##

   * ESP8266 
  * ESP32 
  *  ATmega328 @ 16MHz 
*    ATmega328 @ 12MHz 
 *   ATmega32u4 @ 16MHz 
 *   ATmega2560 @ 16MHz 
  *  ATSAM3X8E 
  *  ATSAM21D
 *   ATtiny85 @ 16MHz 
  *  ATtiny85 @ 8MHz 
* STM32
* Intel Curie @ 32MHz

MCU                | Tested Works | Doesn't Work | Not Tested  | Notes
------------------ | :----------: | :----------: | :---------: | -----
ESP32              |      X       |             |            | 
ESP8266            |      X       |             |            | 
Atmega328 @ 16MHz  |      X       |             |            | 
Atmega328 @ 12MHz  |      X       |             |            | 
Atmega32u4 @ 16MHz |      X       |             |            | 
Atmega32u4 @ 8MHz  |      X       |             |            | 
Atmega2560 @ 16MHz |      X       |             |            | 
ATSAM3X8E          |      X       |             |            | 
ATSAM21D           |      X       |             |            | 
ATtiny85 @ 16MHz*   |             |      X       |            | Change the SPI to 2MHz or less
ATtiny85 @ 8MHz*    |             |      X       |            | Change the SPI to 2MHz or less
Intel Curie @ 32MHz |      X       |             |            | 
STM32              |       x      |             |            | 
