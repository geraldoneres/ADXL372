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
ESP32              |      X       |             |            | Supports 10MHz SPI
ESP8266            |      X       |             |            | Supports 10MHz SPI
Atmega328 @ 16MHz  |      X       |             |            | 
Atmega328 @ 12MHz  |      X       |             |            | 
Atmega32u4 @ 16MHz |      X       |             |            | 
Atmega32u4 @ 8MHz  |      X       |             |            | 
Atmega2560 @ 16MHz |              |             |      x     | 
ATSAM3X8E          |              |             |      x     | 
ATSAM21D           |      X       |             |            | 
ATtiny85 @ 16MHz*   |             |             |       x     | Change the SPI to 2MHz or less
ATtiny85 @ 8MHz*    |      x       |             |            | Change the SPI to 2MHz or less
Intel Curie @ 32MHz |      X       |             |            | Supports 10MHz SPI
STM32              |       x      |             |            | Supports 10MHz SPI
