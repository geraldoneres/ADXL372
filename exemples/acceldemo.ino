// Demonstração básica para leituras do acelerômetro ADXL372

#include <SPI.h>
#include "ADXL372.h"

/* Usado para SPI via software */
#define ADXL372_CLK  18
#define ADXL372_MISO 19
#define ADXL372_MOSI 23
/* Usado para SPI via software e hardware */
#define ADXL372_CS   5

/* SPI via software */
/* ADXL372 adxl = ADXL372(ADXL372_CS, ADXL372_MOSI, ADXL372_MISO, ADXL372_CLK); */
/* SPI via hardware */
ADXL372 adxl = ADXL372(ADXL372_CS);

struct ADXL372_AccelTriplet accel;

void setup(void) {

  Serial.begin(115200);
  adxl.begin();

  delay(1000);

}

void loop() {

  adxl.ReadAccTriplet(&accel);

  Serial.print(accel.x * 100.0 / 1000.0);
  Serial.print("\t " );
  Serial.print(accel.y * 100.0 / 1000.0);                  
  Serial.print("\t " );
  Serial.println((accel.z * 100.0 / 1000.0) - 2);

  delay(500);
}
