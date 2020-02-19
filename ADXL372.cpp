/**/
/*********************************************************************************/
/*********************************************************************************/
/*    @author  Geraldo Daniel Neres dos Santos                                   */
/*    @file    ADXL372.cpp                                                       */
/*    @version v1                                                                */
/*    @date    05-February-2020                                                  */
/*    @brief   SPI Implementation.                                               */
/*             This code implements the spi of the ADXL372                       */
/*             accelerometer via hardware and software.                          */
/*********************************************************************************/
/*********************************************************************************/
/**/

#include "ADXL372.h"
#include "Arduino.h"

/*!
 *   @brief  Instancia uma nova classe ADXL372 usando SPI via hardware
 *   @param  cspin
 *           Pino do Chip Select (5)
 *   @param  *theSPI
 *           Parâmetro opcional, contém o objeto SPI
 */
 
ADXL372::ADXL372(int8_t cspin, SPIClass *theSPI) {
  _cs = cspin;
  _mosi = -1;
  _miso = -1;
  _sck = -1;

  SPIinterface = theSPI;
}

/*!
 *   @brief  Instancia uma nova classe ADXL372 usando SPI via software
 *   @param  cspin
 *           Pino do Chip Select (5)
 *   @param  mosipin
 *           Pino do MOSI (23)
 *   @param  misopin
 *           Pino do MISO (19)
 *   @param  sckpin
 *           Pino do CLK (18)
 */
 
ADXL372::ADXL372(int8_t cspin, int8_t mosipin, int8_t misopin,
                                 int8_t sckpin) {
  _cs = cspin;
  _mosi = mosipin;
  _miso = misopin;
  _sck = sckpin;

}

/*!
 *  @brief  Configura o Hardware.
 *  @return Retorna true se for bem sucedido.
 */
 
bool ADXL372::begin() {

  digitalWrite(_cs, HIGH);
  pinMode(_cs, OUTPUT);

  if (_sck == -1) {
    /* SPI via Hardware */
    SPIinterface->begin();
  } else {
    /* SPI via Software */
    pinMode(_sck, OUTPUT);
    pinMode(_mosi, OUTPUT);
    pinMode(_miso, INPUT);
  }

  reset();
  Serial.println("Registros inicio: \n");
  registryData();
  Serial.println("\n\n");

  SetRegister_MEASURE();
  SetRegister_TIMING();
  SetRegister_HPF();
  SetRegister_POWER_CTL();
  Serial.println(readRegister(ADXL372_POWER_CTL), BIN);

  Serial.println("\n\n");
  Serial.println("Atualizando ADXL372_POWER_CTL: \n");
  writeRegister(ADXL372_POWER_CTL, 0x1F);

  Serial.println("\n\n");

  Serial.println("Registros depois: \n");
  registryData();
 /* Serial.println("\n\n");
  Serial.println(readRegister(ADXL372_MEASURE), BIN);
  Serial.println(readRegister(ADXL372_TIMING), BIN);
  Serial.println(readRegister(ADXL372_HPF), BIN);
  Serial.print(readRegister(ADXL372_POWER_CTL), BIN);
  Serial.println(readRegister(ADXL372_POWER_CTL), HEX);
  writeRegister(ADXL372_POWER_CTL, 0x1F);
  Serial.println(readRegister(ADXL372_POWER_CTL), HEX);*/
  SetAxisOffsets();
  Serial.println("\n\n");
  registryData();
  /*
  uint8_t data = getDeviceID();
  if(data == ADXL372_DEVID_VAL){
    Serial.println("ADXL372 encontrado!");
  }
  else
  {
    Serial.println("ADXL372 não foi encontrado!");
    return false;
  }
  */

  return true;
}

/*!
    @brief  Exibe os bytes de todos os registros em hexadecimal.
*/
void ADXL372::registryData(){

  for (uint8_t i = 0x00; i < 0x43; i++) {
    Serial.print("0x");
    Serial.print(i, HEX);
    Serial.print(" = 0x");
    Serial.println(readRegister(i), HEX);
  }

}

/*!
    @brief  Obtêm o ID do dispositivo.
    @return Valor da ID do acelerômetro.
*/
uint8_t ADXL372::GetDeviceID() {
	return (readRegister(ADXL372_DEVID));
}

/*!
    @brief  Obtêm o PartID do dispositivo.
    @return Valor dp PartID do acelerômetro.
*/
uint8_t ADXL372::GetPartID() {
	return (readRegister(ADXL372_PARTID));
}

/*!
    @brief  Obtêm o RevID do dispositivo.
    @return Valor dp RevID do acelerômetro.
*/
uint8_t ADXL372::GetRevID() {
	return (readRegister(ADXL372_REVID));
}

/*!
    @brief  Obtêm o DevID do dispositivo.
    @return Valor dp DevID do acelerômetro.
*/
uint8_t ADXL372::GetDevIDMst() {
	return (readRegister(ADXL372_DEVID_MST));
}

/*!
 *  @brief  Prepara o byte e escreve no registro.
 *     @param  reg
               Byte a ser gravado.
 */
void ADXL372::SetRegister(uint8_t address, struct ADXL372_Register *value) {
	uint8_t reg = 0;
	reg |= value->b7;
	reg = reg << 1;
	reg |= value->b6;
	reg = reg << 1;
	reg |= value->b5;
	reg = reg << 1;
	reg |= value->b4;
	reg = reg << 1;
	reg |= value->b3;
	reg = reg << 1;
	reg |= value->b2;
	reg = reg << 1;
	reg |= value->b1;
	reg = reg << 1;
	reg |= value->b0;
                                            //	reg = reg << 1;
	writeRegister(address, reg);
}

/*!
 *  @brief  Seta o POWER CONTROL REGISTER.
 */
void ADXL372::SetRegister_POWER_CTL() {
  #define ADXL372_I2C_HSM_EN 		  	0 // Velocidade do I2C
  #define ADXL372_INSTANT_ON_THRESH	0 // LOW - não usado
  #define ADXL372_FILTER_SETTLE	  	1 // 16 ms (Ideal para o HPF)
  #define ADXL372_LPF_DISABLE		  	0 // Disabilitado
  #define ADXL372_HPF_DISABLE		  	0 // Disabilitado
  #define ADXL372_MODE1			      	1
  #define ADXL372_MODE0			       	1 // Full Bandwidth

	struct ADXL372_Register reg;
	reg.b7 = ADXL372_I2C_HSM_EN;
	reg.b6 = ADXL372_RESERVED_BIT;
	reg.b5 = ADXL372_INSTANT_ON_THRESH;
	reg.b4 = ADXL372_FILTER_SETTLE;
	reg.b3 = ADXL372_LPF_DISABLE;
	reg.b2 = ADXL372_HPF_DISABLE;
	reg.b1 = ADXL372_MODE1;
	reg.b0 = ADXL372_MODE0;
	SetRegister(ADXL372_POWER_CTL, &reg);

}

/*!
 *  @brief  Seta o MEASUREMENT CONTROL REGISTER.
 */
void ADXL372::SetRegister_MEASURE() {

  #define ADXL372_USER_OR_DISABLE 0 // Overange disabilitado.
  #define ADXL372_AUTOSLEEP 	  	0 // Autosleep desabilitado.
  #define ADXL372_LINKLOOP1	    	0 // Link/Loop Activity Processing.
  #define ADXL372_LINKLOOP0	    	0 //
  #define ADXL372_LOW_NOISE	     	1 // 0 para normal noise level e 1 para ~1/3 do normal noise level.
  #define ADXL372_BANDWIDTH2	  	1 // BW está definido para 3200 Hz.
  #define ADXL372_BANDWIDTH1	  	0
  #define ADXL372_BANDWIDTH0	  	0

	struct ADXL372_Register reg;
	reg.b7 = ADXL372_USER_OR_DISABLE;
	reg.b6 = ADXL372_AUTOSLEEP;
	reg.b5 = ADXL372_LINKLOOP1;
	reg.b4 = ADXL372_LINKLOOP0;
	reg.b3 = ADXL372_LOW_NOISE;
	reg.b2 = ADXL372_BANDWIDTH2;
	reg.b1 = ADXL372_BANDWIDTH1;
	reg.b0 = ADXL372_BANDWIDTH0;
	SetRegister(ADXL372_MEASURE, &reg);

}

/*!
 *  @brief  Seta o EXTERNAL TIMING CONTROL REGISTER.
 */
void ADXL372::SetRegister_TIMING() {

  #define ADXL372_ODR2 		    	1 // ODR de 6400 Hz.
  #define ADXL372_ODR1 		    	0
  #define ADXL372_ODR0		    	0
  #define ADXL372_WAKEUP_RATE2	0 // Timer Rate para Wake-Up Mode.
  #define ADXL372_WAKEUP_RATE1	0
  #define ADXL372_WAKEUP_RATE0	0 // WAKEUP disabilitado, mas setado em 52 ms.
  #define ADXL372_EXT_CLK		  	0 // Habilita clock externo.
  #define ADXL372_EXT_SYNC	  	0 // Habilita trigger externo.

	struct ADXL372_Register reg;
	reg.b7 = ADXL372_ODR2;
	reg.b6 = ADXL372_ODR1;
	reg.b5 = ADXL372_ODR0;
	reg.b4 = ADXL372_WAKEUP_RATE2;
	reg.b3 = ADXL372_WAKEUP_RATE1;
	reg.b2 = ADXL372_WAKEUP_RATE0;
	reg.b1 = ADXL372_EXT_CLK;
	reg.b0 = ADXL372_EXT_SYNC;
	SetRegister(ADXL372_TIMING, &reg);

}

/*!
 *  @brief  Seta o HIGH-PASS FILTER SETTINGS REGISTER.
 */
void ADXL372::SetRegister_HPF() {

  /* 6 MSBs reservados */
  #define ADXL372_HPF_CORNER1 	1 /* High Pass Filter Corner 3. At ODR 6400 Hz = 3.96 Hz */
  #define ADXL372_HPF_CORNER0 	1

	struct ADXL372_Register reg;
	reg.b7 = ADXL372_RESERVED_BIT;
	reg.b6 = ADXL372_RESERVED_BIT;
	reg.b5 = ADXL372_RESERVED_BIT;
	reg.b4 = ADXL372_RESERVED_BIT;
	reg.b3 = ADXL372_RESERVED_BIT;
	reg.b2 = ADXL372_RESERVED_BIT;
	reg.b1 = ADXL372_HPF_CORNER1;
	reg.b0 = ADXL372_HPF_CORNER0;
	SetRegister(ADXL372_HPF, &reg);

}

uint8_t ADXL372::AddressToSPIData(uint8_t address, uint8_t rw_mask) {

	address = address << 1;
	return (address |= rw_mask);
}

int16_t ADXL372::ConvertFrom2Complement(uint8_t *high_part, uint8_t *low_part) {

	int16_t axis_data = 0;
	axis_data = (*high_part << 4);		// shift H part - 8MSBs to allow storing 4 LSBs
	axis_data &= 0x0FF0;			      	// make sure L part is zeroed
	*low_part = (*low_part  >> 4);		// shift L part to delete reserved bits
	axis_data |= *low_part;

	if ( axis_data & 0x800 ) { 		  	// 2s complement data, MSB indicates the sign
		axis_data = ~(axis_data);
		axis_data &= 0x0FFF;		      	// zeroing higest part
		axis_data += 1;
		axis_data = -axis_data;
	}
	return axis_data;

}

uint8_t ADXL372::ReadAccTriplet(struct ADXL372_AccelTriplet *triplet) {

	uint8_t x_data_h = 0;
	uint8_t x_data_l = 0;
	uint8_t y_data_h = 0;
	uint8_t y_data_l = 0;
	uint8_t z_data_h = 0;
	uint8_t z_data_l = 0;

		// burst transfer, auto-increment of adress
		readRegisterM(ADXL372_X_DATA_H); // ignore received data during transmit
		x_data_h = spiTransfer(0);
		x_data_l = spiTransfer(0);
		y_data_h = spiTransfer(0);
		y_data_l = spiTransfer(0);
		z_data_h = spiTransfer(0);
		z_data_l = spiTransfer(0);
		digitalWrite(_cs, HIGH);

	triplet->x = ConvertFrom2Complement(&x_data_h, &x_data_l);
	triplet->y = ConvertFrom2Complement(&y_data_h, &y_data_l);
	triplet->z = ConvertFrom2Complement(&z_data_h, &z_data_l) + ADXL372_manual_z_offset;

	return true;

}

struct ADXL372_AccelTripletG ADXL372::ConvertAccTripletToG(const struct ADXL372_AccelTriplet *triplet) {

	struct ADXL372_AccelTripletG triplet_g = {0.0, 0.0, 0.0};
	triplet_g.x = triplet->x * 100.0 / 1000.0;
	triplet_g.y = triplet->y * 100.0 / 1000.0;
	triplet_g.z = triplet->z * 100.0 / 1000.0;
	return (triplet_g);

}

void ADXL372::SetAxisOffsets() {

	// 4 MSBs in offset registers are reserved, read-only
	// description - Figure 36, p. 25

	// x axis
	writeRegister(ADXL372_OFFSET_X, 0x0E);
	// y axis
	writeRegister(ADXL372_OFFSET_Y, 0x00);
	// z axis
	writeRegister(ADXL372_OFFSET_Z, 0x07);

}

/*!
 *  @brief  Reseta o dispositivo.
 */
void ADXL372::reset(){

  writeRegister(ADXL372_RESET, ADXL372_RESET_CODE);
}

/*!
 *  @brief  SPI de baixo nível.
 *  @param  x
 *          Valor que será gravado pelo SPI.
 *  @return reply
 */
uint8_t ADXL372::spiTransfer(uint8_t x) {
  if (_sck == -1)
    return SPIinterface->transfer(x);

    /* SPI via Software */
  uint8_t reply = 0;
  for (int i = 7; i >= 0; i--) {
    reply <<= 1;
    digitalWrite(_sck, LOW);
    digitalWrite(_mosi, x & (1 << i));
    digitalWrite(_sck, HIGH);
    if (digitalRead(_miso))
      reply |= 1;
  }
  return reply;
}

/*!
    @brief  Grava 8-bits no registro designado.
    @param  reg
            Endereço do registro.
    @param  value
            Valor que será gravado no registro.
*/
void ADXL372::writeRegister(uint8_t reg, uint8_t value) {

  if (_sck == -1)
    SPIinterface->beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));

  digitalWrite(_cs, LOW);

  spiTransfer((reg << 1 ) & ~0x01); /* Desloca os bits, seta o LSB em 0 e escreve */
  spiTransfer(value);
  digitalWrite(_cs, HIGH);
  if (_sck == -1)
    SPIinterface->endTransaction(); /* Libera o barramento do SPI */

}

/*!
    @brief  Lê 8-bits do registro designado.
    @param  reg  
            Endereço do registro.
    @return Valor lido.
*/
uint8_t ADXL372::readRegister(uint8_t reg) {
  uint8_t value;

  if (_sck == -1)
    SPIinterface->beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs, LOW);
  spiTransfer((reg << 1) | 0x01); /* Desloca os bits, seta o LSB em 1 e faz a leitura*/
  value = spiTransfer(0);
  digitalWrite(_cs, HIGH);
  if (_sck == -1)
    SPIinterface->endTransaction(); // release the SPI bus

  return value;
}

uint8_t ADXL372::readRegisterM(uint8_t reg) {
  uint8_t value;

  if (_sck == -1)
    SPIinterface->beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs, LOW);
  spiTransfer((reg << 1) | 0x01); /* Desloca os bits, seta o LSB em 1 e faz a leitura*/
  //digitalWrite(_cs, HIGH);
  if (_sck == -1)
    SPIinterface->endTransaction(); // release the SPI bus

  return value;
}