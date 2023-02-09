// Código feito por Euler Torres para facilitar o uso da MPU6050 com algumas funções úteis, este é o C.
// Código atualizado no Github: 

#include "MPU6050.h"
#include "Wire.h"

// Funções essenciais -----------------------------------------------------

void MPU6050::resetar(uint8_t MPUnum)
{
  writeByte(MPUnum, PWR_MGMT_1, 0x80); // Set bit 7 para resetar MPU9250
  delay(100); // Espera todos os registradores resetarem
}

uint8_t MPU6050::verificarID(uint8_t MPUnum)
{
  uint8_t c = readByte(MPUnum, WHO_AM_I_MPU6050);  // Read WHO_AM_I register for MPU-9250
  return c;
}

void MPU6050::LerDadosIMU(uint8_t MPUnum, int16_t * destination)
{
  uint8_t rawData[14];  // x/y/z accel register data stored here
  readBytes(MPUnum, ACCEL_XOUT_H, 14, &rawData[0]);  // Read the 14 raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
  destination[3] = ((int16_t)rawData[6] << 8) | rawData[7] ;   
  destination[4] = ((int16_t)rawData[8] << 8) | rawData[9] ;  
  destination[5] = ((int16_t)rawData[10] << 8) | rawData[11] ;  
  destination[6] = ((int16_t)rawData[12] << 8) | rawData[13] ; 
}

void MPU6050::readAccelData(uint8_t MPUnum, int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(MPUnum, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}

void MPU6050::readGyroData(uint8_t MPUnum, int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(MPUnum, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}

void MPU6050::configura_accel_gyro(uint8_t MPUnum){
  //Configuração da MPU-6050
    
	writeByte(MPUnum, PWR_MGMT_1, 0x00); // Set bit 7 para resetar MPU9250 | Escrita no registrador PWR_MGMT_1 endereço 0x6B hexdecimal Ativar o giroscópio enviando 0x00
 
	writeByte(MPUnum, GYRO_CONFIG, 0x08); //Escrita no registrador GYRO_CONFIG endereço 0x1B hex | Envia bits 00001000 para escala de 500dps (graus por segundo)

	writeByte(MPUnum, ACCEL_CONFIG, 0x10); //Escrita no registrador ACCEL_CONFIG endereço 0x1A hex | Insere os bits 00010000 (+/- 8g full scale range)

	//Checagem dos registradores
	if(readByte(MPUnum, GYRO_CONFIG) != 0x08){
		digitalWrite(13,HIGH); 				//Liga LED de aviso
		while(1)delay(10);                                                       //Fica num loop infinito}
	}
	writeByte(MPUnum, CONFIG, 0x03); //Escrita no registrador CONFIG register (1A hex) | Insere os bits 00000011 (Filtro passa baixa ~43Hz) 
}

// Comunicação I²C com os sensores -------------------------------------------

  void MPU6050::writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

  uint8_t MPU6050::readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data = 0;                        // `data` will store the register data   
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, 1);            // Read two bytes from slave register address on MPU9250 
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}

  void MPU6050::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
  Wire.beginTransmission(address);   // Initialize the Tx buffer
  Wire.write(subAddress);            // Put slave register address in Tx buffer
  Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  Wire.requestFrom(address, count);  // Read bytes from slave register address 
  while (Wire.available()) {
        dest[i++] = Wire.read(); }         // Put read results in the Rx buffer
}
