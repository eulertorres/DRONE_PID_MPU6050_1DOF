// Código feito por Euler Torres para facilitar o uso da MPU6050 com algumas funções úteis, este é o header.
// Código atualizado no Github: 

#ifndef MPU6050_h	// poupar esforço computacional
#define MPU6050_h

#include "Arduino.h"
#include "Wire.h"

// Algumas definições úteis
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_XOUT_H     0x3B
#define GYRO_XOUT_H      0x43
#define PWR_MGMT_1       0x6B 	// Device defaults to the SLEEP mode
#define WHO_AM_I_MPU6050 0x75   // Should return 0x--

#define MPU6050_ADDRESS  0x68   // Quando AD0 está em BAIXO

class MPU6050
{
  public: 
  uint8_t verificarID(uint8_t MPUnum);
  void resetar(uint8_t MPUnum);
  void LerDadosIMU(uint8_t MPUnum, int16_t * destination);
  void readAccelData(uint8_t MPUnum, int16_t * destination);
  void readGyroData(uint8_t MPUnum, int16_t * destination);
  void configura_accel_gyro(uint8_t MPUnum);
  void    writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
  uint8_t readByte(uint8_t address, uint8_t subAddress);
  void    readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);
  private:
};

#endif