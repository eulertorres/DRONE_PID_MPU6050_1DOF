// Código feito por Euler Torres para o uso de uma bancada de teste com MPU6050.
// A bancada utiliza de dois ESCs, dois motores brushless e um Arduino UNO para estabilizar uma haste de madeira com 1 grau de liberdade com controlador PID.
// Código atualizado no Github: 

//-------------------------------------------------------------------------------------
//									Termos de uso									  |
//ESTE SOFTWARE FOI DISTRIBUÍDO EM SEU ESTADO ATUAL, SEM NENHUMA GARANTIA, DE FORMA	  |
//IMPLÍCITA OU EXPLÍCITA. DESIGNADO PARA USO PESSOAL.								  |
//OS AUTORES NÃO SÃO RESPONSÁVEIS POR QUALQUER DANO CAUSADO PELO USO DESTE SOFTWARE.  |
//-------------------------------------------------------------------------------------
//								NOTA DE SEGURANÇA									  |																
//SEMPRE REMOVA AS HÉLICES ANTES DE REALIZAR QUALQUER TESTE,						  |
//A MENOS QUE VOCÊ SAIBA O QUE ESTÁ FAZENDO.										  |
//-------------------------------------------------------------------------------------

#define MatlabPlot true

#include <Wire.h>                          //Biblioteca para comunicação I²C com a MPU6050
#include "MPU6050.h"					   // Arquivo com uns comandinhos úteis

MPU6050 IMU;			// Instancia objeto

int16_t dados_brutos[7];		// Armazena Ax | Ay | Az | Temp | Gx | Gy | Gz

float   aRes = 8.0/32768.0, gRes = 500.0/32768.0;  // Escala A = 8G e G = 500gps
float 	ax,ay,az, gx=0, gy=0, gz=0;
unsigned long millisTime, setupTime;		// For printing the time of the data aquisition (matlab only)

void setup(){
	Serial.begin(9600);
	delay(1000);
	Wire.begin();                                                             //Comunicação I²C como Mestre
	Wire.setClock(400000); 			// I2C frequency at 400 kHz
	delay(1000);
	
	IMU.configura_accel_gyro(MPU6050_ADDRESS);
	
	Serial.print("Valor retornardo pelo registrador WHO_AM_I: "); Serial.println(IMU.verificarID(MPU6050_ADDRESS));
  delay(8000);
	setupTime = millis();
}

void loop(){
	IMU.readAccelData(MPU6050_ADDRESS, dados_brutos); 		// Read the first MPU data
	
	ax = dados_brutos[0]*aRes; ay = dados_brutos[1]*aRes; az = dados_brutos[2]*aRes; 
	
  millisTime = millis() - setupTime;

  if(!MatlabPlot){
	  Serial.print("Dados brutos accelerometro: A_x =\t"); Serial.print(ax); Serial.print("\t A_y =\t"); Serial.print(ay); Serial.print("\t A_z =\t"); Serial.println(az);
  } else{
                          Serial.print( ax, 2);
      Serial.print("\t"); Serial.print( ay, 2); 
      Serial.print("\t"); Serial.print( az, 2);
      Serial.print("\t"); Serial.print( gx, 2); 
      Serial.print("\t"); Serial.print( gy, 2); 
      Serial.print("\t"); Serial.print( gz, 2);
      Serial.print("\t");Serial.println(millisTime);
  }
	delay(50);
}
