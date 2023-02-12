// Código feito por Euler Torres para o uso de uma bancada de teste com MPU6050.
// A bancada utiliza de dois ESCs, dois motores brushless e um Arduino UNO para estabilizar uma haste de madeira com 1 grau de liberdade com controlador PID.
// Código atualizado no Github: 

//			|-------------------------------------------------------------------------------------
//			|									                Termos de uso                                       |
//			|ESTE SOFTWARE FOI DISTRIBUÍDO EM SEU ESTADO ATUAL, SEM NENHUMA GARANTIA, DE FORMA    |
//			|IMPLÍCITA OU EXPLÍCITA. DESIGNADO PARA USO PESSOAL.                                  |
//			|OS AUTORES NÃO SÃO RESPONSÁVEIS POR QUALQUER DANO CAUSADO PELO USO DESTE SOFTWARE.   |
//			|-------------------------------------------------------------------------------------
//			|								                  NOTA DE SEGURANÇA                                   |																
//			|             SEMPRE REMOVA AS HÉLICES ANTES DE REALIZAR QUALQUER TESTE,              |
//			|             A MENOS QUE VOCÊ SAIBA O QUE ESTÁ FAZENDO.                              |
//			|-------------------------------------------------------------------------------------

#define LED 12									// LED para debug

#define MatlabPlot		true					// Habilita plotagem de dados no Matlab (Desativa o Serialgraph)
#define Serialgraph		false					// Vizualização de ângulo no gráfico serial [Precisa colocar a taxa de atualização para 1 Hz (modificar Delta)]
#define Calib_receptor	false					// Habilita calibração do receptor (gravado memória EEPROM)
#define PID_ativado		false					// Desabilita a plotagem e abilita o uso na bancada
#define Accel_calib		false					// Calibração única do acelerõmetro (~1 min)			
#define auto_level    true          			// NÃO DESABILITA ISSO A MENOS QUE VOCÊ SEJA PILOTO LEVEL 89!!!!!!!!!!!

#include <Wire.h>							// Biblioteca para comunicação I²C com a MPU6050
#include "MPU6050.h"						// Arquivo com uns comandinhos úteis
#include <EEPROM.h>							//Biblioteca resposável por armazenar informações na memória do Arduino

MPU6050 IMU;								// Instancia objeto
int16_t dados_brutos[7];					// Armazena acc_x | acc_y | acc_z | Temperatura | gyro_roll | gyro_pitch | gyro_yaw

//Ajuste de ganho do PID e Limites. É necessário ser feito manualmente
//-----------------------------(Específico para o drone KRONOS MK I)--------------------------------		  
float pid_p_gain_roll = 1.0;               // Ganho da parte P (Proporcional) do Roll  [Valor do Kronus v1.0 = 1.0]
float pid_i_gain_roll = 0.04;              // Ganho da parte I (Integrador) do Roll   [Valor do Kronus v1.0 = 0.04]
float pid_d_gain_roll = 16;                // Ganho da parte D (Derivativo) do Roll   [Valor do Kronus v1.0 = 16]
int pid_max_roll = 200;                    // Saída máxima e mínima do PID para o roll

//float pid_p_gain_pitch = 1.0;   // Ganho da parte P (Proporcional) do Pitch			[Valor do Kronus v1.0 = 1.0]
//float pid_i_gain_pitch = 0.04;  // Ganho da parte I (Integrador) do Pitch           [Valor do Kronus v1.0 = 0.04]
//float pid_d_gain_pitch = 16;    // Ganho da parte D (Derivativo) do Pitch           [Valor do Kronus v1.0 = 16]
//int pid_max_pitch = 200;          // Saída máxima e mínima do PID para o pitch

// ---------------------Variáveis Locais---------------------------------------------------------------------------------------------------------------------------------------------------------
float   aRes = 8.0/32768.0, gRes = 500.0/32768.0;	// Escala A = 8G e G = 500gps
float 	acc_x=0, acc_y=0, acc_z=0, gyro_roll=0, gyro_pitch=0, gyro_yaw=0;					// Variáveis para dados convertidos
unsigned long millisTime, setupTime;				// Para aquisição de dados e controle da frequência
unsigned long loop_timer, esc_loop_timer, button_timer, delta;					// Variáveis controle frequência ESC
byte last_channel_1, last_channel_2, last_channel_3;									// Nível do sinal anterior do canal
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3;		// Dados escalonados receptor
int esc_1, esc_2;														// Valor final enviado aos ESCs para controle dos motores
int throttle;															// Valor do throttle para controle de altitude
float battery_voltage;													// Valor da bateria adquirido pelo divisor de tensão
int cal_int, start;														// Controle do código
volatile int receiver_input[5];													// Dados brutos do receptor
int temperature;														// Temperatura da IMU [Não utilizado]
float roll_level_adjust, pitch_level_adjust;							// Ajuste fino do Roll e Pitch
long acc_total_vector;													// Vetor da aceleração resultante (Trigonometria)
unsigned long timer_channel_1, timer_channel_2,	timer;					// Controle largura de pulso
unsigned long timer_1, timer_2, timer_3, current_time;					// Controle largura de pulso na interrupção
double gyro_axis_cal[4], accel_axis_cal[2],	accel_offset[3], Ganho;		// Calibração da IMU Média simples, calculo offset
//----------------------------- Varíaveis para o Sistemas de controle ----------------------------------------------------------------------------------------------------------------------
float pid_error_temp;                          // Varíavel temporário utilizada no cálculo do PID
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;    // Roll
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;  // Pitch
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;      // Yaw
float angle_roll_acc, angle_pitch_acc, angle_yaw_acc, angle_pitch, angle_roll, angle_yaw;				// Angulos estimados acelerômetro^
//----------------------------- Varíaveis para calibração Receptor ----------------------------------------------------------------------------------------------------------------------------
int center_channel_1, center_channel_2, center_channel_3;
int high_channel_1, high_channel_2, high_channel_3;
int low_channel_1, low_channel_2, low_channel_3, low_channel_4;
volatile uint8_t invertido = 0;

void setup(){
	Serial.begin(9600);      // Conexão serial com 57600 bps
	delay(1000);
	Wire.begin();					// Comunicação I²C como Mestre
	Wire.setClock(400000);			// Frequência I2C de 400 kHz
	delay(1000);

  // Valores de offset adquirido na calibração:
  accel_offset[0] = 0; accel_offset[1] = 0; accel_offset[2] = 0;

	
	invertido = EEPROM.read(55);	// Verifica se os canais estão invertidos 
	
	// Saidas
	DDRD |= B11000000;             	// Porta 6, 7 output - ESCs
	DDRB |= B00010000;				// Porta 12 Output - LED de alerta	
	// Entradas
	// Portas receptor: 8, 9 e 10 (ARM(ch.5), Pitch(ch.2) e Throttle(ch.3))
	
	IMU.configura_accel_gyro(MPU6050_ADDRESS);	// Ativa IMU e configura registradores
	
	Serial.print("Valor retornardo pelo registrador WHO_AM_I: "); Serial.println(IMU.verificarID(MPU6050_ADDRESS));
	delay(1000);
		
	/////////////////////////////////////////	Calibração do Acelerômetro (apenas uma vez) /////////////////////////////////////////////////////////////////////////////////////
	//	Aquisição de dados aravés de um sistema de equação Saida = Ganho x Vref + offset	. No final serão achados o ganho e o offset (usado para compensar o erro)
	// S1 = G x 1 + Of	->  Of = S1 - G
	// S2 = G x -1 + Of	->	S2 = -G + S1 -G
	// G = (S1 - S2)/2	| Of = S1 - G
	if(Accel_calib) {
		Serial.println(F("================== Calibragem Acelerômetro ================="));
		Serial.println(F("Coloque o acelerômetro na posição X = 1g em 20 segundos"));
		espera_20seg();
		Serial.println(F(" "));
		// Eq. 1 Eixo X
		for (cal_int = 0; cal_int < 2000 ; cal_int ++){                          	//	Realiza a leitura de 2000 amostras do giroscópio
			if(cal_int % 15 == 0)digitalWrite(LED, !digitalRead(LED));                //	Led pisca para indicar calibração a cada segundo
			IMU.readAccelData(MPU6050_ADDRESS, dados_brutos);						//	Lê os primeiros dados do acelerômetro
			acc_x = dados_brutos[0];											// Converção dos dados
			accel_axis_cal[0] += acc_x;												//Acumula o resultado
		}
		Serial.println(F("Coloque o acelerômetro na posição X = -1g em 10 segundos"));
		espera_20seg();
		Serial.println(F(" "));
		// Eq. 2 Eixo X
		for (cal_int = 0; cal_int < 2000 ; cal_int ++){                          	// Realiza a leitura de 2000 amostras do giroscópio
			if(cal_int % 15 == 0)digitalWrite(LED, !digitalRead(LED));              // Led pisca para indicar calibração a cada segundo
			IMU.readAccelData(MPU6050_ADDRESS, dados_brutos);						// Lê os primeiros dados do acelerômetro
			acc_x = dados_brutos[0];											// Converção dos dados
			accel_axis_cal[1] += acc_x;												// Acumula o resultado
		}
		accel_axis_cal[0] /= 2000; accel_axis_cal[1] /= 2000;                       // Divide as saidas 1 e 2 por 2000
		Ganho = (accel_axis_cal[0] - accel_axis_cal[1])/2;
		accel_offset[0] = accel_axis_cal[0] - Ganho;
		Serial.print(F("Ganho e Offset: eixo X: ")); Serial.print(Ganho);Serial.print(F(" | ")); Serial.println(accel_offset[0]);
		
		// Eixo Y -------------------------------------------------------------------------------------------------------------------------
		Serial.println(F("Coloque o acelerômetro na posição Y = 1g em 20 segundos"));
		espera_20seg();
		Serial.println(F(" "));
		// Eq. 1 Eixo Y
		for (cal_int = 0; cal_int < 2000 ; cal_int ++){                          	//	Realiza a leitura de 2000 amostras do giroscópio
			if(cal_int % 15 == 0)digitalWrite(LED, !digitalRead(LED));                //	Led pisca para indicar calibração a cada segundo
			IMU.readAccelData(MPU6050_ADDRESS, dados_brutos);						//	Lê os primeiros dados do acelerômetro
			acc_y = dados_brutos[1];											// Converção dos dados
			accel_axis_cal[0] += acc_y;												//Acumula o resultado
		}
		Serial.println(F("Coloque o acelerômetro na posição Y = -1g em 20 segundos"));
		espera_20seg();
		Serial.println(F(" "));
		// Eq. 2 Eixo Y
		for (cal_int = 0; cal_int < 2000 ; cal_int ++){                          	// Realiza a leitura de 2000 amostras do giroscópio
			if(cal_int % 15 == 0)digitalWrite(LED, !digitalRead(LED));              // Led pisca para indicar calibração a cada segundo
			IMU.readAccelData(MPU6050_ADDRESS, dados_brutos);						// Lê os primeiros dados do acelerômetro
			acc_y = dados_brutos[1];											// Converção dos dados
			accel_axis_cal[1] += acc_y;												// Acumula o resultado
		}
		accel_axis_cal[0] /= 2000; accel_axis_cal[1] /= 2000;                       // Divide as saidas 1 e 2 por 2000
		Ganho = (accel_axis_cal[0] - accel_axis_cal[1])/2;
		accel_offset[1] = accel_axis_cal[0] - Ganho;
		Serial.print(F("Ganho e Offset eixo Y: ")); Serial.print(Ganho);Serial.print(F(" | ")); Serial.println(accel_offset[1]);
		
		// Eixo Z -------------------------------------------------------------------------------------------------------------------------
		Serial.println(F("Coloque o acelerômetro na posição Z = 1g em 20 segundos"));
		espera_20seg();
		Serial.println(F(" "));
		// Eq. 1 Eixo Z
		for (cal_int = 0; cal_int < 2000 ; cal_int ++){                          	//	Realiza a leitura de 2000 amostras do giroscópio
			if(cal_int % 15 == 0)digitalWrite(LED, !digitalRead(LED));                //	Led pisca para indicar calibração a cada segundo
			IMU.readAccelData(MPU6050_ADDRESS, dados_brutos);						//	Lê os primeiros dados do acelerômetro
			acc_z = dados_brutos[2];											// Converção dos dados
			accel_axis_cal[0] += acc_z;												//Acumula o resultado
		}
		Serial.println(F("Coloque o acelerômetro na posição z = -1g em 20 segundos"));
		espera_20seg();
		Serial.println(F(" "));
		// Eq. 2 Eixo Z
		for (cal_int = 0; cal_int < 2000 ; cal_int ++){                          	// Realiza a leitura de 2000 amostras do giroscópio
			if(cal_int % 15 == 0)digitalWrite(LED, !digitalRead(LED));              // Led pisca para indicar calibração a cada segundo
			IMU.readAccelData(MPU6050_ADDRESS, dados_brutos);						// Lê os primeiros dados do acelerômetro
			acc_z = dados_brutos[2];											// Converção dos dados
			accel_axis_cal[1] += acc_z;												// Acumula o resultado
		}
		accel_axis_cal[0] /= 2000; accel_axis_cal[1] /= 2000;                       // Divide as saidas 1 e 2 por 2000
		Ganho = (accel_axis_cal[0] - accel_axis_cal[1])/2;
		accel_offset[2] = accel_axis_cal[0] - Ganho;
		Serial.print(F("Ganho e Offset eixo Z: ")); Serial.print(Ganho);Serial.print(F(" | ")); Serial.println(accel_offset[2]);
		
		Serial.println(F("================== Calibragem finalizada ================="));
		Serial.println(F(" Salve os valores de offset e mude a flag Accel_calib"));
   Serial.print(F("accel_offset[0] = ")); Serial.print(accel_offset[0]); Serial.print(F("; accel_offset[1] = ")); Serial.print(accel_offset[1]); Serial.print(F("; accel_offset[2] = ")); Serial.print(accel_offset[2]); Serial.println(F(";"));
		while(1)delay(1000);
	}
	
	  // Calcula a média do erro do giroscópio a partir de 2000 amostras
	for (cal_int = 0; cal_int < 2000 ; cal_int ++){                           // Realiza a leitura de 2000 amostras do giroscópio
		if(cal_int % 10 == 0)digitalWrite(LED, !digitalRead(LED));                // Led pisca para indicar calibração
		IMU.readGyroData(MPU6050_ADDRESS, dados_brutos);						//	Lê os primeiros dados do acelerômetro
		gyro_roll = dados_brutos[0]; gyro_pitch = dados_brutos[1]; gyro_yaw = dados_brutos[2];	// Converção dos dados
		gyro_axis_cal[1] += gyro_roll;												//Acumula o resultado
		gyro_axis_cal[2] += gyro_pitch;												//Acumula o resultado
		gyro_axis_cal[3] += gyro_yaw;												//Acumula o resultado
    // Envia pulsos de 1500 us para parar o motor [APENAS EM ESCs BIDIRECIONAIS ----CUIDADO-------]
    PORTD |= B11000000;                                                     // Porta 6 e 7 em nivel ALTO
    delayMicroseconds(1500);                                                // Espera 1.5 milisegundos (1500us). (Desliga o motor)
    PORTD &= B00111111;                                                     // Porta 6 e 7 nivel BAIXO
    delayMicroseconds(2500);                                                // Espera 2.5 ms [completa o período de 4 ms]
   //Serial.println(cal_int);
	}
	// Adquirimos a média simples das 2000 amostras para estimar o erro do giroscópio
	gyro_axis_cal[1] /= 2000;                                                 // Divide o roll  por 2000
	gyro_axis_cal[3] /= 2000;                                                 // Divide o yaw   por 2000
	gyro_axis_cal[2] /= 2000;                                                 // Divide o pitch por 2000
	
	
	// Depois da calibração da IMU, habilitar a interrupção do micro
	PCICR |= (1 << PCIE0);                                                    // Registeador PCIE0  em alto para habilitar scaneamento da interrupção através do PCMSK0
	PCMSK0 |= (1 << PCINT0);                                                  // Registrador PCINT0 em alto (entrada digital 8)  para causar uma interrupção em qualquer mudança.
	PCMSK0 |= (1 << PCINT1);                                                  // Registrador PCINT0 em alto (entrada digital 9)  para causar uma interrupção em qualquer mudança.
	PCMSK0 |= (1 << PCINT2);                                                  // Registrador PCINT1 em alto (entrada digital 10 )  para causar uma interrupção em qualquer mudança.
	
	if(Calib_receptor){
		Serial.println(F(""));
		Serial.println(F("==================================================="));
		Serial.println(F("			Checagem do sistema"));
		Serial.println(F("==================================================="));
		delay(300);
		Serial.println(F(""));
		Serial.println(F("==================================================="));
		Serial.println(F("			  Configuração do transmissor RC"));
		Serial.println(F("==================================================="));
		delay(1000);
		Serial.println(F(""));
		
		Serial.println(F("Coloque os manches na posição central do controle nos próximos 10 segundos"));
		espera_10seg();
		Serial.println(" ");
		// Define a posição de CENTRO para este controle remoto
		center_channel_1 = 1500;  // Chave 2 posições apenas
		center_channel_2 = receiver_input[2];
		center_channel_3 = receiver_input[3];
		Serial.println(F(""));
		Serial.println(F("Posições de CENTRO armazenadas"));
		Serial.print(F("GPIO 08 = "));
		Serial.println(center_channel_1);
		Serial.print(F("GPIO 09 = "));
		Serial.println(center_channel_2);
		Serial.print(F("GPIO 10 = "));
		Serial.println(center_channel_3);
		Serial.println(F(""));
		Serial.println(F(""));

		// Verifica inversão dos canais
		Serial.println(F("Movimente o manche do throttle para a maior posição, depois para o centro"));
		checa_inversao(3);
		Serial.println(F("Movimente o manche do pitch para a maior posição, depois para o centro"));
		checa_inversao(2);
		Serial.println(F("Movimente a chave do ARM para a maior posição, depois para o centro"));
		checa_inversao(1);
		
		// Registra os valores mínimos e máximos do receptor
    Serial.println(F("Mova lentamente os manches para cada extremidade"));
    Serial.println(F("Quando estiver pronto, coloque todos no centro novamente"));    
		registra_min_max();
		Serial.println(F(""));
		Serial.println(F(""));
		Serial.println(F("Valores finais de ALTO, CENTRO, BAIXO detectados na configuração"));
		Serial.print(F("GPIO 08:"));
		Serial.print(low_channel_1);
		Serial.print(F(" - "));
		Serial.print(center_channel_1);
		Serial.print(F(" - "));
		Serial.println(high_channel_1);
		Serial.print(F("GPIO 09:"));
		Serial.print(low_channel_2);
		Serial.print(F(" - "));
		Serial.print(center_channel_2);
		Serial.print(F(" - "));
		Serial.println(high_channel_2);
		Serial.print(F("GPIO 10:"));
		Serial.print(low_channel_3);
		Serial.print(F(" - "));
		Serial.print(center_channel_3);
		Serial.print(F(" - "));
		Serial.println(high_channel_3);

		//Registra valores no EEPROM, para nao precisar calibrar novamente
		Serial.println(F(""));
		Serial.println(F("==================================================="));
		Serial.println(F("			Registrando no EEPROM				"));
		Serial.println(F("==================================================="));
		Serial.println(F(""));

		EEPROM.write(36, center_channel_1 >> 8);			// Parte alta	
		EEPROM.write(37, center_channel_2 >> 8);			// Parte alta
		EEPROM.write(38, center_channel_3 >> 8);			// Parte alta
		EEPROM.write(39, center_channel_1 & 0b11111111);	// Parte baixa
		EEPROM.write(40, center_channel_2 & 0b11111111);	// Parte baixa  
		EEPROM.write(41, center_channel_3 & 0b11111111);	// Parte baixa
		EEPROM.write(42, high_channel_1 >> 8);				// Parte alta	
		EEPROM.write(43, high_channel_2 >> 8);              // Parte alta
		EEPROM.write(44, high_channel_3 >> 8);              // Parte alta
		EEPROM.write(45, high_channel_1 & 0b11111111);		// Parte baixa
		EEPROM.write(46, high_channel_2 & 0b11111111);      // Parte baixa
		EEPROM.write(47, high_channel_3 & 0b11111111);      // Parte baixa
		EEPROM.write(48, low_channel_1 >> 8);				// Parte alta
		EEPROM.write(49, low_channel_2 >> 8);               // Parte alta
		EEPROM.write(50, low_channel_3 >> 8);               // Parte alta
		EEPROM.write(51, low_channel_1 & 0b11111111);       // Parte baixa
		EEPROM.write(52, low_channel_2 & 0b11111111);       // Parte baixa
		EEPROM.write(53, low_channel_3 & 0b11111111);       // Parte baixa
		EEPROM.write(55, invertido);       					// Canais invertidos		
		Serial.println(F(""));
		Serial.println(F("===================================================="));
		Serial.println(F("Calibração finalizada, mude a flag	Calib_receptor"));
		Serial.println(F("===================================================="));
		Serial.println(F(""));
    for(int k=1; k<4; k++){
      int centro, baixo, alto;
      centro = (EEPROM.read(35+k) << 8) | EEPROM.read(38+k);
      baixo = (EEPROM.read(47+k) << 8) | EEPROM.read(50+k);
      alto = ((EEPROM.read(41+k) << 8) | EEPROM.read(44+k));
      Serial.print(F("Centro canal: ")); Serial.print(k); Serial.print(F(" = ")); Serial.print(centro);
      Serial.print(F(" Low = ")); Serial.print(baixo);
      Serial.print(F(" HIGH = ")); Serial.println(alto);
    }
		while(1)delay(1000);								// Trava o programa
	}

  receiver_input_channel_3 = convert_receiver_channel(3);                 // Escalonamento para a faixa de 1500 a 2000 us (throtle)
  receiver_input_channel_1 = 1500;                                        // Não há interrupção do ARM até esse momento
  
	//Espera o throttle estar na menor posição (valor bruto da interrupção) e ARM estiver ligado (posição para baixo)
	while(receiver_input_channel_3 > 1525 || receiver_input_channel_1 < 1700){
		receiver_input_channel_3 = convert_receiver_channel(3);                 // Escalonamento para a faixa de 1500 a 2000 us (throtle)
		receiver_input_channel_1 = convert_receiver_channel(1);                 // Escalonamento para a faixa de 1500 a 2000 us (ARM)
		start ++;                                                               // Incrementa o contador para piscar a LED a cada 500 ms
		//Envia pulsos de 1500 us aos ESCs bidirecionais (para rotação)            
		PORTD |= B11000000;                                                     // Porta 4, 5 em nivel ALTO
		delayMicroseconds(1500);                                                // Espera 1.5 milisegundos (1500us) Para desligar os motores
		PORTD &= B00111111;                                                     // Porta 4, 5 nivel BAIXO
		delayMicroseconds(2500);												//Espera 2.5 ms
		if(start == 125){                                                       // A cada 500 ms
			digitalWrite(LED, !digitalRead(LED));                                 // O led fica piscando
			start = 0;                                                          // Reseta o contador
		}
   Serial.print(F(" Throttle: ")); Serial.print(receiver_input_channel_3); Serial.print(F(" ARM: ")); Serial.println(receiver_input_channel_1);
	}                                                                            
	start = 0;

	//Cálculo da tensão atual da bateria 4S (importante) [Divisor de tensão R1 = 5.1K e R2 = 1K]
	//O diodo apresenta queda de tensão equivalente de 0.8V [49 (decimal)]
	//A bateria cheia (16,8V) deve mostrar na porta analógica 5V (valor máximo da porta analógica do Arduino)
	//16,8V deve ser lido como 1023 na porta analógica A0
	//16,8 / 1023 = 0,01642V/b. Cada valor incrementado [em bits] representa 0,016422 V da bateria (Resolução)
	//Exemplo: Caso seja lido 787 na porta A0, o valor de tensão da bateria será 12.92 V
	battery_voltage = (analogRead(0) + 49) * 0.01642;

	digitalWrite(LED,LOW);									//Desliga a LED
	setupTime = millis();
	loop_timer = micros();									//Set the timer for the next loop.
}

void loop(){
	if(PID_ativado){
    //Serial.print(receiver_input_channel_3); Serial.print(F(" ARM: ")); Serial.println(receiver_input_channel_1);
		// Cada 65.5 entregue pelo giroscópio corresponde a 1 grau/seg
		gyro_roll_input = (gyro_roll_input * 0.7) + ((gyro_roll / 65.5) * 0.3);   //Entrada do pid em graus por segundo [Não utilizado nessa bancada]
		//gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro_pitch / 65.5) * 0.3);	//Entrada do pid em graus por segundo
		//gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyro_yaw / 65.5) * 0.3);      //Entrada do pid em graus por segundo [Não utilizado nessa bancada]

		// Cáluclos Estimativa de atitude, ângulos de Euler
		// Giroscópio
		//0.0000611 = 1 / (250Hz / 65.5)
		//angle_pitch += gyro_pitch * 0.0000611;                                    // Integração simples para transformar graus/segundo em graus a partir do período 1/250hz (Pitch)
		angle_roll += gyro_roll * 0.0000611;                                      // Integração simples para transformar graus/segundo em graus a partir do período 1/250hz (Roll) [Não utilizado nessa bancada]

		//0.000001066 = 0.0000611 * (3.142(PI) / 180degr) A função seno no arduino é em radianos		
		//angle_pitch -= angle_roll * sin(gyro_yaw * 0.000001066);                  // Compensa a rotação da IMU (Yaw) transferindo o angulo roll para o pitch [Não utilizado nessa bancada]
		//angle_roll += angle_pitch * sin(gyro_yaw * 0.000001066);                  // Compensa a rotação da IMU (Yaw) transferindo o angulo pitch para o roll [Não utilizado nessa bancada]
																				   
		// Acelerômetro                                           
		acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));       // Vetor resultante da acleração (raiz quadrada da soma dos quadrados)
																				   
		//if(abs(acc_y) < acc_total_vector){                                        // Apenas calcula se |acc_y| < vetor resultante.
		//angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;				// ângulo do pitch baseado na aceleração, convertido de rad para graus
		//}                                                                            
		if(abs(acc_x) < acc_total_vector){                                        // Apenas calcula se |acc_x| < vetor resultante.
		angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;          // Ângulo do roll baseado na aceleração, convertido de rad para graus
		}
		
		//Fusão sensorial
		//angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;            // Filtro complementar para correção do acumulo de erro no pitch
		angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;               // Filtro complementar para correção do acumulo de erro no roll

		//pitch_level_adjust = angle_pitch * 7.5;                                    // Correção do ângulo do erro do pitch
		roll_level_adjust = angle_roll * 7.5;                                      // Correção do ângulo do erro do roll

		if(!auto_level){                                                          /////// AUTO BALANCEAMENTO (PRECISA SER PILOTO LEVEL 98 PARA DESABILITAR ISSO)
		pitch_level_adjust = 0;                                                 // Pitch não será ajustado caso auto_level esteja em falso
		roll_level_adjust = 0;                                                  // Roll  não será ajustado caso auto_level esteja em falso
		}

		//Converção dos valores do receptor e ajuste fino da calibração
		receiver_input_channel_1 = convert_receiver_channel(1);	// ARM
		receiver_input_channel_2 = convert_receiver_channel(2);	// Roll
		receiver_input_channel_3 = convert_receiver_channel(3);	// Throttle
		
		//			PROCESSO PARA ARMAR E DESARMAR O DRONE          \\
		
		//Para iniciar os motores: Coloque o throttle na menor posição e ARM para baixo
		if(receiver_input_channel_3 < 1525 && receiver_input_channel_1 < 1525){
			start = 2;
			//angle_pitch = angle_pitch_acc;                                          // Ângulo inicial do pitch será do acelerômetro (gravidade como referência)
			angle_roll = angle_roll_acc;                                            // Ângulo inicial do roll  será do acelerômetro (gravidade como referência)

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			////////////////////////////////////////////////////////////////	BANCADA ARMADA   /////////////////////////////////////////////////////////////////////////////////
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //Reseta os dados do sistema PID, para uma decolagem suave. (Apaga a memória do último voo)
      pid_i_mem_roll = 0;
      pid_last_roll_d_error = 0;
      //pid_i_mem_pitch = 0;
      //pid_last_pitch_d_error = 0;
      //pid_i_mem_yaw = 0;
      //pid_last_yaw_d_error = 0;
		}

		//Para parar os motores: Throttle na menor posição e ARM para cima
		if(start == 2 && receiver_input_channel_1 > 1900)start = 0;
		
		
		//O ponto de referência do PID é em graus por segundo. O roll será a entrada deste sistema-------------------
		//Para converter a largura de pulso para graus por segundo, iremos dividir por 1.5. O ângulo maximo será então (250-4)/1.5 = 164 [graus/s]
		 pid_roll_setpoint = 0;
		//O ponto de referência só é definido quando o canal é +/- 8 us diferente do CENTRO [Evitar flutuaçoes afetarem o PID]
		if(receiver_input_channel_1 > 1754)pid_roll_setpoint = receiver_input_channel_1 - 1754;
		else if(receiver_input_channel_1 < 1746)pid_roll_setpoint = receiver_input_channel_1 - 1746;
		pid_roll_setpoint -= roll_level_adjust;                                   // Subtract the angle correction from the standardized receiver roll input value.
		pid_roll_setpoint /= 1.5;                                                 // Converção para entrada do PID_roll ser em graus por segundo

		//Entrada do sistema de controle para o pitch -------------------------------------------------------------------------
//		pid_pitch_setpoint = 0;
//		if(receiver_input_channel_2 > 1754)pid_pitch_setpoint = receiver_input_channel_2 - 1754;
//		else if(receiver_input_channel_2 < 1746)pid_pitch_setpoint = receiver_input_channel_2 - 1746;
//
//		pid_pitch_setpoint -= pitch_level_adjust;                                  // Subtract the angle correction from the standardized receiver pitch input value.
//		pid_pitch_setpoint /= 1.5;                                                 // Converção para entrada do PID_pitch ser em graus por segundo

		//Entrada do sistema de controle para o Yaw -------------------------------------------------------------------------
		//pid_yaw_setpoint = 0;
		////We need a little dead band of 16us for better results.
		//if(receiver_input_channel_3 > 1525){ //Não rotacionar o drone quando o throttle tiver em BAIXO
		//if(receiver_input_channel_4 > 1754)pid_yaw_setpoint = (receiver_input_channel_4 - 1754)/1.5;
		//else if(receiver_input_channel_4 < 1746)pid_yaw_setpoint = (receiver_input_channel_4 - 1746)/1.5;
		//}

		calculate_pid();                                                            // Com as entradas prontas para o PID, podemos calcular a saída

		//Compesação do descarga da bateria no contolador PID (com filtro complementar para reudizr o ruído da leitura analógica)
		//0.09853 = 0.08 * 0.01642.
		battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 49) * 0.0013136;
		if(battery_voltage < 14 && battery_voltage > 12.8)digitalWrite(12, HIGH);// Aviso caso o nivel da bateria esteja muito baixo (12.8 a  14V)

		throttle = receiver_input_channel_3;                                      // O throttle será o sinal de controle principal

		if (start == 2){                                                          // Motores estão armados
		if (throttle > 1900) throttle = 1900;                                   // O PID precisa de uma parte da largura de pulso para atuar no drone
		esc_1 = throttle + pid_output_roll;									  // Esc 1: direito  ||	  							||
		esc_2 = throttle - pid_output_roll;										// Esc 2: esquerdo || IMPORTANTE SEGUIR ESSA ORDEM ||

		if (battery_voltage < 16.4 && battery_voltage > 12.4){                   // Compensa a bateria apenas se tiver entre 12.4 e 16.4 Volts
		  esc_1 += esc_1 * ((16.4 - battery_voltage)/(float)70);              // Compoensação para o ESC 1 para a descarga da bateria	
		  esc_2 += esc_2 * ((16.4 - battery_voltage)/(float)70);              // Compoensação para o ESC 2 para a descarga da bateria	Porcentagem
		} 

		if (esc_1 < 1550) esc_1 = 1550;                                         // Os motores continuam rodando com o valor de throttle no baixo
		if (esc_2 < 1550) esc_2 = 1550;                                         // Os motores continuam rodando com o valor de throttle no baixo
																				   
		if(esc_1 > 2000)esc_1 = 2000;                                           // Limite da largura de pulso deve ser 2000 us
		if(esc_2 > 2000)esc_2 = 2000;                                           // Limite da largura de pulso deve ser 2000 us
		}                                                                            
																				   
		else{                                                                        
		esc_1 = 1500;                                                           // Se não estiver armado, mantém o esc 1 no valor mínimo
		esc_2 = 1500;                                                           // Se não estiver armado, mantém o esc 2 no valor mínimo
		}

		//====================================== ATENÇÃO =================================================
		// O loop precisa necessáriamente ser menor que 4000 us (taxa de atualização dos ESCs ) - 250 Hz.|
		//================================================================================================

		if(micros() - loop_timer > 4050)digitalWrite(LED, HIGH);                   // Se o loop for maior que 4050 us, ligamos  LED de alerta
		while(micros() - loop_timer < 4000);                                      // Esperamos o tempo que sobra no loop até comletar 4000us
		loop_timer = micros();                                                    // Zeramos o loop_timer para os próximos 4000 us
																				   
		PORTD |= B11000000;                                                       // Coloca os pinos 6 e 7 em alto
		timer_channel_1 = esc_1 + loop_timer;                                     // Calcula o tempo em que o pulso do ESC 1 vai permanecer em ALTO
		timer_channel_2 = esc_2 + loop_timer;                                     // Calcula o tempo em que o pulso do ESC 2 vai permanecer em ALTO

		//Sempre teremos 1550 us com o sinal em alto (valor mínimo enviado aos ESCs), então podemos aproveitar fazendo outra coisa
		//Adquire os valores de velocidade angular e aceleração da IMU para serem utilizado nos cálculos (isso demora ~600 us)
		IMU.LerDadosIMU(MPU6050_ADDRESS, dados_brutos);
		acc_x = dados_brutos[0] + accel_offset[0];
		acc_y = dados_brutos[1] + accel_offset[1];
		acc_z = dados_brutos[2] + accel_offset[2];	// Compensação Offset
		gyro_roll = dados_brutos[4] - gyro_axis_cal[1];	// Nao utilizado nessa bancada
		//gyro_pitch = dados_brutos[5] - gyro_axis_cal[2];
		//gyro_yaw = dados_brutos[6] - gyro_axis_cal[3];	// Nao utilizado nessa bancada

		while(PORTD >= 64){                                                       // Entre neste loop até que todos os sinais estejam em BIAXO
		esc_loop_timer = micros();                                              // Lê o tempo atual
		if(timer_channel_1 <= esc_loop_timer)PORTD &= B10111111;                // Se o tempo atual for >= do tempo definido, o pino 6 é colocado em BAIXO
		if(timer_channel_2 <= esc_loop_timer)PORTD &= B01111111;                // Se o tempo atual for >= do tempo definido, o pino 7 é colocado em BAIXO
	  }
	}else{
		// Se o PID não estiver ativado --------------------------------------------- DEBUG SERIAL -------------------------------------------------------------------------------------
		IMU.LerDadosIMU(MPU6050_ADDRESS, dados_brutos);
		acc_x = dados_brutos[0]*aRes;// + accel_offset[0];
		acc_y = dados_brutos[1]*aRes;// + accel_offset[1];
		acc_z = dados_brutos[2]*aRes;// + accel_offset[2];	// Converção dos dados
		gyro_roll = dados_brutos[4] - gyro_axis_cal[1];	
		//gyro_pitch = dados_brutos[5]- gyro_axis_cal[2];
		//gyro_yaw = dados_brutos[6] - gyro_axis_cal[3];	

    delta = (millis() - loop_timer)/1000;
    // Cáluclos Estimativa de atitude, ângulos de Euler
		// Giroscópio -------------------------
		//angle_pitch += gyro_pitch * 0.00006103515625 * delta;                                    // Integração simples para transformar graus/segundo em graus a partir do período delta (Pitch)
		angle_roll += gyro_roll * gRes * delta;                                      // Integração simples para transformar graus/segundo em graus a partir do período delta (Roll)
		//angle_yaw += gyro_yaw * 0.00006103515625 * delta;										                    // Integração simples para transformar graus/segundo em graus a partir do período delta (yaw)																		
		//angle_pitch -= angle_roll * sin(gyro_yaw * 0.000001066 * delta);                  // Compensa a rotação da IMU (Yaw) transferindo o angulo roll para o pitch
		//angle_roll += angle_pitch * sin(gyro_yaw * 0.000001066 * delta);                  // Compensa a rotação da IMU (Yaw) transferindo o angulo pitch para o roll
    loop_timer = millis();                                                    // Zeramos o loop_timer para os próximos 2000 us
                                           
		// Acelerômetro                                           
		acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));       // Vetor resultante da acleração (raiz quadrada da soma dos quadrados)
																				   
		//if(abs(acc_y) < acc_total_vector){                                        // Apenas calcula se |acc_y| < vetor resultante.
	  //angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;				// ângulo do pitch baseado na aceleração
		//}                                                                            
		if(abs(acc_x) < acc_total_vector){                                        // Apenas calcula se |acc_x| < vetor resultante.
		angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;          // Ângulo do roll baseado na aceleração
		}
		
		//Fusão sensorial
		//angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;            // Filtro complementar para correção do acumulo de erro no pitch
		angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;               // Filtro complementar para correção do acumulo de erro no roll
		//angle_yaw = angle_yaw * 0.9996 + angle_yaw_acc * 0.0004;
		
		if(Serialgraph){
		  //Serial.print(" Angulo eixos: X = "); Serial.print(angle_pitch); Serial.print("\t Y =\t"); Serial.print(angle_roll); Serial.print("\t Z =\t"); Serial.println(angle_yaw);
      Serial.println("Atualize Delta primeiro para ter 1 Hz de frequencia de amostragem, depois comente a linha e descomnte a acima");
		} else if(MatlabPlot){
      //millisTime = millis() - setupTime;
			//          				  Serial.print( acc_x, 2);
		  //Serial.print("\t"); Serial.print( acc_y, 2); 
		  //Serial.print("\t"); Serial.print( acc_z, 2);
		  //Serial.print("\t"); Serial.print( gyro_roll, 2); 
		  //erial.print("\t"); Serial.print( gyro_pitch, 2); 
		  //Serial.print("\t"); Serial.print( gyro_yaw, 2);
		  //Serial.print("\t"); Serial.print( angle_pitch, 2); 
		  Serial.print("\t"); Serial.print( angle_roll, 2); 
		  //Serial.print("\t"); Serial.print( angle_yaw, 2);
		  Serial.print("\t");Serial.println(millisTime);
		}
	}
}

//Essa rotina acontece sempre que há uma mudança nos pinos 8, 9 e 10 (sinais do receptor, canais 6, 2 e 3 Respectivamente)
ISR(PCINT0_vect){
  current_time = micros();
  //Canal 1==================================================ARM=========================================================
  if(PINB & B00000001){                                                    // A entrada 8 está em alto?
    if(last_channel_1 == 0){                                                // Input 8 mudou de baixo para alto.
      last_channel_1 = 1;                                                   // grava o estado atual
      timer_1 = current_time;                                               // Set timer_1 to current_time.
    }                                                                          
  }                                                                            
  else if(last_channel_1 == 1){                                             // Input 8 is not high and changed from 1 to 0.
    last_channel_1 = 0;                                                     // grava o estado atual
    receiver_input[1] = current_time - timer_1;                             // Channel 1 is current_time - timer_1.
  }
  //Canal 2 ================================================PITCH========================================================
  if(PINB & B00000010){                                                     // A entrada 8 está em alto?
    if(last_channel_2 == 0){                                                // Input 8 mudou de baixo para alto.
      last_channel_2 = 1;                                                   // grava o estado atual
      timer_2 = current_time;                                               // Set timer_2 to current_time.
    }                                                                          
  }                                                                            
  else if(last_channel_2 == 1){                                             // Input 8 is not high and changed from 1 to 0.
    last_channel_2 = 0;                                                     // grava o estado atual
    receiver_input[2] = current_time - timer_2;                             // Channel 2 is current_time - timer_2.
  }
  //Canal 3===============================================THROTTLE=========================================================
  if(PINB & B00000100 ){                                                    // A entrada 10 está em alto?
    if(last_channel_3 == 0){                                                // Input 10 mudou de baixo para alto.
      last_channel_3 = 1;                                                   // grava o estado atual
      timer_3 = current_time;                                               // Set timer_3 to current_time.
    }                                                                          
  }                                                                            
  else if(last_channel_3 == 1){                                             // Input 10 is not high and changed from 1 to 0.
    last_channel_3 = 0;                                                     // grava o estado atual
    receiver_input[3] = current_time - timer_3;                             // Channel 3 is current_time - timer_3.
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////|
//												Subroutina para cálculo das saídas PID 													|
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////|

void calculate_pid(){
  //Cálculo PID Pitch -------------------------------------------------------------------------------------------------------------------------
//  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;							// Cálculo do erro do ângulo roll
//  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;								// Ganho controlador I multiplicado pelo erro
//  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;				// Delimita esse valor para ficar entre +/- o valor máximo
//  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;// +200 ou -200
  
// PID PITCH: ------|		Ganho_P * Erro [P]		  |Ganho_I * Erro [I]| 		Ganho_D * (Erro[i]-Erro[i-1]) [D]          |-------------------
//  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
//  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;					// Delimita esse valor para ficar entre +/- o valor máximo
//  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;  // + 200 ou -200
//  pid_last_pitch_d_error = pid_error_temp;                                              // Salva o erro para o prox cálculo

    //Cálculo PID Roll -------------------------------------------------------------------------------------------------------------------------
    pid_error_temp = gyro_roll_input - pid_roll_setpoint;                // Cálculo do erro do ângulo roll
    pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;               // Ganho controlador I multiplicado pelo erro
    if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;         // Delimita esse valor para ficar entre +/- o valor máximo
    else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;  
    
    // PID ROLL: ------|    Ganho_P * Erro [P]     |Ganho_I * Erro [I]|     Ganho_D * (Erro[i]-Erro[i-1]) [D]                 |------------
    pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
    if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;         // Delimita esse valor para ficar entre +/- o valor máximo
    else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;  // + 200 ou -200
    pid_last_roll_d_error = pid_error_temp;                     // Salva o erro para o prox cálculo
  
}

//Essa função garante que que os sinais brutos do receptor serão escalonados para a faixa de 1500-2000 uS (configuração unidirecional)
//**********Esta faixa é apenas para ESC's BIDIRECIONAIS. ESCs unidirecionais não serão armados ou o motor não consigirá parar************
//Os dados armazenados na memória EEPROM (baixo, medio e alto) calibrados são utilizados aqui
int convert_receiver_channel(byte channel){
  // Variáveis locais
  int low, center, high, actual;                                                  
  int difference;   
																			  
  actual = receiver_input[channel];                                            // Lê o valor atual do receptor (1000 a 2000 us)
  low = (EEPROM.read(47+channel) << 8) | EEPROM.read(50+channel);  // valor BAIXO para esse canal
  center = (EEPROM.read(35+channel) << 8) | EEPROM.read(38+channel); // valor de CENTRO para esse canal
  high = (EEPROM.read(41+channel) << 8) | EEPROM.read(44+channel);   // valor ALTO para esse canal
																				  
  if(actual < center){                                                         // Caso o valor atual seja menor que o de centro
    if(actual < low)actual = low;                                              // Limite mais baixo do sinal
    difference = ((long)(center - actual) * (long)500) / (center - low);       // A diferença sempre estará entre 0 e 500
    if ((invertido >> (channel-1)) & 0b00000001) return 1750 + difference/2;
	else return 1750 - difference/2;                                           // Caso não esteja invertido
  }                                                                               
  else if(actual > center){                                                    // Quando o valor recebido é maior que o de CENTRO
    if(actual > high)actual = high;                                            // Limite do valor mais baixo, adquirido no programa de configuração
    difference = ((long)(actual - center) * (long)500) / (high - center);      // A diferença sempre estará entre 0 e 500
    if ((invertido >> (channel-1)) & 0b00000001) return 1750 - difference/2;
	return 1750 + difference/2;                                           		// Caso não esteja invertido
  }
  else return 1750;																// Quando é exatamente igual o de centro
}

void registra_min_max(){
  byte zero = 0;
  low_channel_1 = receiver_input[1];
  low_channel_2 = receiver_input[2];
  low_channel_3 = receiver_input[3];
  while(receiver_input[3] < center_channel_3 + 20 && receiver_input[3] > center_channel_3 - 20)delay(250);
  Serial.println(F(" Medindo as posições máximas e mínimas"));
  while( // Enquanto os manches não estiverem na posição neutra
    receiver_input[2] < center_channel_2 - 20 || receiver_input[2] > center_channel_2 + 20 ||
    receiver_input[3] < center_channel_3 - 20 || receiver_input[3] > center_channel_3 + 20){
    
	if(receiver_input[1] < low_channel_1)low_channel_1 = receiver_input[1];
    if(receiver_input[2] < low_channel_2)low_channel_2 = receiver_input[2];
    if(receiver_input[3] < low_channel_3)low_channel_3 = receiver_input[3];
    if(receiver_input[1] > high_channel_1)high_channel_1 = receiver_input[1];
    if(receiver_input[2] > high_channel_2)high_channel_2 = receiver_input[2];
    if(receiver_input[3] > high_channel_3)high_channel_3 = receiver_input[3];
	
	Serial.print(F("|Canal 1 (ARM): "));
	Serial.print(low_channel_1);Serial.print(F(" - "));Serial.print(high_channel_1);Serial.print(F(" |Canal 2 (Pitch): "));
	Serial.print(low_channel_2);Serial.print(F(" - "));Serial.print(high_channel_2);Serial.print(F(" |Canal 3 (Throttle):"));
	Serial.print(low_channel_3);Serial.print(F(" - "));Serial.println(high_channel_3);
    delay(100);
	}
}

void espera_10seg(){
	for(int i = 9;i > 0;i--){
	  delay(1000);
	  Serial.print(i);
	  Serial.print(" | ");
	}
	Serial.println(" ");
}

void espera_20seg(){
  for(int i = 19;i > 0;i--){
    delay(1000);
    Serial.print(i);
    Serial.print(" | ");
  }
  Serial.println(" ");
}

void checa_inversao(int movimento){
	timer = millis() + 30000;
	while(timer > millis()){
    if(movimento == 1){
  		if(receiver_input[1] < 1050){invertido |= 0b00000001; Serial.println(F("registrado")); delay(2000); return;}	// Tá invertido
  		if(receiver_input[1] > 1950){invertido &= 0b11111110; Serial.println(F("registrado")); delay(2000); return;}	// Não ta invertido
    } else if(movimento == 2){
		  if(receiver_input[2] < 1050){invertido |= 0b00000010; Serial.println(F("registrado")); delay(2000); return;}	// Tá invertido
		  if(receiver_input[2] > 1950){invertido &= 0b11111101; Serial.println(F("registrado")); delay(2000); return;}	// Não ta invertido
    }else if(movimento == 3){
		  if(receiver_input[3] < 1050){invertido |= 0b00000100; Serial.println(F("registrado")); delay(2000); return;}	// Tá invertido
		  if(receiver_input[3] > 1950){invertido &= 0b11111011; Serial.println(F("registrado")); delay(2000); return;}	// Não ta invertido
    }			
	}
  Serial.println(F("Não foi feixo nenhum movimento"));
}
