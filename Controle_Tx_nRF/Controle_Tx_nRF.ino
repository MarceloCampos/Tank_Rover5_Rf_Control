/**
 * Controle_Tx_nRF.ino + Radio_API.ino 
 * Mirf  Ardunio based.
 * Usar Library Kalman Filter: https://github.com/MarceloCampos/Kalman_Filter
 *
 *
 *
 *
 * VERSÃO Carrinho Rover 5 - Esteira
 * protocolo 16 bytes
 * off set	|	byte	| função	
 * 0		| 0x03 0x15     | Start
 * 2		| 0x00		| 0=tras 1=frente
 * 3		| 0x00		| Velocidade 0 - 255 (0=parado)
 * 4		| 0x00		| Direção 0=reto 1=direita 2=esquerda
 * 5		| 0x00		| camera pos x
 * 6		| 0x00		| camera pos y
 * 7		| 7bytes	| reservado uso futuro 	
 * 14		| 0x00 0x00	| (int) Check Sum incluindo Start 
 *
 *
 */
#include <Kalman_Filter.h> // disponível em https://github.com/MarceloCampos/Kalman_Filter

//------------------------------ Definições e variáveis do Rádio -----------------------------
#define RADIO_PAYLOAD  0x10
#define RADIO_CHANNEL  18

byte radio_tx_buffer[RADIO_PAYLOAD];
byte radio_rx_buffer[RADIO_PAYLOAD];
//------------------------------ FIM Definições e variáveis do Rádio -------------------------



#define Xmax 730
#define Xmin 240
#define Ymax 785
#define Ymin 285
// -> Xmax 730 e XMin 240 
// -> Ymax 785 e Ymin 280

int tempo_espera_ack = 300;     // tempo que espera pelo outro lado mandar um ACK
int packetIntervalTime = 100;   // intervalo de tempo mínimo entre envios em mS
int LED = 0x09;
int count = 0x00;

#define DEBUG  1                // debug valores pela serial

volatile byte Flag_Acelerometro = 1;      // usdo do acelerômetro ao invés do joystick
volatile int sw_mode = 0x02;    // D2 - Interrupt 0 - chave modo controle acelerômetro ou joystick

Kalman_Filter Kalman_Filter_Y;
Kalman_Filter Kalman_Filter_X;

void setup()
{
  pinMode(LED, OUTPUT);
  pinMode(sw_mode,INPUT_PULLUP);// chave modo controle acelerômetro ou joystick  
  digitalWrite(LED, HIGH);
  delay (1000);

#if defined DEBUG  
  Serial.begin(115200);         // Debug somente
#endif 

  Radio_Init();


  analogReference(EXTERNAL);   // refência 3,3Volts p/ acelerômetro e joystick, ligar pino Vref ao 3,3volts

#if defined DEBUG   
  Serial.println("Beginning ... "); 
#endif

  Kalman_Filter_X.kalman_init();
  Kalman_Filter_Y.kalman_init();
  
  delay (500);

  attachInterrupt(0, InterruptService, FALLING);
  digitalWrite(LED, LOW);
}

void loop()
{
  unsigned long time ;
  unsigned long wait_time = tempo_espera_ack;

  time = millis();
  digitalWrite(LED, LOW);

  if (Flag_Acelerometro)
    Le_Acelerometro();
  else
    Le_Sensores();
  
  Monta_Envia_Frame();
  
#if defined DEBUG
  printBuffer();
#endif  

  while(!Radio_Receive ())                        // timeout de espera receber
  {    
    if ( millis() - time > wait_time )
    {
      #if defined DEBUG 
      Serial.println("TOT"); 
      #endif 
      return;
    }       
  }

#if defined DEBUG  
  Serial.println("> Rx Packet");
#endif  
 
  digitalWrite(LED, HIGH);

  if ( millis() - time <100 )
  {
    delay( packetIntervalTime - (millis() - time) );  // espera intervalo de tempo mínimo entre envios em mS, variável "packetIntervalTime"
  }
  else
  {
    delay(100);
  }
  
} 

void Monta_Envia_Frame ()
{
  radio_tx_buffer[0] = 0x03;          // header
  radio_tx_buffer[1] = 0x15;
  
  Radio_Send( radio_tx_buffer );
}

void Le_Acelerometro()
{
  int adc_rd_Eixo_Y;
  int adc_rd_Eixo_X;

  int adc_ft_Eixo_Y;  // valores filtrados
  int adc_ft_Eixo_X;
  
  int temp;
  
  adc_rd_Eixo_Y = analogRead(A3);
  adc_rd_Eixo_X = analogRead(A2);
  
  
  adc_ft_Eixo_X = (int)Kalman_Filter_X.kalman_filter( (float)adc_rd_Eixo_X );
  adc_ft_Eixo_Y = (int)Kalman_Filter_Y.kalman_filter( (float)adc_rd_Eixo_Y );
  adc_ft_Eixo_Y = (Ymax+Ymin) - adc_ft_Eixo_Y;  // inverte
  
  if ( adc_ft_Eixo_Y > ((Ymax-Ymin)/2 + Ymin) )// Velocidade 0-255, se maior que 512 (p/ frente) normaliza p/ 0-512 ou ...
  {      
    temp =  (adc_ft_Eixo_Y - ((Ymax-Ymin)/2 + Ymin) );
    radio_tx_buffer[2] = 1;            // frente 
  }
  else if ( adc_ft_Eixo_Y < ((Ymax-Ymin)/2 + Ymin) )
  {							
    temp = ((Ymax-Ymin)/2 + Ymin) - adc_ft_Eixo_Y ;	// ... se < 512 inverte pois joystick puxando pra tras o nível DC que envia é invertido
    radio_tx_buffer[2] = 0;             // tras
  }
     	
  radio_tx_buffer[3] = byte(temp);      // Velocidade 0-255
  	
  // Direção 0=reto 1=direita 2=esquerda 
  if (adc_ft_Eixo_X >= ((Xmax-Xmin)/2 + Xmin + 45) )
    radio_tx_buffer[4] = 0x02; 		
  else if (adc_ft_Eixo_X <= ((Xmax-Xmin)/2 + Xmin - 45))
    radio_tx_buffer[4] = 0x01; 
  else 
    radio_tx_buffer[4] = 0x00; 

#if defined DEBUG 
//  Serial.print("0000, "); 
//  Serial.print(adc_rd_Eixo_X);
//  Serial.print(", ");   
//  Serial.print(adc_ft_Eixo_X);
//  Serial.print(", ");   
//  Serial.print(adc_rd_Eixo_Y);
//  Serial.print(", ");   
//  Serial.print(adc_ft_Eixo_Y);   
//  Serial.println("  ");
#endif

}

void Le_Sensores ()
{
  int adc_rd_Eixo_Y;
  int adc_rd_Eixo_X;
  int temp;
  
  adc_rd_Eixo_Y = analogRead(A0);
  adc_rd_Eixo_X = analogRead(A1);
  
  if ( adc_rd_Eixo_Y >= 512 )		// Velocidade 0-255, se maior que 512 (p/ frente) normaliza p/ 0-512 ou ...
  {  
    temp =  adc_rd_Eixo_Y - 512;
    radio_tx_buffer[2] = 0;            // frente
  }
  else
  {							
    temp = 511 - adc_rd_Eixo_Y;	// ... se < 512 inverte pois joystick puxando pra tras o nível DC que envia é invertido
    radio_tx_buffer[2] = 1;             // tras
  }
  	
  radio_tx_buffer[3] = ( (float)temp / 512 ) * 255; 	// Velocidade 0-255
  	
  // Direção 0=reto 1=direita 2=esquerda
  if (adc_rd_Eixo_X >= 560)
    radio_tx_buffer[4] = 0x02; 		
  else if (adc_rd_Eixo_X <= 462)
    radio_tx_buffer[4] = 0x01; 
  else 
    radio_tx_buffer[4] = 0x00; 

}

void  printBuffer()
{
  for (int i = 0; i < RADIO_PAYLOAD; i++)
  {
    Serial.print(radio_tx_buffer[i]);
    Serial.print(" ");
  }
  Serial.println();
}


void InterruptService()
{
  volatile int count = 1000;
  
  detachInterrupt( digitalPinToInterrupt(0) );

  while (count--)
  {
    if (digitalRead (sw_mode))
      goto exit;
  }  
  
  Flag_Acelerometro = !Flag_Acelerometro;

exit:  
  delayMicroseconds(50000);
  attachInterrupt(0, InterruptService, FALLING);

}


//  Serial.print("temp: ");	
//  Serial.print(temp);	
//  Serial.print(" | adc_ft_Eixo_Y: ");	
//  Serial.println(adc_ft_Eixo_Y);	

/*
  digitalWrite(LED, LOW);
 
  for (int i = 1; i < RADIO_PAYLOAD; i++)
    radio_tx_buffer[i] = 0x30 + i;
    
  radio_tx_buffer[0] = count++;
  
  Radio_Send( radio_tx_buffer );
 
  Serial.println("Finished sending");
  delay(10);

  
  while(!Radio_Receive ())
  {
    
  }
  
  digitalWrite(LED, HIGH);

  for (int i = 0; i < RADIO_PAYLOAD; i++)
  {
    Serial.print(radio_rx_buffer[i]);
    Serial.print(" ");
  }
  Serial.println();
   
  delay(100);
*/
