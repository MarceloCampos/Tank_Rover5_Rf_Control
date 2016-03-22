/**
 * Robot_Rx_nRF.ino + Radio_API.ino 
 * Mirf  Ardunio based.
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
 * Arduino  |  Conec pci atmega-nRF
 * 3            1 PWM
 * 10           2 Direção
 * 6            3 PWM
 * 5            4 (PWM) Direção
 * Marcelo Campos - Garoa HC
 */


//------------------------------ Definições e variáveis do Rádio -----------------------------

#define RADIO_PAYLOAD  0x10
#define RADIO_CHANNEL  18

byte radio_tx_buffer[RADIO_PAYLOAD];
byte radio_rx_buffer[RADIO_PAYLOAD];
//------------------------------ FIM Definições e variáveis do Rádio -------------------------

int LED = 0x09;
byte start_byte_1  = 0x03;
byte start_byte_2  = 0x15;

//motor A
int IN1 = 3 ;     // ~PWM - Velocidade 
int IN2 = 5 ;     // Direção 
//motor B
int IN3 = 10 ;    // ~PWM - Velocidade
int IN4 = 6 ;     //  Direção

byte Flag_Stop = 0;

void setup()
{    
  pinMode(LED, OUTPUT); 
  digitalWrite(LED, HIGH); 
  
  Serial.begin(115200);        // Debug somente

  Radio_Init();

  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  
  digitalWrite(IN2,LOW); 
  analogWrite(IN1, 0);

  digitalWrite(IN4, LOW);
  analogWrite(IN3, 0); 

  Serial.println("Init Ok");

  delay (1000);
  digitalWrite(LED, LOW); 
  delay(500);
}

void loop()
{
  unsigned long time = millis();
  unsigned long wait_time = 3000;
   
  while(!Radio_Receive ())                        // timeout de espera receber
  {
    
    if ( millis() - time > wait_time )
    {
      Serial.println("No Signal!"); 
      Stop();            
      return;
    } 
      
  }
  
    digitalWrite(LED, HIGH);
    
    if ( radio_rx_buffer[0] == start_byte_1 && radio_rx_buffer[1] == start_byte_2 )
    {      
      Analiza_Recebido ();
    }

    printRxData();    // DEBUG

    Send_ACK();
    digitalWrite(LED, LOW);

}

void printRxData()
{
  for (int i = 0; i < RADIO_PAYLOAD; i++)
  {
    Serial.print(radio_rx_buffer[i]);
    Serial.print(" ");
  }
  Serial.println();
}

void Send_ACK()
{
    flush_radio_buffer();
    radio_tx_buffer[0] = 'O';
    radio_tx_buffer[1] = 'K';
    Radio_Send(radio_tx_buffer);
}


void Analiza_Recebido ()
{
// ----- Processa o frame recebido de acordo com protocolo ( vide descritivo)
  // 2    | 0x00    | 0=tras 1=frente
  // 3    | 0x00    | Velocidade 0 - 255 (0=parado)
  // 4    | 0x00    | Direção 0=reto 1=direita 2=esquerda
  // sentido frente / tras
  if (radio_rx_buffer[2] == 1 && radio_rx_buffer[4] == 0)   
  { // para Frente - Reto
    digitalWrite (IN2, 0);
    digitalWrite (IN4, 0);
//    Serial.println("Frente - Reto");
  }
  else if (radio_rx_buffer[2] == 1 && radio_rx_buffer[4] == 1)
  { // Frente - direita
    digitalWrite (IN2, 0);
    digitalWrite (IN4, 1);
//    Serial.println("Frente - direita");
  }
  else if (radio_rx_buffer[2] == 1 && radio_rx_buffer[4] == 2)
  { // Frente - Esquerda
    digitalWrite (IN2, 1);
    digitalWrite (IN4, 0);
//    Serial.println("Frente - Esquerda");
  }
  else if (radio_rx_buffer[2] == 0 && radio_rx_buffer[4] == 0)
  { // tras - reto
    digitalWrite (IN2, 1);
    digitalWrite (IN4, 1);
//    Serial.println("tras - reto");
  }
  else if (radio_rx_buffer[2] == 0 && radio_rx_buffer[4] == 1)
  { // tras - direita
    digitalWrite (IN2, 1);
    digitalWrite (IN4, 0);
//    Serial.println("tras - direita");
  }
  else if (radio_rx_buffer[2] == 0 && radio_rx_buffer[4] == 2)
  { // tras - direita
    digitalWrite (IN2, 0);
    digitalWrite (IN4, 1);
//    Serial.println("tras - direita");
  }

  if (radio_rx_buffer[3] < 5  && radio_rx_buffer[4]) // virando sem acelerar, volta próprio eixo
  {

    if (radio_rx_buffer[4] == 1)
    {
      digitalWrite (IN2, 0);
      digitalWrite (IN4, 1);
      analogWrite(IN1, 25 );  
      analogWrite(IN3, 250);        
    }
    else
    {
      digitalWrite (IN2, 1);
      digitalWrite (IN4, 0);
      analogWrite(IN1, 250);  
      analogWrite(IN3, 25);     
    }
    Serial.println("virando sem acelerar, volta próprio eixo");
  }
  else
  {
      if (!radio_rx_buffer[2])
      {
        analogWrite(IN1, 255 - radio_rx_buffer[3]);  
        analogWrite(IN3, 255 - radio_rx_buffer[3]);   
      }
      else
      {
        analogWrite(IN1, radio_rx_buffer[3]);  
        analogWrite(IN3, radio_rx_buffer[3]);           
      }
        
  }

  
}


void Stop ()  // para motores - perda sinal
{
  digitalWrite(IN2,LOW); 
  digitalWrite(IN4,LOW);  
  analogWrite(IN1, 0);  
  analogWrite(IN3, 0);   
}





