/**
 * A Mirf example to test the latency between two Ardunio.
 *
 * Pins:
 * Hardware SPI:
 * MISO -> 12
 * MOSI -> 11
 * SCK -> 13
 *
 * Configurable:
 * CE -> 8
 * CSN -> 7
 *
 * Note: To see best case latency comment out all Serial.println
 * statements not displaying the result and load 
 * 'ping_server_interupt' on the server.
 */


#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>


void Radio_Init()
{
  Mirf.spi = &MirfHardwareSpi;
  Mirf.init();
   
  Mirf.setTADDR((byte *)"clie1");   
  Mirf.setRADDR((byte *)"serv1");
    
  Mirf.payload = RADIO_PAYLOAD;
  Mirf.channel = RADIO_CHANNEL;

  Mirf.configRegister(RF_SETUP, B00100110);

  Mirf.config();
  flush_radio_buffer();
}


void Radio_Send( byte *tx_data)
{  
  
  Mirf.send(tx_data);
  while(Mirf.isSending());    // Block !!
} 

void flush_radio_buffer()
{
  for (int i = 0; i < RADIO_PAYLOAD; i++)
  {
    radio_tx_buffer[i] = 0;
    radio_rx_buffer[i] = 0;
  }
}
 
byte Radio_Receive ()
{
  unsigned char dr = 0;
  
  if ( !Mirf.isSending() && Mirf.dataReady() ) //
  {
    flush_radio_buffer();
    Mirf.getData(radio_rx_buffer);
    dr = 1;
  }

  return dr;
  
}
    
