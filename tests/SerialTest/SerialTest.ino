#include <Arduino.h>   // required before wiring_private.h
#include "wiring_private.h" // pinPeripheral() function


// Serial 2
Uart Serial2( &sercom3, 13, 12, SERCOM_RX_PAD_1, UART_TX_PAD_0 ) ;
void SERCOM3_0_Handler()
{
  Serial2.IrqHandler();
}
void SERCOM3_1_Handler()
{
  Serial2.IrqHandler();
}
void SERCOM3_2_Handler()
{
  Serial2.IrqHandler();
}
void SERCOM3_3_Handler()
{
  Serial2.IrqHandler();
}

// Serial3
Uart Serial3 (&sercom4, A3, A2, SERCOM_RX_PAD_1, UART_TX_PAD_0);
void SERCOM4_0_Handler()
{
  Serial3.IrqHandler();
}
void SERCOM4_1_Handler()
{
  Serial3.IrqHandler();
}
void SERCOM4_2_Handler()
{
  Serial3.IrqHandler();
}
void SERCOM4_3_Handler()
{
  Serial3.IrqHandler();
}

// Serial4
Uart Serial4 (&sercom0, A1, A4, SERCOM_RX_PAD_1, UART_TX_PAD_0);
void SERCOM0_0_Handler()
{
  Serial4.IrqHandler();
}
void SERCOM0_1_Handler()
{
  Serial4.IrqHandler();
}
void SERCOM0_2_Handler()
{
  Serial4.IrqHandler();
}
void SERCOM0_3_Handler()
{
  Serial4.IrqHandler();
}


void setup() {


  Serial.begin(115200);

  Serial1.begin(9600);
  
  Serial2.begin(9600);
  
  Serial3.begin(9600);
  
  Serial4.begin(115200);


  // Assign pins A1,A4 and A2,A3 SERCOM functionality
  pinPeripheral(A1, PIO_SERCOM_ALT);
  pinPeripheral(A2, PIO_SERCOM_ALT);
  pinPeripheral(A3, PIO_SERCOM_ALT);
  pinPeripheral(A4, PIO_SERCOM_ALT);
  pinPeripheral(13, PIO_SERCOM);
  pinPeripheral(12, PIO_SERCOM);

  pinMode(A0, OUTPUT); // nreset of lora
  digitalWrite(A0, LOW);
  delay(1000);
  digitalWrite(A0, HIGH);
  
  //Serial4.begin(9600);
  
  delay(3000);
}

void loop() {

  /*Serial1.println("Hello from Serial1 !");
  Serial2.println("Hello from Serial2 !");
  Serial3.println("Hello from Serial3 !");
  Serial4.println("Hello from Serial4 !");
  delay(500);
  
  while (Serial1.available() > 0)
    Serial.write(Serial1.read());
    
  while (Serial2.available() > 0)
    Serial.write(Serial2.read());
    
  while (Serial3.available())
    Serial.write(Serial3.read());
    
  while (Serial4.available())
    Serial.write(Serial4.read());*/
    
  while( Serial.available() ){
    Serial4.write(Serial.read());
  }
  
  while( Serial4.available() ){
    Serial.write(Serial4.read());
  }
  

}
