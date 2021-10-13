// TelemSpoofer firmware for AMTPS-FTA project
// Matt Ruffner 2021
// This software is meant to simulate the telemetry that the capsule would send 
// to the groundstation, for testing the range and functionality of the
// 915Mhz LoRa telemetry link


#include <SPI.h>
#include <Wire.h>
#include <FreeRTOS_SAMD21.h>
#include <semphr.h>
#include "wiring_private.h"

#include "include/delay_helpers.h" // rtos delay helpers
#include "include/config.h"        // project wide defs
#include "include/packet.h"        // data packet defs

#define DEBUG 1

#define PIN_LORA_RST A0

#define TLM_SEND_PERIOD   1000 // in scheduler ticks (should be 1ms)
#define RX_TIMEOUT_PERIOD 500  

TaskHandle_t Handle_radTask;
//TaskHandle_t Handle_monitorTask;

// freeRTOS queues
QueueHandle_t qRadData; // data queue for sending recvd radio data to the sd card to be logged

SemaphoreHandle_t dbSem; // serial debug logging (Serial)

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

#define SERIAL Serial
#define RADIO_SERIAL Serial4


/**********************************************************************************
 *  Radio task
*/
#define RBUF_SIZE 300
#define SBUF_SIZE 240
static void radThread( void *pvParameters )
{
  uint8_t rbuf[RBUF_SIZE];
  char sbuf[SBUF_SIZE];
  tlm_t dataOut;
  unsigned long lastSendTime = 0;
  
    /* 0: waiting for +OK from reset
     1: configuring address
     2:   waiting for address config +OK
     3: configuring network id
     4:   waiting for network id config +OK
     5: configuring rf params
     6:   waiting for rf param config +OK
     7: ready
  */
  int state = 0;  
  
  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    SERIAL.println("Radio thread started");
    xSemaphoreGive( dbSem );
  }
  #endif
  
  
  digitalWrite(PIN_LORA_RST, LOW);
  myDelayMs(1000);
  digitalWrite(PIN_LORA_RST, HIGH);
  
  
  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    SERIAL.println("Reset radio");
    xSemaphoreGive( dbSem );
  }
  #endif
  
  
  memset(rbuf, 0, RBUF_SIZE);
  memset(sbuf, 0, SBUF_SIZE);
  
  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    SERIAL.println("zerod buffers");
    xSemaphoreGive( dbSem );
  }
  #endif
  
  char buff[40];
  
  while(1) {
    // check the data source queue and see if there is something to send
    // in capsule firmware this will need to peek at many other queus to aggregate the needed data,
    // here we just spoof values in the struct and send it
    // only send once per second
    
    if( state == 1 ){
      int len = sprintf(sbuf, "AT+ADDRESS=1\r\n");
      RADIO_SERIAL.write(sbuf, len);
      state = 2;
    }
    else if( state == 3 ){
      int len = sprintf(sbuf, "AT+NETWORKID=1\r\n");
      RADIO_SERIAL.write(sbuf, len);
      state = 4;
    }
    else if( state == 5 ){
      int len = sprintf(sbuf, "AT+PARAMETER=12,4,1,7\r\n"); // recommended for more than 3km
      RADIO_SERIAL.write(sbuf, len);
      state = 6;
    }
    
    if( xTaskGetTickCount() - lastSendTime > TLM_SEND_PERIOD && state == 7){
      
      dataOut.t = xTaskGetTickCount();
      dataOut.lat = 37.1838432;
      dataOut.lon = -84.824233;
      dataOut.vel = 1029.235;
      dataOut.alt_gps = 1200.92;
      dataOut.alt_bar = 1190.32;
      dataOut.barp = 995.127;
      dataOut.tmp = 23.81;
      dataOut.irsig = 2;
      dataOut.pardep = 1;
      dataOut.thread_status[0] = 0;
      dataOut.thread_status[1] = 0;
      dataOut.thread_status[2] = 0;
      dataOut.thread_status[3] = 1;
      dataOut.thread_status[4] = 0;
      dataOut.thread_status[5] = 0;
      dataOut.thread_status[6] = 0;
      dataOut.thread_status[7] = 0;
      dataOut.thread_status[8] = 0;
      dataOut.thread_status[9] = 0;      
      
      const int dataSize = sizeof(tlm_t);
      sprintf(sbuf, "AT+SEND=2,%d,", dataSize);
      int pre = strlen(sbuf);
      
      // TODO: can we embed binary data ?????
      // now copy binary data from struct to send buffer
      memcpy(&sbuf[pre], (void*)&dataOut, dataSize);
      
      sbuf[pre+dataSize] = '\r';
      sbuf[pre+dataSize+1] = '\n';
      sbuf[pre+dataSize+2] = 0;
      
      // send to lora module
      RADIO_SERIAL.write(sbuf, pre+dataSize+2);
      //RADIO_SERIAL.write(sbuf, pre);
      
      SERIAL.print("sending: ");
      SERIAL.write(sbuf, pre+dataSize+3);
      SERIAL.println("DONE");
      
      // update timestamp, not including data fill time from above lines
      lastSendTime = dataOut.t; 
      
      // go to state 8 so that we wait for a response
      state = 8;
    }
    
  
    // handle incoming message from LORA radio
    // AT message from module
    bool eol = false;
    int pos = 0;
    bool timeout = false;
    unsigned long timeoutStart = 0;
    if( RADIO_SERIAL.peek() == '+' ){
      
      SERIAL.println("saw a +");
    
      unsigned long timeoutStart = xTaskGetTickCount();
      while(!eol && !timeout && pos < RBUF_SIZE-1) {
        if( RADIO_SERIAL.available() ){
          rbuf[pos] = RADIO_SERIAL.read();
          if( pos > 1 ){
            if( rbuf[pos]=='\n' && rbuf[pos-1]=='\r' ){
              memset(&rbuf[pos+1], 0, RBUF_SIZE-(pos+1));
              eol = true;
            }
          }
          
          if( pos++ >= RBUF_SIZE ){
            break;
          }
        }
        if( xTaskGetTickCount() - timeoutStart > RX_TIMEOUT_PERIOD ){
          memset(rbuf, 0, RBUF_SIZE);
          timeout = true;
        }
      }
    
    
      if( timeout ){
        #ifdef DEBUG
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          SERIAL.println("Radio rx timed out");
          xSemaphoreGive( dbSem );
        }
        #endif
      } else if (!timeout && eol) {
        #ifdef DEBUG
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          SERIAL.println("Radio got packet!");
          xSemaphoreGive( dbSem );
        }
        #endif
      } else if( !timeout && !eol) {
        #ifdef DEBUG
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          SERIAL.println("Radio receive buffer overrun!");
          xSemaphoreGive( dbSem );
        }
        #endif
      }
      
      // if first byte is non-zero then we received data ( also check timeout and eol vars to be safe)
      // now process the data line received
      if( (rbuf[0] == '+') && !timeout && eol){
        
        
        int eqpos = 1;
        while( rbuf[eqpos] != '=' &&  eqpos < pos){
          eqpos++;
        }
        if( eqpos == pos ){
          #ifdef DEBUG
          if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
            SERIAL.print("We think we got a +READY or +OK message, we actually got: ");
            SERIAL.write(rbuf, pos);
            SERIAL.println();
            xSemaphoreGive( dbSem );
          }
          #endif
          
          if( state < 7 ){
            state ++;
            if( state == 7 ){
              #ifdef DEBUG
              if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
                SERIAL.println("STATE = 7, successfully configured radio!");
                xSemaphoreGive( dbSem );
              }
              #endif
            }
          } else if( state > 7 ){
            #ifdef DEBUG
            state = 7;
            if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
              SERIAL.println("STATE was 8, received +OK from a data send operation!");
              xSemaphoreGive( dbSem );
            }
            #endif 
          }
          
        } else {
          // found an '=', parse rest of message
          #ifdef DEBUG
          if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
            SERIAL.print("We think we got a message with an '=' in it, we actually got: ");
            SERIAL.write(rbuf, pos);
            xSemaphoreGive( dbSem );
          }
          #endif
          
          //
          // TODO: parse the packet and fill the rxtlm_t struct with data
          //       and send to the logging and display queues
          // 
          
        }
      }
    } 
    
    taskYIELD();
  }
  
  vTaskDelete( NULL );  
}

void setup() {
  SERIAL.begin(115200);
  delay(10);
  RADIO_SERIAL.begin(115200);
  delay(10);
  
  delay(4000);
  
  pinPeripheral(A1, PIO_SERCOM_ALT);
  pinPeripheral(A4, PIO_SERCOM_ALT);
  
  pinMode(A0, OUTPUT);
  
  SERIAL.println("Starting...");

  // CREATE RTOS QUEUES 
  // temperature data queue
  qRadData = xQueueCreate( 5, sizeof( struct tlm_t ) );
  if( qRadData == NULL ) {
    /* Queue was not created and must not be used. */
    #if DEBUG
    SERIAL.println("Failed to create qRadData queue");
    #endif
  }
  
  SERIAL.println("Created queues...");
  
  // setup debug serial log semaphore
  if ( dbSem == NULL ) {
    dbSem = xSemaphoreCreateMutex();  // create mutex
    if ( ( dbSem ) != NULL )
      xSemaphoreGive( ( dbSem ) );  // make available
  }
  
  SERIAL.println("Created semaphores...");
  
  /**************
  * CREATE TASKS
  **************/
  xTaskCreate(radThread, "Radio Control", 2048, NULL, tskIDLE_PRIORITY + 2, &Handle_radTask);
  //xTaskCreate(taskMonitor, "Task Monitor", 256, NULL, tskIDLE_PRIORITY + 4, &Handle_monitorTask);

  SERIAL.println("Starting scheduler...");

  // start the scheduler
  vTaskStartScheduler();

  // error scheduler failed to start
  while(1)
  {
	  SERIAL.println("Scheduler Failed! \n");
	  delay(1000);
  }
  
}

void loop() {
  // tasks!
}
