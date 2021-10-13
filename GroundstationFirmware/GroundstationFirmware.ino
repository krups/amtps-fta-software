// Groundstation firmware for AMTPS-FTA project
// Matt Ruffner 2021
// This software is meant to run on the receiving end of debug radio 
// and interpret in flight telemetry from the capsule, display on OLED 
// and log to SD card

#include <SPI.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <FreeRTOS_SAMD21.h>
#include <SerialTransfer.h>
#include <semphr.h>
#include <SD.h>

#define DEBUG 1

#define MAX_PAGES 3 // max pages of info on screen to scroll through

#include "include/delay_helpers.h" // rtos delay helpers
#include "include/config.h"        // project wide defs
#include "include/packet.h"        // data packet defs
#include "pins.h"                  // groundstation system pinouts


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an display
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);


// freertos task handles
TaskHandle_t Handle_logTask;
TaskHandle_t Handle_radTask;
TaskHandle_t Handle_lcdTask;
TaskHandle_t Handle_monitorTask;

// freeRTOS semaphores
// shouldnt need any because the radio uses uart, the sd card uses spi and the display uses i2c
//SemaphoreHandle_t i2csem; // data transmit to CDH semaphore
SemaphoreHandle_t dbSem; // serial debug logging (Serial)
//SemaphoreHandle_t i2c1Sem; // i2c port 1 access semaphore

// freeRTOS queues
QueueHandle_t qLogData; // data queue for sending recvd radio data to the sd card to be logged
QueueHandle_t qDisData; // data queue for sending recvd radio data to the display

#define SERIAL Serial
#define RADIO_SERIAL Serial1

/**********************************************************************************
 * SD logging task
*/
static void logThread( void *pvParameters )
{
  bool ready = false;
  rxtlm_t tlmData;
  File logfile; 
  
  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    SERIAL.println("SD Logging thread started");
    xSemaphoreGive( dbSem );
  }
  #endif
  
  String filename = "TLM00.txt";
  
  // INIT CARD
  while (!SD.begin(PIN_SD_CS)) {
    #if DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      SERIAL.println("ERROR: sd logging thread couldn't init sd card");
      xSemaphoreGive( dbSem );
    }
    #endif
    ready = false;
    myDelayMs(1000);
  } 
  
  #if DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    SERIAL.println("SD CARD INIT OK");
    xSemaphoreGive( dbSem );
  }
  #endif

  // if we make it to hear, we are ready
  ready = true;
  
  
  // CREATE UNIQUE FILE NAME
  for( uint8_t i=0; i < 100; i++) {
    filename[3] = '0' + i/10;
    filename[4] = '0' + i%10;
    if (! SD.exists(filename)) {
      break;
    }
  }
  
  while(1) {
    
    // check for log packet in queue and log to sd
    // need to format data in one row with correct column names
    
    // if telem queue has been created successfully
    if( qLogData != NULL ) {
      if( xQueueReceive( qLogData, &tlmData, portMAX_DELAY ) == pdPASS) {
        #ifdef DEBUG_VERBOSE
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          SERIAL.println("sd log: received telem data!");
          xSemaphoreGive( dbSem );
        }
        #endif
        
        // now we have telem data in tlmData
        
        // try to open logfile
        logfile = SD.open(filename, FILE_WRITE);
      
        // logfile no good
        if( ! logfile ) {
          #if DEBUG
          if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
            SERIAL.print("ERROR: sd logging thread couldn't open logfile for writing: ");
            SERIAL.println(filename);
            xSemaphoreGive( dbSem );
          }
          #endif
        }       
        else { // LOGFILE OPEN!
          
          // write telem data to file
          logfile.print(tlmData.tlm.t);
          logfile.print(", ");
          logfile.print(tlmData.tlm.lat, 6);
          logfile.print(", ");
          logfile.print(tlmData.tlm.lon, 6);
          logfile.print(", ");
          logfile.print(tlmData.tlm.vel, 2);
          logfile.print(", ");
          logfile.print(tlmData.tlm.alt_gps, 2);
          logfile.print(", ");
          logfile.print(tlmData.tlm.alt_bar, 2);
          logfile.print(", ");
          logfile.print(tlmData.tlm.barp, 3);
          logfile.print(", ");
          logfile.print(tlmData.tlm.tmp, 2);
          logfile.print(", ");
          logfile.print(tlmData.tlm.irsig);
          logfile.print(", ");
          logfile.print(tlmData.tlm.pardep);
          // TODO: log status codes for threads
          logfile.print(", "); logfile.print(tlmData.rssi);
          logfile.print(", "); logfile.println(tlmData.snr);
        }
        
        logfile.close();
      }
    }
    
    taskYIELD();
  }
  
  vTaskDelete( NULL );  
}

/**********************************************************************************
 * Oled screen task
*/
static void oledThread( void *pvParameters )
{
  rxtlm_t tlmData;
  
  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    SERIAL.println("OLED thread started");
    xSemaphoreGive( dbSem );
  }
  #endif
  
  
  u8g2.begin();

  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    SERIAL.println("OLED initialized");
    xSemaphoreGive( dbSem );
  }
  #endif
  
  u8g2.clearBuffer();					// clear the internal memory
  u8g2.setFontMode(1);
  u8g2.setFont(u8g2_font_6x10_mr);	// choose a suitable font
  u8g2.setCursor(0,10);				// set write position
  u8g2.print("Hello World!");			// write something to the internal memory
  u8g2.sendBuffer();					// transfer internal memory to the display
  
  
  myDelayMs(2000); // Pause for 2 seconds
  
  uint8_t page = 0;
  
  while(1) {
    SERIAL.println("oled loop");
    // main telem display loop
        
    // check data queue for new data to display
    if( qDisData != NULL ) {
      if( xQueueReceive( qDisData, &tlmData, 500 ) == pdPASS) {
        #ifdef DEBUG_VERBOSE
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          SERIAL.println("oled thrd: received telem data!");
          xSemaphoreGive( dbSem );
        }
        #endif
        
      }
        
    }
    
    if( digitalRead(A3) == LOW ){
      page = (page + 1) % MAX_PAGES;
      SERIAL.println("page incremented");
    }
    if( digitalRead(A2) == LOW ){
      page = (page - 1) % MAX_PAGES;
      SERIAL.println("serial decremented");
    }
    
    // put corresponding data on each page
    if( page == 0 ){
      
      u8g2.clearBuffer();
      u8g2.setFontMode(1);
      u8g2.setFont(u8g2_font_6x10_mr);	// choose a suitable font
      u8g2.setCursor(0,10);
      u8g2.print(tlmData.tlm.lat, 5); 
      u8g2.print(","); u8g2.println(tlmData.tlm.lon, 5);
      u8g2.setCursor(0,20);
      u8g2.print("Vel: "); u8g2.println(tlmData.tlm.vel, 1); u8g2.print(" m/s");
      u8g2.setCursor(0,30);
      u8g2.print("G alt: "); u8g2.println(tlmData.tlm.alt_gps, 1); u8g2.print(" m");
      u8g2.setCursor(0,40);
      u8g2.print("B alt: "); u8g2.println(tlmData.tlm.alt_bar, 1); u8g2.print(" m");
      u8g2.setCursor(0,50);
      u8g2.print("Prs: "); u8g2.println(tlmData.tlm.barp, 2); u8g2.print(" hPa");
      u8g2.setCursor(0,60);
      u8g2.print("Tmp: "); u8g2.println(tlmData.tlm.tmp, 1); u8g2.print(" C");
      u8g2.sendBuffer();
      
      
    } else if( page == 1 ){
      u8g2.clearBuffer();
      u8g2.setFontMode(1);
      u8g2.setFont(u8g2_font_6x10_mr);	// choose a suitable font
      u8g2.setCursor(0,10);
      u8g2.print("Ir. sig: "); u8g2.println(tlmData.tlm.irsig);
      u8g2.setCursor(0,20);
      u8g2.print("Par. dep: "); u8g2.println(tlmData.tlm.pardep);
      u8g2.sendBuffer();
    } else if( page == 2 ){
      //show thread statuses
      //tlmData.tlm.thread_status
    }
    
    
    myDelayMs(500);
  }
  
  vTaskDelete( NULL );  
}

/**********************************************************************************
 *  Radio task
*/
#define RBUF_SIZE 300
#define SBUF_SIZE 240
static void radThread( void *pvParameters )
{
  uint8_t rbuf[RBUF_SIZE];
  tlm_t dat;
  rxtlm_t outDat;
  char sbuf[SBUF_SIZE];

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
  
  
  pinMode(PIN_LORA_RST, OUTPUT); // nreset of lora
  digitalWrite(PIN_LORA_RST, LOW);
  myDelayMs(100);
  digitalWrite(PIN_LORA_RST, HIGH);
  
  
  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    SERIAL.println("Reset radio");
    xSemaphoreGive( dbSem );
  }
  #endif
  
  memset(rbuf, 0, RBUF_SIZE);
  memset(sbuf, 0, SBUF_SIZE);
  
  RADIO_SERIAL.print("AT+RESET\r\n");
  
  while(1) {
  
    if( state == 1 ){
      int len = sprintf(sbuf, "AT+ADDRESS=2\r\n");
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
  
    // handle incoming message from LORA radio
    // AT message from module
    bool eol = false;
    int pos = 0;
    bool timeout = false;
    unsigned long timeoutStart = 0;
    if( RADIO_SERIAL.peek() == '+' ){
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
        if( xTaskGetTickCount() - timeoutStart > 1000 ){
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
            xSemaphoreGive( dbSem );
          }
          #endif
          
          if( state != 7 ){
            state ++;
            if( state == 7 ){
              #ifdef DEBUG
              if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
                SERIAL.print("STATE = 7! successfully configured radio!");
                xSemaphoreGive( dbSem );
              }
              #endif
            }
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
          
          // check if its a receive message
          if( rbuf[0]=='+' &&
              rbuf[1]=='R' &&
              rbuf[2]=='C' &&
              rbuf[3]=='V'){
            
            // parse data
            memcpy((void*)&dat, &rbuf[10], 48);
            outDat.tlm = dat;
            outDat.rssi = -19;
            outDat.snr = 30;
            
            if( xQueueSend( qDisData, ( void * ) &outDat, ( TickType_t ) 200 ) != pdTRUE ) {
              /* Failed to post the message, even after 100 ticks. */
              #ifdef DEBUG
              if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
                SERIAL.println("ERROR: failed to put telem data into queue");
                xSemaphoreGive( dbSem );
              }
              #endif
            }
            
            
            SERIAL.print("GOT DATA: ");
            SERIAL.write(rbuf, pos);
            SERIAL.println("DONE");
            
          }
          
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
  
  delay(3000);
  
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);
  
  SERIAL.println("Starting...");

  // CREATE RTOS QUEUES 
  // temperature data queue
  qDisData = xQueueCreate( 2, sizeof( struct rxtlm_t ) );
  if( qDisData == NULL ) {
    /* Queue was not created and must not be used. */
    #if DEBUG
    SERIAL.println("Failed to create qDisData queue");
    #endif
  }
  // pressure data queue
  qLogData = xQueueCreate( 2, sizeof( struct rxtlm_t ) );
  if( qLogData == NULL ) {
    /* Queue was not created and must not be used. */
    #if DEBUG
    SERIAL.println("Failed to create qLogData queue");
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
  //xTaskCreate(logThread, "SD Logging", 1024, NULL, tskIDLE_PRIORITY + 2, &Handle_logTask);
  xTaskCreate(radThread, "Radio Control", 800, NULL, tskIDLE_PRIORITY + 2, &Handle_radTask);
  xTaskCreate(oledThread, "OLED Control", 1024, NULL, tskIDLE_PRIORITY + 3, &Handle_lcdTask);
  
  //xTaskCreate(taskMonitor, "Task Monitor", 256, NULL, tskIDLE_PRIORITY + 4, &Handle_monitorTask);

  SERIAL.println("Created tasks...");
  
  delay(100);
  
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
