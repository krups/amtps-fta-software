// Groundstation firmware for AMTPS-FTA project
// Matt Ruffner 2021
// This software is meant to run on the receiving end of debug radio 
// and interpret in flight telemetry from the capsule, display on OLED 
// and log to SD card

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <FreeRTOS_SAMD21.h>
#include <SerialTransfer.h>
#include <semphr.h>
#include <SD.h>


#include "include/delay_helpers.h" // rtos delay helpers
#include "include/config.h"        // project wide defs
#include "include/packet.h"        // data packet defs
#include "pins.h"                  // groundstation system pinouts


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// freertos task handles
TaskHandle_t Handle_logTask;
TaskHandle_t Handle_radTask;
TaskHandle_t Handle_lcdTask;
TaskHandle_t Handle_monitorTask;

// freeRTOS semaphores
// shouldnt need any because the radio uses uart, the sd card uses spi and the display uses i2c
//SemaphoreHandle_t i2csem; // data transmit to CDH semaphore
//SemaphoreHandle_t dbSem; // serial debug logging (Serial)
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
  
  
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  while(!display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) { // Address 0x3D for 128x64
    
    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      SERIAL.println("Failed to init OLED, retrying");
      xSemaphoreGive( dbSem );
    }
    #endif
    
    myDelayMs(1000);
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.clearDisplay();
  display.setTextSize(2);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.print("Starting...");
  display.display();
  delay(2000); // Pause for 2 seconds
  
  int page = 1;
  
  while(1) {
  
    // main telem display loop
        
    // check data queue for new data to display
    if( qDisData != NULL ) {
      if( xQueueReceive( qLogData, &tlmData, portMAX_DELAY ) == pdPASS) {
        #ifdef DEBUG_VERBOSE
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          SERIAL.println("oled thrd: received telem data!");
          xSemaphoreGive( dbSem );
        }
        #endif
        
      }
        
    }
    
    // put corresponding data on each page
    if( page == 1 ){
      
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0,0);
      display.print("Loc: "); display.print(tlmData.tlm.lat, 4); 
      display.print(", "); display.println(tlmData.tlm.lon, 4);
      display.print("Vel (m/s): "); display.println(tlmData.tlm.vel, 1);
      display.print("GPS alt (m): "); display.println(tlmData.tlm.alt_gps, 1);
      display.print("Bar alt (m): "); display.println(tlmData.tlm.alt_bar, 1);
      display.print("Int.prs. (hPa): "); display.println(tlmData.tlm.barp, 2);
      display.print("Int.tmp. (C): "); display.println(tlmData.tlm.tmp, 1);
      display.print("Ir. sig: "); display.println(tlmData.tlm.irsig, 4);
      display.print("Par. dep: "); display.println(tlmData.tlm.pardep);
    } else if( page == 2 ){
      //show thread statuses
    }
    
    
    myDelayMs(500);
  }
  
  vTaskDelete( NULL );  
}

/**********************************************************************************
 *  Radio task
*/
#define RBUF_SIZE 300
static void radThread( void *pvParameters )
{
  uint8_t rbuf[RBUF_SIZE];
  
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
  myDelayMs(900);
  
  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    SERIAL.println("Reset radio");
    xSemaphoreGive( dbSem );
  }
  #endif
  
  memset(rbuf, 0, RBUF_SIZE);
  
  while(1) {
  
    // handle incoming message from LORA radio
    // AT message from module
    bool eol = false;
    int pos = 0;
    bool timeout = false;
    if( RADIO_SERIAL.peek() == '+' ){
      unsigned long timeout = xTaskGetTickCount();
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
        if( xTaskGetTickCount() - timeout > 1000 ){
          memset(rbuf, 0, RBUF_SIZE);
          timeout = true;
        }
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
          SERIAL.print("We think we got a +READY message, we actually got: ");
          SERIAL.println(rbuf);
          xSemaphoreGive( dbSem );
        }
        #endif
      } else {
        // found an '=', parse rest of message
        #ifdef DEBUG
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          SERIAL.print("We think we got a message with an '=' in it, we actually got: ");
          SERIAL.println(rbuf);
          xSemaphoreGive( dbSem );
        }
        #endif
        
        //
        // TODO: parse the packet and fill the rxtlm_t struct with data
        //       and send to the logging and display queues
        // 
        
      }
      
    } else {
      // shouldn't happen
      #ifdef DEBUG
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
        SERIAL.println("ERROR: we think we got a packet but the first byte of the buffer is 0!");
        xSemaphoreGive( dbSem );
      }
      #endif
    }
    
  }
  
  vTaskDelete( NULL );  
}

void setup() {
  SERIAL.begin(115200);
  delay(10);
  RADIO_SERIAL.begin(115200);
  delay(10);
  
  delay(1000);
  
  SERIAL.println("Starting...");

  // CREATE RTOS QUEUES 
  // temperature data queue
  qDisData = xQueueCreate( 5, sizeof( struct rxtlm_t ) );
  if( qDisData == NULL ) {
    /* Queue was not created and must not be used. */
    #if DEBUG
    SERIAL.println("Failed to create qDisData queue");
    #endif
  }
  // pressure data queue
  qLogData = xQueueCreate( 5, sizeof( struct rxtlm_t ) );
  if( qLogData == NULL ) {
    /* Queue was not created and must not be used. */
    #if DEBUG
    SERIAL.println("Failed to create qLogData queue");
    #endif
  }
  
  SERIAL.println("Created queues...");
  
  /**************
  * CREATE TASKS
  **************/
  xTaskCreate(logThread, "SD Logging", 1024, NULL, tskIDLE_PRIORITY + 2, &Handle_logTask);
  xTaskCreate(oledThread, "OLED Control", 1024, NULL, tskIDLE_PRIORITY + 2, &Handle_lcdTask);
  xTaskCreate(radThread, "Radio Control", 1024, NULL, tskIDLE_PRIORITY + 2, &Handle_radTask);
  //xTaskCreate(taskMonitor, "Task Monitor", 256, NULL, tskIDLE_PRIORITY + 4, &Handle_monitorTask);

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
