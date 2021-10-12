// Groundstation firmware for AMTPS-FTA project
// Matt Ruffner 2021
// This software is meant to run on the receiving end of debug radio 
// and interpret in flight telemetry from the capsule, display on OLED 
// and log to SD card

#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <FreeRTOS_SAMD21.h>
#include <SerialTransfer.h>
#include <semphr.h>

#include "include/delay_helpers.h" // rtos delay helpers
#include "include/config.h"        // project wide defs
#include "include/packet.h"        // data packet defs
#include "pins.h"                  // groundstation system pinouts


#define RADIO_SERIAL Serial1

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


/**********************************************************************************
 * SD logging task
*/
static void logThread( void *pvParameters )
{
    
  
  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("SD Logging thread started");
    xSemaphoreGive( dbSem );
  }
  #endif
  
  while(1) {
    
  }
  
  vTaskDelete( NULL );  
}

/**********************************************************************************
 * Oled screen task
*/
static void oledThread( void *pvParameters )
{
    
  
  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("OLED thread started");
    xSemaphoreGive( dbSem );
  }
  #endif
  
  
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  while(!display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) { // Address 0x3D for 128x64
    
    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      Serial.println("OLED thread started");
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
    // put corresponding data on each page
    
    
    if( page == 1 ){
    
    
    } else if( page == 2 ){
    
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
    Serial.println("Radio thread started");
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
    Serial.println("Reset radio");
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
        Serial.println("Radio rx timed out");
        xSemaphoreGive( dbSem );
      }
      #endif
    } else if (!timeout && eol) {
      #ifdef DEBUG
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
        Serial.println("Radio got packet!");
        xSemaphoreGive( dbSem );
      }
      #endif
    } else if( !timeout && !eol) {
      #ifdef DEBUG
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
        Serial.println("Radio receive buffer overrun!");
        xSemaphoreGive( dbSem );
      }
      #endif
    }
    
    // if first byte is non-zero then we received data ( also check timeout and eol vars to be safe)
    // now process the data line received
    if( (rbuf[0] == '+') && !timeout && eol){
      
      /*
      int eqpos = 1;
      while( rbuf[eqpos] != '=' &&  eqpos < pos){
        eqpos++;
      }
      if( eqpos == pos ){
        // '=' not found, must be +READY message
      } else {
        // found an '=', parse rest of message
        
      }
      */
      
      #ifdef DEBUG
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
        Serial.print("Got message: ");
        Serial.println(rbuf);
        xSemaphoreGive( dbSem );
      }
      #endif
      
      
      
      
    } else {
      // shouldn't happen
      #ifdef DEBUG
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
        Serial.println("ERROR: we think we got a packet but the first byte of the buffer is 0!");
        xSemaphoreGive( dbSem );
      }
      #endif
    }
    
  }
  
  vTaskDelete( NULL );  
}

void setup() {
}

void loop() {
}
