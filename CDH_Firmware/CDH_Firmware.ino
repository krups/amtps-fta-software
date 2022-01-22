/* 
 * AMPTS FTA Firmware
 * Command and Data Handling (CDH) subsystem
 * 
 * Matt Ruffner, University of Kentucky Fall 2021
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <Adafruit_MPL3115A2.h>
#include <ArduinoNmeaParser.h>
#include <FreeRTOS_SAMD51.h>
#include <SerialTransfer.h>
#include <IridiumSBD.h>
#include <semphr.h>
#include <SD.h>
#include <SPI.h>
#include <Servo.h>
#include "wiring_private.h"

#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define CO2SERVO_POS_ACT  17
#define CO2SERVO_POS_HOME 150

Servo co2servo;

#define DEBUG 1 // usb serial debug switch
#ifdef DEBUG
  //#define DEBUG_GPS 1 // print raw gga to serial 
  //#define DEBUG_QUEUE 1 // print info on log queue operations
  //#define DEBUG_VERBOSE 1
  //#define DEBUG_BARO 1
  #define DEBUG_IRD 1
  //#define DEBUG_LOG 1
  #define DEBUG_PAR 1
  //#define DEBUG_RAD 1
#endif

bool sendPackets = 0;

//#include <Mahony_DPEng.h>
//#include <Madgwick_DPEng.h>

#include "include/delay_helpers.h" // rtos delay helpers
#include "include/config.h"        // project wide defs
#include "include/packet.h"        // packet definitions
#include "pins.h"                  // CDH system pinouts

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


// debug serial
#define SERIAL      Serial  // debug serial (USB) all uses should be conditional on DEBUG define
#define SERIAL_TPM  Serial1 // to TPM subsystem
#define SERIAL_GPS  Serial3 // to gps
#define SERIAL_IRD  Serial2 // to iridium modem
#define SERIAL_LOR  Serial4 // to telemetry radio

// Serial transfer object for receiving data from TPM processor
SerialTransfer myTransfer;

// High G accel sensor object
H3LIS100 accel = H3LIS100(12345);

// capsule internal barometric pressure object
// and struct for passing data through queue
Adafruit_MPL3115A2 baro;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

// freertos task handles
TaskHandle_t Handle_tpmTask; // data receive from TPM subsystem task
TaskHandle_t Handle_logTask; // sd card logging task
TaskHandle_t Handle_gpsTask; // gps data receive task
TaskHandle_t Handle_irdTask; // iridium transmission task
TaskHandle_t Handle_parTask; // parachute deployment task
TaskHandle_t Handle_barTask; // barometric sensor task
TaskHandle_t Handle_radTask; // telem radio task handle
TaskHandle_t Handle_monitorTask; // debug running task stats over uart task

// freeRTOS queues
QueueHandle_t qTmpData; // temperature or heat flux data to be logged
QueueHandle_t qPrsData; // pressure data to be logged
QueueHandle_t qGgaData; // GGA GPS fix data to be logged
QueueHandle_t qRmcData; // RMC GPS data to be logged
QueueHandle_t qBarData; // barometric pressure data

// freeRTOS semaphores
SemaphoreHandle_t tpmSerSem; // data from CDH semaphore
SemaphoreHandle_t dbSem; // serial debug logging (Serial)
SemaphoreHandle_t i2c1Sem; // i2c port 1 access semaphore
SemaphoreHandle_t gpsSerSem; // gps serial port acces
SemaphoreHandle_t irdSerSem; // iridium serial semaphore
SemaphoreHandle_t depSem; // deployment status protector
SemaphoreHandle_t sigSem; // iridium signal protector

bool globalDeploy = false;
bool irSig = 0;

// GPS update callbacks
void onRmcUpdate(nmea::RmcData const);
void onGgaUpdate(nmea::GgaData const);

// GPS parser object
ArduinoNmeaParser parser(onRmcUpdate, onGgaUpdate);

// IRIDIUM MODEM OBJECT
IridiumSBD modem(SERIAL_IRD);

void onRmcUpdate(nmea::RmcData const rmc)
{
  #ifdef DEBUG_GPS
  //if (rmc.is_valid) {
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 200 ) == pdTRUE ) {
      writeRmc(rmc, SERIAL);  
      xSemaphoreGive( dbSem );
    }
  //}
  #endif

  if( xQueueSend( qRmcData, ( void * ) &rmc, ( TickType_t ) 200 ) != pdTRUE ) {
    /* Failed to post the message, even after 100 ticks. */
    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
      SERIAL.println("ERROR: failed to put rmc data into queue");
      xSemaphoreGive( dbSem );
    }
    #endif
  }
}

void writeRmc(nmea::RmcData const rmc, Stream &pipe)
{
  pipe.print("RMC ");

  if      (rmc.source == nmea::RmcSource::GPS)     pipe.print("GPS");
  else if (rmc.source == nmea::RmcSource::GLONASS) pipe.print("GLONASS");
  else if (rmc.source == nmea::RmcSource::Galileo) pipe.print("Galileo");
  else if (rmc.source == nmea::RmcSource::GNSS)    pipe.print("GNSS");

  pipe.print(" ");
  pipe.print(rmc.time_utc.hour);
  pipe.print(":");
  pipe.print(rmc.time_utc.minute);
  pipe.print(":");
  pipe.print(rmc.time_utc.second);
  pipe.print(".");
  pipe.print(rmc.time_utc.microsecond);

  if (rmc.is_valid)
  {
    pipe.print(" : LON ");
    pipe.print(rmc.longitude, 5);
    pipe.print(" ° | LAT ");
    pipe.print(rmc.latitude, 5);
    pipe.print(" ° | VEL ");
    pipe.print(rmc.speed);
    pipe.print(" m/s | HEADING ");
    pipe.print(rmc.course);
    pipe.print(" °");
  }

  pipe.println();
}

// write formatted gps string to stream i.e. file or serial port
void writeGga(nmea::GgaData const gga, Stream &pipe)
{
  pipe.print("GGA ");

  if      (gga.source == nmea::GgaSource::GPS)     pipe.print("GPS");
  else if (gga.source == nmea::GgaSource::GLONASS) pipe.print("GLONASS");
  else if (gga.source == nmea::GgaSource::Galileo) pipe.print("Galileo");
  else if (gga.source == nmea::GgaSource::GNSS)    pipe.print("GNSS");

  pipe.print(" ");
  pipe.print(gga.time_utc.hour);
  pipe.print(":");
  pipe.print(gga.time_utc.minute);
  pipe.print(":");
  pipe.print(gga.time_utc.second);
  pipe.print(".");
  pipe.print(gga.time_utc.microsecond);

  if (gga.fix_quality != nmea::FixQuality::Invalid)
  {
    pipe.print(" : LON ");
    pipe.print(gga.longitude, 5);
    pipe.print(" ° | LAT ");
    pipe.print(gga.latitude, 5);
    pipe.print(" ° | Num Sat. ");
    pipe.print(gga.num_satellites);
    pipe.print(" | HDOP =  ");
    pipe.print(gga.hdop);
    pipe.print(" m | Altitude ");
    pipe.print(gga.altitude);
    pipe.print(" m | Geoidal Separation ");
    pipe.print(gga.geoidal_separation);
    pipe.print(" m");
  }

  pipe.println();
}

// hopefully not called from an interrup by the GPS parser library
// this function takes the gps fix data and pushes it into the logging queue
void onGgaUpdate(nmea::GgaData const gga)
{
  #ifdef DEBUG_GPS
  if (gga.fix_quality != nmea::FixQuality::Invalid) {
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 200 ) == pdTRUE ) {
      writeGga(gga, SERIAL);
      xSemaphoreGive( dbSem );
    }
  }
  #endif

  if( xQueueSend( qGgaData, ( void * ) &gga, ( TickType_t ) 200 ) != pdTRUE ) {
    /* Failed to post the message, even after 100 ticks. */
    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
      SERIAL.println("ERROR: failed to put gga data into queue");
      xSemaphoreGive( dbSem );
    }
    #endif
  }
}

/**********************************************************************************
/**********************************************************************************
/**********************************************************************************
/**********************************************************************************
 * barometric pressure monitoring thread
*/
static void barThread( void *pvParameters )
{
  bool init = false;
  bar_t sensorData;

  #ifdef DEBUG_BARO
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("Barometer thread started");
    xSemaphoreGive( dbSem );
  }
  #endif

  while (!init) {
    myDelayMs(1000);
    
    if ( xSemaphoreTake( i2c1Sem, ( TickType_t ) 100 ) == pdTRUE ) {
      // initialize barometer
      if (baro.begin()) {
        init = true;
        baro.setSeaPressure(1013.26);

      }      
      xSemaphoreGive( i2c1Sem );
    }
  }
  
  #ifdef DEBUG_BARO
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("Barometer initialized!");
    xSemaphoreGive( dbSem );
  }
  #endif
    
  while(1) {
  
    if ( xSemaphoreTake( i2c1Sem, ( TickType_t ) 100 ) == pdTRUE ) {
      sensorData.prs = baro.getPressure();
      sensorData.alt = baro.getAltitude();
      sensorData.tmp = baro.getTemperature();
      xSemaphoreGive( i2c1Sem );
    }
  
    if( xQueueSend( qBarData, ( void * ) &sensorData, ( TickType_t ) 100 ) != pdTRUE ) {
        /* Failed to post the message, even after 100 ticks. */
        #ifdef DEBUG_BARO
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
          SERIAL.println("BAR: failed to put sensor data into queue");
          xSemaphoreGive( dbSem );
        }
        #endif
      } else {
        #ifdef DEBUG_QUEUE
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
          SERIAL.println("BAR: added prs/alt/tmp packet to queue!");
          xSemaphoreGive( dbSem );
        }
        #endif
      }
    
    #ifdef DEBUG_BARO
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      Serial.print("pressure = "); Serial.print(sensorData.prs); Serial.println(" hPa");
      Serial.print("altitude = "); Serial.print(sensorData.alt); Serial.println(" m");
      Serial.print("temperature = "); Serial.print(sensorData.tmp); Serial.println(" C");
      
      xSemaphoreGive( dbSem );
    }
    #endif

    myDelayMs(1000);
  }
}



/**********************************************************************************
/**********************************************************************************
/**********************************************************************************
/**********************************************************************************
 * GPS serial monitoring thread
*/
static void gpsThread( void *pvParameters )
{
  #ifdef DEBUG_GPS
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("GPS thread started");
    xSemaphoreGive( dbSem );
  }
  #endif
  
  while(1) {
    if ( xSemaphoreTake( gpsSerSem, ( TickType_t ) 100 ) == pdTRUE ) {
      while (SERIAL_GPS.available()) {
        parser.encode((char)SERIAL_GPS.read());
      }
      xSemaphoreGive( gpsSerSem );
    }
    
    taskYIELD();
  }
  
  vTaskDelete( NULL );  
}


/**********************************************************************************
/**********************************************************************************
/**********************************************************************************
/**********************************************************************************
 * Iridium thread
*/
static void irdThread( void *pvParameters )
{
  bool fix_valid = false;
  char buf[330];
  int mSq = 0, irerr; // signal quality, modem operation return code
  unsigned long lastSignalCheck = 0, lastPacketSend = 0;
  bool gpsAvailable = false;
  nmea::RmcData rmcData;
  
  sprintf(buf, "No GPS fix yet.");
  
  #ifdef DEBUG_IRD
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("Iridium thread started");
    xSemaphoreGive( dbSem );
  }
  #endif

  // enable modem
  digitalWrite(PIN_IRIDIUM_EN, HIGH);
  
  myDelayMs(5000); // give modem time to power up
    
    
  #ifdef DEBUG_IRD
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("Iridium thread: trying to start modem!...");
    xSemaphoreGive( dbSem );
  }
  #endif
    
  // init the iridium library and check signal strength
  irerr = modem.begin();
  
  #ifdef DEBUG_IRD
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    SERIAL.println("IRIDIUM: called modem.begin()");
    xSemaphoreGive( dbSem );
  }
  #endif
    
  
  while( irerr != ISBD_SUCCESS ){
    #ifdef DEBUG_IRD
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      SERIAL.println("IRIDIUM: Begin failed: error " + String(irerr));
      xSemaphoreGive( dbSem );
    }
    #endif
    
    if( irerr == ISBD_NO_MODEM_DETECTED ){
      #ifdef DEBUG_IRD
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
        SERIAL.println("IRIDIUM: No modem detected: check wiring. ");
        xSemaphoreGive( dbSem );
      }
      #endif
    }
    
    irerr = modem.begin();
    
    myDelayMs(1000);
  }
  
  #ifdef DEBUG_IRD
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    SERIAL.println("IRIDIUM: modem initialized!");
    xSemaphoreGive( dbSem );
  }
  #endif
    
  
  // Test the signal quality.
  // This returns a number between 0 and 5.
  // 2 or better is preferred.
  irerr = modem.getSignalQuality(mSq);
  if( irerr != ISBD_SUCCESS ){
    
    #ifdef DEBUG_IRD
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      SERIAL.println("IRIDIUM: SignalQuality failed: error " + String(irerr));
      //syslog("IRIDIUM: failed to get first signal strength reading");
      //TODO: error handling
      //return;
      xSemaphoreGive( dbSem );
    }
    #endif
    
    
  } else {
    #ifdef DEBUG_IRD
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      SERIAL.println("IRIDIUM: first signal strength: " + String(mSq));
      //syslog("IRIDIUM: failed to get first signal strength reading");
      //TODO: error handling
      //return;
      xSemaphoreGive( dbSem );
    }
    #endif
  }
  
  //
  // MAIN TASK LOOP
  //
  while(1) {
    // handle a thread asking to send a packet, also periodically check the signal strength
    
    // periodically check the signal strength
    if( xTaskGetTickCount() - lastSignalCheck > CHECK_SIGNAL_PERIOD ){
      irerr = modem.getSignalQuality(mSq);
      if( irerr != ISBD_SUCCESS ){
        
        #ifdef DEBUG_IRD
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          SERIAL.println("IRIDIUM: get SignalQuality failed: error " + String(irerr));
          //syslog("IRIDIUM: failed to get first signal strength reading");
          //TODO: error handling
          //return;
          xSemaphoreGive( dbSem );
        }
        #endif 
      } else {
        #ifdef DEBUG_IRD
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          SERIAL.println("IRIDIUM: signal strength: " + String(mSq));
          //syslog("IRIDIUM: failed to get first signal strength reading");
          //TODO: error handling
          //return;
          xSemaphoreGive( dbSem );
        }
        #endif
      }
      
      if ( xSemaphoreTake( sigSem, ( TickType_t ) 100 ) == pdTRUE ) {
        irSig = mSq;
        xSemaphoreGive( sigSem );
      }
      
      lastSignalCheck = xTaskGetTickCount();
    }
    
    //
    // block for 400ms on the GGA GPS queue and build a packet with our latest location
    // send a packet every IRIDIUM_PACKET_PERIOD
    if( qRmcData != NULL ) {
    
      // if there was a fix in the queue, create a message in buf
      if( xQueuePeek( qRmcData, &rmcData, (TickType_t) 400 ) == pdPASS) {

        /*#ifdef DEBUG
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
        SERIAL.println("IRIDIUM: formatting packet");
        xSemaphoreGive( dbSem );
        }
        #endif*/

        if( rmcData.is_valid ){
          fix_valid = true;
          sprintf(buf, 
                  "%d:%d:%f.%f : LON %f ° | LAT %f ° | VEL %d  m/s | HEADING %f °",
                  rmcData.time_utc.hour, 
                  rmcData.time_utc.minute, 
                  rmcData.time_utc.second, 
                  rmcData.time_utc.microsecond,
                  rmcData.longitude,
                  rmcData.latitude,
                  rmcData.speed,
                  rmcData.course);
        } else {
          fix_valid = false;
          sprintf(buf, 
                  "%d:%d:%f.%f",
                  rmcData.time_utc.hour, 
                  rmcData.time_utc.minute, 
                  rmcData.time_utc.second, 
                  rmcData.time_utc.microsecond);
        }

        /*#ifdef DEBUG
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
        SERIAL.println("IRIDIUM: done formatting packet");
        xSemaphoreGive( dbSem );
        }
        #endif*/

        
      } 
    }
    
    // 
    // IS IT TIME TO SEND A PACKKAGE??
    if( xTaskGetTickCount() - lastPacketSend > IRIDIUM_PACKET_PERIOD && 
        (mSq > 0) && 
        fix_valid &&
        sendPackets ){
      
      #ifdef DEBUG_IRD
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      SERIAL.println("IRIDIUM: sending packet");
      xSemaphoreGive( dbSem );
      }
      #endif

      
      irerr = modem.sendSBDText(buf);
            
      if (irerr != ISBD_SUCCESS) { // sending failed
        #ifdef DEBUG_IRD
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          SERIAL.println("IRIDIUM: failed to send packet :( error " + String(irerr));
          xSemaphoreGive( dbSem );
        }
        #endif 
      }
      else { // send success
        #ifdef DEBUG_IRD
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          SERIAL.println("IRIDIUM: successfully sent a packet!!!!!!");
          xSemaphoreGive( dbSem );
        }
        #endif 
        // only update lastPacketSned timestamp if we were successful so that
        // the modem will try again asap
        lastPacketSend = xTaskGetTickCount();
      }
    }
    
    myDelayMs(1000);
    
  } // end task loop
  
  vTaskDelete( NULL );  
}

/**********************************************************************************
/**********************************************************************************
/**********************************************************************************
/**********************************************************************************
 * TPM Serial monitoring thread
*/
static void tpmThread( void *pvParameters )
{

  #ifdef DEBUG_TPM
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("TPM comms thread started");
    xSemaphoreGive( dbSem );
  }
  #endif
  
  bool  newTmp = false, newPrs = false;
  tc_t  tcData;
  prs_t prsData;
  
  
  while(1) {
    
    //myDelayMs(10);
    
    int result = 0;
    
    // acquire lock on serial port to TPM board
    if ( xSemaphoreTake( tpmSerSem, ( TickType_t ) 100 ) == pdTRUE ) {
      
      // AAHHH
      // UNTESTED !!
      // check for data received over serial
      // HOPEFULLY each instance of 'available' > 1 will correspond to the one byte of type data 
      // followed by the full packet of actual data we want...
      // .....
      if( myTransfer.available() ){
        uint16_t recSize = 0;
        uint8_t type; // what type of data is being received
        recSize = myTransfer.rxObj(type, recSize);
        if( type == PTYPE_TMP ){
          newTmp = true;
          recSize = myTransfer.rxObj(tcData, recSize);
        } 
        else if( type == PTYPE_PRS ){
          newPrs = true;
          recSize = myTransfer.rxObj(prsData, recSize);
        }
        else {
          // problem!!!
          result = -1;
        }
      }
      xSemaphoreGive( tpmSerSem );
    }
    
          
    // NOW HOPEFULLY WE HAVE DATA OH BUDDTY
    
    if( newTmp ){
      // new temperature data received over serial, put it in the queue for logging
      if( xQueueSend( qTmpData, ( void * ) &tcData, ( TickType_t ) 100 ) != pdTRUE ) {
        /* Failed to post the message, even after 100 ticks. */
        #ifdef DEBUG
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
          SERIAL.println("TPM: failed to put temperature data into queue");
          xSemaphoreGive( dbSem );
        }
        #endif
      } else {
        #ifdef DEBUG_QUEUE
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
          SERIAL.println("TPM: added tc packet to queue!");
          xSemaphoreGive( dbSem );
        }
        #endif
      }
      newTmp = false;  
    }
    
    if( newPrs ){
      // new pressure data receive over serial, put it in queue for logging
      if( xQueueSend( qPrsData, ( void * ) &prsData, ( TickType_t ) 100 ) != pdTRUE ) {
        /* Failed to post the message, even after 100 ticks. */
        #ifdef DEBUG
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
          SERIAL.println("ERROR: failed to put pressure data into queue");
          xSemaphoreGive( dbSem );
        }
        #endif
      } else {
        #ifdef DEBUG_QUEUE
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
          SERIAL.println("TPM: added pressure packet to queue!!!!");
          xSemaphoreGive( dbSem );
        }
        #endif
      }
      newPrs = false; // maybe put this is else so it will try to resend if it fails to put it in the queue the first time?
    }
    
    if( result == -1 ){
      #ifdef DEBUG
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
        SERIAL.println("TPM: error while receiveing data with serial transfer object, result=-1");
        xSemaphoreGive( dbSem );
      }
      #endif
    
    }
    
  }
  
  vTaskDelete( NULL );  
}


/**********************************************************************************
/**********************************************************************************
/**********************************************************************************
/**********************************************************************************
 * sd card logging thread
*/
static void logThread( void *pvParameters )
{
  static bool ready = false;
  int numToLog;
  uint8_t toLog[NUM_LOG_FILES];
  tc_t  tmpData;
  prs_t prsData;
  acc_t accData;
  imu_t imuData;
  nmea::GgaData ggaData;
  nmea::RmcData rmcData;
  bar_t barData;
  File logfile;
  
  #ifdef DEBUG_LOG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("sd logging thread thread started");
    xSemaphoreGive( dbSem );
  }
  #endif
  
  String filenames[NUM_LOG_FILES];
  filenames[0] = LOGFILE0; // TLM
  
  // INIT CARD
  while (!SD.begin(PIN_SD_CS)) {
    #if DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      Serial.println("ERROR: sd logging thread couldn't init sd card");
      xSemaphoreGive( dbSem );
    }
    #endif
    ready = false;
    myDelayMs(1000);
  } 
  //else {
    #if DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      Serial.println("SD CARD INIT OK");
      xSemaphoreGive( dbSem );
    }
    #endif
    ready = true;
  //}
  
  
  // CREATE UNIQUE FILE NAMES (UP TO 100)
  char filename[15];
  for( uint8_t j=0; j<NUM_LOG_FILES; j++ ){
    strcpy(filename, filenames[j].c_str());
    for( uint8_t i=0; i < 100; i++) {
      filename[3] = '0' + i/10;
      filename[4] = '0' + i%10;
      // create if does not exist, do not open existing, write, sync after write
      if (! SD.exists(filename)) {
        break;
      }
    }
    filenames[j] = String(filename);
  }

  while(1) {
    // check for a packet in each of the log data queues
    numToLog = 0;
    
    // TC / HEAT FLUX
    if( qTmpData != NULL ) {
      if( xQueueReceive( qTmpData, &tmpData, (TickType_t) 1 ) == pdTRUE ) {
         toLog[numToLog] = LOGID_TMP;
         numToLog++;
      }
    }
    
    // PRESSURE
    if( qPrsData != NULL ) {
      if( xQueueReceive( qPrsData, &prsData, (TickType_t) 1 ) == pdTRUE ) {
         toLog[numToLog] = LOGID_PRS;
         numToLog++;
      }
    }
    
    // GGA GPA DATA
    if( qGgaData != NULL ) {
      if( xQueueReceive( qGgaData, &ggaData, (TickType_t) 5 ) == pdTRUE ) {
         toLog[numToLog] = LOGID_GGA;
         numToLog++;
      }
    }
    
    // RMC GPS DATA
    if( qRmcData != NULL ) {
      if( xQueueReceive( qRmcData, &rmcData, (TickType_t) 5 ) == pdTRUE ) {
         toLog[numToLog] = LOGID_RMC;
         numToLog++;
      }
    }
    
    // BAROMETER ALT/PRS/TMP DATA
    if( qBarData != NULL ) {
      if( xQueueReceive( qBarData, &barData, (TickType_t) 1 ) == pdTRUE ) {
         toLog[numToLog] = LOGID_BAR;
         numToLog++;
      }
    }
    /*********************
    Now write any new data received above to the SD card
    HOPEFULLY  no need for mutex since this thread is the only one using the SPI port
    *********************/
    
    // open last log file for all telem (single log file configuration)
    logfile = SD.open(filenames[NUM_LOG_FILES-1], FILE_WRITE);
    // logfile no good
    if( ! logfile ) {
      #if DEBUG
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
        Serial.print("ERROR: sd logging thread couldn't open logfile for writing: ");
        Serial.println(filenames[0]);
        xSemaphoreGive( dbSem );
      }
      #endif
    }
      
    // LOGFILE OPEN!
    // log in CSV format for now because we are lazy
    for( int i=0; i<numToLog; i++ ){
    
      // tc / heat flux
      if( toLog[i] == LOGID_TMP ){
        logfile.print(tmpData.t);
        logfile.print(", ");
        logfile.print(LOGID_TMP);
        logfile.print(", ");
        for( int j=0; j<NUM_TC_CHANNELS; j++ ){
          logfile.print(tmpData.data[j]);
          if( j<NUM_TC_CHANNELS-1 ){
            logfile.print(", ");
          } else {
            logfile.println();
          }
        }
      }
      
      // pressure
      else if( toLog[i] == LOGID_PRS ){
        logfile.print(prsData.t);
        logfile.print(", ");
        logfile.print(LOGID_PRS);
        logfile.print(", ");
        for( int j=0; j<NUM_PRS_CHANNELS; j++ ){
          logfile.print(prsData.data[j]);
          if( j<NUM_PRS_CHANNELS-1 ){
            logfile.print(", ");
          } else {
            logfile.println();
          }
        }
      }
      
      // GGA GPS data
      else if( toLog[i] == LOGID_GGA ){
        logfile.print(LOGID_GGA);
        logfile.print(", ");
        writeGga(ggaData, logfile);
      }
      
      // RMC GPS data
      else if( toLog[i] == LOGID_RMC ){
        logfile.print(LOGID_RMC);
        logfile.print(", ");
        writeRmc(rmcData, logfile);
      } 
      
      else if( toLog[i] == LOGID_BAR ){
        logfile.print(xTaskGetTickCount());
        logfile.print(", ");
        logfile.print(LOGID_BAR);
        logfile.print(", ");
        logfile.print(barData.prs);
        logfile.print(", ");
        logfile.print(barData.alt);
        logfile.print(", ");
        logfile.println(barData.tmp);
      }
      
      // should not happen
      else {
        // errrrrr
      }
    }
    
    // done writing to this file
    logfile.close();
    
    taskYIELD();
  }
  
  vTaskDelete( NULL );  
}


/**********************************************************************************
/**********************************************************************************
/**********************************************************************************
/**********************************************************************************
 * parachute deployment thread
 * monitor gps fixes and more importantly pressure data
*/
static void parThread( void *pvParameters )
{
  prs_t prsData;
  nmea::GgaData ggaData;
  float alt=1.3e6, prs=0;
  bool deployed = false;

  #ifdef DEBUG_PAR
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("parachute deployment thread started");
    xSemaphoreGive( dbSem );
  }
  #endif
  
  while(1) {
    // peek at gga and pressure queues to see if activation criteria have been met
    
    myDelayMs(100);
    
    if (deployed) continue;
    
    // PRESSURE
    if( qPrsData != NULL ) {
      if( xQueuePeek( qPrsData, &prsData, portMAX_DELAY ) == pdPASS) {

        // TODO: DECIDE ON WHICH PRESSURE CHANNEL TO USE
        // USING ONLY ONE CHANNEL FOR NOW
        prs = prsData.data[0];
        
        #ifdef DEBUG_VERBOSE
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          Serial.println("parachute: peeked at pressure data!");
          xSemaphoreGive( dbSem );
        }
        #endif
      }
    }
    
    if( qGgaData != NULL ) {
      if( xQueuePeek( qGgaData, &ggaData, (TickType_t) 1000 ) == pdPASS) {
        alt = ggaData.altitude;
        
        #ifdef DEBUG_VERBOSE
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          Serial.println("parachute: peeked at gps data!");
          xSemaphoreGive( dbSem );
        }
        #endif
      } 
    }
    
    //if( ( alt <= ALT_THRESH ) && ( prs >= PRS_THRESH ) ){ // DEPLOYYY!
    if( ( prs >= PRS_THRESH ) ){
      deployed = true;
    }
    
    if ( /* received deploy over radio */ false ) {
      deployed = true;
    }
    
    if( deployed ){
      #ifdef DEBUG
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
        Serial.println("{INFO} !!! parachute deployment !!!");
        Serial.println("{INFO} !!! parachute deployment !!!");
        Serial.println("{INFO} !!! parachute deployment !!!");
        xSemaphoreGive( dbSem );
      }
      #endif
      
      if ( xSemaphoreTake( depSem, ( TickType_t ) portMAX_DELAY ) == pdTRUE ) {
        globalDeploy = true;
        xSemaphoreGive( depSem );
      }
      
      // now start paracture deployment sequence
      // first trigger c02 servo
      
      co2servo.write(CO2SERVO_POS_ACT);
      
    }
    
  }
  
  vTaskDelete( NULL );  
}



/**********************************************************************************
 *  Radio task
*/
#define RBUF_SIZE 260
#define SBUF_SIZE 240
uint8_t rbuf[RBUF_SIZE];
char sbuf[SBUF_SIZE];

static void radThread( void *pvParameters )
{

  unsigned long lastSendTime = 0;
  tlm_t dataOut;
  nmea::GgaData ggaData;
  nmea::RmcData rmcData;
  bar_t barData; 
  
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
  myDelayMs(100);
  digitalWrite(PIN_LORA_RST, HIGH);
  
    
  //SERIAL_LOR.print("AT+RESET\r\n");
  //SERIAL_LOR.flush();
  
  #ifdef DEBUG_RAD
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    SERIAL.println("Reset radio");
    xSemaphoreGive( dbSem );
  }
  #endif
  
  
  memset(rbuf, 0, RBUF_SIZE);
  memset(sbuf, 0, SBUF_SIZE);
  
  #ifdef DEBUG_RAD
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
    
    /*#ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      SERIAL.print("STATE = ");
      SERIAL.print(state);
      SERIAL.print(", LOR_RADIO.peek()= ");
      SERIAL.println(SERIAL_LOR.peek());
      xSemaphoreGive( dbSem );
    }
    #endif*/
    
    if( state == 1 ){
      int len = sprintf(sbuf, "AT+ADDRESS=1\r\n");
      SERIAL_LOR.write(sbuf, len);
      state = 2;
      #ifdef DEBUG_RAD
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
        Serial.println("sent at+address, setting state to 2");
        xSemaphoreGive( dbSem );
      }
      #endif
      taskYIELD();
    }
    else if( state == 3 ){
      int len = sprintf(sbuf, "AT+NETWORKID=1\r\n");
      SERIAL_LOR.write(sbuf, len);
      state = 4;
      #ifdef DEBUG_RAD
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
        Serial.println("sent at+networkid, setting state to 4");
        xSemaphoreGive( dbSem );
      }
      #endif
      taskYIELD();
    }
    else if( state == 5 ){
      int len = sprintf(sbuf, "AT+PARAMETER=12,4,1,7\r\n"); // recommended for more than 3km
      SERIAL_LOR.write(sbuf, len);
      state = 6;
      #ifdef DEBUG_RAD
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
        Serial.println("sent at+param, setting state to 6");
        xSemaphoreGive( dbSem );
      }
      #endif
      taskYIELD();
    }
    
    if( xTaskGetTickCount() - lastSendTime > TLM_SEND_PERIOD && state == 7){
      
      // peek at GGA data
      if( qGgaData != NULL ) {
        if( xQueuePeek( qGgaData, &ggaData, (TickType_t) 2000 ) == pdPASS) {
          dataOut.lat = ggaData.latitude;
          dataOut.lon = ggaData.longitude;
          dataOut.alt_gps = ggaData.altitude;
          
          #ifdef DEBUG_QUEUE
          if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
            Serial.println("lora: peeked at GGA data!");
            xSemaphoreGive( dbSem );
          }
          #endif
        } 
      }
      
      // peek at RMC data
      if( qRmcData != NULL ) {
        if( xQueuePeek( qRmcData, &rmcData, (TickType_t) 2000 ) == pdPASS) {
          dataOut.vel = rmcData.speed;
          
          
          #ifdef DEBUG_QUEUE
          if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
            Serial.println("lora: peeked at RMC data!");
            xSemaphoreGive( dbSem );
          }
          #endif
        } 
      }
      
      // peek at barometric pressure sensor data
      if( qBarData != NULL ) {
        if( xQueuePeek( qBarData, &barData, (TickType_t) 2000 ) == pdPASS) {
          dataOut.alt_bar = barData.alt;
          dataOut.barp = barData.prs;
          dataOut.tmp = barData.tmp;
          
          #ifdef DEBUG_QUEUE
          if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
            Serial.println("lora: peeked at RMC data!");
            xSemaphoreGive( dbSem );
          }
          #endif
        } 
      }
      
      // check parachute deploy status
      bool dep = false;
      if ( xSemaphoreTake( depSem, ( TickType_t ) portMAX_DELAY ) == pdTRUE ) {
        dep = globalDeploy;
        xSemaphoreGive( depSem );
      }
      dataOut.pardep = dep;
      
      // check iridium signal
      if ( xSemaphoreTake( sigSem, ( TickType_t ) 100 ) == pdTRUE ) {
        dataOut.irsig = irSig;
        xSemaphoreGive( sigSem );
      }
      
      dataOut.t = xTaskGetTickCount();
      /*dataOut.lat = 37.1838432;
      dataOut.lon = -84.824233;
      dataOut.vel = 1029.235;
      dataOut.alt_gps = 1200.92;
      dataOut.alt_bar = 1190.32;
      dataOut.barp = 995.127;
      dataOut.tmp = 23.81;
      dataOut.irsig = 2;
      dataOut.pardep = 1;
      */
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
      
      
      #ifdef DEBUG_RAD
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
        SERIAL.print("sending radio binary packet: ");
        //SERIAL.write(sbuf, pre+dataSize+3);
        //SERIAL.println("DONE");
        xSemaphoreGive( dbSem );
      }
      #endif
      
      // send to lora module
      SERIAL_LOR.write(sbuf, pre+dataSize+2);
      //SERIAL_LOR.write(sbuf, pre);
      
      // update timestamp, not including data fill time from above lines
      lastSendTime = dataOut.t; 
      
      // go to state 8 so that we wait for a response
      state = 8;
      taskYIELD();
    }
    
  
    // handle incoming message from LORA radio
    // AT message from module
    bool eol = false;
    int pos = 0;
    bool timeout = false;
    unsigned long timeoutStart = 0;
    if( SERIAL_LOR.peek() == '+' ){
      
      #ifdef DEBUG_RAD
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
        SERIAL.println("saw a +");
        xSemaphoreGive( dbSem );
      }
      #endif

    
      unsigned long timeoutStart = xTaskGetTickCount();
      while(!eol && !timeout && pos < RBUF_SIZE-1) {
        if( SERIAL_LOR.available() ){
          rbuf[pos] = SERIAL_LOR.read();
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
        #ifdef DEBUG_RAD
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          SERIAL.println("Radio rx timed out");
          xSemaphoreGive( dbSem );
        }
        #endif
      } else if (!timeout && eol) {
        #ifdef DEBUG_RAD
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          SERIAL.println("Radio got packet!");
          xSemaphoreGive( dbSem );
        }
        #endif
      } else if( !timeout && !eol) {
        #ifdef DEBUG_RAD
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
          #ifdef DEBUG_RAD
          if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
            SERIAL.print("Received ");
            SERIAL.write(rbuf, pos);
            SERIAL.println();
            xSemaphoreGive( dbSem );
          }
          #endif
          
          if( state < 7 ){
            state ++;
            if( state == 7 ){
              #ifdef DEBUG_RAD
              if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
                SERIAL.println("STATE = 7, successfully configured radio!");
                xSemaphoreGive( dbSem );
              }
              #endif
              taskYIELD();
            }
          } else if( state > 7 ){
            #ifdef DEBUG_RAD
            state = 7;
            if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
              SERIAL.println("STATE was 8, received +OK from a data send operation!");
              xSemaphoreGive( dbSem );
            }
            #endif
            taskYIELD(); 
          }
          
        } else {
          // found an '=', parse rest of message
          #ifdef DEBUG
          if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
            SERIAL.print("Received ");
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


/**********************************************************************************/
/**********************************************************************************/
/**********************************************************************************/
/**********************************************************************************/
/**********************************************************************************/
/**********************************************************************************/
void setup() {

  // attach servo
  co2servo.attach(PIN_C02SERVO);
  co2servo.write(CO2SERVO_POS_HOME);

  #if DEBUG
  SERIAL.begin(115200); // init debug serial
  #endif
  
  delay(100);
  
  delay(10);
  SERIAL_TPM.begin(TPM_SERIAL_BAUD); // init serial to tpm subsystem
  delay(10);
  SERIAL_GPS.begin(9600); // init gps serial
  delay(10);
  SERIAL_IRD.begin(9600); // init iridium serial
  delay(10);
  SERIAL_LOR.begin(115200); // init LORA telemetry radio serial
  
  
  // Assign SERCOM functionality to enable 3 more UARTs
  pinPeripheral(A1, PIO_SERCOM_ALT);
  pinPeripheral(A4, PIO_SERCOM_ALT);
  pinPeripheral(A2, PIO_SERCOM_ALT);
  pinPeripheral(A3, PIO_SERCOM_ALT);
  pinPeripheral(13, PIO_SERCOM);
  pinPeripheral(12, PIO_SERCOM);
  
  // setup reliable serial datagram between CDH and TPM processors
  //myTransfer.begin(SERIAL_TPM, false, SERIAL, 500);
  myTransfer.begin(SERIAL_TPM);
  
  // reset pin for lora radio
  pinMode(PIN_LORA_RST, OUTPUT);
  
  // battery voltage divider 
  pinMode(PIN_VBAT, INPUT);
  
  // servo control
  pinMode(PIN_C02SERVO, OUTPUT);
  
  // iridium power switch
  pinMode(PIN_IRACT, OUTPUT);
  
  // pyro power switch
  pinMode(PIN_PYROACT, OUTPUT);
  
  // scheduler control pin to TPM subsystem
  pinMode(PIN_TPM_SCHEDULER_CTRL, OUTPUT);
  digitalWrite(PIN_TPM_SCHEDULER_CTRL, LOW); // TPM scheduler starts when high

  // modem power on/off control
  pinMode(PIN_IRIDIUM_EN, OUTPUT);
  digitalWrite(PIN_IRIDIUM_EN, LOW); // modem on when this output high

  delay(3000);
  
  #ifdef DEBUG
  SERIAL.println("Starting...");
  #endif

  // CREATE RTOS QUEUES 
  // temperature data queue
  qTmpData = xQueueCreate( 10, sizeof( struct tc_t ) );
  if( qTmpData == NULL ) {
    /* Queue was not created and must not be used. */
    #if DEBUG
    SERIAL.println("Failed to create qTmpData queue");
    #endif
  }
  // pressure data queue
  qPrsData = xQueueCreate( 10, sizeof( struct prs_t ) );
  if( qPrsData == NULL ) {
    /* Queue was not created and must not be used. */
    #if DEBUG
    SERIAL.println("Failed to create qPrsData queue");
    #endif
  }
  // gga gps data queue
  qGgaData = xQueueCreate( 5, sizeof( nmea::GgaData ) );
  if( qGgaData == NULL ) {
    /* Queue was not created and must not be used. */
    #if DEBUG
    SERIAL.println("Failed to create qGgaData queue");
    #endif
  }
  // rmc gps data queue
  qRmcData = xQueueCreate( 5, sizeof( nmea::RmcData ) );
  if( qRmcData == NULL ) {
    /* Queue was not created and must not be used. */
    #if DEBUG
    SERIAL.println("Failed to create qRmcData queue");
    #endif
  }  
  // barometrics pressure data queue
  qBarData = xQueueCreate( 5, sizeof( struct bar_t ) );
  if( qBarData == NULL ) {
    /* Queue was not created and must not be used. */
    #if DEBUG
    SERIAL.println("Failed to create qBarData queue");
    #endif
  }  
  // should take action if not all queues were created properly
  
  #ifdef DEBUG
  SERIAL.println("Created queues...");
  #endif
  
  // SETUP RTOS SEMAPHORES
  // setup cdh serial port smphr
  if ( tpmSerSem == NULL ) {
    tpmSerSem = xSemaphoreCreateMutex();  // create mutex
    if ( ( tpmSerSem ) != NULL )
      xSemaphoreGive( ( tpmSerSem ) );  // make available
  }
  // setup debug serial log semaphore
  if ( dbSem == NULL ) {
    dbSem = xSemaphoreCreateMutex();  // create mutex
    if ( ( dbSem ) != NULL )
      xSemaphoreGive( ( dbSem ) );  // make available
  }
  // setup i2c port semaphore
  if ( i2c1Sem == NULL ) {
    i2c1Sem = xSemaphoreCreateMutex();  // create mutex
    if ( ( i2c1Sem ) != NULL )
      xSemaphoreGive( ( i2c1Sem ) );  // make available
  }
  // setup gps serial semaphore
  if ( gpsSerSem == NULL ) {
    gpsSerSem = xSemaphoreCreateMutex();  // create mutex
    if ( ( gpsSerSem ) != NULL )
      xSemaphoreGive( ( gpsSerSem ) );  // make available
  }
  // setup iridium serial semaphore
  if ( irdSerSem == NULL ) {
    irdSerSem = xSemaphoreCreateMutex();  // create mutex
    if ( ( irdSerSem ) != NULL )
      xSemaphoreGive( ( irdSerSem ) );  // make available
  }
  // setup DEPLOYMENT bool protector
  if ( depSem == NULL ) {
    depSem = xSemaphoreCreateMutex();  // create mutex
    if ( ( depSem ) != NULL )
      xSemaphoreGive( ( depSem ) );  // make available
  }
  // setup iridium signal protector
  if ( sigSem == NULL ) {
    sigSem = xSemaphoreCreateMutex();  // create mutex
    if ( ( sigSem ) != NULL )
      xSemaphoreGive( ( sigSem ) );  // make available
  }
  
  #ifdef DEBUG
  SERIAL.println("Created semaphores...");
  #endif
  
  /**************
  * CREATE TASKS
  **************/
  xTaskCreate(logThread, "SD Logging", 1024, NULL, tskIDLE_PRIORITY, &Handle_logTask);
  xTaskCreate(gpsThread, "GPS Reception", 512, NULL, tskIDLE_PRIORITY, &Handle_gpsTask);
  xTaskCreate(irdThread, "Iridium thread", 512, NULL, tskIDLE_PRIORITY, &Handle_irdTask);
  xTaskCreate(parThread, "Parachute Deployment", 512, NULL, tskIDLE_PRIORITY, &Handle_parTask);
  xTaskCreate(tpmThread, "TPM Communication", 512, NULL, tskIDLE_PRIORITY, &Handle_tpmTask);
  xTaskCreate(barThread, "Capsule internals", 512, NULL, tskIDLE_PRIORITY, &Handle_barTask);
  xTaskCreate(radThread, "Telem radio", 1024, NULL, tskIDLE_PRIORITY, &Handle_radTask);
  //xTaskCreate(taskMonitor, "Task Monitor", 256, NULL, tskIDLE_PRIORITY + 4, &Handle_monitorTask);
  
  // Start the RTOS, this function will never return and will schedule the tasks.
  // signal to the TPM subsystem to start the task scheduler so that xTaskGetTickCount() is 
  // consistent between the two
  digitalWrite(PIN_TPM_SCHEDULER_CTRL, HIGH);
  vTaskStartScheduler();
  
  co2servo.write(CO2SERVO_POS_HOME);

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

