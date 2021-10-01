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
#include "wiring_private.h"

#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define DEBUG 1 // usb serial debug switch
#define DEBUG_GPS 1

//#include <Mahony_DPEng.h>
//#include <Madgwick_DPEng.h>

#include "H3LIS100.h"
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


// debug serial
#define SERIAL      Serial  // debug serial (USB) all uses should be conditional on DEBUG define
#define SERIAL_TPM  Serial1 // to TPM subsystem
#define SERIAL_GPS  Serial2 // to gps
#define SERIAL_IRD  Serial3 // to iridium modem

// Serial transfer object for receiving data from TPM processor
SerialTransfer myTransfer;

// High G accel sensor object
H3LIS100 accel = H3LIS100(12345);

// capsule internal barometric pressure object
// and struct for passing data through queue
Adafruit_MPL3115A2 baro;
struct barSenseData {
  float prs;
  float alt;
  float tmp;
};

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

// freertos task handles
TaskHandle_t Handle_tpmTask; // data receive from TPM subsystem task
TaskHandle_t Handle_logTask; // sd card logging task
TaskHandle_t Handle_accTask; // high g imu data collection task
TaskHandle_t Handle_imuTask; // 9 axis imu data collection task
TaskHandle_t Handle_gpsTask; // gps data receive task
TaskHandle_t Handle_irdTask; // iridium transmission task
TaskHandle_t Handle_parTask; // parachute deployment task
TaskHandle_t Handle_barTask;
TaskHandle_t Handle_monitorTask; // debug running task stats over uart task

// freeRTOS queues
QueueHandle_t qTmpData; // temperature or heat flux data to be logged
QueueHandle_t qPrsData; // pressure data to be logged
QueueHandle_t qAccData; // high g accelerometer data to be logged
QueueHandle_t qImuData; // 6 axis log g imu data to be logged
QueueHandle_t qGgaData; // GGA GPS fix data to be logged
QueueHandle_t qRmcData; // RMC GPS data to be logged
QueueHandle_t qBarData; // barometric pressure data

// freeRTOS semaphores
SemaphoreHandle_t tpmSerSem; // data from CDH semaphore
SemaphoreHandle_t dbSem; // serial debug logging (Serial)
SemaphoreHandle_t i2c1Sem; // i2c port 1 access semaphore
SemaphoreHandle_t gpsSerSem; // gps serial port acces
SemaphoreHandle_t irdSerSem; // iridium serial semaphore

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
    pipe.print(rmc.longitude);
    pipe.print(" ° | LAT ");
    pipe.print(rmc.latitude);
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
    pipe.print(gga.longitude);
    pipe.print(" ° | LAT ");
    pipe.print(gga.latitude);
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
  /*#ifdef DEBUG_GPS
  if (gga.fix_quality != nmea::FixQuality::Invalid) {
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 200 ) == pdTRUE ) {
      writeGga(gga, SERIAL);
      xSemaphoreGive( dbSem );
    }
  }
  #endif*/

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
  barSenseData sensorData;

  #ifdef DEBUG
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
  
  #ifdef DEBUG
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
        #ifdef DEBUG
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
          SERIAL.println("BAR: failed to put sensor data into queue");
          xSemaphoreGive( dbSem );
        }
        #endif
      } else {
        #ifdef TPM_PACKET_DEBUG
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
          SERIAL.println("BAR: added prs/alt/tmp packet to queue!");
          xSemaphoreGive( dbSem );
        }
        #endif
      }
    
    #ifdef DEBUG_VERBOSE
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
  #ifdef DEBUG
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
  
  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("Iridium thread started");
    xSemaphoreGive( dbSem );
  }
  #endif

  // enable modem
  digitalWrite(PIN_IRIDIUM_EN, HIGH);
  
  myDelayMs(5000); // give modem time to power up
    
    
   #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("Iridium thread: trying to start modem!/...");
    xSemaphoreGive( dbSem );
  }
  #endif
    
  // init the iridium library and check signal strength
  irerr = modem.begin();
  
  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    SERIAL.println("IRIDIUM: called modem.begin()");
    xSemaphoreGive( dbSem );
  }
  #endif
    
  
  while( irerr != ISBD_SUCCESS ){
    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      SERIAL.println("IRIDIUM: Begin failed: error " + String(irerr));
      xSemaphoreGive( dbSem );
    }
    #endif
    
    if( irerr == ISBD_NO_MODEM_DETECTED ){
      #ifdef DEBUG
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
        SERIAL.println("IRIDIUM: No modem detected: check wiring. ");
        xSemaphoreGive( dbSem );
      }
      #endif
    }
    
    irerr = modem.begin();
    
    myDelayMs(1000);
  }
  
  #ifdef DEBUG
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
    
    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      SERIAL.println("IRIDIUM: SignalQuality failed: error " + String(irerr));
      //syslog("IRIDIUM: failed to get first signal strength reading");
      //TODO: error handling
      //return;
      xSemaphoreGive( dbSem );
    }
    #endif
    
    
  } else {
    #ifdef DEBUG
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
        
        #ifdef DEBUG
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          SERIAL.println("IRIDIUM: get SignalQuality failed: error " + String(irerr));
          //syslog("IRIDIUM: failed to get first signal strength reading");
          //TODO: error handling
          //return;
          xSemaphoreGive( dbSem );
        }
        #endif 
      } else {
        #ifdef DEBUG
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          SERIAL.println("IRIDIUM: signal strength: " + String(mSq));
          //syslog("IRIDIUM: failed to get first signal strength reading");
          //TODO: error handling
          //return;
          xSemaphoreGive( dbSem );
        }
        #endif
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
    // IS TIT TIME TO SEND A PACKKAGE??
    if( xTaskGetTickCount() - lastPacketSend > IRIDIUM_PACKET_PERIOD && (mSq > 0) && fix_valid ){
      
      #ifdef DEBUG
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      SERIAL.println("IRIDIUM: sending packet");
      xSemaphoreGive( dbSem );
      }
      #endif

      
      irerr = modem.sendSBDText(buf);
            
      if (irerr != ISBD_SUCCESS) { // sending failed
        #ifdef DEBUG
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          SERIAL.println("IRIDIUM: failed to send packet :( error " + String(irerr));
          xSemaphoreGive( dbSem );
        }
        #endif 
      }
      else { // send success
        #ifdef DEBUG
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

  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("TPM comms thread started");
    xSemaphoreGive( dbSem );
  }
  #endif
  
  bool  newTmp = false, newPrs = false;
  tc_t  tcData;
  prs_t prsData;
  
  
  while(1) {
    
    myDelayMs(10);
    
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
        #ifdef TPM_PACKET_DEBUG
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
        #ifdef TPM_PACKET_DEBUG
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
          SERIAL.println("TPM: added pressure packet to queue!!!!");
          xSemaphoreGive( dbSem );
        }
        #endif
      }
      newPrs = false; // maybe put this is else so it will try to resend if it fails to put it in the queue the first time?
    }
    
  }
  
  vTaskDelete( NULL );  
}


/**********************************************************************************
/**********************************************************************************
/**********************************************************************************
/**********************************************************************************
 * High g accel data collection thread
*/
static void accThread( void *pvParameters )
{

  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("High g accelerometer  thread started");
    xSemaphoreGive( dbSem );
  }
  #endif
  

  // init the sensor and try to restart the thread if this fails.. or somehow 
  // ensure that the sensor is eventually initialized
  while( !accel.begin() ){
    myDelayMs(100);
    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      Serial.println("High g accelerometer  failed to init");
      xSemaphoreGive( dbSem );
    }
    #endif
  }


  while(1) {
    // goal of 20 hz logging rate, 
    // read from the high g accel and push a message to the data log queue
    myDelayMs(49);

    sensors_event_t event; 
    
    if ( xSemaphoreTake( i2c1Sem, ( TickType_t ) 5 ) == pdTRUE ) {
      accel.getEvent(&event);
      xSemaphoreGive( i2c1Sem );
    }
    
    // handle the data in the event var    
    acc_t data;
    data.t = xTaskGetTickCount();
    data.data[0] = event.acceleration.x;
    data.data[1] = event.acceleration.y;
    data.data[2] = event.acceleration.z;
    
    #ifdef ACC_DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 10 ) == pdTRUE ) {
      SERIAL.println("ACC: " + String(data.data[0]) + ", " + String(data.data[1]) + ", " + String(data.data[2]));
      xSemaphoreGive( dbSem );
    }
    #endif
    
    // send the data to the data log queue
    if( xQueueSend( qAccData, ( void * ) &data, ( TickType_t ) 5 ) != pdTRUE ) {
      /* Failed to post the message, even after 100 ticks. */
      #ifdef DEBUG
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
        SERIAL.println("ERROR: failed to put accel data into queue");
        xSemaphoreGive( dbSem );
      }
      #endif
    } else {
      #ifdef DEBUG_VERBOSE
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
        SERIAL.println("ACC: put pkt in queue");
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
 * 9 axis imu data collection thread
*/
static void imuThread( void *pvParameters )
{
  bool init = false;
  imu::Vector<3> acc, gyr;
  imu_t imuData;

  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("9 axis imu thread started");
    xSemaphoreGive( dbSem );
  }
  #endif
  
  // init imu
  while (!init) {
    myDelayMs(500);
    if ( xSemaphoreTake( i2c1Sem, ( TickType_t ) 100 ) == pdTRUE ) {  
      if (bno.begin()) {
        init = true;
      }      
      xSemaphoreGive( i2c1Sem );
    }
  }
  
  // delay and set config data
  myDelayMs(100);
  if ( xSemaphoreTake( i2c1Sem, ( TickType_t ) 100 ) == pdTRUE ) {  
    // set a setting 
    bno.setExtCrystalUse(true);
    xSemaphoreGive( i2c1Sem );
  } 
  
  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("SUCCESS: INITIALIZED 9 axis imu");
    xSemaphoreGive( dbSem );
  }
  #endif
  
  while(1) {
    // goal log rate of 100Hz for accelerometer and gyroscope data 
    // can log queue handle >100 message passes per second??
    myDelayMs(100);
    
    // get data over i2c
    if ( xSemaphoreTake( i2c1Sem, ( TickType_t ) 10 ) == pdTRUE ) {  
      acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
      gyr = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
      xSemaphoreGive( i2c1Sem );
    } 
    
    imuData.data[0] = acc.x();
    imuData.data[1] = acc.y();
    imuData.data[2] = acc.z();
    imuData.data[3] = gyr.x();
    imuData.data[4] = gyr.y();
    imuData.data[5] = gyr.z();
    
    // send the data to the data log queue
    if( xQueueSend( qImuData, ( void * ) &imuData, ( TickType_t ) 5 ) != pdTRUE ) {
      /* Failed to post the message, even after 5 ticks. */
      #ifdef DEBUG
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
        SERIAL.println("ERROR: failed to put imu data into queue");
        xSemaphoreGive( dbSem );
      }
      #endif
    } else {
      #ifdef DEBUG_VERBOSE
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
        SERIAL.println("IMU: put pkt in queue");
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
  barSenseData barData;
  File logfile;
  
  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("sd logging thread thread started");
    xSemaphoreGive( dbSem );
  }
  #endif
  
  String filenames[NUM_LOG_FILES];
  filenames[0] = LOGFILE0; // TMP
  filenames[1] = LOGFILE1; // PRS
  filenames[2] = LOGFILE2; // ACC
  filenames[3] = LOGFILE3; // IMU
  filenames[4] = LOGFILE4; // GGA
  filenames[5] = LOGFILE5; // RMC
  filenames[6] = LOGFILE6; // BAR
  
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
      if( xQueueReceive( qTmpData, &tmpData, (TickType_t) 5 ) == pdTRUE) {
         toLog[numToLog] = LOGID_TMP;
         numToLog++;
      }
    }
    
    // PRESSURE
    if( qPrsData != NULL ) {
      if( xQueueReceive( qPrsData, &prsData, (TickType_t) 5 ) == pdTRUE) {
         toLog[numToLog] = LOGID_PRS;
         numToLog++;
      }
    }
    
    // HIGH G ACCELEROMETER
    if( qAccData != NULL ) {
      if( xQueueReceive( qAccData, &accData, (TickType_t) 5 ) == pdTRUE) {
         toLog[numToLog] = LOGID_ACC;
         numToLog++;
      }
    }
    
    // IMU DATA
    if( qImuData != NULL ) {
      if( xQueueReceive( qImuData, &imuData, (TickType_t) 5 ) == pdTRUE) {
         toLog[numToLog] = LOGID_IMU;
         numToLog++;
      }
    }
    
    // GGA GPA DATA
    if( qGgaData != NULL ) {
      if( xQueueReceive( qGgaData, &ggaData, (TickType_t) 5 ) == pdTRUE) {
         toLog[numToLog] = LOGID_GGA;
         numToLog++;
      }
    }
    
    // RMC GPS DATA
    if( qRmcData != NULL ) {
      if( xQueueReceive( qRmcData, &rmcData, (TickType_t) 5 ) == pdTRUE) {
         toLog[numToLog] = LOGID_RMC;
         numToLog++;
      }
    }
    
    // BAROMETER ALT/PRS/TMP DATA
    if( qBarData != NULL ) {
      if( xQueueReceive( qBarData, &barData, (TickType_t) 5 ) == pdTRUE) {
         toLog[numToLog] = LOGID_BAR;
         numToLog++;
      }
    }
    /*********************
    Now write any new data received above to the SD card
    HOPEFULLY  no need for mutex since this thread is the only one using the SPI port
    *********************/
    
    for( int i=0; i<numToLog; i++ ){
      logfile = SD.open(filenames[toLog[i]], FILE_WRITE);
      
      // logfile no good
      if( ! logfile ) {
        #if DEBUG
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          Serial.print("ERROR: sd logging thread couldn't open logfile for writing: ");
          Serial.println(filenames[toLog[i]]);
          xSemaphoreGive( dbSem );
        }
        #endif
      }       
      else { // LOGFILE OPEN!
        // write to respective files
        // for now log in CSV because flight times are short and we have GB to spare
        
        // tc / heat flux
        if( toLog[i] == LOGID_TMP ){
          logfile.print(tmpData.t);
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
          for( int j=0; j<NUM_PRS_CHANNELS; j++ ){
            logfile.print(prsData.data[j]);
            if( j<NUM_PRS_CHANNELS-1 ){
              logfile.print(", ");
            } else {
              logfile.println();
            }
          }
        }
        
        // high g accelerometer
        else if( toLog[i] == LOGID_ACC ){
          logfile.print(accData.t);
          logfile.print(", ");
          for( int j=0; j<NUM_HIGHG_CHANNELS; j++ ){
            logfile.print(accData.data[j]);
            if( j<NUM_HIGHG_CHANNELS-1 ){
              logfile.print(", ");
            } else {
              logfile.println();
            }
          }
        } 
        
        // IMU
        else if( toLog[i] == LOGID_IMU ){
          logfile.print(imuData.t);
          logfile.print(", ");
          for( int j=0; j<NUM_IMU_CHANNELS; j++ ){
            logfile.print(imuData.data[j]);
            if( j<NUM_IMU_CHANNELS-1 ){
              logfile.print(", ");
            } else {
              logfile.println();
            }
          }
        }
        
        // GGA GPS data
        else if( toLog[i] == LOGID_GGA ){
          writeGga(ggaData, logfile);
        }
        
        // RMC GPS data
        else if( toLog[i] == LOGID_RMC ){
          writeRmc(rmcData, logfile);
        } 
        
        else if( toLog[i] == LOGID_BAR ){
          logfile.print(xTaskGetTickCount());
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
    }
    
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

  #ifdef DEBUG
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
    if( ( prs >= PRS_THRESH ) ){ // DEPLOYYY!
      #ifdef DEBUG
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
        Serial.println("{INFO} !!! parachute deployment !!!");
        Serial.println("{INFO} !!! parachute deployment !!!");
        Serial.println("{INFO} !!! parachute deployment !!!");
        xSemaphoreGive( dbSem );
      }
      #endif
      
      deployed = true;
      
      // TODO: interface with the parachute deployment hardware
      //...
    }
    
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

  #if DEBUG
  SERIAL.begin(115200); // init debug serial
  #endif
  delay(10);
  SERIAL_TPM.begin(34800); // init serial to tpm subsystem
  delay(10);
  SERIAL_GPS.begin(9600); // init gps serial
  delay(10);
  SERIAL_IRD.begin(9600); // init iridium serial
  delay(10);
  
  
  // Assign pins A2 & A3 SERCOM functionality
  pinPeripheral(A2, PIO_SERCOM_ALT);
  pinPeripheral(A3, PIO_SERCOM_ALT);
  pinPeripheral(13, PIO_SERCOM);
  pinPeripheral(12, PIO_SERCOM);
  
  myTransfer.begin(SERIAL_TPM, false, SERIAL, 500);
  myTransfer.begin(SERIAL_TPM);
  
  // scheduler control pin to TPM subsystem
  pinMode(PIN_TPM_SCHEDULER_CTRL, OUTPUT);
  digitalWrite(PIN_TPM_SCHEDULER_CTRL, LOW); // TPM scheduler starts when high

  // modem power on/off control
  pinMode(PIN_IRIDIUM_EN, OUTPUT);
  digitalWrite(PIN_IRIDIUM_EN, LOW); // modem on when this output high

  delay(3000);
  
  SERIAL.println("Starting...");

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
  // high g accel data queue
  qAccData = xQueueCreate( 20, sizeof( struct acc_t ) );
  if( qAccData == NULL ) {
    /* Queue was not created and must not be used. */
    #if DEBUG
    SERIAL.println("Failed to create qAccData queue");
    #endif
  }
  // imu data queue
  qImuData = xQueueCreate( 20, sizeof( struct imu_t ) );
  if( qImuData == NULL ) {
    /* Queue was not created and must not be used. */
    #if DEBUG
    SERIAL.println("Failed to create qImuData queue");
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
  qBarData = xQueueCreate( 5, sizeof( struct barSenseData ) );
  if( qBarData == NULL ) {
    /* Queue was not created and must not be used. */
    #if DEBUG
    SERIAL.println("Failed to create qBarData queue");
    #endif
  }  
  // should take action if not all queues were created properly
  
  SERIAL.println("Created queues...");
  
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
  
  SERIAL.println("Created semaphores...");

  
  /**************
  * CREATE TASKS
  **************/
  xTaskCreate(logThread, "SD Logging", 1024, NULL, tskIDLE_PRIORITY + 3, &Handle_logTask);
  xTaskCreate(accThread, "High-g Accel", 512, NULL, tskIDLE_PRIORITY + 2, &Handle_accTask);
  xTaskCreate(imuThread, "9 Axis IMU", 512, NULL, tskIDLE_PRIORITY + 2, &Handle_imuTask);
  xTaskCreate(gpsThread, "GPS Reception", 512, NULL, tskIDLE_PRIORITY + 2, &Handle_gpsTask);
  xTaskCreate(irdThread, "Iridium thread", 512, NULL, tskIDLE_PRIORITY + 2, &Handle_irdTask);
  xTaskCreate(parThread, "Parachute Deployment", 512, NULL, tskIDLE_PRIORITY + 4, &Handle_parTask);
  xTaskCreate(tpmThread, "TPM Communication", 512, NULL, tskIDLE_PRIORITY + 2, &Handle_tpmTask);
  xTaskCreate(barThread, "Capsule internals", 512, NULL, tskIDLE_PRIORITY + 2, &Handle_barTask);
  //xTaskCreate(taskMonitor, "Task Monitor", 256, NULL, tskIDLE_PRIORITY + 4, &Handle_monitorTask);
  
  // Start the RTOS, this function will never return and will schedule the tasks.
  // signal to the TPM subsystem to start the task scheduler so that xTaskGetTickCount() is 
  // consistent between the two
  digitalWrite(PIN_TPM_SCHEDULER_CTRL, HIGH);
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
//*****************************************************************
// Task will periodically print out useful information about the tasks running
// Is a useful tool to help figure out stack sizes being used
// Run time stats are generated from all task timing collected since startup
// No easy way yet to clear the run time stats yet
//*****************************************************************
static char ptrTaskList[400]; //temporary string bufer for task stats

void taskMonitor(void *pvParameters)
{
  int x;
  int measurement;
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
    SERIAL.println("Task Monitor: Started");
    xSemaphoreGive( dbSem );
  }
  // run this task afew times before exiting forever
  while(1)
  {
  	myDelayMs(10000); // print every 10 seconds

    if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
    	SERIAL.println("****************************************************");
    	SERIAL.print("Free Heap: ");
    	SERIAL.print(xPortGetFreeHeapSize());
    	SERIAL.println(" bytes");

    	SERIAL.print("Min Heap: ");
    	SERIAL.print(xPortGetMinimumEverFreeHeapSize());
    	SERIAL.println(" bytes");

    	SERIAL.println("****************************************************");
    	SERIAL.println("Task            ABS             %Util");
    	SERIAL.println("****************************************************");

    	vTaskGetRunTimeStats(ptrTaskList); //save stats to char array
    	SERIAL.println(ptrTaskList); //prints out already formatted stats

	    /*SERIAL.println("****************************************************");
	    SERIAL.println("Task            State   Prio    Stack   Num     Core" );
	    SERIAL.println("****************************************************");

	    vTaskList(ptrTaskList); //save stats to char array
	    SERIAL.println(ptrTaskList); //prints out already formatted stats

	    SERIAL.println("****************************************************");
	    SERIAL.println("[Stacks Free Bytes Remaining] ");

	    measurement = uxTaskGetStackHighWaterMark( Handle_logTask );
	    SERIAL.print("Log Thread: ");
	    SERIAL.println(measurement);

	    measurement = uxTaskGetStackHighWaterMark( Handle_accTask );
	    SERIAL.print("ACC thread: ");
	    SERIAL.println(measurement);
	    
	    measurement = uxTaskGetStackHighWaterMark( Handle_imuTask );
	    SERIAL.print("IMU Stack: ");
	    SERIAL.println(measurement);
	    
	    measurement = uxTaskGetStackHighWaterMark( Handle_gpsTask );
	    SERIAL.print("GPS Stack: ");
	    SERIAL.println(measurement);
	    
	    measurement = uxTaskGetStackHighWaterMark( Handle_irdTask );
	    SERIAL.print("IRD Stack: ");
	    SERIAL.println(measurement);

      measurement = uxTaskGetStackHighWaterMark( Handle_parTask );
	    SERIAL.print("PAR Stack: ");
	    SERIAL.println(measurement);
	    
	    measurement = uxTaskGetStackHighWaterMark( Handle_tpmTask );
	    SERIAL.print("TPM Stack: ");
	    SERIAL.println(measurement);*/

	    SERIAL.println("****************************************************");
      xSemaphoreGive( dbSem );
    }
  }

  // delete ourselves.
  // Have to call this or the system crashes when you reach the end bracket and then get scheduled.
  vTaskDelete( NULL );
}
