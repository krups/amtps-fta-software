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
#include <ArduinoNmeaParser.h>
#include <FreeRTOS_SAMD21.h>
#include <SerialTransfer.h>
#include <IridiumSBD.h>
#include <semphr.h>


#define NUM_TC_CHANNELS 18


//#include <Mahony_DPEng.h>
//#include <Madgwick_DPEng.h>

#include "H3LIS100.h"
#include "include/delay_helpers.h" // rtos delay helpers
#include "include/config.h"        // project wide defs
#include "include/packet.h"        // packet definitions
#include "pins.h"                  // CDH system pinouts

#define I2CMUX_ADDR (0x70) 

#define DEBUG 1 // usb serial debug switch

// debug serial
#define SERIAL      Serial  // debug serial (USB) all uses should be conditional on DEBUG define
#define SERIAL_TPM  Serial1 // to TPM subsystem
#define SERIAL_GPS  Serial2 // to gps
#define SERIAL_IRD  Serial3 // to iridium modem

// Serial transfer object for sending data to CDH processor
SerialTransfer myTransfer;

// TODO: SD card logging object

// High G accel sensor object
H3LIS100 accel = H3LIS100(12345);


// freertos task handles
TaskHandle_t Handle_tpmTask; // data receive from TPM subsystem task
TaskHandle_t Handle_logTask; // sd card logging task
TaskHandle_t Handle_accTask; // high g imu data collection task
TaskHandle_t Handle_imuTask; // 9 axis imu data collection task
TaskHandle_t Handle_gpsTask; // gps data receive task
TaskHandle_t Handle_irdTask; // iridium transmission task
TaskHandle_t Handle_monitorTask; // debug running task stats over uart task

// freeRTOS queues
QueueHandle_t qTmpData; // temperature or heat flux data to be logged
QueueHandle_t qPrsData; // pressure data to be logged
QueueHandle_t qAccData; // high g accelerometer data to be logged
QueueHandle_t qImuData; // 6 axis log g imu data to be logged
QueueHandle_t qGgaData; // GGA GPS fix data to be logged
QueueHandle_t qRmcData; // RMC GPS data to be logged

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

// IRIDIUM VARS
#define IRIDIUM_SERIAL SERIAL_IRD
#define DIAGNOSTICS false// Change this to see diagnostics
#define SBD_TX_SZ 340
IridiumSBD modem(SERIAL_IRD);


void onRmcUpdate(nmea::RmcData const rmc)
{
  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
    SERIAL.print("RMC ");

    if      (rmc.source == nmea::RmcSource::GPS)     SERIAL.print("GPS");
    else if (rmc.source == nmea::RmcSource::GLONASS) SERIAL.print("GLONASS");
    else if (rmc.source == nmea::RmcSource::Galileo) SERIAL.print("Galileo");
    else if (rmc.source == nmea::RmcSource::GNSS)    SERIAL.print("GNSS");

    SERIAL.print(" ");
    SERIAL.print(rmc.time_utc.hour);
    SERIAL.print(":");
    SERIAL.print(rmc.time_utc.minute);
    SERIAL.print(":");
    SERIAL.print(rmc.time_utc.second);
    SERIAL.print(".");
    SERIAL.print(rmc.time_utc.microsecond);

    if (rmc.is_valid)
    {
      SERIAL.print(" : LON ");
      SERIAL.print(rmc.longitude);
      SERIAL.print(" ° | LAT ");
      SERIAL.print(rmc.latitude);
      SERIAL.print(" ° | VEL ");
      SERIAL.print(rmc.speed);
      SERIAL.print(" m/s | HEADING ");
      SERIAL.print(rmc.course);
      SERIAL.print(" °");
    }

    SERIAL.println();
    xSemaphoreGive( dbSem );
  }
  #endif
  
  if( xQueueSend( qRmcData, ( void * ) &rmc, ( TickType_t ) 100 ) != pdTRUE ) {
    /* Failed to post the message, even after 100 ticks. */
    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
      SERIAL.print("ERROR: failed to put rmc data into queue");
      xSemaphoreGive( dbSem );
    }
    #endif
  }
}

// hopefully not called from an interrup by the GPS parser library
// this function takes the gps fix data and pushes it into the logging queue
void onGgaUpdate(nmea::GgaData const gga)
{
  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
    SERIAL.print("GPS UPDATE: GGA ");

    if      (gga.source == nmea::GgaSource::GPS)     SERIAL.print("GPS");
    else if (gga.source == nmea::GgaSource::GLONASS) SERIAL.print("GLONASS");
    else if (gga.source == nmea::GgaSource::Galileo) SERIAL.print("Galileo");
    else if (gga.source == nmea::GgaSource::GNSS)    SERIAL.print("GNSS");

    SERIAL.print(" ");
    SERIAL.print(gga.time_utc.hour);
    SERIAL.print(":");
    SERIAL.print(gga.time_utc.minute);
    SERIAL.print(":");
    SERIAL.print(gga.time_utc.second);
    SERIAL.print(".");
    SERIAL.print(gga.time_utc.microsecond);

    if (gga.fix_quality != nmea::FixQuality::Invalid)
    {
      SERIAL.print(" : LON ");
      SERIAL.print(gga.longitude);
      SERIAL.print(" ° | LAT ");
      SERIAL.print(gga.latitude);
      SERIAL.print(" ° | Num Sat. ");
      SERIAL.print(gga.num_satellites);
      SERIAL.print(" | HDOP =  ");
      SERIAL.print(gga.hdop);
      SERIAL.print(" m | Altitude ");
      SERIAL.print(gga.altitude);
      SERIAL.print(" m | Geoidal Separation ");
      SERIAL.print(gga.geoidal_separation);
      SERIAL.print(" m");
    }

    SERIAL.println();
    xSemaphoreGive( dbSem );
  }
  #endif
  

  if( xQueueSend( qGgaData, ( void * ) &gga, ( TickType_t ) 100 ) != pdTRUE ) {
    /* Failed to post the message, even after 100 ticks. */
    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
      SERIAL.print("ERROR: failed to put gga data into queue");
      xSemaphoreGive( dbSem );
    }
    #endif
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
    xSemaphoreGive( dbSem ); // Now free or "Give" the Serial Port for others.
  }
  #endif
  
  while(1) {

    if ( xSemaphoreTake( gpsSerSem, ( TickType_t ) 100 ) == pdTRUE ) {
      while (SERIAL_GPS.available()) {
        parser.encode((char)SERIAL_GPS.read());
      }
      xSemaphoreGive( gpsSerSem ); // Now free or "Give" the Serial Port for others.
    }
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

  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("Iridium thread started");
    xSemaphoreGive( dbSem ); // Now free or "Give" the Serial Port for others.
  }
  #endif
  
  // init the iridium library and check signal streength
  
  
  while(1) {
    // handle a thread asking to send a packet, also periodically check the signal strength
   
  }
  
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
    xSemaphoreGive( dbSem ); // Now free or "Give" the Serial Port for others.
  }
  #endif
  
  bool  newTmp = false, newPrs = false;
  tc_t  tcData;
  prs_t prsData;
  
  while(1) {
    
    
    
    // acquire lock on serial port to TPM board
    if ( xSemaphoreTake( tpmSerSem, ( TickType_t ) 100 ) == pdTRUE ) {
      
      // AAHHH
      // UNTESTED !!
      // check for data received over serial
      // HOPEFULLY each instance of 'available' > 1 will correspond to the one byte of type data 
      // followed by the full packet of actual data we want...
      // .....
      if( myTransfer.available() > 1){
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
      xSemaphoreGive( tpmSerSem ); // Now free or "Give" the Serial Port for others.
    }
    
          
    // NOW HOPEFULLY WE HAVE DATA OH BUDDTY
    
    if( newTmp ){
      // new temperature data received over serial, put it in the queue for logging
      if( xQueueSend( qTmpData, ( void * ) &tcData, ( TickType_t ) 100 ) != pdTRUE ) {
        /* Failed to post the message, even after 100 ticks. */
        #ifdef DEBUG
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
          SERIAL.print("ERROR: failed to put accel data into queue");
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
          SERIAL.print("ERROR: failed to put accel data into queue");
          xSemaphoreGive( dbSem );
        }
        #endif
      }
      newPrs = false;
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
    xSemaphoreGive( dbSem ); // Now free or "Give" the Serial Port for others.
  }
  #endif
  

  // init the sensor and try to restart the thread if this fails.. or somehow 
  // ensure that the sensor is eventually initialized
  while( !accel.begin() ){
    myDelayMs(50);
  }


  while(1) {
    // goal of 10 hz logging rate, 
    // read from the high g accel and push a message to the data log queue
    sensors_event_t event; 
    
    if ( xSemaphoreTake( i2c1Sem, ( TickType_t ) 100 ) == pdTRUE ) {
      accel.getEvent(&event);
      xSemaphoreGive( i2c1Sem ); // Now free or "Give" the Serial Port for others.
    }
    
    // handle the data in the event var
    
    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      Serial.print("High g accelerometer data: ");
      // Serial.println(thedata);
      xSemaphoreGive( dbSem ); // Now free or "Give" the Serial Port for others.
    }
    #endif
    
    acc_t data;
    data.t = xTaskGetTickCount();
    data.data[0] = event.acceleration.x;
    data.data[1] = event.acceleration.y;
    data.data[2] = event.acceleration.z;
    
    // send the data to the data log queue
    if( xQueueSend( qAccData, ( void * ) &data, ( TickType_t ) 100 ) != pdTRUE ) {
      /* Failed to post the message, even after 100 ticks. */
      #ifdef DEBUG
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
        SERIAL.print("ERROR: failed to put accel data into queue");
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

  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("9 axis imu thread started");
    xSemaphoreGive( dbSem ); // Now free or "Give" the Serial Port for others.
  }
  #endif
  
  // make sure the IMU is properly configured

  while(1) {
    // goal log rate of 100Hz for accelerometer and gyroscope data 
    
    // can log queue handle >100 message passes per second??
    
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

  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("sd logging thread thread started");
    xSemaphoreGive( dbSem ); // Now free or "Give" the Serial Port for others.
  }
  #endif
  
  // open file, figure out logging format
  

  while(1) {
    // check for a packet in the log data queue
    // if there is a packet, write it to the respective log file dependin on the type
    
    // one by one, check all the packet queueus and either print them out in the debiug case or
    // write them to sd card or both whatevererr
    
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
  
  SERIAL_TPM.begin(115200); // init serial to tpm subsystem
  SERIAL_GPS.begin(9600); // init gps serial
  SERIAL_IRD.begin(9600); // init iridium serial
  


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
  qPrsData = xQueueCreate( 10, sizeof( struct prs_t *) );
  if( qPrsData == NULL ) {
    /* Queue was not created and must not be used. */
    #if DEBUG
    SERIAL.println("Failed to create qPrsData queue");
    #endif
  }
  // high g accel data queue
  qAccData = xQueueCreate( 10, sizeof( struct acc_t ) );
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
  qGgaData = xQueueCreate( 20, sizeof( nmea::GgaData ) );
  if( qGgaData == NULL ) {
    /* Queue was not created and must not be used. */
    #if DEBUG
    SERIAL.println("Failed to create qGgaData queue");
    #endif
  }
  // rmc gps data queue
  qRmcData = xQueueCreate( 20, sizeof( nmea::RmcData ) );
  if( qRmcData == NULL ) {
    /* Queue was not created and must not be used. */
    #if DEBUG
    SERIAL.println("Failed to create qRmcData queue");
    #endif
  }  
  // should take action if not all queues were created properly
  
  
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
  
  /**************
  * CREATE TASKS
  **************/
  xTaskCreate(tpmThread, "TPM Communication", 512, NULL, tskIDLE_PRIORITY + 3, &Handle_tpmTask);
  xTaskCreate(logThread, "SD Logging", 512, NULL, tskIDLE_PRIORITY + 3, &Handle_logTask);
  xTaskCreate(accThread, "High-g Accel", 512, NULL, tskIDLE_PRIORITY + 3, &Handle_accTask);
  xTaskCreate(imuThread, "9 Axis IMU", 512, NULL, tskIDLE_PRIORITY + 3, &Handle_imuTask);
  xTaskCreate(gpsThread, "GPS Reception", 512, NULL, tskIDLE_PRIORITY + 3, &Handle_gpsTask);
  xTaskCreate(irdThread, "9 Axis IMU", 512, NULL, tskIDLE_PRIORITY + 3, &Handle_irdTask);
  //xTaskCreate(taskMonitor, "Task Monitor", 256, NULL, tskIDLE_PRIORITY + 3, &Handle_monitorTask);
  
  
  SERIAL.println("Created Tasks");

  // Start the RTOS, this function will never return and will schedule the tasks.
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
