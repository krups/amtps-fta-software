/* IMU logger firmware for AMTPS-FTA project
Matt Ruffner 2021
This subsystem is responsible for logging 6 axis IMU data and high g accelerometer
data to an SD card at 100HZ from each
*/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <FreeRTOS_SAMD21.h>
#include <SerialTransfer.h>
#include <semphr.h>
#include <SD.h>


#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include "H3LIS100.h"

#define DEBUG 1
//#define DEBUG_ACC 1
//#define DEBUG_VERBOSE 1

#define NUM_LOG_FILES 2 // only acc and imu here

#include "include/delay_helpers.h" // rtos delay helpers
#include "include/config.h"        // project wide defs
#include "include/packet.h"        // data packet defs
#include "pins.h"                  // groundstation system pinouts



// High G accel sensor object
H3LIS100 accel = H3LIS100(12345);

TaskHandle_t Handle_logTask; // sd card logging task
TaskHandle_t Handle_accTask; // high g imu data collection task
TaskHandle_t Handle_imuTask; // 9 axis imu data collection task

QueueHandle_t qAccData; // high g accelerometer data to be logged
QueueHandle_t qImuData; // 6 axis log g imu data to be logged


SemaphoreHandle_t dbSem; // serial debug logging (Serial)
SemaphoreHandle_t i2c1Sem; // i2c port 1 access semapho



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
  acc_t accData;
  imu_t imuData;
  File logfile;
  
  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("sd logging thread thread started");
    xSemaphoreGive( dbSem );
  }
  #endif
  
  String filenames[NUM_LOG_FILES];
  filenames[0] = LOGFILE2; // ACC
  filenames[1] = LOGFILE3; // IMU
  
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



void setup() {
  SERIAL.begin(115200);
  
  
  delay(3000);
  
  // TODO: configure interrupt pins for acc and imu so 
  //       we can get accurate measurements at a known sample rate
  
  SERIAL.println("Starting...");

  // CREATE RTOS QUEUES 
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
  // should take action if not all queues were created properly
  
  SERIAL.println("Created queues...");
  
  // SETUP RTOS SEMAPHORES
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
  
  SERIAL.println("Created semaphores...");

  
  /**************
  * CREATE TASKS
  **************/
  xTaskCreate(logThread, "SD Logging", 1024, NULL, tskIDLE_PRIORITY + 3, &Handle_logTask);
  xTaskCreate(accThread, "High-g Accel", 512, NULL, tskIDLE_PRIORITY + 2, &Handle_accTask);
  xTaskCreate(imuThread, "9 Axis IMU", 512, NULL, tskIDLE_PRIORITY + 2, &Handle_imuTask);
  //xTaskCreate(taskMonitor, "Task Monitor", 256, NULL, tskIDLE_PRIORITY + 4, &Handle_monitorTask);
  
  // start scheduler
  vTaskStartScheduler();

  // error scheduler failed to start
  while(1)
  {
	  SERIAL.println("Scheduler Failed! \n");
	  delay(1000);
  }
}

void loop() {
  // tasks!!
}
