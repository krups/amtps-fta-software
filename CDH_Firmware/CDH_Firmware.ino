/* 
 * AMPTS FTA Firmware
 * Command and Data Handling (CDH) subsystem
 * 
 * Matt Ruffner, University of Kentucky 2021
 */

#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <ArduinoNmeaParser.h>
#include <FreeRTOS_SAMD21.h>
#include <semphr.h>


#define NUM_TC_CHANNELS 18


//#include <Mahony_DPEng.h>
//#include <Madgwick_DPEng.h>

#include "delay_helpers.h" // rtos delay helpers
#include "pins.h" // CDH system pinouts

#define I2CMUX_ADDR (0x70) 

#define DEBUG 1

// debug serial
#define SERIAL Serial

// data line to CDH processor
#define SERIAL_CDH Serial1

// software serial to iridium
#define irdSerial Serial2
//SoftwareSerial irdSerial(IRD_RX, IRD_TX);

// software serial to GPS
#define gpsSerial Serial3
//SoftwareSerial gpsSerial(GPS_RX, GPS_TX);

// IMU interface object


// SD card logging object

// freertos task handles
TaskHandle_t Handle_tpmTask; // data receive from TPM subsystem task
TaskHandle_t Handle_logTask; // sd card logging task
TaskHandle_t Handle_accTask; // high g imu data collection task
TaskHandle_t Handle_imuTask; // 9 axis imu data collection task
TaskHandle_t Handle_gpsTask; // gps data receive task
TaskHandle_t Handle_irdTask; // iridium transmission task
TaskHandle_t Handle_monitorTask; // debug running task stats over uart task

// freeRTOS queues
QueueHandle_t qGgaData;
QueueHandle_t qRmcData;
QueueHandle_t qAccData;
QueueHandle_t qImuData;


// freeRTOS semaphores
SemaphoreHandle_t tpmSerSem; // data from CDH semaphore
SemaphoreHandle_t dbSem; // serial debug logging (Serial)
SemaphoreHandle_t i2c1Sem; // i2c port 1 access semaphore
SemaphoreHandle_t gpsSerSem; // gps serial port acces
SemaphoreHandle_t irdSerSem; //

// GPS update callbacks
void onRmcUpdate(nmea::RmcData const);
void onGgaUpdate(nmea::GgaData const);

// GPS parser object
ArduinoNmeaParser parser(onRmcUpdate, onGgaUpdate);



void onRmcUpdate(nmea::RmcData const rmc)
{
  Serial.print("RMC ");

  if      (rmc.source == nmea::RmcSource::GPS)     Serial.print("GPS");
  else if (rmc.source == nmea::RmcSource::GLONASS) Serial.print("GLONASS");
  else if (rmc.source == nmea::RmcSource::Galileo) Serial.print("Galileo");
  else if (rmc.source == nmea::RmcSource::GNSS)    Serial.print("GNSS");

  Serial.print(" ");
  Serial.print(rmc.time_utc.hour);
  Serial.print(":");
  Serial.print(rmc.time_utc.minute);
  Serial.print(":");
  Serial.print(rmc.time_utc.second);
  Serial.print(".");
  Serial.print(rmc.time_utc.microsecond);

  if (rmc.is_valid)
  {
    Serial.print(" : LON ");
    Serial.print(rmc.longitude);
    Serial.print(" ° | LAT ");
    Serial.print(rmc.latitude);
    Serial.print(" ° | VEL ");
    Serial.print(rmc.speed);
    Serial.print(" m/s | HEADING ");
    Serial.print(rmc.course);
    Serial.print(" °");
  }

  Serial.println();
}

void onGgaUpdate(nmea::GgaData const gga)
{
  Serial.print("GGA ");

  if      (gga.source == nmea::GgaSource::GPS)     Serial.print("GPS");
  else if (gga.source == nmea::GgaSource::GLONASS) Serial.print("GLONASS");
  else if (gga.source == nmea::GgaSource::Galileo) Serial.print("Galileo");
  else if (gga.source == nmea::GgaSource::GNSS)    Serial.print("GNSS");

  Serial.print(" ");
  Serial.print(gga.time_utc.hour);
  Serial.print(":");
  Serial.print(gga.time_utc.minute);
  Serial.print(":");
  Serial.print(gga.time_utc.second);
  Serial.print(".");
  Serial.print(gga.time_utc.microsecond);

  if (gga.fix_quality != nmea::FixQuality::Invalid)
  {
    Serial.print(" : LON ");
    Serial.print(gga.longitude);
    Serial.print(" ° | LAT ");
    Serial.print(gga.latitude);
    Serial.print(" ° | Num Sat. ");
    Serial.print(gga.num_satellites);
    Serial.print(" | HDOP =  ");
    Serial.print(gga.hdop);
    Serial.print(" m | Altitude ");
    Serial.print(gga.altitude);
    Serial.print(" m | Geoidal Separation ");
    Serial.print(gga.geoidal_separation);
    Serial.print(" m");
  }

  Serial.println();
}



/**********************************************************************************
 * Pressure monitoring thread
*/
static void gpsThread( void *pvParameters )
{
  float pressures[5];  
  

  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("GPS thread started");
    xSemaphoreGive( dbSem ); // Now free or "Give" the Serial Port for others.
  }
  #endif
  
  while(1) {

    if ( xSemaphoreTake( gpsSerSem, ( TickType_t ) 100 ) == pdTRUE ) {
      while (gpsSerial.available()) {
        parser.encode((char)gpsSerial.read());
      }
      xSemaphoreGive( gpsSerSem ); // Now free or "Give" the Serial Port for others.
    }
  }
  
  vTaskDelete( NULL );  
}


void setup() {
  SERIAL.begin(115200); // init debug serial
  
  gpsSerial.begin(9600); // init sercom3 to gps
  
  irdSerial.begin(9600); // init sercom4 to iridium
  
  //
  // CREATE RTOS QUEUES 
  qGgaData = xQueueCreate( 5, sizeof( nmea::GgaData *) );
  if( qGgaData == NULL ) {
    /* Queue was not created and must not be used. */
    SERIAL.println("Failed to create GPS GGA data queue");
  }
  qRmcData = xQueueCreate( 5, sizeof( nmea::RmcData *) );
  if( qRmcData == NULL ) {
    /* Queue was not created and must not be used. */
    SERIAL.println("Failed to create GPS RMC data queue");
  }
  
  //qAccData = xQueueCreate( 10, sizeof( unsigned long ) );
  //qImuData = xQueueCreate( 10, sizeof( unsigned long ) );

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
  // setup debug serial log semaphore
  if ( i2c1Sem == NULL ) {
    i2c1Sem = xSemaphoreCreateMutex();  // create mutex
    if ( ( i2c1Sem ) != NULL )
      xSemaphoreGive( ( i2c1Sem ) );  // make available
  }
  // freeRTOS semaphores
SemaphoreHandle_t tpmSerSem; // data from CDH semaphore
SemaphoreHandle_t dbSem; // serial debug logging (Serial)
SemaphoreHandle_t i2c1Sem; // i2c port 1 access semaphore
SemaphoreHandle_t gpsSerSem; // gps serial port acces
SemaphoreHandle_t irdSerSem; //
  
  
}

void loop() {
  while (gpsSerial.available()) {
    parser.encode((char)gpsSerial.read());
  }
}
