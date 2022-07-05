/* 
 * AMPTS FTA Firmware
 * Command and Data Handling (CDH) subsystem
 * 
 * Matt Ruffner, University of Kentucky Fall 2021
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <Adafruit_MPL3115A2.h>
#include <ArduinoNmeaParser.h>
#include <FreeRTOS_SAMD51.h>
#include <SerialTransfer.h>
#include <SerialCommands.h>
#include <IridiumSBD.h>
#include <semphr.h>
#include <SD.h>
#include <SPI.h>
#include <Servo.h>
#include "wiring_private.h"


#define DEBUG 1 // usb serial debug switch
#ifdef DEBUG
  #define DEBUG_GPS 1 // print raw gga to serial 
  //#define DEBUG_QUEUE 1 // print info on log queue operations
  //#define DEBUG_RAD_VERBOSE 1
  //#define DEBUG_VERBOSE 1
  //#define DEBUG_BARO 1
  //#define DEBUG_IRD 1
  #define DEBUG_LOG 1
  //#define DEBUG_PAR 1
  #define DEBUG_RAD 1
  #define DEBUG_DUMP 1
  //#define DEBUG_TPMS_TRANSFER 1
#endif

bool sendPackets = 0;

//#include <Mahony_DPEng.h>
//#include <Madgwick_DPEng.h>

#include "include/delay_helpers.h" // rtos delay helpers
#include "include/config.h"        // project wide defs
#include "include/packet.h"        // packet definitions
#include "include/commands.h"      // command spec
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

// global logfile name
char filename[LOGFILE_NAME_LENGTH] = LOGFILE_NAME;


// include here to things are already defined
#include "include/sample_datfile.h"


// debug serial
#define SERIAL      Serial  // debug serial (USB) all uses should be conditional on DEBUG define
#define SERIAL_TPM  Serial1 // to TPM subsystem
#define SERIAL_GPS  Serial3 // to gps
#define SERIAL_IRD  Serial2 // to iridium modem
#define SERIAL_LOR  Serial4 // to telemetry radio
#define DDSERIAL    Serial // for data dump thread

// Serial transfer object for receiving data from TPM processor
SerialTransfer myTransfer;

// capsule internal barometric pressure object
// and struct for passing data through queue
Adafruit_MPL3115A2 baro;


// freertos task handles
TaskHandle_t Handle_tpmTask; // data receive from TPM subsystem task
TaskHandle_t Handle_logTask; // sd card logging task
TaskHandle_t Handle_gpsTask; // gps data receive task
TaskHandle_t Handle_irdTask; // iridium transmission task
TaskHandle_t Handle_parTask; // parachute deployment task
TaskHandle_t Handle_barTask; // barometric sensor task
TaskHandle_t Handle_radTask; // telem radio task handle
TaskHandle_t Handle_monitorTask; // debug running task stats over uart task
TaskHandle_t Handle_dumpTask; // data transfer over serial task

// freeRTOS queues
// QueueHandle_t qTmpData; // temperature or heat flux data to be logged
// QueueHandle_t qPrsData; // pressure data to be logged
// QueueHandle_t qGgaData; // GGA GPS fix data to be logged
// QueueHandle_t qRmcData; // RMC GPS data to be logged
// QueueHandle_t qBarData; // barometric pressure data

// freeRTOS semaphores
SemaphoreHandle_t tpmSerSem; // data from CDH semaphore
SemaphoreHandle_t dbSem; // serial debug logging (Serial)
SemaphoreHandle_t i2c1Sem; // i2c port 1 access semaphore
SemaphoreHandle_t gpsSerSem; // gps serial port acces
SemaphoreHandle_t irdSerSem; // iridium serial semaphore
SemaphoreHandle_t depSem; // deployment status protector
SemaphoreHandle_t sigSem; // iridium signal protector
SemaphoreHandle_t wbufSem; // SD buffer write semaphore
SemaphoreHandle_t ledSem; // neopixel sepaphore
SemaphoreHandle_t sdSem;

// for radio task
#define RBUF_SIZE 260
#define SBUF_SIZE 240
uint8_t rbuf[RBUF_SIZE];
char sbuf[SBUF_SIZE];
char printbuf[100];

// for sd logging
static uint8_t logBuf1[LOGBUF_BLOCK_SIZE];
static uint8_t logBuf2[LOGBUF_BLOCK_SIZE];
volatile uint32_t logBuf1Pos = 0, // current write index in buffer1
         logBuf2Pos = 0; // current write index in buffer2
volatile uint8_t activeLog = 1;   // which buffer should be used fo writing, 1 or 2
volatile bool gb1Full = false, gb2Full = false;

volatile bool globalDeploy = false;
volatile bool irSig = 0;
volatile bool packetReady = 0;
int gPacketSize = 0;
char gIrdBuf[SBD_TX_SZ];


// GPS update callbacks
void onRmcUpdate(nmea::RmcData const);
void onGgaUpdate(nmea::GgaData const);

// GPS parser object
ArduinoNmeaParser parser(onRmcUpdate, onGgaUpdate);

// IRIDIUM MODEM OBJECT
IridiumSBD modem(SERIAL_IRD);

// Parachute C02 servo setup
#define CO2SERVO_POS_ACT  17
#define CO2SERVO_POS_HOME 150
Servo co2servo;

Adafruit_NeoPixel led(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

void ledError(int type) {
  switch (type) {
  case ERR_BOOT:
    led.setPixelColor(0, led.Color(150, 0, 0));
    break;
  case ERR_2:
    led.setPixelColor(0, led.Color(150, 0, 150));
    break;
  case ERR_3:
    led.setPixelColor(0, led.Color(150, 150, 0));
    break;
  case ERR_4:
    led.setPixelColor(0, led.Color(0, 150, 150));
    break;
  default:
    led.setPixelColor(0, led.Color(100, 100, 100));
  }
  led.show();
}

void ledOk() {
  led.setPixelColor(0, led.Color(0, 150, 0));
  led.show();
}

void logStruct(uint8_t type, char* data, size_t size)
{
  // try to write  data to the SD log buffer
  if ( xSemaphoreTake( wbufSem, ( TickType_t ) 500 ) == pdTRUE ) {

    if( activeLog == 1 ){
      // is this the last data we will put in before considering the
      // buffer full?
      logBuf1[logBuf1Pos++] = type; // set packet type byte
      memcpy(&logBuf1[logBuf1Pos], data, size);
      logBuf1Pos += size;
      if( logBuf1Pos >= LOGBUF_FULL_SIZE ){
        activeLog = 2;
        logBuf1Pos = 0;
        gb1Full = true;
      }
    } else if( activeLog == 2 ){
      // is this the last data we will put in before considering the
      // buffer full?
      logBuf2[logBuf2Pos++] = type; // set packet type byte
      memcpy(&logBuf2[logBuf2Pos], data, size);
      logBuf2Pos += size;
      if( logBuf2Pos >= LOGBUF_FULL_SIZE ){
        activeLog = 1;
        logBuf2Pos = 0;
        gb2Full = true;
      }
    }

    xSemaphoreGive( wbufSem );
  }
}

void initSD(SerialCommands* sender)
{
	// list files on SD card
	// sender->GetSerial()->print("Initializing SD card...");
  if (!SD.begin(PIN_SD_CS)) {
    #if DEBUG
    sender->GetSerial()->println("initialization failed. Things to check:");
    sender->GetSerial()->println("1. is a card inserted?");
    sender->GetSerial()->println("2. is your wiring correct?");
    sender->GetSerial()->println("3. did you change the chipSelect pin to match your shield or module?");
    sender->GetSerial()->println("Note: press reset or reopen this serial monitor after fixing your issue!");
    #endif
    while (true){
      ledError(0);
      myDelayMs(500);
      ledError(-1);
      myDelayMs(500);
    }
  }
}

// remove files on SD card
void cmd_rm(SerialCommands* sender)
{
  initSD(sender);
  File root = SD.open("/");
  clearFiles(sender, root);
  root.close();
  sender->GetSerial()->println();
}

// list files on SD card
void cmd_ls(SerialCommands* sender)
{
  initSD(sender);
  File root = SD.open("/");
  printDirectory(sender, root, 0);
  root.close();
  sender->GetSerial()->println();
}

void cmd_dump(SerialCommands* sender)
{
	//Note: Every call to Next moves the pointer to next parameter

	char* file_str = sender->Next();
	if (file_str == NULL)
	{
		sender->GetSerial()->println("Need filename as argument");
		return;
	}
	
	initSD(sender);
  
  File dataFile = SD.open(file_str);
  if (dataFile) {
    while (dataFile.available()) {
      sender->GetSerial()->write(dataFile.read());
    }
    dataFile.close();
  } else {
    sender->GetSerial()->print("error opening ");
    sender->GetSerial()->println(file_str);
  }
  sender->GetSerial()->println();
}

void clearFiles(SerialCommands* sender, File dir) {

  while (true) {
    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    if (entry.isDirectory()) {
      clearFiles(sender, entry);
    } else {
      if ( strcmp(entry.name(), "CONFIG.TXT") != 0){
        SD.remove(entry.name());
      }
    }
  }
}


void printDirectory(SerialCommands* sender, File dir, int numTabs) {

  while (true) {
    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++) {
      sender->GetSerial()->print('\t');
    }
    sender->GetSerial()->print(entry.name());
    if (entry.isDirectory()) {
      sender->GetSerial()->println("/");
      printDirectory(sender, entry, numTabs + 1);
    } else {
      // files have sizes, directories do not
      sender->GetSerial()->print("\t\t");
      sender->GetSerial()->println(entry.size(), DEC);
    }
    entry.close();
  }
}

//This is the default handler, and gets called when no other command matches. 
void cmd_unrecognized(SerialCommands* sender, const char* cmd)
{
  sender->GetSerial()->print("Unrecognized command [");
  sender->GetSerial()->print(cmd);
  sender->GetSerial()->println("]");
}

void onRmcUpdate(nmea::RmcData const rmc)
{
  rmc_t data;

  #ifdef DEBUG_GPS
  //if (rmc.is_valid) {
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 200 ) == pdTRUE ) {
      writeRmc(rmc, SERIAL);  
      xSemaphoreGive( dbSem );
    }
  //}
  #endif

  data.t = xTaskGetTickCount();
  data.time[0] = (uint16_t)rmc.time_utc.hour;
  data.time[1] = (uint16_t)rmc.time_utc.minute;
  data.time[2] = (uint16_t)rmc.time_utc.second;
  data.time[3] = (uint16_t)rmc.time_utc.microsecond;
  data.lat = rmc.latitude;
  data.lon = rmc.longitude;
  data.speed = rmc.speed;
  data.course = rmc.course;

  logStruct(PTYPE_RMC, (char*)(&data), sizeof(rmc_t));
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
  gga_t data;

  #ifdef DEBUG_GPS
  if (gga.fix_quality != nmea::FixQuality::Invalid) {
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 200 ) == pdTRUE ) {
      writeGga(gga, SERIAL);
      xSemaphoreGive( dbSem );
    }
  }
  #endif

  data.t = xTaskGetTickCount();
  data.time[0] = (uint16_t)gga.time_utc.hour;
  data.time[1] = (uint16_t)gga.time_utc.minute;
  data.time[2] = (uint16_t)gga.time_utc.second;
  data.time[3] = (uint16_t)gga.time_utc.microsecond;
  data.lat = gga.latitude;
  data.lon = gga.longitude;
  data.hdop = gga.hdop;
  data.alt = gga.altitude;

  logStruct(PTYPE_GGA, (char*)(&data), sizeof(gga_t));
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
  bar_t data;

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
      data.prs = baro.getPressure();
      data.alt = baro.getAltitude();
      data.tmp = baro.getTemperature();
      xSemaphoreGive( i2c1Sem );
    }

    data.t = xTaskGetTickCount();

    // try to write this data into the current log buffer
    logStruct(PTYPE_BAR, (char*)(&data), sizeof(bar_t));
    
    #ifdef DEBUG_BARO
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      Serial.print("pressure = "); Serial.print(data.prs); Serial.println(" hPa");
      Serial.print("altitude = "); Serial.print(data.alt); Serial.println(" m");
      Serial.print("temperature = "); Serial.print(data.tmp); Serial.println(" C");
      
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
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          Serial.write(SERIAL_GPS.peek());
          xSemaphoreGive( dbSem );
        }
        parser.encode((char)SERIAL_GPS.read());
      }
      xSemaphoreGive( gpsSerSem );
    }
    
    myDelayMs(20);
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
    // IS IT TIME TO SEND A PACKKAGE??
    if( xTaskGetTickCount() - lastPacketSend > IRIDIUM_PACKET_PERIOD && 
        (mSq > 0) && 
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
    
    myDelayMs(50);
    
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
    
    //myDelayMs(1);
    
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

           #ifdef DEBUG_TPMS_TRANSFER
          if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
            Serial.println("TPM:  PROBLEM");
            xSemaphoreGive( dbSem );
          }
          #endif
        }
      } else {
        //  #ifdef DEBUG_TPMS_TRANSFER
        //   if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
        //     Serial.println("TPM:  transfer not available");
        //     xSemaphoreGive( dbSem );
        //   }
        //   #endif
      }
      xSemaphoreGive( tpmSerSem );
    }
    
          
    // NOW HOPEFULLY WE HAVE DATA OH BUDDTY
    
    if( newTmp ){
      // new temperature data received over serial, put it in a log buffer
      logStruct(PTYPE_TMP, (char*)(&tcData), sizeof(tc_t));
      newTmp = false;  

      #ifdef DEBUG
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
        SERIAL.println("TPM: wrote to tc logfile: ");
        xSemaphoreGive( dbSem );
      }
      #endif
    }
    
    if( newPrs ){
      // new pressure data receive over serial, put it in a log file
      logStruct(PTYPE_PRS, (char*)(&prsData), sizeof(prs_t));
      newPrs = false; 

      #ifdef DEBUG
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
        SERIAL.println("TPM: wrote to prs logfile");
        xSemaphoreGive( dbSem );
      }
      #endif
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

//static uint8_t logRingBuffer[CDH_LOGBUFFERSIZE];

static void logThread( void *pvParameters )
{
  static bool ready = false;
  bool b1full = false, b2full = false;
  uint32_t written = 0;
  uint32_t filesize = 0;
  File logfile;

  #ifdef DEBUG_LOG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("sd logging thread thread started");
    xSemaphoreGive( dbSem );
  }
  #endif
  
  // wait indefinitely for the SD mutex
  while( xSemaphoreTake( sdSem, portMAX_DELAY ) != pdPASS );
  // wait indefinitely for the writebuf mutex
  while( xSemaphoreTake( wbufSem, portMAX_DELAY ) != pdPASS );

  memset(logBuf1, 0, LOGBUF_BLOCK_SIZE);
  memset(logBuf2, 0, LOGBUF_BLOCK_SIZE);

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

  // CREATE UNIQUE FILE NAMES (UP TO 1000)
  for( int i=0; i < 1000; i++) {
    filename[2] = '0' + (int)(i/100);
    filename[3] = '0' + (i-((int)(i/100)))/10;
    filename[4] = '0' + i%10;

    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }

  // write a header block to the file to start
  // the first two bytes are the block size
  logfile = SD.open(filename, FILE_WRITE);
  *(uint16_t*)(&logBuf1[0]) = (uint16_t)(LOGBUF_BLOCK_SIZE);

  // the second 128 bytes are packet types (as 16 bit uints)
  *(uint16_t*)(&logBuf1[128]) = (uint16_t)(PTYPE_ACC);
  *(uint16_t*)(&logBuf1[130]) = (uint16_t)(PTYPE_IMU);
  *(uint16_t*)(&logBuf1[132]) = (uint16_t)(PTYPE_TMP);
  *(uint16_t*)(&logBuf1[134]) = (uint16_t)(PTYPE_PRS);
  *(uint16_t*)(&logBuf1[136]) = (uint16_t)(PTYPE_BAR);
  *(uint16_t*)(&logBuf1[138]) = (uint16_t)(PTYPE_RMC);
  *(uint16_t*)(&logBuf1[140]) = (uint16_t)(PTYPE_GGA);
  *(uint16_t*)(&logBuf1[142]) = (uint16_t)(PTYPE_TLM);

  // the third 128 bytes are packet sizes ( as 16 bit uints)
  *(uint16_t*)(&logBuf1[256]) = (uint16_t)(sizeof(acc_t));
  *(uint16_t*)(&logBuf1[258]) = (uint16_t)(sizeof(imu_t));
  *(uint16_t*)(&logBuf1[260]) = (uint16_t)(sizeof(tc_t));
  *(uint16_t*)(&logBuf1[262]) = (uint16_t)(sizeof(prs_t));
  *(uint16_t*)(&logBuf1[264]) = (uint16_t)(sizeof(bar_t));
  *(uint16_t*)(&logBuf1[266]) = (uint16_t)(sizeof(rmc_t));
  *(uint16_t*)(&logBuf1[268]) = (uint16_t)(sizeof(gga_t));
  *(uint16_t*)(&logBuf1[270]) = (uint16_t)(sizeof(tlm_t));
  logfile.write(logBuf1, LOGBUF_BLOCK_SIZE);
  logfile.close();

  memset(logBuf1, 0, LOGBUF_BLOCK_SIZE);

  xSemaphoreGive( wbufSem );
  xSemaphoreGive( sdSem );

  #if DEBUG_LOG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.print("Wrote header to filename: ");
    Serial.println(filename);
    xSemaphoreGive( dbSem );
  }
  #endif

  while(1) {
    if( !gb1Full && !gb2Full ){
      myDelayMs(10);
      taskYIELD();
      continue;
    }

    // wait indefinitely for the SD mutex
    //while( xSemaphoreTake( sdSem, portMAX_DELAY ) != pdPASS );
    if ( xSemaphoreTake( sdSem, ( TickType_t ) 1000 ) == pdTRUE ) {

      // open log file for all telem (single log file configuration)
      logfile = SD.open(filename, FILE_WRITE);

      filesize = logfile.size();

      // if we could not open the file
      if( ! logfile ) {
        #if DEBUG
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          Serial.print("ERROR: sd logging thread couldn't open logfile for writing: ");
          Serial.println(filename);
          xSemaphoreGive( dbSem );
        }
        #endif
      } else {
        #if DEBUG_LOG
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          Serial.print("SDLOG: logfile size is ");
          Serial.print(filesize);
          Serial.println(" bytes");
          xSemaphoreGive( dbSem );
        }
        #endif
      }

      // we assume we can open it and start writing the buffer
      // check if one of the buffers is full and lets write it
      if( gb1Full ){

        logfile.seek(logfile.size());

        written = logfile.write(logBuf1, LOGBUF_BLOCK_SIZE);

        // clean the buffer so we are gauranteed to see zeros where there hasn't
        // been recetn data written to
        memset(logBuf1, 0, LOGBUF_BLOCK_SIZE);

        #if DEBUG_LOG
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          Serial.print("SD: wrote  ");
          Serial.print(written);
          Serial.print("/");
          Serial.print(LOGBUF_BLOCK_SIZE);
          Serial.println(" bytes from logBuf1");
          xSemaphoreGive( dbSem );
        }
        #endif

        gb1Full = false;
      }

      // check the other buffer
      if( gb2Full ){

        logfile.seek(logfile.size());

        written = logfile.write(logBuf2, LOGBUF_BLOCK_SIZE);

        // clean the buffer so we are gauranteed to see zeros where there hasn't
        // been recetn data written to
        memset(logBuf2, 0, LOGBUF_BLOCK_SIZE);

        #if DEBUG_LOG
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          Serial.print("SD: wrote  ");
          Serial.print(written);
          Serial.print("/");
          Serial.print(LOGBUF_BLOCK_SIZE);
          Serial.println(" bytes from logBuf2");
          xSemaphoreGive( dbSem );
        }
        #endif

        gb2Full = false;
      }

      // done writing to this file
      logfile.close();

      xSemaphoreGive( sdSem );
    }

    myDelayMs(10);
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
  gga_t ggaData;
  float alt=1.3e6, prs=0;
  bool deployed = false;
  int temp = 0;

  #ifdef DEBUG_PAR
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("parachute deployment thread started");
    xSemaphoreGive( dbSem );
  }
  #endif
  
  while(1) {
    // peek at gga and pressure queues to see if activation criteria have been met
    
    myDelayMs(1000);
    
    if (deployed) continue;
    
    // PRESSURE
    if( ( temp = sample_datfile(PTYPE_PRS, 1, (unsigned char *)(&prsData)) ) != ERR_SD_BUSY) {
      //bytesRead = temp;
      #ifdef DEBUG_PAR
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
        Serial.println("PAR: read pressure data from logfile");
        xSemaphoreGive( dbSem );
      }
      #endif
    }
        
    // GPS
    // aslo need to compare system time and GPS altiture
    if( ( temp = sample_datfile(PTYPE_GGA, 1, (unsigned char *)(&ggaData)) ) != ERR_SD_BUSY) {
      //bytesRead = temp;
      #ifdef DEBUG_PAR
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
        Serial.println("PAR: read GPS data from logfile");
        xSemaphoreGive( dbSem );
      }
      #endif
    }
    
    //if( ( alt <= ALT_THRESH ) && ( prs >= PRS_THRESH ) ){ // DEPLOYYY!
    if( ( prs >= PRS_THRESH ) ){
      deployed = true;
    }
    
    // TODO: implement manual deployment over debug radio
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

static void radThread( void *pvParameters )
{

  unsigned long lastSendTime = 0;
  int temp = 0;
  tlm_t dataOut;
  gga_t ggaData;
  rmc_t rmcData;
  tc_t tmpData;
  prs_t prsData;
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
  
  #ifdef DEBUG_RAD_VERBOSE
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    SERIAL.println("LORA: Reset radio");
    xSemaphoreGive( dbSem );
  }
  #endif
  
  
  memset(rbuf, 0, RBUF_SIZE);
  memset(sbuf, 0, SBUF_SIZE);
  
  #ifdef DEBUG_RAD_VERBOSE
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    SERIAL.println("LORA: zerod buffers");
    xSemaphoreGive( dbSem );
  }
  #endif
  
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
      #ifdef DEBUG_RAD_VERBOSE
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
        Serial.println("LORA: sent at+address, setting state to 2");
        xSemaphoreGive( dbSem );
      }
      #endif
      taskYIELD();
      //myDelayMs(10);
    }
    else if( state == 3 ){
      int len = sprintf(sbuf, "AT+NETWORKID=1\r\n");
      SERIAL_LOR.write(sbuf, len);
      state = 4;
      #ifdef DEBUG_RAD_VERBOSE
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
        Serial.println("LORA: sent at+networkid, setting state to 4");
        xSemaphoreGive( dbSem );
      }
      #endif
      //myDelayMs(10);
      taskYIELD();
    }
    else if( state == 5 ){
      int len = sprintf(sbuf, "AT+PARAMETER=10,7,1,7\r\n"); // less than 3km
      SERIAL_LOR.write(sbuf, len);
      state = 6;
      #ifdef DEBUG_RAD_VERBOSE
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
        Serial.println("LORA: sent at+param, setting state to 6");
        xSemaphoreGive( dbSem );
      }
      #endif
      //myDelayMs(10);
      taskYIELD();
    }
    
    if( xTaskGetTickCount() - lastSendTime > TLM_SEND_PERIOD && state == 7){

      if( ( temp = sample_datfile(PTYPE_GGA, 1, (unsigned char *)(&ggaData)) ) != ERR_SD_BUSY) {
        //bytesRead = temp;
        #ifdef DEBUG_RAD
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          Serial.println("RAD: read GGA GPS data from logfile");
          xSemaphoreGive( dbSem );
        }
        #endif
      }

      // if( ( temp = sample_datfile(PTYPE_RMC, 1, (unsigned char *)(&rmcData)) ) != ERR_SD_BUSY) {
      //   //bytesRead = temp;
      //   #ifdef DEBUG_RAD
      //   if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      //     Serial.println("RAD: read RMC GPS data from logfile");
      //     xSemaphoreGive( dbSem );
      //   }
      //   #endif
      // }     
      
      if( ( temp = sample_datfile(PTYPE_TMP, 1, (unsigned char *)(&tmpData)) ) != ERR_SD_BUSY) {
        //bytesRead = temp;
        #ifdef DEBUG_RAD
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          Serial.println("RAD: read TC data from logfile");
          xSemaphoreGive( dbSem );
        }
        #endif
      }

      if( ( temp = sample_datfile(PTYPE_PRS, 1, (unsigned char *)(&prsData)) ) != ERR_SD_BUSY) {
        //bytesRead = temp;
        #ifdef DEBUG_RAD
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          Serial.println("RAD: read Pressure data from logfile");
          xSemaphoreGive( dbSem );
        }
        #endif
      }

      if( ( temp = sample_datfile(PTYPE_BAR, 1, (unsigned char *)(&barData)) ) != ERR_SD_BUSY) {
        //bytesRead = temp;
        #ifdef DEBUG_RAD
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          Serial.println("RAD: read barometer data from logfile");
          xSemaphoreGive( dbSem );
        }
        #endif
      }

      #ifdef DEBUG_RAD
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          Serial.print("RAD: tc0: ");
          Serial.print(tmpData.data[0]);
          Serial.print(", barp: ");
          Serial.println(barData.prs);
          xSemaphoreGive( dbSem );
        }
      #endif

      //
      // FILL STRUCT TO SEND TO GROUNDSTATION
      // copy in baseline telemetry
      dataOut.t = xTaskGetTickCount(); 
      dataOut.lat = ggaData.lat;
      dataOut.lon = ggaData.lon;
      dataOut.vel = rmcData.speed;
      dataOut.alt_gps = ggaData.alt;
      dataOut.alt_bar = barData.alt;
      dataOut.barp = barData.prs;
      dataOut.tmp = barData.tmp;
      dataOut.bat = 3.30 * (analogRead(PIN_VBAT) / 1023.0);
      dataOut.irsig = irSig;
      // copy in parachute status
      if ( xSemaphoreTake( depSem, ( TickType_t ) 100 ) == pdTRUE ) {
        dataOut.pardep = globalDeploy;
        xSemaphoreGive( depSem );
      }
      // copy tc and pressure data in
      memcpy(&(dataOut.tc), &tmpData, sizeof(tc_t));
      memcpy(&(dataOut.prs), &prsData, sizeof(prs_t));


      // START SENDING 
      const int dataSize = sizeof(tlm_t);
      sprintf(sbuf, "AT+SEND=2,%d,", dataSize); // where 2 is the address
      int pre = strlen(sbuf);
         
      // TODO: can we embed binary data ?????
      // now copy binary data from struct to send buffer
      memcpy(&sbuf[pre], (void*)&dataOut, dataSize);
      
      sbuf[pre+dataSize] = '\r';
      sbuf[pre+dataSize+1] = '\n';
      sbuf[pre+dataSize+2] = 0;
      
      
      #ifdef DEBUG_RAD
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
        SERIAL.print("sending radio binary packet of size ");
        SERIAL.println(pre+dataSize+2);
        SERIAL.print("actual data was (bytes): ");
        SERIAL.println(dataSize);
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
      //myDelayMs(10);
      //taskYIELD();
    }
    
  
    // handle incoming message from LORA radio
    // AT message from module
    bool eol = false;
    int pos = 0;
    bool timeout = false;
    unsigned long timeoutStart = 0;
    if( SERIAL_LOR.peek() == '+' ){
      
      #ifdef DEBUG_RAD_VERBOSE
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
        #ifdef DEBUG_RAD_VERBOSE
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          SERIAL.println("Radio got message!");
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
        while( ( rbuf[eqpos] != '=' ) &&  ( eqpos < pos) ){
          eqpos++;
        }
        
        if( eqpos == pos ){
          #ifdef DEBUG_RAD
          if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
            SERIAL.print("Received ");
            SERIAL.write(rbuf, pos);
            xSemaphoreGive( dbSem );
          }
          #endif
          
          if( state < 7 ){
            state ++;
            if( state == 7 ){
              #ifdef DEBUG_RAD_VERBOSE
              if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
                SERIAL.println("STATE = 7, successfully configured radio!");
                xSemaphoreGive( dbSem );
              }
              #endif
              taskYIELD();
            }
          } else if( state > 7 ){
            state = 7;
            #ifdef DEBUG_RAD_VERBOSE
            if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
              SERIAL.println("STATE was 8, received +OK from a data send operation!");
              xSemaphoreGive( dbSem );
            }
            #endif
            taskYIELD(); 
          }
          
        } else {
          // found an '=', parse rest of message
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
            // example rx string: +RCV=50,5,HELLO,-99,40
            const char *comma = ",";
            char *token;
            char *data;
            int rssi;
            int snr;
            int addr = -1;
            int datalen = -1;
            int cc = 0;
            
            // find start of data chunk
            int dataPos = 0;
            while( dataPos < pos){
              if( rbuf[dataPos] == ',' ){
                cc++;
                if( cc == 2 ){
                  dataPos++;
                  break;
                }
                  
              }
              dataPos++;
            }
            
            //SERIAL.print("dataPos is ");
            //SERIAL.println(dataPos);
            
            // parse target address
            token = strtok((char *) &rbuf[5], comma);
            addr = atoi(token);
            
            // extract data length
            token = strtok(NULL, comma);
            datalen = atoi(token);
            
            // get pointer to start of data 
            //data = strtok(NULL, comma);
            
            // get the rssi
            token = strtok((char *) &rbuf[8+datalen], comma);
            token = strtok(NULL, comma);
            rssi = atoi(token);
            
            // get the SNR
            token = strtok(NULL, comma);
            snr = atoi(token);
            
            #ifdef DEBUG
            int pblen = sprintf(printbuf, "Received %d bytes from address %d\n  rssi: %d, snr: %d\n", datalen, addr, rssi, snr);
            if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
              SERIAL.write(printbuf, pblen);
              //SERIAL.write(data, datalen);
              SERIAL.println((int)rbuf[dataPos]);
              xSemaphoreGive( dbSem );
            }
            #endif
            
            // check what the data was
            switch ((int)rbuf[dataPos]) {
              case CMDID_DEPLOY_DROGUE:
                #if DEBUG
                if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
                  SERIAL.println("Got deploy drogue command");
                  xSemaphoreGive( dbSem );
                }
                #endif
                break;
              case CMDID_FIRE_PYRO:
                #if DEBUG
                if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
                  SERIAL.println("Got fire pyro command");
                  xSemaphoreGive( dbSem );
                }
                #endif
                break;
            }
            
             
          }
        }
      }
    } 
    taskYIELD();
    //myDelayMs(100);
  }
  
  vTaskDelete( NULL );  
}


/*********************************************
 * /*********************************************
 * /*********************************************
 * DATA DUMP THREAD
 * */
char serial_command_buffer_[32];
SerialCommands serial_commands_(&DDSERIAL, serial_command_buffer_, sizeof(serial_command_buffer_), "\r\n", " ");
SerialCommand cmd_ls_("ls", cmd_ls);
SerialCommand cmd_dump_("dump", cmd_dump);
SerialCommand cmd_rm_("rm", cmd_rm);

static void dumpThread( void *pvParameters )
{
  #ifdef DEBUG_DUMP
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("Data dump thread started");
    xSemaphoreGive( dbSem );
  }
  #endif
  
  serial_commands_.SetDefaultHandler(cmd_unrecognized);
	serial_commands_.AddCommand(&cmd_ls_);
	serial_commands_.AddCommand(&cmd_rm_);
	serial_commands_.AddCommand(&cmd_dump_);
  
  while (1) {
    serial_commands_.ReadSerial();
    if( digitalRead(PC_BOOT_PIN) == LOW ){
      myDelayMs(1000);
      if( digitalRead(PC_BOOT_PIN) == LOW )
        NVIC_SystemReset();
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
  SERIAL_GPS.begin(115200); // init gps serial
  delay(10);
  SERIAL_IRD.begin(19200); // init iridium serial
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

  // setup neopixel
  led.begin();
  led.setPixelColor(0, led.Color(0,150,0));
  led.show();

  // for sd log dump mode
  pinMode(PC_BOOT_PIN, INPUT_PULLUP);
  
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
  // setup write buffer sem
  if ( wbufSem == NULL ) {
    wbufSem = xSemaphoreCreateMutex();  // create mutex
    if ( ( wbufSem ) != NULL )
      xSemaphoreGive( ( wbufSem ) );  // make available
  }
  // setup led sem
  if ( ledSem == NULL ) {
    ledSem = xSemaphoreCreateMutex();  // create mutex
    if ( ( ledSem ) != NULL )
      xSemaphoreGive( ( ledSem ) );  // make available
  }
  // setup sd sem
  if ( sdSem == NULL ) {
    sdSem = xSemaphoreCreateMutex();  // create mutex
    if ( ( sdSem ) != NULL )
      xSemaphoreGive( ( sdSem ) );  // make available
  }

  
  #ifdef DEBUG
  SERIAL.println("Created semaphores...");
  #endif

  /**************
  * CREATE TASKS
  **************/
  bool pcdump = false;
  
  if( digitalRead(PC_BOOT_PIN) == LOW ){
    bool stable = true;
    for( int i=0; i<10; i++ ){
      if( digitalRead(PC_BOOT_PIN) == HIGH ){
        stable = false;
        break;
      }
      delay(10);
    }
    if( stable ){
      pcdump = true;
      ledOk();
    }
  }
  // if button was held down during boot, only start the data offload thread
  if( pcdump ){
    ledError(-1); //white
    xTaskCreate(dumpThread, "Data dump", 1024, NULL, tskIDLE_PRIORITY + 2, &Handle_dumpTask);
  } 
  // otherwise start mission threads
  else {
    xTaskCreate(logThread, "SD Logging", 1024, NULL, tskIDLE_PRIORITY, &Handle_logTask);
    xTaskCreate(gpsThread, "GPS Reception", 512, NULL, tskIDLE_PRIORITY, &Handle_gpsTask);
    xTaskCreate(irdThread, "Iridium thread", 512, NULL, tskIDLE_PRIORITY, &Handle_irdTask);
    xTaskCreate(parThread, "Parachute Deployment", 512, NULL, tskIDLE_PRIORITY, &Handle_parTask);
    xTaskCreate(barThread, "Capsule internals", 512, NULL, tskIDLE_PRIORITY, &Handle_barTask);
    xTaskCreate(radThread, "Telem radio", 1000, NULL, tskIDLE_PRIORITY, &Handle_radTask);
    xTaskCreate(tpmThread, "TPM Communication", 1024, NULL, tskIDLE_PRIORITY, &Handle_tpmTask);

    //xTaskCreate(taskMonitor, "Task Monitor", 256, NULL, tskIDLE_PRIORITY + 4, &Handle_monitorTask);
    
    // signal to the TPM subsystem to start the task scheduler so that xTaskGetTickCount() is 
    // consistent between the two
    digitalWrite(PIN_TPM_SCHEDULER_CTRL, HIGH);
  }

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

