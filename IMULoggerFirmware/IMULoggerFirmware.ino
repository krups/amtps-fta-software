/* IMU logger firmware for AMTPS-FTA project
Matt Ruffner 2021
This subsystem is responsible for logging 6 axis IMU data and high g accelerometer
data to an SD card at 100HZ from each
*/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <FreeRTOS_SAMD51.h>
#include <SerialTransfer.h>
#include <Adafruit_NeoPixel.h>
#include <semphr.h>
#include <SD.h>
#include <SerialCommands.h>
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

#define DDSERIAL Serial

#define DEBUG 1
//#define DEBUG_ACC 1
//#define DEBUG_VERBOSE 1

#define NUM_LOG_FILES 1 // only acc and imu here
#define LOGFILE              "imu00.csv"

#include "include/delay_helpers.h" // rtos delay helpers
#include "include/config.h"        // project wide defs
#include "include/packet.h"        // data packet defs
//#include "src/PCInterface.h"
#include "src/H3LIS100.h"
#include "pins.h"                  // imu logger system pinouts

Adafruit_NeoPixel pixel = Adafruit_NeoPixel(1, 8, NEO_GRB + NEO_KHZ800);

// IMU vars
ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object
#define AD0_VAL 0
#define WIRE_PORT Wire 
#define BUFFER_SAMPLE_NUM 32
#define SERIAL Serial

// High G accel sensor object
H3LIS100 accel = H3LIS100(12345);

TaskHandle_t Handle_logTask; // sd card logging task
TaskHandle_t Handle_dataTask; // 9 axis imu data collection task
TaskHandle_t Handle_dumpTask; // dump file over serial task

QueueHandle_t qAccData; // high g accelerometer data to be logged
QueueHandle_t qImuData; // 6 axis log g imu data to be logged


SemaphoreHandle_t dbSem; // serial debug logging (Serial)
SemaphoreHandle_t i2c1Sem; // i2c port 1 access semapho

void printDirectory(SerialCommands* sender, File dir, int numTabs);

void setLED(uint8_t r, uint8_t g, uint8_t b)
{
  pixel.setPixelColor(0, pixel.Color(r,g,b)); // white to signify mode
  pixel.show();
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
      setLED(0,0,0);
      myDelayMs(500);
      setLED(100,100,100);
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
  
  DDSERIAL.begin(115200);
  
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

/**********************************************************************************
/**********************************************************************************
/**********************************************************************************
/**********************************************************************************
 * 9 axis imu data collection thread
*/
static void dataThread( void *pvParameters )
{
  imu_t imuData;
  sensors_event_t event; 
  acc_t data;
  unsigned long nowtime = 0;

  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("IMU+ACC thread started");
    xSemaphoreGive( dbSem );
  }
  #endif
  
  while(1) {
  
    /*if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      SERIAL.println("reading imu ");
      xSemaphoreGive( dbSem );
    }*/
  
    //isrFired = false;
    //if ( xSemaphoreTake( i2c1Sem, ( TickType_t ) 10 ) == pdTRUE ) {  
      //myICM.getAGMT();            // get the A, G, M, and T readings
      //myICM.clearInterrupts(); 
      //xSemaphoreGive( i2c1Sem );
    //} 
    
    /*if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      SERIAL.println("read imu ");
      xSemaphoreGive( dbSem );
    }*/
    
    nowtime =  xTaskGetTickCount();

    accel.getEvent(&event);
    myICM.getAGMT();   

    imuData.t = nowtime;
    imuData.data[0] = myICM.accX();
    imuData.data[1] = myICM.accY();
    imuData.data[2] = myICM.accZ();
    imuData.data[3] = myICM.gyrX();
    imuData.data[4] = myICM.gyrY();
    imuData.data[5] = myICM.gyrZ();
    
    data.t = nowtime;
    data.data[0] = event.acceleration.x;
    data.data[1] = event.acceleration.y;
    data.data[2] = event.acceleration.z;
    
    // send the data to the data log queue
    if( xQueueSend( qImuData, ( void * ) &imuData, ( TickType_t ) 1 ) != pdTRUE ) {
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
    
    // send the data to the data log queue
    if( xQueueSend( qAccData, ( void * ) &data, ( TickType_t ) 1 ) != pdTRUE ) {
      // Failed to post the message, even after 100 ticks. 
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

    //taskYIELD();
  
    myDelayMs(8);
    

  }
  
  vTaskDelete( NULL );  
}



/**********************************************************************************
/**********************************************************************************
/**********************************************************************************
/**********************************************************************************
 * sd card logging thread
*/

static acc_t accData[50];
static imu_t imuData[50];
static int fillThresh = 40;
static char printBuffer[1000];
static void logThread( void *pvParameters )
{
  static bool ready = false;
  int numToLog;
  uint8_t imuBufferLoc=0, accBufferLoc=0;
  bool flushAcc=false, flushImu=false;
  File logfile;
  
  #ifdef DEBUG_LOG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("SD logging thread thread started");
    xSemaphoreGive( dbSem );
  }
  #endif
  
  String filename = LOGFILE;
  
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
  for( uint8_t i=0; i < 100; i++) {
    filename[3] = '0' + i/10;
    filename[4] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }

  while(1) {
   
    
    /*********************
    Only write packets if more than 400ms worth of data is queued
    *********************/

    if( flushAcc || flushImu ){
    
      pixel.setPixelColor(0, pixel.Color(0,0,150)); // Moderately bright green color.
      pixel.show();
   
      // open one log file and write all data to it (single log file configuration)
      logfile = SD.open(filename, FILE_WRITE);
      // logfile no good
      if( ! logfile ) {
        #if DEBUG
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          Serial.print("ERROR: sd logging thread couldn't open logfile for writing: ");
          Serial.println(filename);
          xSemaphoreGive( dbSem );
        }
        #endif
      }
      
      if( flushAcc ){
  
        // LOGFILE OPEN!
        for( int i=0; i<accBufferLoc; i++ ){ 
          logfile.print(accData[i].t);
          logfile.print(", ");
          logfile.print(PTYPE_ACC);
          logfile.print(", ");
          logfile.print(accData[i].data[0]);
          logfile.print(", ");
          logfile.print(accData[i].data[1]);
          logfile.print(", ");
          logfile.print(accData[i].data[2]);
          logfile.println();
        }
        flushAcc = false;
        accBufferLoc = 0;
        taskYIELD();
      }
      
      if( flushImu ){
        // IMU
        for( int i=0; i<imuBufferLoc; i++ ){
          logfile.print(imuData[i].t);
          logfile.print(", ");
          logfile.print(PTYPE_IMU);
          logfile.print(", ");
          for( int j=0; j<NUM_IMU_CHANNELS; j++ ){
            logfile.print(imuData[i].data[j]);
            if( j<NUM_IMU_CHANNELS-1 ){
              logfile.print(", ");
            } else {
              logfile.println();
            }
          }
        }
        flushImu = false;
        imuBufferLoc = 0;
        taskYIELD();
      }
    
      pixel.setPixelColor(0, pixel.Color(0,0,0)); // Moderately bright green color.
      pixel.show();
    
      // done writing to this file
      logfile.close();
    }    
        
    // check for a packet in each of the log data queues
    // HIGH G ACCELEROMETER
    if( qAccData != NULL ) {
      if( xQueueReceive( qAccData, &accData[accBufferLoc++], (TickType_t) 1 ) == pdTRUE) {
        if( accBufferLoc > fillThresh ){
          flushAcc = true;
        }
      } else {
        accBufferLoc--;
      }
    }
    
    // check for IMU DATA in queue
    if( qImuData != NULL ) {
      if( xQueueReceive( qImuData, &imuData[imuBufferLoc++], (TickType_t) 1 ) == pdTRUE) {
        if( imuBufferLoc > fillThresh ){
          flushImu = true;
        }
      } else {
        imuBufferLoc--;
      }
    }
    
  }
  
  vTaskDelete( NULL );  
}

void initializeICM() {
  
  bool initialized = false;
  while (!initialized)
  {
    myICM.begin(WIRE_PORT, AD0_VAL);

    SERIAL.print(F("Initialization of the sensor returned: "));
    SERIAL.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      SERIAL.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }
   // In this advanced example we'll cover how to do a more fine-grained setup of your sensor
  SERIAL.println("Device connected!");

  // Here we are doing a SW reset to make sure the device starts in a known state
  myICM.swReset();
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    SERIAL.print(F("Software Reset returned: "));
    SERIAL.println(myICM.statusString());
  }
  delay(250);

  // Now wake the sensor up
  myICM.sleep(false);
  myICM.lowPower(false);

  myICM.startupMagnetometer();
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    SERIAL.print(F("startupMagnetometer returned: "));
    SERIAL.println(myICM.statusString());
  }

  // The next few configuration functions accept a bit-mask of sensors for which the settings should be applied.
  myICM.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous);
  SERIAL.print(F("setSampleMode returned: "));
  SERIAL.println(myICM.statusString());

  ICM_20948_smplrt_t mySmplrt;
  mySmplrt.g = 14; //11=~100Hz
  mySmplrt.a = 14;
  myICM.setSampleRate((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), mySmplrt);
  SERIAL.print(F("setSampleRate returned: "));
  SERIAL.println(myICM.statusString());

  // Set full scale ranges for both acc and gyr
  ICM_20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors

  myFSS.a = gpm16; // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                  // gpm2
                  // gpm4
                  // gpm8
                  // gpm16

  myFSS.g = dps2000; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                    // dps250
                    // dps500
                    // dps1000
                    // dps2000

  myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    SERIAL.print(F("setFullScale returned: "));
    SERIAL.println(myICM.statusString());
  }


  SERIAL.println();
  SERIAL.println(F("Configuration complete!"));
}

void setup() {
  SERIAL.begin(115200);
  
  pinMode(ICM_INT, INPUT);
  pinMode(ICM_SYNC, INPUT); // should be OUTPUT if used, currently unused
  pinMode(LIS_INT1, INPUT);
  pinMode(LIS_INT2, INPUT);
  
  pinMode(PC_BOOT_PIN, INPUT_PULLUP);
  
  WIRE_PORT.begin();
  WIRE_PORT.setClock(800000);
  
  pixel.begin();
  pixel.setPixelColor(0, pixel.Color(0,150,0));
  pixel.show();
  
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV8);
  
  delay(5000);
  
  #ifdef DEBUG
  SERIAL.println("Starting...");
  #endif

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
  
  #ifdef DEBUG
  SERIAL.println("Created queues...");
  #endif
  
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
  
  #ifdef DEBUG
  SERIAL.println("Created semaphores...");
  #endif
  
  initializeICM();
  
    // init the sensor and try to restart the thread if this fails.. or somehow 
  // ensure that the sensor is eventually initialized
  while( !accel.begin() ){
    delay(100);
    #ifdef DEBUG
    Serial.println("High g accelerometer failed to init");

    #endif
  }
  
  pixel.setPixelColor(0, pixel.Color(150,0,0)); // Moderately bright green color.
  pixel.show();
  

  
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
    }
    if( stable ){
      pcdump = true;
    }
  } 
  
  if( pcdump ){
    pixel.setPixelColor(0, pixel.Color(75,75,75)); // white to signify mode
    pixel.show();
    
    xTaskCreate(dumpThread, "Data dump", 1024, NULL, tskIDLE_PRIORITY + 2, &Handle_dumpTask);
  } else {
    xTaskCreate(logThread, "SD Logging", 1024, NULL, tskIDLE_PRIORITY + 3, &Handle_logTask);
    xTaskCreate(dataThread, "IMU + ACC", 2000, NULL, tskIDLE_PRIORITY + 2, &Handle_dataTask);
  }
  
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
