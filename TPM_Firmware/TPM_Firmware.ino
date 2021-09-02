/* 
 * AMPTS FTA Firmware
 * Temperature and Pressure Monitoring (TPM) subsystem
 * 
 * Matt Ruffner, University of Kentucky 2021
 */

#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <FreeRTOS_SAMD21.h>
#include <Adafruit_MCP9600.h>
#include <Honeywell_ABP.h>
#include <semphr.h>



#define NUM_TC_CHANNELS 18


//#include <Mahony_DPEng.h>
//#include <Madgwick_DPEng.h>

// rtos delay helpers
#include "delay_helpers.h"
#include "pins.h" // TPM system pinouts

#define I2CMUX_ADDR (0x70) 

#define DEBUG 1

// debug serial
#define SERIAL Serial

// data line to CDH processor
#define SERIAL_CDH Serial1



// freertos task handles
TaskHandle_t Handle_mcpTask;
TaskHandle_t Handle_prsTask;
TaskHandle_t Handle_cdhTask;
TaskHandle_t Handle_monitorTask;
// freeRTOS semaphores
SemaphoreHandle_t cdhSem; // data transmit to CDH semaphore
SemaphoreHandle_t dbSem; // serial debug logging (Serial)
SemaphoreHandle_t i2c1Sem; // i2c port 1 access semaphore

// create Honeywell_ABP instances
Honeywell_ABP ps1(0x38, 0, 103.4, "kpa");
Honeywell_ABP ps2(0x38, 0, 103.4, "kpa");
Honeywell_ABP ps3(0x38, 0, 103.4, "kpa");
Honeywell_ABP ps4(0x38, 0, 103.4, "kpa");
Honeywell_ABP ps5(0x78, 0, 160.0, "kpa");

Adafruit_MCP9600 mcps[18];

// I2C addresses on MCP breakout board, use this to assign proper channels 
// need to specify per breakout if addressing not consistent between boards
const uint8_t MCP_ADDRS[6] = {0x62, 0x61, 0x60, 0x63, 0x64, 0x67};
// which i2c channel the 6xMCP boards are connected to
const uint8_t MCP_I2CMUX_CHANNELS[3] = {0, 1, 2};

/**********************************************************************************
 * Pressure monitoring thread
*/
static void mcpThread( void *pvParameters )
{
  float current_temps[NUM_TC_CHANNELS];  
  unsigned long lastDebug = 0;

  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("MCP thread started");
    xSemaphoreGive( dbSem );
  }
  #endif
  
  while(1) {
    //myDelayMs(1000);

    for( int i=0; i<NUM_TC_CHANNELS; i++ ){
      current_temps[i] = readMCP(i);
      myDelayMs(50);
    }
    
    #ifdef DEBUG
    if( millis() - lastDebug > 1000 ){
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
        // print tc temps over serial
        SERIAL.print("TC temps: ");
        for( int i=0; i<NUM_TC_CHANNELS; i++ ){
          SERIAL.print(current_temps[i]); if(i<NUM_TC_CHANNELS-1) SERIAL.print(", ");
        }
        SERIAL.println();
        xSemaphoreGive( dbSem ); 
      }
      lastDebug = millis();
    }
    #endif
    
    // assign tc temps from MCP objects to local vars
    
    // check cdh serial acccess
    if ( xSemaphoreTake( cdhSem, ( TickType_t ) 5 ) == pdTRUE ) {
      // send TC data to CDH
      
      xSemaphoreGive( cdhSem );
    }
      
    // but eh more blinks
    ledToggle();
  }
  
  vTaskDelete( NULL );  
}


/**********************************************************************************
 * Pressure monitoring thread
*/
static void prsThread( void *pvParameters )
{
  float pressures[5];  
  

  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("Pressure sensing thread started");
    xSemaphoreGive( dbSem ); // Now free or "Give" the Serial Port for others.
  }
  #endif
  
  while(1) {
    myDelayMs(1000);

    if ( xSemaphoreTake( i2c1Sem, ( TickType_t ) 100 ) == pdTRUE ) {
      select_i2cmux_channel(3);
      ps1.update();
      select_i2cmux_channel(4);
      ps2.update();
      select_i2cmux_channel(5);
      ps3.update();
      select_i2cmux_channel(6);
      ps4.update();
      select_i2cmux_channel(7);
      ps5.update();
      xSemaphoreGive( i2c1Sem ); // Now free or "Give" the Serial Port for others.
    }
    
    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
      Serial.print("Pressures: ");
      SERIAL.print(ps1.pressure()); SERIAL.print(", ");
      SERIAL.print(ps2.pressure()); SERIAL.print(", ");
      SERIAL.print(ps3.pressure()); SERIAL.print(", ");
      SERIAL.print(ps4.pressure()); SERIAL.print(", ");
      SERIAL.print(ps5.pressure()); SERIAL.println();
      xSemaphoreGive( dbSem ); // Now free or "Give" the Serial Port for others.
    }
    #endif
    
    pressures[0] = ps1.pressure();
    pressures[1] = ps2.pressure();
    pressures[2] = ps3.pressure();
    pressures[3] = ps4.pressure();
    pressures[4] = ps5.pressure();
    
    // check rosserial publish semaphore wait 5 ticks if not available
    if ( xSemaphoreTake( cdhSem, ( TickType_t ) 5 ) == pdTRUE ) {
      // send temperature and pressure data over serial
      
      xSemaphoreGive( cdhSem );
    }
      
    // but eh more blinks
    ledToggle();
  }
  
  vTaskDelete( NULL );  
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

	    SERIAL.println("****************************************************");
	    SERIAL.println("Task            State   Prio    Stack   Num     Core" );
	    SERIAL.println("****************************************************");

	    vTaskList(ptrTaskList); //save stats to char array
	    SERIAL.println(ptrTaskList); //prints out already formatted stats

	    SERIAL.println("****************************************************");
	    SERIAL.println("[Stacks Free Bytes Remaining] ");

	    measurement = uxTaskGetStackHighWaterMark( Handle_mcpTask );
	    SERIAL.print("TC Thread: ");
	    SERIAL.println(measurement);

	    measurement = uxTaskGetStackHighWaterMark( Handle_prsTask );
	    SERIAL.print("Pressure thread: ");
	    SERIAL.println(measurement);
	    
	    measurement = uxTaskGetStackHighWaterMark( Handle_monitorTask );
	    SERIAL.print("Monitor Stack: ");
	    SERIAL.println(measurement);

	    SERIAL.println("****************************************************");
      xSemaphoreGive( dbSem );
    }
  }

  // delete ourselves.
  // Have to call this or the system crashes when you reach the end bracket and then get scheduled.
  vTaskDelete( NULL );
}

void ledToggle() {
  digitalWrite(LED_ACT, !digitalRead(LED_ACT));
}


/* safely read from an MCP device
 * 
*/
float readMCP(int id) {
  uint8_t muxchn = MCP_I2CMUX_CHANNELS[(int)(id/6)];
  float hot=0.0;//ambient=0.0,adcval=0.0;
  
  if ( xSemaphoreTake( i2c1Sem, ( TickType_t ) 100 ) == pdTRUE ) {
      select_i2cmux_channel( muxchn );
      myDelayMs(5);
      // TODO: use struct for data transfer + read adc and convert to heat flux 
      hot     = mcps[id].readThermocouple();
      //ambient = mcps[id].readAmbient();
      //adcval  = mcps[id].readADC();
      xSemaphoreGive( i2c1Sem );
  }

  return hot;
}


// initialize an MCP device (0-17)
// made to run before task scheduler is started
bool initMCP(int id) {
  bool ok = true;
  uint8_t muxchn = MCP_I2CMUX_CHANNELS[(int)(id/6)];
  uint8_t addr = MCP_ADDRS[id%6];
  uint8_t tries = 0;
  uint8_t max_tries = 100;
  
  SERIAL.print("Starting MCP #");SERIAL.print(id);
  
  select_i2cmux_channel( muxchn );
  delay(10);
  //if ( xSemaphoreTake( i2c1Sem, ( TickType_t ) 100 ) == pdTRUE ) {
      while ( !mcps[id].begin(addr) && tries < max_tries) {
        //myDelayMs(100);
        delay(10);
        tries++;
      }
  //    xSemaphoreGive( i2c1Sem );
  //}
  if( tries == max_tries) {
    ok = false;
    SERIAL.println("Sensor not found. Check wiring!");
  } else {
    SERIAL.println("  Found MCP9600!");
  }
  
  mcps[id].setADCresolution(MCP9600_ADCRESOLUTION_14);
  /*SERIAL.print("ADC resolution set to ");
  switch (mcps[id].getADCresolution()) {
    case MCP9600_ADCRESOLUTION_18:   SERIAL.print("18"); break;
    case MCP9600_ADCRESOLUTION_16:   SERIAL.print("16"); break;
    case MCP9600_ADCRESOLUTION_14:   SERIAL.print("14"); break;
    case MCP9600_ADCRESOLUTION_12:   SERIAL.print("12"); break;
  }
  SERIAL.println(" bits");*/

  mcps[id].setThermocoupleType(MCP9600_TYPE_K);
  /*SERIAL.print("Thermocouple type set to ");
  switch (mcps[id].getThermocoupleType()) {
    case MCP9600_TYPE_K:  SERIAL.print("K"); break;
    case MCP9600_TYPE_J:  SERIAL.print("J"); break;
    case MCP9600_TYPE_T:  SERIAL.print("T"); break;
    case MCP9600_TYPE_N:  SERIAL.print("N"); break;
    case MCP9600_TYPE_S:  SERIAL.print("S"); break;
    case MCP9600_TYPE_E:  SERIAL.print("E"); break;
    case MCP9600_TYPE_B:  SERIAL.print("B"); break;
    case MCP9600_TYPE_R:  SERIAL.print("R"); break;
  }
  SERIAL.println(" type");*/

  mcps[id].setFilterCoefficient(3);
  //SERIAL.print("Filter coefficient value set to: ");
  //SERIAL.println(mcps[id].getFilterCoefficient());

  mcps[id].enable(true);

  return ok;
}

void select_i2cmux_channel(uint8_t c)
{
  if( c > 7 ) c = 7;
  uint8_t val = 1 << c;
  Wire.beginTransmission(I2CMUX_ADDR);
  Wire.write(val);
  Wire.endTransmission();
}

/**********************************************************************************
 * Pressure monitoring thread
*/
void setup() {
  SERIAL.begin(115200);
  SERIAL_CDH.begin(115200);
  delay(3000);
  SERIAL.println("Starting..");
  
  // status signal configuration
  ledToggle();
  
  Wire.begin();
  Wire.setClock(100000);

  //delay(10);
  //select_i2cmux_channel(1);
  
  //TODO: watchdog or to reset device if MCP initialization fails
  
  // initialize all MCP9600 sensors
  bool ok = true;
  for( int i=0; i<18; i++) {
    ok &= initMCP(i);
    delay(10);
  }
  if (!ok) {
    SERIAL.println("failed to start all MCP devices");
  }
  
  // setup cdh serial port smphr
  if ( cdhSem == NULL ) {
    cdhSem = xSemaphoreCreateMutex();  // create mutex
    if ( ( cdhSem ) != NULL )
      xSemaphoreGive( ( cdhSem ) );  // make available
  }
  // setup debug serial log semaphore
  if ( dbSem == NULL ) {
    dbSem = xSemaphoreCreateMutex();  // create mutex
    if ( ( dbSem ) != NULL )
      xSemaphoreGive( ( dbSem ) );  // make available
  }
  // setup pressureSensor data access semaphore
  if ( i2c1Sem == NULL ) {
    i2c1Sem = xSemaphoreCreateMutex();  // create mutex
    if ( ( i2c1Sem ) != NULL )
      xSemaphoreGive( ( i2c1Sem ) );  // make available
  }
  
  xTaskCreate(mcpThread, "MCP9600 Conversion", 512, NULL, tskIDLE_PRIORITY + 3, &Handle_mcpTask);
  xTaskCreate(prsThread, "Pressure Sensing", 512, NULL, tskIDLE_PRIORITY + 3, &Handle_prsTask);
  //xTaskCreate(cdhThread, "Data Transmission", 512, NULL, tskIDLE_PRIORITY + 3, &Handle_cdhTask);
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
}

