/* 
 * AMPTS FTA Firmware
 * Temperature and Pressure Monitoring (TPM) subsystem
 * 
 * Matt Ruffner, University of Kentucky Fall 2021
 */

#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <FreeRTOS_SAMD21.h>
#include <Adafruit_MCP9600.h>
#include <SerialTransfer.h>
#include <Honeywell_ABP.h>
#include <semphr.h>

#include "include/delay_helpers.h" // rtos delay helpers
#include "include/config.h"        // project wide defs
#include "include/packet.h"        // data packet defs
#include "pins.h"                  // TPM system pinouts

#define I2CMUX_ADDR (0x70) 

#define DEBUG 1

// debug serial
#define SERIAL Serial

// data line to CDH processor
#define SERIAL_CDH Serial1

// serial transfer object for sending data to CDH processor
SerialTransfer myTransfer;


// freertos task handles
TaskHandle_t Handle_mcpTask;
TaskHandle_t Handle_prsTask;
TaskHandle_t Handle_cdhTask;
TaskHandle_t Handle_monitorTask;

// freeRTOS semaphores
SemaphoreHandle_t cdhSerialSem; // data transmit to CDH semaphore
SemaphoreHandle_t dbSem; // serial debug logging (Serial)
SemaphoreHandle_t i2c1Sem; // i2c port 1 access semaphore

// freeRTOS queues
QueueHandle_t qMcpData; // data queue for sending thermal data to CDH processor over serial
QueueHandle_t qPrsData; // data queue for sending pressure data to CDH processor over serial

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
 * Temperature and heat flux monitoring thread
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

    // assign tc temps from MCP objects to local vars
    for( int i=0; i<NUM_TC_CHANNELS; i++ ){
      current_temps[i] = readMCP(i);
      myDelayMs(5);
    }
    
    #ifdef DEBUG
    if( xTaskGetTickCount() - lastDebug > 1000 ){
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
        // print tc temps over serial
        SERIAL.print("TC temps: ");
        for( int i=0; i<NUM_TC_CHANNELS; i++ ){
          SERIAL.print(current_temps[i]); if(i<NUM_TC_CHANNELS-1) SERIAL.print(", ");
        }
        SERIAL.println();
        xSemaphoreGive( dbSem ); 
      }
      lastDebug = xTaskGetTickCount();
    }
    #endif
    
    // build data struct to send over serial
    tc_t data;
    data.t = xTaskGetTickCount();
    for( int i=0; i<NUM_TC_CHANNELS; i++ ){
      data.data[i] = current_temps[i];
    }
    
    // check cdh serial acccess
    if ( xSemaphoreTake( cdhSerialSem, ( TickType_t ) 100 ) == pdTRUE ) {
      // send TC data to CDH
      uint16_t sendSize = 0;
      uint8_t type = PTYPE_TMP;
      sendSize = myTransfer.txObj(type, sendSize);
      sendSize = myTransfer.txObj(data, sendSize);
      myTransfer.sendData(sendSize);
      xSemaphoreGive( cdhSerialSem );
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
    myDelayMs(995);

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
    
    // copy pressure data to struct to be sent
    prs_t data;
    data.t = xTaskGetTickCount();
    for( int i=0; i<NUM_PRS_CHANNELS; i++ ){
      data.data[i] = pressures[i];
    }
    
    // send pressure packet to cdh
    if ( xSemaphoreTake( cdhSerialSem, ( TickType_t ) 100 ) == pdTRUE ) {
      // send temperature and pressure data over serial
      uint8_t type = PTYPE_PRS;
      uint16_t sendSize = 0;
      sendSize = myTransfer.txObj(type, sendSize);
      sendSize = myTransfer.txObj(data, sendSize); 
      myTransfer.sendData(sendSize);
      xSemaphoreGive( cdhSerialSem );
    }
      
    // but eh more blinks
    ledToggle();
  }
  
  vTaskDelete( NULL );  
}

void ledToggle() {
  digitalWrite(PIN_LED_ACT, !digitalRead(PIN_LED_ACT));
}


/* safely read from an MCP device
 * 
*/
float readMCP(int id) {
  uint8_t muxchn = MCP_I2CMUX_CHANNELS[(int)(id/6)];
  float hot=0.0;//ambient=0.0,adcval=0.0;
  
  if ( xSemaphoreTake( i2c1Sem, ( TickType_t ) 100 ) == pdTRUE ) {
      select_i2cmux_channel( muxchn );
      myDelayMs(1);
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
  
  #if DEBUG
  SERIAL.print("Starting MCP #");SERIAL.print(id);
  #endif
  
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
    #if DEBUG
    SERIAL.println("Sensor not found. Check wiring!");
    #endif    
  } else {
    #if DEBUG
    SERIAL.println("  Found MCP9600!");
    #endif
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
 * Main setup
*/
void setup() {

  #if DEBUG
  SERIAL.begin(115200);
  #endif
  
  SERIAL_CDH.begin(TPM_SERIAL_BAUD);
  delay(3000);
  
  #if DEBUG
  SERIAL.println("Starting..");
  #endif
  
  pinMode(PIN_SCHED_CTRL, INPUT);
  pinMode(PIN_LED_ACT, OUTPUT);
  
  myTransfer.begin(SERIAL_CDH);
  
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
    ledToggle();
  }
  if (!ok) {
    #if DEBUG
    SERIAL.println("failed to start all MCP devices");
    #endif
  }
  
  // setup cdh serial port smphr
  if ( cdhSerialSem == NULL ) {
    cdhSerialSem = xSemaphoreCreateMutex();  // create mutex
    if ( ( cdhSerialSem ) != NULL )
      xSemaphoreGive( ( cdhSerialSem ) );  // make available
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
  //xTaskCreate(taskMonitor, "Task Monitor", 256, NULL, tskIDLE_PRIORITY + 3, &Handle_monitorTask);
  
  //SERIAL.println("Created Tasks, waiting for signal from CDH to start scheduler");
  
  // blink to signify waiting
  unsigned long lastToggle = 0;
  bool state = 0;
  while( digitalRead(PIN_SCHED_CTRL) == LOW ) {
    if( xTaskGetTickCount() - lastToggle > 500 ){
      digitalWrite(PIN_LED_ACT, state);
      state = !state;
      lastToggle = xTaskGetTickCount();
    }    
  }

  // Start the RTOS, this function will never return and will schedule the tasks.
  vTaskStartScheduler();

  // error scheduler failed to start
  while(1)
  {
    #if DEBUG
	  SERIAL.println("Scheduler Failed! \n");
	  #endif
	  
	  delay(1000);
  }
}

void loop() {
  // tasks!
}

