/* 
 * AMPTS FTA Firmware
 * Command and Data Handling (CDH) subsystem
 * 
 * Matt Ruffner, University of Kentucky 2021
 */

#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
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

// software serial to GPS

// IMU interface object

// GPS data parsing object

// SD card logging object

// freertos task handles
TaskHandle_t Handle_tpmTask; // data receive from TPM subsystem task
TaskHandle_t Handle_logTask; // sd card logging task
TaskHandle_t Handle_accTask; // high g imu data collection task
TaskHandle_t Handle_imuTask; // 9 axis imu data collection task
TaskHandle_t Handle_gpsTask; // gps data receive task
TaskHandle_t Handle_irdTask; // iridium transmission task
TaskHandle_t Handle_monitorTask; // debug running task stats over uart task

// freeRTOS semaphores
SemaphoreHandle_t cdhSem; // data transmit to CDH semaphore
SemaphoreHandle_t dbSem; // serial debug logging (Serial)
SemaphoreHandle_t i2c1Sem; // i2c port 1 access semaphore



void setup() {
}

void loop() {
}
