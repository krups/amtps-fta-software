// Groundstation firmware for AMTPS-FTA project
// Matt Ruffner 2021
// This software is meant to run on the receiving end of debug radio 
// and interpret in flight telemetry from the capsule, display on OLED 
// and log to SD card

// https://medium.com/@benjaminmbrown/real-time-data-visualization-with-d3-crossfilter-and-websockets-in-python-tutorial-dba5255e7f0e

#include <SPI.h>
#include <Wire.h>
//#include <U8g2lib.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <Adafruit_NeoPixel.h>
#include <FreeRTOS_SAMD21.h>
#include <SerialCommands.h>
#include <semphr.h>
#include <SD.h>

//#define DEBUG 1
#define DEBUG_RAD 1
//#define PRINT_RX_STATS 1

#include "include/delay_helpers.h" // rtos delay helpers
#include "include/config.h"        // project wide defs
#include "include/packet.h"        // data packet defs
#include "include/commands.h"      // command spec
#include "pins.h"                  // groundstation system pinouts

// freertos task handles
TaskHandle_t Handle_radTask;
TaskHandle_t Handle_serTask;

// freeRTOS semaphores
SemaphoreHandle_t dbSem; // serial debug logging (Serial)

#define SERIAL Serial
#define SCSERIAL Serial
#define RADIO_SERIAL Serial1

volatile bool newCmdToSend = false;
volatile unsigned char cmdToSend;

// prototypes for serial commands
void printDirectory(SerialCommands* sender, File dir, int numTabs);

char serial_command_buffer_[32]; // max received command length

// serial command parser object
SerialCommands serial_commands_(&SCSERIAL, serial_command_buffer_, sizeof(serial_command_buffer_), "\r\n", " ");


// serial command handler for initiating a send of the C02 paraachute deploy
void cmd_send_pdep(SerialCommands* sender)
{
  sender->GetSerial()->print("sending parachute deploy...");
  cmdToSend = CMDID_DEPLOY_DROGUE;
  newCmdToSend = true;
  // do work
}

// serial command handler for initiating a send of the pyro cutter firing
void cmd_send_pyro(SerialCommands* sender)
{
  sender->GetSerial()->print("sending pyro cut command...");
  cmdToSend = CMDID_FIRE_PYRO;
  newCmdToSend = true;
}


// This is the default handler, and gets called when no other command matches. 
void cmd_help(SerialCommands* sender, const char* cmd)
{
  sender->GetSerial()->print("Unrecognized command [");
  sender->GetSerial()->print(cmd);
  sender->GetSerial()->println("]");
}

SerialCommand cmd_pdep_("D", cmd_send_pdep); // deploy parachute command
SerialCommand cmd_pyro_("C", cmd_send_pyro); // fire pyro cutters command

unsigned long lastSendTime = 0;

/**********************************************************************************
 *  Radio task
*/
#define RBUF_SIZE 300
#define SBUF_SIZE 240

class RYLR896 {
public:
  RYLR896() {};
  static void thread( void *pvParam );
  static uint8_t rbuf[RBUF_SIZE];
  static char sbuf[SBUF_SIZE];
  static char printbuf[1000];
  static tlm_t rxtlm;
};
tlm_t RYLR896::rxtlm;
char RYLR896::printbuf[1000];
char RYLR896::sbuf[SBUF_SIZE];
uint8_t RYLR896::rbuf[RBUF_SIZE];
//void RYLR896::thread( void *pvParameters )


//char printbuf[100];
//char sbuf[SBUF_SIZE];
//uint8_t rbuf[RBUF_SIZE];
void RYLR896::thread( void *pvParameters )
//void radioThread( void *pvParameters )
{

  tlm_t dat;
  rxtlm_t outDat;

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
  
  
  pinMode(PIN_LORA_RST, OUTPUT); // nreset of lora
  digitalWrite(PIN_LORA_RST, LOW);
  myDelayMs(100);
  digitalWrite(PIN_LORA_RST, HIGH);
  
  
  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    SERIAL.println("Reset radio");
    xSemaphoreGive( dbSem );
  }
  #endif
  
  memset(rbuf, 0, RBUF_SIZE);
  memset(sbuf, 0, SBUF_SIZE);
  
  //RADIO_SERIAL.print("AT+RESET\r\n");
  
  while(1) {
  
    if( state == 1 ){
      int len = sprintf(sbuf, "AT+ADDRESS=2\r\n");
      RADIO_SERIAL.write(sbuf, len);
      state = 2;
    }
    else if( state == 3 ){
      int len = sprintf(sbuf, "AT+NETWORKID=1\r\n");
      RADIO_SERIAL.write(sbuf, len);
      state = 4;
    }
    else if( state == 5 ){
      int len = sprintf(sbuf, "AT+PARAMETER=10,7,1,7\r\n"); // recommended for less than 3km
      //int len = sprintf(sbuf, "AT+PARAMETER=12,4,1,7\r\n"); // recommended for more than 3km
      RADIO_SERIAL.write(sbuf, len);
      state = 6;
    }
  
    if( newCmdToSend && state == 7){
      // START SENDING 
      const int dataSize = sizeof(uint8_t);
      sprintf(sbuf, "AT+SEND=1,%d,", dataSize); // where 2 is the address
      int pre = strlen(sbuf);
         
      // embed command
      sbuf[pre] = cmdToSend;
      
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
      RADIO_SERIAL.write(sbuf, pre+dataSize+2);
      //SERIAL_LOR.write(sbuf, pre);
      
      // go to state 8 so that we wait for a response
      state = 8;

      newCmdToSend = false;
    }

    // handle incoming message from LORA radio
    // AT message from module
    bool eol = false;
    int pos = 0;
    bool timeout = false;
    int sawComma = 0;
    unsigned long timeoutStart = 0;
    int expectedDataLen = 0;
    
    char rbts[4]; // receive buffer text length i.e "127"
    int rbtsLen = 0; // number of chars in rbts
    int payloadSize = 0;

    
    if( RADIO_SERIAL.peek() == '+' ){
      //SERIAL.print("in peek '");
      //SERIAL.write((char)RADIO_SERIAL.peek());
    
      unsigned long timeoutStart = xTaskGetTickCount();
      while(!eol && !timeout && pos < RBUF_SIZE-1) {

        if( RADIO_SERIAL.available() ){
          rbuf[pos] = RADIO_SERIAL.read();
          if( pos > 1 ){
           
            // look for payload length in receive at command
            // +RCV=50,5,HELLO,-99,40
            if( sawComma == 1 ){
              if( rbuf[pos] == ',' ){
                sawComma = 2;
                if( rbtsLen < 4 ){
                  rbts[rbtsLen] = 0;
                }
                payloadSize = atoi(rbts);
                
              } else {
                rbts[rbtsLen] = rbuf[pos];
                rbtsLen++;
              }
            }
            if( ( sawComma == 0) && ( rbuf[pos] == ',' ) ){
              sawComma = 1;
            }
          
            /*
            if( rbuf[pos]=='\n' && rbuf[pos-1]=='\r' && sawComma==2 ){
              memset(&rbuf[pos+1], 0, RBUF_SIZE-(pos+1));
              eol = true;
              Serial.println("saw eol");
            }
            */
            
            if( rbuf[pos]=='\n' && rbuf[pos-1]=='\r' ){
              memset(&rbuf[pos+1], 0, RBUF_SIZE-(pos+1));
              eol = true;
            }
          }
          
          if( pos++ >= RBUF_SIZE ){
            break;
          }
        }
        if( xTaskGetTickCount() - timeoutStart > 5000 ){
          memset(rbuf, 0, RBUF_SIZE);
          timeout = true;
        }
      }
    
    
      if( timeout ){
        #ifdef DEBUG
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          SERIAL.println("Radio rx timed out");
          xSemaphoreGive( dbSem );
        }
        #endif
      } else if (!timeout && eol) {
        #ifdef DEBUG
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          SERIAL.println("Radio got packet!");
          xSemaphoreGive( dbSem );
        }
        #endif
      } else if( !timeout && !eol) {
        #ifdef DEBUG
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
          #ifdef DEBUG
          if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
            SERIAL.print("We think we got a +READY or +OK message, we actually got: ");
            SERIAL.write(rbuf, pos);
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
            state = 7;
            #ifdef DEBUG_RAD
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
            SERIAL.print("We think we got a message with an '=' in it, we actually got: ");
            SERIAL.write(rbuf, pos);
            xSemaphoreGive( dbSem );
          }
          #endif
          
          
          // check if its a receive message
          if( rbuf[0]=='+' &&
              rbuf[1]=='R' &&
              rbuf[2]=='C' &&
              rbuf[3]=='V'){
            
            
            #ifdef PRINT_RX_STATS
            //int pblen = sprintf(printbuf, "Received %d bytes from address %d\n  rssi: %d, snr: %d\n", datalen, addr, rssi, snr);
            if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
              //SERIAL.write(printbuf, pblen);
              SERIAL.print("data: "); SERIAL.write(rbuf, pos);
              SERIAL.println();
              xSemaphoreGive( dbSem );
            }
            #endif
            
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
            
            // assume that data coming from the capsule 
            // is a specific data structure for now
            memcpy((void*)&rxtlm, (void *)&rbuf[dataPos], sizeof(tlm_t));
            
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
                      
            
            String latstr = String(rxtlm.lat);
            String lonstr = String(rxtlm.lon);
            String velstr = String(rxtlm.vel);
            String alt_gpsstr = String(rxtlm.alt_gps);
            String alt_barstr = String(rxtlm.alt_bar);
            String barpstr = String(rxtlm.barp);
            String tmpstr = String(rxtlm.tmp);
            String tcstr = "";
            String nanstr = "nan";
            
            if( latstr.compareTo(nanstr) == 0 ){
              latstr = "\"nan\"";
            }
            
            for( int i=0; i<NUM_TC_CHANNELS; i++ ){
              String istr = String(i+1);
              String tstr = String(rxtlm.tc.data[i]);
              if( tstr.compareTo("nan")==0 ){
                tstr = "'nan'";
              }
              tcstr += "\"tc" + istr + "\":" + tstr;
              if( i < NUM_TC_CHANNELS-1 ){
                tcstr += ",";
              }
            }
            String prstr = "";
            for( int i=0; i<NUM_PRS_CHANNELS; i++ ){
              String istr = String(i+1);
              String pstr = String(rxtlm.prs.data[i]);
              if( pstr.compareTo("nan")==0 ){
                pstr = "'nan'";
              }
              prstr += "\"prs" + istr + "\":" + pstr;
              if( i < NUM_PRS_CHANNELS-1 ){
                prstr += ",";
              }
            }
            
            // print out all received data in JSON format
            int dlen = sprintf(printbuf, "{\"time\": %d,\"lat\":%s,\"lon\":%s,\"vel\":%s,\"alt_gps\":%s,\"alt_bar\":%s,\"barp\":%s,\"tmp\":%s,\"irsig\":%d,\"pardep\":%d,\"tc\":{%s},\"prs\":{%s}}",
                   rxtlm.t,
                   latstr.c_str(),
                   lonstr.c_str(),
                   velstr.c_str(),
                   alt_gpsstr.c_str(),
                   alt_barstr.c_str(),
                   barpstr.c_str(),
                   tmpstr.c_str(),
                   rxtlm.irsig,
                   (uint8_t)rxtlm.pardep,
                   tcstr.c_str(),
                   prstr.c_str());
            


            if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
              SERIAL.write(printbuf, dlen);
              SERIAL.println();
              xSemaphoreGive( dbSem );
            }
            
            
            
          }
          
        }
      }
    }
    
    taskYIELD();
  }
  
  vTaskDelete( NULL );  
}

RYLR896 radio;

void serialTask( void *param ){

  serial_commands_.SetDefaultHandler(cmd_help);
	serial_commands_.AddCommand(&cmd_pdep_);
	serial_commands_.AddCommand(&cmd_pyro_);
  
  while (1) {
    serial_commands_.ReadSerial();
  }
  
  vTaskDelete (NULL);

}

void setup() {
  SERIAL.begin(115200);
  delay(10);
  RADIO_SERIAL.begin(115200);
  delay(10);
  
  delay(4000);
  
  #if DEBUG
  SERIAL.println("Starting...");
  #endif

  // CREATE RTOS QUEUES 
  
  // setup debug serial log semaphore
  if ( dbSem == NULL ) {
    dbSem = xSemaphoreCreateMutex();  // create mutex
    if ( ( dbSem ) != NULL )
      xSemaphoreGive( ( dbSem ) );  // make available
  }
  
  /**************
  * CREATE TASKS
  **************/
  xTaskCreate(RYLR896::thread, "Radio Control", 1000, NULL, tskIDLE_PRIORITY + 2, &Handle_radTask);
  xTaskCreate(serialTask, "Serial Interface", 1000, NULL, tskIDLE_PRIORITY + 2, &Handle_serTask);
  //xTaskCreate(taskMonitor, "Task Monitor", 256, NULL, tskIDLE_PRIORITY + 4, &Handle_monitorTask);
  
  #if DEBUG
  SERIAL.println("Created tasks...");
  #endif
  
  delay(100);
  
  // start the scheduler
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
