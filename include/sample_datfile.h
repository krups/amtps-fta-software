#ifndef SAMPLE_DATFILE_H
#define SAMPLE_DATFILE_H

#include <FreeRTOS_SAMD51.h>
#include "config.h"
#include "packet.h"

extern volatile bool gb1Full;
extern volatile bool gb2Full;
extern SemaphoreHandle_t ledSem;
extern SemaphoreHandle_t sdSem;
extern void ledError(int type);
void ledOk();

// sample a data file taking numPackets of each type ptype and put them
// sequenctially into the output buffer
// cam only sample one packet type at a time
int sample_datfile(uint8_t ptype, int numToSample, unsigned char *output)
{
  File logfile;
  int bytesRead = 0;
  int outputPos = 0;
  int numSampled = 0;
  uint32_t fileSize = 0;
  int curBlock = 0;
  int numPackets = 0, curPacket = 0;
  int i, offset, stride, strideCount;
  double fstride;
  char type;

  if( gb1Full || gb2Full ){
    return ERR_SD_BUSY;
  }

  if ( xSemaphoreTake( ledSem, ( TickType_t ) 5 ) == pdTRUE ) {
    ledError( ERR_4 );
    xSemaphoreGive( ledSem );
  }

//  #if DEBUG
//  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
//    Serial.print("SAMPLE: acquiring SD lock to open file ");
//    Serial.print(filename);
//    Serial.println();
//    xSemaphoreGive( dbSem );
//  }
//  #endif

  // first open the specified log file and count how many packets of the
  // specified type exist in the file
  if ( xSemaphoreTake( sdSem, ( TickType_t ) 400 ) == pdTRUE ) {
    // make sure file exists
    if (! SD.exists(filename)) {
      return ERR_SD_BUSY;
    }
    // open log file and read file info
    logfile = SD.open(filename, FILE_READ);

    if( !logfile ){
//      #if DEBUG
//      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
//        Serial.print("SAMPLE: unable to open file ");
//        Serial.print(filename);
//        Serial.println();
//        xSemaphoreGive( dbSem );
//      }
//      #endif
      return  ERR_SD_BUSY;
    }

    fileSize = logfile.size();

    if( numToSample == 1){
      logfile.seek(fileSize - LOGBUF_BLOCK_SIZE);

    }

    // get timestamp of first logged data
    while( logfile.position() < fileSize ){
      // go until the end of data in this block
      while( (type = logfile.read()) != 0 ){
        // check if its a packet we want to sample
        if( type == ptype ) {
          numPackets++;
          if( numToSample == 1){
            logfile.seek(fileSize);
          }
        }

        // calculate number of bytes until next packet
        if     ( type==PTYPE_TMP   ) offset = sizeof (tc_t);
        //else if( type==PTYPE_SPEC ) offset = sizeof (spec_t);
        else if( type==PTYPE_ACC  ) offset = sizeof (acc_t);
        else if( type==PTYPE_IMU  ) offset = sizeof (imu_t);
        else if( type==PTYPE_GGA  ) offset = sizeof (gga_t);
        else if( type==PTYPE_RMC  ) offset = sizeof (rmc_t);
        else if( type==PTYPE_BAR  ) offset = sizeof (bar_t);
        else if( type==PTYPE_PRS  ) offset = sizeof (prs_t);
        //else if( type==PTYPE_PACKET ) offset = sizeof (packet_t);
        else break; // we are lost, break so we seek to next block

//        #if DEBUG
//        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
//          Serial.print("  SAMPLE: seeking to:  ");
//          Serial.println(logfile.position()+offset);
//          xSemaphoreGive( dbSem );
//        }
//        #endif

        // slurp the rest of the packet depending on what type it is
        logfile.seek( logfile.position() + offset );
      }
      // seek to the next block
      logfile.seek( ++curBlock * LOGBUF_BLOCK_SIZE );
    }
    // close the file
    logfile.close();
    xSemaphoreGive( sdSem );
  } else {
    return ERR_SD_BUSY;
  }

  // are there at least as many packets as we requested in the dat file?
  if( numToSample > numPackets ){
    numToSample = numPackets;
  }

  if( numPackets == 0 ){
//    #if DEBUG
//    if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
//      Serial.println("SAMPLE: file has no packets");
//      xSemaphoreGive( dbSem );
//    }
//    #endif
    return ERR_SD_BUSY;
  }

  stride = floor( (double)(numPackets) / (double)(numToSample) );
  if( stride < 1 ) stride = 1;
  strideCount = 0;

//  #if DEBUG
//  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
//    Serial.print(F("SAMPLE: file size: "));
//    Serial.print(fileSize);
//    Serial.print(F(", num packets of type ( "));
//    Serial.print(ptype);
//    Serial.print("): ");
//    Serial.print(numPackets);
//    Serial.print(", numToSample: ");
//    Serial.print(numToSample);
//    Serial.print(", stride: ");
//    Serial.println(stride);
//    xSemaphoreGive( dbSem );
//  }
//  #endif


  // now re-open the file and
  if ( xSemaphoreTake( sdSem, ( TickType_t ) 400 ) == pdTRUE ) {

    curBlock = 0;
    // open log file and read file info
    logfile = SD.open(filename, FILE_READ);

    // if we only want one packet, take it from the most recent buffer write
    // this wont necessarily give the latest packet, but it will be within the most recent 
    // write to disk at least
    if( numToSample == 1 ){
      logfile.seek(fileSize - LOGBUF_BLOCK_SIZE);
    }

    // loop through blocks until we go through the file or get enough samples
    while( (logfile.position() < fileSize) && (numSampled < numToSample) ){
      // go until the end of data in this last block
      // and find the most recent timestamp recorded by a packet
      // first read the packet type which should be nonzero otherwise
      // we have found the end of the file
      while( (type = logfile.read()) != 0 ){

        // calculate number of bytes until next packet
        if     ( type==PTYPE_TMP   ) offset = sizeof (tc_t);
        //else if( type==PTYPE_SPEC ) offset = sizeof (spec_t);
        else if( type==PTYPE_ACC  ) offset = sizeof (acc_t);
        else if( type==PTYPE_IMU  ) offset = sizeof (imu_t);
        else if( type==PTYPE_GGA  ) offset = sizeof (gga_t);
        else if( type==PTYPE_RMC  ) offset = sizeof (rmc_t);
        else if( type==PTYPE_BAR  ) offset = sizeof (bar_t);
        else if( type==PTYPE_PRS  ) offset = sizeof (prs_t);
        //else if( type==PTYPE_PACKET ) offset = sizeof (packet_t);
        else break; // we are lost, break so we seek to next block

        if( type==ptype ) curPacket++;

        // check if its a packet we want to sample
        // if it is, read it into the output buffer
        if( (type == ptype) && ( (strideCount == stride) || (numToSample==1 && curPacket==numPackets))){
          strideCount = 0;
          if( numSampled < numToSample ){
            //output[outputPos++] = type;
            bytesRead = logfile.read( &output[outputPos], offset);
            outputPos += offset;
            numSampled++;
          } else {
            break;
          }
        } else { // we didnt read from the file
          // if it was the right packet type but we need to stride past it
          // make sure to increment the stride counter
          if( type == ptype ) strideCount++;

          // slurp the rest of the packet depending on what type it is
          logfile.seek( logfile.position() + offset );
        }
      }
      // seek to the next block
      logfile.seek( ++curBlock * LOGBUF_BLOCK_SIZE );
    }
    // close the file
    logfile.close();
    xSemaphoreGive( sdSem );
  } else {
    return ERR_SD_BUSY;
  }

//  #if DEBUG
//  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
//    Serial.print("SAMPLE: sampled ");
//    Serial.print(numSampled);
//    Serial.print(" packets of type (");
//    Serial.print(ptype);
//    Serial.print("), wrote  ");
//    Serial.print(outputPos);
//    Serial.println(" bytes to output buffer");
//    xSemaphoreGive( dbSem );
//  }
//  #endif

  if ( xSemaphoreTake( ledSem, ( TickType_t ) 5 ) == pdTRUE ) {
    ledError( 1 );
    xSemaphoreGive( ledSem );
  }

  // return number of bytes written to buffer;
  return outputPos;
}



#endif
