#ifndef PACKET_H
#define PACKET_H

#include "config.h"

// logging packet structure

#define PTYPE_GGA  ((unsigned char)0x01) // nmea::GgaData
#define PTYPE_RMC  ((unsigned char)0x02) // nmea::RmcData
#define PTYPE_ACC  ((unsigned char)0x03) 
#define PTYPE_IMU  ((unsigned char)0x04)
#define PTYPE_TMP  ((unsigned char)0x05)
#define PTYPE_PRS  ((unsigned char)0x06)
#define PTYPE_TLM  ((unsigned char)0x07)
#define PTYPE_BAR  ((unsigned char)0x08)

// type PTYPE_ACC
struct acc_t {
  uint32_t t;
  float data[3];
};

// type PTYPE_IMU
struct imu_t {
  uint32_t t;
  float data[6];
};

// type PTYPE_TMP
struct tc_t {
  uint32_t t;
  float data[NUM_TC_CHANNELS];
};

// type PTYPE_PRS
struct prs_t {
  uint32_t t;
  float data[NUM_PRS_CHANNELS];
};

//type PTYPE_BAR
struct bar_t {
  uint32_t t;
  float prs;
  float alt;
  float tmp;
}; 

struct rmc_t {
  uint32_t t; // microprocessor time in ms
  uint16_t time[4]; // hh:mm:ss:us GPS time
  float lat;
  float lon;
  float speed;
  float course;
};

struct gga_t {
  uint32_t t;
  uint16_t time[4];
  float lat;
  float lon;
  float hdop;
  float alt;
};

// type PTYPE_TELEM
struct tlm_t {
  uint32_t t; // system time when packet was sent in # of scheduler ticks (ms)
  float lat;     // gps latitude
  float lon;     // gps longitude
  float vel;     // gps velocity
  float alt_gps; // gps altitude
  float alt_bar; // barometer altitude
  float barp;    // capsule internal barometric pressure
  float tmp;     // capsule internal temperature
  float bat;     // battery voltage
  int   irsig;   // iridium signal strength
  bool  pardep;  // parachute deployed yes/no
  tc_t tc;      // thermocouple data
  prs_t prs;     // external pressure sensors
};

// not a packet type, used in the groundstation firmware to hold the extra radio receive info
struct rxtlm_t {
  tlm_t tlm;
  int rssi;
  int snr;
};

#endif
