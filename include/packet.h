#ifndef PACKET_H
#define PACKET_H

// logging packet structure

#define PTYPE_GGA 1 // nmea::GgaData
#define PTYPE_RMC 2 // nmea::RmcData
#define PTYPE_ACC 3 
#define PTYPE_IMU 4
#define PTYPE_TMP 5
#define PTYPE_PRS 6
#define PTYPE_TLM 7

// type PTYPE_ACC
struct acc_t {
  unsigned long t;
  float data[3];
};

// type PTYPE_IMU
struct imu_t {
  unsigned long t;
  float data[6];
};

// type PTYPE_TMP
struct tc_t {
  unsigned long t;
  float data[NUM_TC_CHANNELS];
};

// type PTYPE_PRS
struct prs_t {
  unsigned long t;
  float data[NUM_PRS_CHANNELS];
};

//type PTYPE_BAR
struct bar_t {
  unsigned long t;
  float prs;
  float alt;
  float tmp;
}; 

// type PTYPE_TELEM
struct tlm_t {
  unsigned long t; // system time when packet was sent in # of scheduler ticks (ms)
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
