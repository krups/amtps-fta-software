#ifndef PACKET_H
#define PACKET_H

// logging packet structure

#define PTYPE_GGA 1 // nmea::GgaData
#define PTYPE_RMC 2 // nmea::RmcData
#define PTYPE_ACC 3 
#define PTYPE_IMU 4
#define PTYPE_TMP 5
#define PTYPE_PRS 6

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
  unsigned long t;
  float lat;     // gps latitude
  float lon;     // gps longitude
  float vel;     // gps velocity
  float alt_gps; // gps altitude
  float alt_bar; // barometer altitude
  float barp;    // capsule internal barometric pressure
  float tmp;     // capsule internal temperature
  int   irsig;   // iridium signal strength
  bool  pardep;  // parachute deployed yes/no
  uint8_t thread_status[10]; // status codes for theads
};


#endif
