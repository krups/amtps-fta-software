#ifndef CONFIG_H
#define CONFIG_H

#define IRIDIUM_PACKET_PERIOD 60000 // send a packet every minute
#define CHECK_SIGNAL_PERIOD   5000 // seconds
#define DIAGNOSTICS false// Change this to see diagnostics
#define SBD_TX_SZ 340

#define ALT_THRESH            1500.0f // in feet, need to be BELOW THIS
#define PRS_THRESH            90.0f // in kpa, NEED TO BE ABOVE THIS

#define NUM_TC_CHANNELS       18 // deg celcius
#define NUM_PRS_CHANNELS      5  // kPa pressure sensors
#define NUM_HF_CHANNELS       0  // W/cm^2 heat flux. units?
#define NUM_HIGHG_CHANNELS    3  // m/s^2  x,y,z of high g accel 
#define NUM_IMU_CHANNELS      6  // TODO:

#define I2CMUX_ADDR (0x70) 

// TODO: improve logfile system
#define NUM_LOG_FILES         6
#define LOGFILE0              "tmp00.csv"
#define LOGFILE1              "prs00.csv"
#define LOGFILE2              "acc00.csv"
#define LOGFILE3              "imu00.csv"
#define LOGFILE4              "gga00.csv"
#define LOGFILE5              "rmc00.csv"

#define LOGID_TMP             0
#define LOGID_PRS             1
#define LOGID_ACC             2
#define LOGID_IMU             3
#define LOGID_GGA             4
#define LOGID_RMC             5

#endif
