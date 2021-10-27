#ifndef CONFIG_H
#define CONFIG_H

#define IRIDIUM_PACKET_PERIOD 60000 // milliseconds, send a packet every minute
#define CHECK_SIGNAL_PERIOD   5000 // milliseconds
#define DIAGNOSTICS false// Change this to see diagnostics
#define SBD_TX_SZ 340

#define TPM_SERIAL_BAUD       34800 // uart between tpms and cdh boards

#define ALT_THRESH            1500.0f // in feet, need to be BELOW THIS
#define PRS_THRESH            100.0f // in kpa, NEED TO BE ABOVE THIS

#define NUM_TC_CHANNELS       18 // deg celcius
#define NUM_PRS_CHANNELS      5  // kPa pressure sensors
#define NUM_HF_CHANNELS       0  // W/cm^2 heat flux. units?
#define NUM_BAR_CHANNELS      3  // pressure in hPa, altitude in meters, capsule internal temperature in deg. celcius

#define I2CMUX_ADDR (0x70) 


#define TLM_SEND_PERIOD   1000 // in scheduler ticks (should be 1ms)
#define RX_TIMEOUT_PERIOD 500  // also in scheduler ticks


// TODO: improve logfile system
#define NUM_LOG_FILES         5
#define LOGFILE0              "tmp00.csv"
#define LOGFILE1              "prs00.csv"
#define LOGFILE2              "gga00.csv"
#define LOGFILE3              "rmc00.csv"
#define LOGFILE4              "bar00.csv"

#define LOGID_TMP             0
#define LOGID_PRS             1
#define LOGID_GGA             2
#define LOGID_RMC             3
#define LOGID_BAR             4

#endif
