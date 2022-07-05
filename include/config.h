#ifndef CONFIG_H
#define CONFIG_H

#define USELEDS
#define ERR_LED_ONSTATE 1

#define IRIDIUM_PACKET_PERIOD 60000 // milliseconds, send a packet every minute
#define CHECK_SIGNAL_PERIOD   5000 // milliseconds
#define DIAGNOSTICS false// Change this to see diagnostics
#define SBD_TX_SZ 340

#define TPM_SERIAL_BAUD       115200 // uart between tpms and cdh boards

#define ALT_THRESH            1500.0f // in feet, need to be BELOW THIS
#define PRS_THRESH            100.0f // in kpa, NEED TO BE ABOVE THIS

// NUM_TC_CHANNELS + NUM_HF_CHANNELS should always be equal to the total number of MCP9600 chips (TOT_MCP_COUNT)
#define NUM_TC_CHANNELS       12 // deg celcius
#define NUM_HF_CHANNELS       0  // W/cm^2 heat flux. units?
#define TOT_MCP_COUNT         12

#define NUM_PRS_CHANNELS      5  // kPa pressure sensors
#define NUM_BAR_CHANNELS      3  // pressure in hPa, altitude in meters, capsule internal temperature in deg. celcius
#define NUM_IMU_CHANNELS      6  // only acc and IMU for now
#define NUM_HIGHG_CHANNELS    3  // x/y/z

#define I2CMUX_ADDR (0x70) 


#define TLM_SEND_PERIOD   5000 // in scheduler ticks (should be 1ms)
#define RX_TIMEOUT_PERIOD 500  // also in scheduler ticks

#define CDH_LOGBUFFERSIZE 10000


// the logfilename to use in the format [A-Z]{3}[0-9]{2}.CSV
// see https://regex101.com/
#define LOGFILE_NAME              "LG000.DAT"
#define LOGFILE_NAME_LENGTH 10 // including null terminator

#define LOGBUF_HEADER_SIZE 2048

// log buffer size in bytes (how many to accumulate before a write)
#define LOGBUF_BLOCK_SIZE         2048              // 32768 / 32
#define LOGBUF_FULL_SIZE    LOGBUF_BLOCK_SIZE - 512 // compressed iridium packet gauranteed to fit


#define ERR_BOOT              0
#define ERR_2                 1
#define ERR_3                 2
#define ERR_4                 3
#define ERR_SD_BUSY           -1
#define OK                    123

#endif
