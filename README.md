# amtps-fta-software
Avionics software and firmware associated with the AMTPS FTA project

CDH Firmware is runs on a feather m4 express and communicates with a coretex m0 acting as the TPM processor. Another feather m4 is the IMU preocessor (could potentially be brought into the main (CDH) board) 

Telemetry data is streamed over the ora radio on the 915Mhz band and samples of all telemetry data are perodically sent oover the iridium netowrk (need to comress this data).

Tasks qate implemented as freertos tasts and need to be split up into their own files (file per task) so that the main program isnt 1500 lines long.

SD logging could be improved too, maybe by haveing each data creating task write text for logging to a shared ring buffer and buid up a good chunk of data (>5k) before writing to SD instead of wrigin on a per packet basis like is implemented now.
