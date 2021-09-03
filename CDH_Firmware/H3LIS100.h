// class definition of 100g acceleromter H3LIS100
// using adafruit unified sensor driver
// matt ruffner Fall 2021

#ifndef H3LIS100_H
#define H3LIS100_H

#if ARDUINO >= 100
 #include "Arduino.h"
 #include "Print.h"
#else
 #include "WProgram.h"
#endif

#include <Adafruit_Sensor.h>

class H3LIS100 : public Adafruit_Sensor {

 public: 
  /* Constructor */
  H3LIS100(int32_t);

  bool begin();
  bool getEvent(sensors_event_t*);
  void getSensor(sensor_t*);
  
 private:
   int32_t _sensorID;
};

#endif
