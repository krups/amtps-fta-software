// class definition of 100g acceleromter H3LIS100
// using adafruit unified sensor driver
// matt ruffner Fall 2021

#include "H3LIS100.h"

#include <avr/pgmspace.h>
#include <limits.h>
#include "pins_arduino.h"
#include "wiring_private.h"

H3LIS100::H3LIS100(int32_t sensorID) {
  _sensorID = sensorID;
}

bool H3LIS100::begin()
{
  // TODO: i2c config for sensor
  return true;
}

bool H3LIS100::getEvent(sensors_event_t *event)
{

  // get the values from the accelerometer
  float x,y,z;

  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version   = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type      = SENSOR_TYPE_ACCELEROMETER;
  event->timestamp = 0;
  event->acceleration.x = x;
  event->acceleration.y = y;
  event->acceleration.z = z;
  
  return true;
}

void H3LIS100::getSensor(sensor_t *sensor)
{
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy (sensor->name, "H3LIS100", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name)- 1] = 0;
  sensor->version     = 1;
  sensor->sensor_id   = _sensorID;
  sensor->type        = SENSOR_TYPE_ACCELEROMETER;
  sensor->min_delay   = 0;
  sensor->max_value   = 100.0F;               // -100 to 100 g
  sensor->min_value   = -100.0F;
  sensor->resolution  = 0.780F;                // 0.780 mg resolution

  /* Clear the reserved section ??? */
  //memset(sensor->reserved0, 0, sizeof(sensor->reserved0));
}


