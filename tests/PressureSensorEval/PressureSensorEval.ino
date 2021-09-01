/* Pressure sensor tester
 * 
 * for SSCSRNN015PA3A3 and SSCSRNN1.6BA7A3
 * sensors from honeywell. 
 * 
 * one of each sensor on channel 0 on an TCA9548 I2C mux
 * a single 1.6bar (103.4 kPa) sensor on channels 1,2,3 
 */

#include <Wire.h>

#define I2CMUX_ADDR 0x70
#define I2CMUX_RST  5

#include "Honeywell_ABP.h"

// create Honeywell_ABP instances
Honeywell_ABP ps1(0x38, 0, 103.4, "kpa");
Honeywell_ABP ps2(0x38, 0, 103.4, "kpa");
Honeywell_ABP ps3(0x38, 0, 103.4, "kpa");
Honeywell_ABP ps4(0x38, 0, 103.4, "kpa");
Honeywell_ABP ps5(0x78, 0, 160.0, "kpa");

#define Serial Serial1

void setup() {
  //Serial.begin(38400);
  Serial.begin(38400);
  delay(4000);

  pinMode(I2CMUX_RST, OUTPUT);

  Serial.println("Starting Pressure Sensor Test");

  digitalWrite(I2CMUX_RST, LOW);
  delay(10);
  digitalWrite(I2CMUX_RST, HIGH);

  Wire.begin();
}

void select_i2cmux_channel(uint8_t c)
{
  if( c > 7 ) c = 7;
  uint8_t val = 1 << c;
  Wire.beginTransmission(I2CMUX_ADDR);
  Wire.write(val);
  Wire.endTransmission();
}

void loop() {

  select_i2cmux_channel(0);
  ps1.update();
  select_i2cmux_channel(1);
  ps2.update();
  select_i2cmux_channel(2);
  ps3.update();
  select_i2cmux_channel(3);
  ps4.update();
  select_i2cmux_channel(4c);
  ps5.update();
  
  
  SERIAL.print(ps1.pressure()); Serial.print(", ");

  select_i2cmux_channel(1);
  ps2.update();
  Serial.print(ps2.pressure()); Serial.print(", ");
  
  select_i2cmux_channel(2);
  ps3.update();
  Serial.print(ps3.pressure()); Serial.print(", ");

  select_i2cmux_channel(3);
  ps4.update();
  Serial.print(ps4.pressure()); Serial.print(", ");

  select_i2cmux_channel(4);
  ps5.update();
  Serial.print(ps5.pressure()); Serial.println();

  delay(100);
}
