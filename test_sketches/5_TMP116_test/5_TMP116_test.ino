/* 5_TMP116_test.ino
 *  
 *  Works with RevB board and RevC VNCL4040 heart sensor breakout
 *  that was carrying a TMP117 (not TMP116).
 *  The TMP117 returns 2.56C every couple of readings, which is almost
 *  certainly some kind of misread/misconversion of the lowest 
 *  or highest bit value (hex 8000 or 7FFF). Need to explore this more.
 *  
 */

#include "ClosedCube_TMP116.h" // https://github.com/closedcube/ClosedCube_TMP116_Arduino
                               // also requires https://github.com/closedcube/ClosedCube_I2C_Arduino

ClosedCube::Sensor::TMP116 tmp116;

#define REDLED 11   // red LED
#define GRNLED 8    // green LED
#define VREG_EN 24  // voltage regulator enable

void setup() {
  Wire.begin();
  Serial.begin(57600);
  pinMode(REDLED, OUTPUT);
  pinMode(GRNLED, OUTPUT);
  digitalWrite(REDLED, HIGH); // set high to turn OFF
  digitalWrite(GRNLED, HIGH); // set high to turn OFF
  pinMode(VREG_EN, OUTPUT);
  digitalWrite(VREG_EN, HIGH); // set low to turn off, high to turn on (~150usec to wake)
  Serial.println("TMP116 Test");
  tmp116.address(0x48); // Sensor I2C address either 0x48 or x49
  Serial.print("Device ID: 0x");
  Serial.println(tmp116.readDeviceId(), HEX); // Device ID = 0x116  

}

void loop() {
  Serial.print("T=");
  Serial.print(tmp116.readTemperature());
  Serial.println("C");

  delay(300);

}
