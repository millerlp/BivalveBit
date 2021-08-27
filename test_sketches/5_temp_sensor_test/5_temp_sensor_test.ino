/* 5_temp_sensor_test.ino
 *  
 *  Use this to test the temperature sensor mounted on the heart rate
 *  sensor board. Works with RevB BivalveBit board and RevC VNCL4040 
 *  heart sensor breakout that was carrying a TMP117 (not TMP116).
 *  
 */

#include <Wire.h>            // Used to establish serial communication on the I2C bus
#include <SparkFun_TMP117.h> // https://github.com/sparkfun/SparkFun_TMP117_Arduino_Library

// The default address of the device is 0x48 = (GND)
TMP117 TMP117sensor; // Initalize sensor

#define REDLED 11   // red LED
#define GRNLED 8    // green LED
#define VREG_EN 24  // voltage regulator enable

void setup() {
  Wire.begin();
  Serial.begin(57600);
  Wire.setClock(400000);   // Set clock speed to be the fastest for better communication (fast mode)

  pinMode(REDLED, OUTPUT);
  pinMode(GRNLED, OUTPUT);
  digitalWrite(REDLED, HIGH); // set high to turn OFF
  digitalWrite(GRNLED, HIGH); // set high to turn OFF
  pinMode(VREG_EN, OUTPUT);
  digitalWrite(VREG_EN, HIGH); // set low to turn off, high to turn on (~150usec to wake)
  if (TMP117sensor.begin() == true) // Function to check if the sensor will correctly self-identify with the proper Device ID/Address
  {
    Serial.println("Begin");
  }  else {
    Serial.println("Device failed to setup- Freezing code.");
    while (1){
      digitalWrite(REDLED,!digitalRead(REDLED));
      delay(100);
      // Runs forever
    }
  }
}

void loop() {
  // Data Ready is a flag for the conversion modes - in continous conversion the dataReady flag should always be high
  if (TMP117sensor.dataReady() == true) // Function to make sure that there is data ready to be printed, only prints temperature values when data is ready
  {
    float tempC = TMP117sensor.readTempC();
    // Print temperature in °C
    Serial.println(tempC);
    digitalWrite(GRNLED,!digitalRead(GRNLED));
    delay(300); // Delay added for easier readings
  }

}
