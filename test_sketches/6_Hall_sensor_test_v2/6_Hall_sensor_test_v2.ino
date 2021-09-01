/* 6_Hall_sensor_test_v2.ino
 *  
 *  TODO: figure out why sensor is reading high at rest (~623)
 *  
 *  Tested with RevB hardware, on 3V3 supply and 3.9V LiIon battery
 *  This v2 version will use the internal bandgap voltage reference just for fun,
 *  but NOTE that the external voltage connected to AREF pin should be disconnected
 *  for safety (pull out the resistor  R14 on RevB board)
 *  Have the OLED hooked up also for testing purposes
 *  
 */

#include <Wire.h>
#include "SSD1306Ascii.h" // https://github.com/greiman/SSD1306Ascii
#include "SSD1306AsciiWire.h" // https://github.com/greiman/SSD1306Ascii
//#include "MusselGapeTrackerlib.h" // https://github.com/millerlp/MusselGapeTrackerlib

SSD1306AsciiWire oled; // create OLED display object, using I2C Wire1 port
#define I2C_ADDRESS1 0x3C // for OLED. DIY mall units list 0x78 as address, but need 0x3C to work here

#define ANALOG_IN A0  // Input for Hall sensor
#define REDLED 11   // red LED
#define GRNLED 8    // green LED
#define VREG_EN 24  // voltage regulator enable
#define HALL_SLEEP 10 // Hall sensor sleep pin, pull high to wake
 
void setup() {
  pinMode(REDLED, OUTPUT);
  pinMode(GRNLED, OUTPUT);
  digitalWrite(REDLED, HIGH); // set high to turn OFF
  digitalWrite(GRNLED, HIGH); // set high to turn OFF
  pinMode(VREG_EN, OUTPUT);
  digitalWrite(VREG_EN, HIGH); // set low to turn off, high to turn on (~150usec to wake)
  pinMode(HALL_SLEEP, OUTPUT);
  digitalWrite(HALL_SLEEP, LOW); // set high to wake, set low to sleep (~60usec to wake)
  analogReference(EXTERNAL); // using voltage regulator value on external pin (will this fail if vreg is off?)
//  analogReference(INTERNAL2V5); // using internal bandgap 2.5V VREF value
  pinMode(ANALOG_IN, INPUT); // Hall sensor input channel

  // Set up OLED display
  Wire.begin();
  Wire.setClock(400000L);
  oled.begin(&Adafruit128x32, I2C_ADDRESS1);  
  oled.setFont(Adafruit5x7);    
  oled.clear(); 
  oled.home();
//  oled.set2X();
  oled.print("Hello");
  oled.println();
  delay(1000);

}


// Function to take a few readings from hall sensor and average them
unsigned int readHall(byte ANALOG_IN){
  unsigned int rawAnalog = 0;
  analogRead(ANALOG_IN); // throw away 1st reading
  for (byte i = 0; i<4; i++){
    rawAnalog = rawAnalog + analogRead(ANALOG_IN);
    delay(1);
  }
  // Do a 2-bit right shift to divide rawAnalog
  // by 4 to get the average of the 4 readings
  rawAnalog = rawAnalog >> 2;   
  return rawAnalog;
}

//-----------------------------------------------
// Main loop
void loop() {
  digitalWrite(VREG_EN, HIGH); // turn on voltage regulator
  digitalWrite(HALL_SLEEP, HIGH); // turn on hall effect sensor
  delayMicroseconds(250); // give voltage regulator time to turn on
  unsigned int HallValue = readHall(ANALOG_IN);
  digitalWrite(VREG_EN, LOW); // turn voltage regulator off again
  digitalWrite(HALL_SLEEP, LOW); // put hall sensor to sleep
  digitalWrite(GRNLED,!digitalRead(GRNLED));
  oled.clear();
  oled.home();
  oled.print(HallValue);
  delay(500);


}
