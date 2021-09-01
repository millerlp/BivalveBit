/* BivalveBit_logger.ino
 *  Initial draft of a BivalveBit data logger program
 *  
 */

#include "SdFat.h" // https://github.com/greiman/SdFat-beta (you must be using ver. 2.1.0 or higher)
#include <Wire.h>  // built in library, for I2C communications
#include "SSD1306Ascii.h" // https://github.com/greiman/SSD1306Ascii
#include "SSD1306AsciiWire.h" // https://github.com/greiman/SSD1306Ascii


// SPI pins for SD card
const uint8_t SD_CHIP_SELECT = 7;
#define MOSI 4
#define MISO 5
#define SCK 6

SdFat sd;


#define REDLED 11
#define GRNLED 8



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
  pinMode(ANALOG_IN, INPUT); // Hall sensor input channel

  Serial.begin(57600);
  // Wait until serial port is opened
  while (!Serial) { delay(1); }
  
  // SD card initialization
  if (!sd.begin(SD_CHIP_SELECT)) {
    Serial.println("Card failed, or not present");
    failFlag = true;
    // don't do anything more:
    
    return;
  }
  Serial.println("card initialized.");


}

void loop() {


}
