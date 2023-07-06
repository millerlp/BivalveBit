/* BivalveBit_MAX30102_test.ino
 *  
 *  Used to test for a working MAX30102 heart sensor (or MAX30101)
 *  Note that a MAX30102 RevB sensor board will probably cause a 
 *  brownout on the BivalveBit if only powered with a 3V3 FTDI USB
 *  adapter. You either need to include a Li-Ion battery plugged
 *  into the board, or use a 5V FTDI-USB adapter to power the
 *  BivalveBit for testing. 
 * 
 */

#include "MAX30105.h"         // https://github.com/millerlp/SparkFun_MAX3010x_Sensor_Library
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/cpufunc.h>
#include "BivalveBit_lib.h" // https://github.com/millerlp/BivalveBit_lib

// MAX30105 sensor parameters
MAX30105 max3010x;
// sensor configurations
byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green. Only use 2
byte REDledBrightness = 1; // low value of 0 shuts it off, 1 is barely on
byte IRledBrightness = 60;  // Starting value around 60 is probably reasonable for bivalves
byte sampleAverage = 1; //Options: 1, 2, 4, 8, 16, 32, but only use 1 or 2. The others are too slow
int pulseWidth = 215; //Options: 69, 118, 215, 411, units microseconds. Applies to all active LEDs. Recommend 215
// For 118us, max sampleRate = 1000; for 215us, max sampleRate = 800, for 411us, max sampleRate = 400
int sampleRate = 800; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
int adcRange = 4096; //Options: 2048, 4096, 8192, 16384. 4096 is standard

/************************************************************
 * Hall effect sensor definitions
 ************************************************************/
#define ANALOG_IN A0  // Analog input pin for Hall sensor
#define HALL_SLEEP 10 // Hall sensor sleep pin, pull high to wake
unsigned int HallValue = 0; // Variable for Hall sensor reading

// SPI pins for SD card
const uint8_t SD_CHIP_SELECT = 7;
#define MOSI 4
#define MISO 5
#define SCK 6

#define REDLED 11   // Red LED pin
#define GRNLED 8    // Green LED pin
#define VREG_EN 24  // voltage regulator enable
unsigned int myIntervalMS = 50;  // units milliseconds
unsigned long myMillis;

/* Define heartMinute, to be used to set which minutes of the hour to take
 *  heart rate samples. This value will be divided into the current minute
 *  value, and if the modulo (remainder) is 0, then the heart rate sampling
 *  will be activated. 
 *  A value of 1 would sample every minute
 *  A value of 2 would sample every even-numbered minute
 *  A value of 5 would sample every 5 minutes
 */
unsigned int heartMinute = 5; 
unsigned int heartSampleLength = 240; // Number of heart samples to take in 1 minute
unsigned int heartCount = 0; // Current number of heart samples taken in this minute
uint16_t heartBuffer[240] = {0}; // 



void setup() {
  Serial.begin(57600);
  Serial.println("Hello");delay(10);
  pinMode(GRNLED, OUTPUT);
  pinMode(REDLED, OUTPUT);
  digitalWrite(GRNLED, HIGH); // turn off
  digitalWrite(REDLED, HIGH); // turn off
  
  for (int i = 0; i<5; i++){
    digitalWrite(GRNLED, LOW);
    delay(100);
    digitalWrite(GRNLED, HIGH);
    delay(100);
  }
  analogReference(EXTERNAL);
//  setUnusedPins(); // in BivalveBit_lib
//  disableUnusedPeripherals(); // in BivalveBit_lib
  pinMode(VREG_EN, OUTPUT);   // Voltage regulator pin
  Serial.println(F("Engaging VREG"));delay(10);
  digitalWrite(VREG_EN, HIGH); // set low to turn off, high to turn on (~150usec to wake)
  Serial.println(F("VREG engaged"));delay(100);

//  pinMode(20, INPUT_PULLUP); // pin PF0, attached to RTC multi-function pin

  // SD card initialization
//  pinMode(SD_CHIP_SELECT, OUTPUT); // SD card chip select pin

//  Wire.begin();

  if (max3010x.begin(Wire, I2C_SPEED_STANDARD)) //Use default I2C port, 100kHz speed
  {
    // Quick green flash to show that the heart sensor was found
    Serial.println(F("MAX heart sensor initialized"));
    digitalWrite(GRNLED,LOW); // set low to turn on
    delay(250);
    digitalWrite(GRNLED,HIGH); // set high to turn off
  } else {
    Serial.println(F("Did not find MAX heart sensor"));
    digitalWrite(REDLED,LOW); // set low to turn on, leave on due to the error
//    delay(250);
//    digitalWrite(REDLED,HIGH); // set high to turn off
  }
  
  max3010x.setup(IRledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  max3010x.enableDIETEMPRDY(); //enable temp ready interrupt. Required to log temp, but each read takes 29ms
  // Tweak individual settings
  max3010x.setPulseAmplitudeRed(REDledBrightness); // essentially turn off red LED to save power, we only want IR LED. **** commented for testing only
  max3010x.setPulseAmplitudeIR(IRledBrightness); // set IR led brightness to user's chosen value 0x00 (off) to 0xFF(full power)

  myMillis = millis();

} // ******************end of setup loop

void loop() {

  // Run in a loop so every 50ms we try to take a new sample
  if ( (millis() - myMillis) >= myIntervalMS) {
    myMillis = millis();

    max3010x.clearFIFO(); // Clearing FIFO potentially lets you only have to grab one value from the FIFO once it starts to refill
    
    long watchdog = millis();
    bool sampleFlag = false;
    // Here we use a watchdog to make sure the collection of a new sample doesn't take
    // longer than our pre-defined watchdog time (in milliseconds)
    while( millis() - watchdog < 5) {
        // With a cleared FIFO these two pointers will match initially
        byte readPointer = max3010x.getReadPointer();
        byte writePointer = max3010x.getWritePointer();
        
        if (readPointer != writePointer){
          // If they don't match, that means a new sample arrived in the FIFO buffer on the MAX3010x
          sampleFlag = true;
          break; // escape the while loop once a new sample has appeared
        }
        delayMicroseconds(20);
        sampleFlag = false;
    }
    if (sampleFlag){
        max3010x.check();  // retrieve the new sample(s) and put them in the max3010x private buffer
        // Calling getIR() should get the most recent value from the buffer of values
        Serial.println(max3010x.getIR());  // modify getIR in the library to remove safeCheck() function        
    } else {
        // If sampleFlag was still false, a new sample didn't arrive from the MAX3010x sensor in time
        Serial.println("0");  // modify getIR in the library to remove safeCheck() function   
    }
  }

}  //************************* end of main loop
