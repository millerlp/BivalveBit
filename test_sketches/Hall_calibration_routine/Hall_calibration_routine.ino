/* Hall_calibration_routine.ino
 *  Luke Miller 2022
 *  
 *  Meant to work with a BivalveBit board. Connect
 *  to a KD Scientific Model 200 syringe dispenser to move the 
 *  dispenser a known distances, causing the 
 *  magnetic field of a Hall effect sensor to change in response.
 *  If you don't have a Model 200 syringe dispenser sitting around,
 *  don't bother running this program.
 *  
 *  Connect Syringe Trigger pin to MusselTracker pin PD3 (Button2 line)
 *  Connect Syringe GND to MusselTracker GND
 *  
 *  
 *  
 *  Make sure the syringe pump is set as follows:
 *  Diam: 50.00 mm
 *  Vol: 0.623 ml
 *  Direction: Withdraw
 *  When you dispense 0.623mL in this setting, you get 1/2 of a full 
 *  rotation of the drive screw.
 *  The drive screw is 40 teeth per inch, so one full rotation moves
 *  the carriage 1/40 of an inch, which is 0.025 inches = 0.635mm, and 
 *  a half rotation = 0.3175 mm.
 *  Usage:
 *  1. Power up the BivalveBit and connect via Serial Monitor 
 *  2. Place Hall sensor on syringe pump fixed block. Place magnet on syringe
 *  pump traveling block (carriage). Fire up Serial Monitor to show Hall reading
 *  3. Release carriage feed, slide magnet near Hall sensor. 
 *  4. Make sure Hall reading is more than ~ 12, as this is the saturated 
 *  reading. If Hall reading is > 500, turn the magnet over (or make 
 *  sure the bottom of the sensor board is oriented towards the magnet).
 *  For consistency's sake we try to set these up so that when the magnet
 *  is near the board the reading is near zero, and as the magnet moves
 *  further from the board the signal goes up towards 500.
 *  5. Engage carriage feed. 
 *  6. Press and hold Button 1 for at least 5 seconds, then release. This
 *  will immediately start the first trial.
 *  7.  A  Hall reading will be taken at each distance (actually the average
 *  of 4 readings), starting from a distance = 0.0 for the initial reading. Every 
 *  subsequent distance is relative to that initial starting point (and is 
 *  assumed to be happening in 0.3175mm steps if the syringe pump is
 *  correctly set to 0.623mL volume,  50mm diameter syringe. I recommend 40ml/min
 *  rate as a reasonable movement speed.
 *  **Any different settings on the 
 *  syringe pump will result in an incorrect distance being recorded.**
 *  8. When the last movement is made, the Serial Monitor or OLED screeen 
 *  will return to just outputing the current Hall reading. The filename
 *  that is being written to for this session will be displayed.
 *  9. You may now release the traveling block and reposition the magnet
 *  near the Hall sensor to run another trial. I recommend starting 
 *  the magnet/Hall combination in several positions and angles to 
 *  make sure you span the range of possible starting positions and
 *  signal curves. 
 *  10. Press Button 1 briefly to start the next trial, which will be
 *  saved to the same output file.
 *  11. Repeat steps 7-10 to carry out multiple trials. 
 *  12. When finished with trials for this Hall effect channel, press 
 *  the RESET button on the circuit board to restart the program and
 *  choose a new Hall channel. 
 */


#include "SdFat.h" // https://github.com/greiman/SdFat
#include <Wire.h>  // built in library, for I2C communications
#include "SSD1306Ascii.h" // https://github.com/greiman/SSD1306Ascii
#include "SSD1306AsciiWire.h" // https://github.com/greiman/SSD1306Ascii
#include <SPI.h>  // built in library, for SPI communications
#include "RTClib.h" // https://github.com/millerlp/RTClib
#include <EEPROM.h> // built in library, for reading the serial number stored in EEPROM
#include "MusselGapeTrackerlib.h" // https://github.com/millerlp/MusselGapeTrackerlib

RTC_DS3231 rtc; // Create real time clock object 

#define ERRLED 4    // Red error LED pin
#define GREENLED A3    // Green LED pin
#define BUTTON1 2     // BUTTON1 on INT0, pin PD2
#define TRIGGER 3  // Trigger on syringe dispenser, pin D3 on RevC
#define ANALOG_IN A7  // Hall effect analog input from multiplexer, pin ADC7
#define CS_SD 10    // Chip select for SD card, pin PB2
#define CS_SHIFT_REG A2 // Chip select for shift registers, pin PC2
#define SHIFT_CLEAR A1  // Clear (erase) line for shift registers, pin PC1

#define MUX_S0  9   // Multiplexer channel select line, pin PB1
#define MUX_S1  5   // Multiplexer channel select line, pin PD5
#define MUX_S2  6   // Multiplexer channel select line, pin PD6
#define MUX_S3  7   // Multiplexer channel select line, pin PD7
#define MUX_EN  8   // Multiplexer enable line, pin PB0
//*************************************
float moveDist = 0.3175; // units mm, if pump set to dispense 0.1mL
float moveLimit = 12.5; // units millimeters
float distanceVal = 0; // keep track of distance moved
//*************
// Create sd card objects
SdFat sd;
SdFile logfile;  // for sd card, this is the file object to be written to
// Declare initial name for output files written to SD card
char filename[] = "Hall00_CALIB_00_SN00.csv";
//*************

//******************************************
// 0X3C+SA0 - 0x3C or 0x3D for OLED screen on I2C bus
#define I2C_ADDRESS1 0x3C   // Typical default address
SSD1306AsciiWire oled1; // create OLED display object, using I2C Wire
//******************************************

// Placeholder serialNumber
char serialNumber[] = "SN00";
// Define a flag to show whether the serialNumber value is real or just zeros
bool serialValid = false;
byte debounceTime = 20; // milliseconds to wait for debounce
int shortPress = 3000;
volatile bool buttonFlag = false; // Flag to mark when button was pressed
unsigned long prevMillis;  // counter for faster operations
unsigned long newMillis;  // counter for faster operations
unsigned int trialNum = 0; // counter for trial number
byte channel = 0; // keep track of current hall channel
DateTime newtime; // keep track of time
char buf[20]; // declare a string buffer to hold the time result
//******************
// Create ShiftReg and Mux objects
ShiftReg shiftReg;
Mux mux;
unsigned int newReading = 0; // Reading from Hall effect sensor analog input
//--------------- buttonFunc --------------------------------------------------
// buttonFunc
void buttonFunc(void){
  detachInterrupt(0); // Turn off the interrupt
  delay(debounceTime);
  if (!digitalRead(BUTTON1)) {
    buttonFlag = true; // Set flag to true
  } else {
    buttonFlag = false; // False alarm
  }
  // Execution will now return to wherever it was interrupted, and this
  // interrupt will still be disabled. 
}
//--------------------------------------------------------------------------


void setup() {
    // Set button1 as an input
  pinMode(BUTTON1, INPUT_PULLUP);
  // Register an interrupt on INT0, attached to button1
  // which will call buttonFunc when the button is pressed.
  attachInterrupt(0, buttonFunc, LOW);
  // Set up the LEDs as output
  pinMode(ERRLED,OUTPUT);
  digitalWrite(ERRLED, LOW);
  pinMode(GREENLED,OUTPUT);
  digitalWrite(GREENLED, LOW);
  // Set up pin for syringe dispenser drive
  pinMode(TRIGGER, OUTPUT); // Same as Button2 physical pin
  digitalWrite(TRIGGER, HIGH);
  // Set up pins for Hall effect sensor
  pinMode(A2, OUTPUT); // sleep line for hall effect 1
  digitalWrite(A2, LOW);
  pinMode(A6, INPUT); // Read the Hall effect sensor on this input
  // Initialize the shift register object
  shiftReg.begin(CS_SHIFT_REG, SHIFT_CLEAR);
  // Initialize the multiplexer object
  mux.begin(MUX_EN,MUX_S0,MUX_S1,MUX_S2,MUX_S3);

  Serial.begin(57600);
  Serial.println("Hello");

  // Grab the serial number from the EEPROM memory
  // The character array serialNumber was defined in the preamble
  EEPROM.get(0, serialNumber);
  if (serialNumber[0] == 'S') {
    serialValid = true; // set flag
  } 

  //----------------------------------
  // Start up the oled display
  oled1.begin(&Adafruit128x64, I2C_ADDRESS1);
  Wire.setClock(400000L); //  oled1.set400kHz();  
  oled1.setFont(Adafruit5x7);    
  oled1.clear(); 
  oled1.home();
  oled1.set2X();
  // Initialize the real time clock DS3231M
  Wire.begin(); // Start the I2C library with default options
  rtc.begin();  // Start the rtc object with default options
  newtime = rtc.now(); // read a time from the real time clock
  newtime.toString(buf, 20); 
  // Now extract the time by making another character pointer that
  // is advanced 10 places into buf to skip over the date. 
  char *timebuf = buf + 10;
  for (int i = 0; i<11; i++){
    oled1.print(buf[i]);
  }
  oled1.println();
  oled1.println(timebuf);

  if (serialValid){
    Serial.print(F("Read serial number: "));
    Serial.println(serialNumber);
    for (byte i = 0; i<4; i++){
      oled1.print(serialNumber[i]);
    }
    oled1.println();
  } else {
    Serial.print(F("No valid serial number: "));
    serialValid = false;    
  }

  delay(2500);
  //***********************************************
  // Check that real time clock has a reasonable time value
  bool stallFlag = true; // Used in error handling below
  if ( (newtime.year() < 2019) | (newtime.year() > 2035) ) {
    // There is an error with the clock, halt everything
    oled1.home();
    oled1.clear();
    oled1.set1X();
    oled1.println(F("RTC ERROR"));
    oled1.println(buf);
    oled1.set2X();
    oled1.println();
    oled1.println(F("Continue?"));
    oled1.println(F("Press 1"));
    Serial.println(F("Clock error, press BUTTON1 on board to continue"));

    bool rtcErrorFlag = true;
    // Consider removing this while loop and allowing user to plow
    // ahead without rtc (use button input?)
    while(stallFlag){
    // Flash the error led to notify the user
    // This permanently halts execution, no data will be collected
      digitalWrite(ERRLED, !digitalRead(ERRLED));
      delay(100);
      if (digitalRead(BUTTON1) == LOW){
        delay(40);  // debounce pause
        if (digitalRead(BUTTON1) == LOW){
          // If button is still low 40ms later, this is a real press
          // Now wait for button to go high again
          while(digitalRead(BUTTON1) == LOW) {;} // do nothing
          stallFlag = false; // break out of while(stallFlag) loop
          oled1.home();
          oled1.clear();
        } 
      }              
    } // end of while(stallFlag)
  } else {
    oled1.home();
    oled1.clear();
    oled1.set2X();
    oled1.println(F("RTC OKAY"));
    Serial.println(F("Clock okay"));
    Serial.println(buf); // Print the time to Serial
    delay(1000);
  } // end of if ( (newtime.year() < 2019) | (newtime.year() > 2035) ) {

  //*************************************************************
  // SD card setup and read (assumes Serial output is functional already)
  pinMode(CS_SD, OUTPUT);  // set chip select pin for SD card to output
  // Initialize the SD card object
  // Try SPI_FULL_SPEED, or SPI_HALF_SPEED if full speed produces
  // errors on a breadboard setup. 
  if (!sd.begin(CS_SD, SPI_FULL_SPEED)) {
  // If the above statement returns FALSE after trying to 
  // initialize the card, enter into this section and
  // hold in an infinite loop.
    while(1){ // infinite loop due to SD card initialization error
                        digitalWrite(ERRLED, HIGH);
                        delay(100);
                        digitalWrite(ERRLED, LOW);
                        digitalWrite(GREENLED, HIGH);
                        delay(100);
                        digitalWrite(GREENLED, LOW);
    }
  }

//******************************************
  // Have the user choose which channel to calibrate
  stallFlag = true;

  // Update oled
  oled1.home();
  oled1.clear();
  oled1.set1X();
  oled1.println(F("Press button 1"));
  oled1.println(F("to choose channel"));
  oled1.println(F("Long press = start"));
  oled1.set2X();
  Serial.println(F("Press button 1 to choose channel"));
  Serial.println(F("Long press = start"));

  long cycleMillis = millis();
  while (stallFlag){
    // Take Hall reading and update oled very 200 ms
    if (millis()-cycleMillis > 200){
      cycleMillis = millis(); // update
      // Choose which channel to sample based on channel value
      // and take a reading to display
      digitalWrite(MUX_EN, LOW); // enable multiplexer
      delayMicroseconds(2);
      shiftReg.shiftChannelSet(channel);
      mux.muxChannelSet(channel);
      newReading = readHall(ANALOG_IN);
      digitalWrite(MUX_EN, HIGH); // disable multiplexer
  
      oled1.setCursor(0,3); // Column 0, row 3 (8-pixel rows)
      oled1.clear(0,128,3,4);
      oled1.print(F("Ch"));
      oled1.print(channel);
      oled1.print(F(": "));
      oled1.print(newReading); // current Hall value
      Serial.print(F("Ch"));
      Serial.print(channel);
      Serial.print(F(": "));
      Serial.println(newReading);
    }
    // React to a Button1 press
    if (digitalRead(BUTTON1) == LOW){
      delay(40);  // debounce pause
      if (digitalRead(BUTTON1) == LOW){
        // If button is still low 40ms later, this is a real press
        // Now wait for button to go high again
        long buttonMillis = millis();
        while(digitalRead(BUTTON1) == LOW) {;} // do nothing
        if (millis() - buttonMillis <= shortPress){
          // Short press registered, increment channel
          ++channel;
          if (channel > 15){
            channel = 0; // cycle back to 0 when 16 is reached
          } 
        } else if (millis() - buttonMillis > shortPress){
          // Long press registered, break out of while(stallFlag) loop
          stallFlag = false; // break out of while(stallFlag) loop
        }
      } 
    }

  } // end of while(stallFlag) loop

  newtime = rtc.now();
  initFileName(newtime);
  oled1.clear();
  oled1.home();
  oled1.set1X();
  oled1.println(F("Starting trial"));
//  oled1.println(F("to start trial"));
  oled1.set2X();

} // end of setup loop ************************************

void loop() {
  // Don't collect a round of data until buttonFlag goes true
  newMillis = millis();
  if (newMillis >= prevMillis + 500){
    prevMillis = newMillis; // Update prevMillis to new value
    digitalWrite(MUX_EN, LOW); // enable multiplexer
    delayMicroseconds(2);
    shiftReg.shiftChannelSet(channel);
    mux.muxChannelSet(channel);
    newReading = readHall(ANALOG_IN);
    digitalWrite(MUX_EN, HIGH); // disable multiplexer
    Serial.print(F("Value: "));
    Serial.println(newReading);
    
    oled1.setCursor(0,3);
    oled1.clear(0,128,3,4);
    oled1.print(F("Ch"));
    oled1.print(channel);
    oled1.print(F(": "));
    oled1.print(newReading);
  }

  if (buttonFlag){
    buttonFlag = false; // reset buttonFlag
    distanceVal = 0.0; // initialize distance
    ++trialNum; // increment trial number
    oled1.set1X();
    oled1.clear(0,128,0,2);
    oled1.home();
    oled1.print(F("Running Trial "));
    oled1.println(trialNum);
    oled1.println(filename);
    oled1.set2X();
    while (distanceVal < moveLimit){ 

      // Take a reading 
      digitalWrite(MUX_EN, LOW); // enable multiplexer
      delayMicroseconds(2);
      shiftReg.shiftChannelSet(channel);
      mux.muxChannelSet(channel);
      delay(10);
      newReading = readHall(ANALOG_IN);
      digitalWrite(MUX_EN, HIGH); // disable multiplexer

      Serial.print(F("Hall "));
      Serial.print(channel);
      Serial.print(F("\t Trial: "));
      Serial.print(trialNum);
      Serial.print(F("\t Distance "));
      Serial.print(distanceVal,4);
      Serial.print(F(" mm, Value: "));
      Serial.println(newReading);
      
      oled1.setCursor(0,3);
      oled1.clear(0,128,3,3);
      oled1.print(F("Ch"));
      oled1.print(channel);
      oled1.print(F(": "));
      oled1.println(newReading);
      oled1.set1X();
      oled1.print(F("Distance: "));
      oled1.println(distanceVal,4);
      oled1.set2X();
      
      // Write the reading to the output file
        // Reopen logfile. If opening fails, notify the user
      if (!logfile.isOpen()) {
        if (!logfile.open(filename, O_RDWR | O_CREAT | O_AT_END)) {
          digitalWrite(ERRLED, HIGH); // turn on error LED
        }
      }
      logfile.print(serialNumber);
      logfile.print(F(","));
      logfile.print(channel);
      logfile.print(F(","));
      logfile.print(trialNum);
      logfile.print(F(","));
      logfile.print(distanceVal,5);
      logfile.print(F(","));
      logfile.println(newReading);  
      logfile.close();

      // Actuate the Trigger pin to make the syringe dispenser
      // run
      digitalWrite(TRIGGER, LOW); // Transition High to Low to trigger movement
      delay(1200); // Give dispenser time to move
      digitalWrite(TRIGGER, HIGH); // Reset Trigger line
      // Increment the distanceVal
      distanceVal = distanceVal + moveDist;  // units mm
    
    } // end of while loop 
    Serial.println(F("Finished"));
    Serial.print(F("Current file: "));
    Serial.println(filename);
    oled1.clear();
    oled1.home();
    oled1.set1X();
    oled1.println(F("Press button 1"));
    oled1.println(F("to start next trial"));
    oled1.println(filename);
    oled1.set2X();
    // Flash the green LED to notify the user we're done
//    for (int i; i < 5; i++){
//      digitalWrite(GREENLED, HIGH);
//      delay(10);
//      digitalWrite(GREENLED, LOW);
//      delay(100);
//    }
    // Reattach the interrupt to allow another trial on this same data file
    attachInterrupt(0, buttonFunc, LOW);
  } 
}

//------------------------------------------------------------------------------

//-------------- initFileName --------------------------------------------------
// initFileName - a function to create a filename for the SD card. 
// The character array 'filename' was defined as a global array 
// at the top of the sketch in the form "Hall00_CALIB_00_SN00.csv"
void initFileName(DateTime time1) {
  // Insert the Hall sensor channel number in positions 4-5 (count from 0)
  if (channel < 10){
    filename[5] = channel + '0';
  } else if (channel >= 10){
    filename[4] = channel /10 + '0'; // 10's digit
    filename[5] = channel % 10 + '0'; // 1's digit
  }

  // If there is a valid serialnumber, insert it into 
  // the file name in positions 16-19. 
  if (serialValid) {
    byte serCount = 0;
    for (byte i = 16; i < 20; i++){
      filename[i] = serialNumber[serCount];
      serCount++;
    }
  }
  // Change the counter on the end of the filename
  // (digits 13,14) to increment count for files generated on
  // the same day. This shouldn't come into play
  // during a normal data run, but can be useful when 
  // troubleshooting.
  for (uint16_t i = 0; i < 100; i++) {
    filename[13] = (i % 100)/10 + '0'; // 10's digit
    filename[14] = i % 10 + '0'; // 1's digit
    // Check and see if this filename is already on the card
    // and if so, repeat the for loop with a value 1 digit higher
    // until you find a non-existent filename.
    if (!sd.exists(filename)) {
      // when sd.exists() returns false, this block
      // of code will be executed to open the file
      if (!logfile.open(filename, O_RDWR | O_CREAT | O_AT_END)) {
        // If there is an error opening the file, notify the
        // user. Otherwise, the file is open and ready for writing
        // Turn both indicator LEDs on to indicate a failure
        // to create the log file
        digitalWrite(ERRLED, !digitalRead(ERRLED)); // Toggle error led 
        digitalWrite(GREENLED, !digitalRead(GREENLED)); // Toggle indicator led 
        delay(5);
      }
      break; // Break out of the for loop when the
      // statement if(!logfile.exists())
      // is finally false (i.e. you found a new file name to use).
    } // end of if(!sd.exists())

  } // end of file-naming for loop
  //------------------------------------------------------------

  // write a header line to the SD file
  logfile.println(F("Serial,HallChannel,Trial,Distance.mm,Reading"));
  // Update the file's creation date, modify date, and access date.
  logfile.timestamp(T_CREATE, time1.year(), time1.month(), time1.day(), 
      time1.hour(), time1.minute(), time1.second());
  logfile.timestamp(T_WRITE, time1.year(), time1.month(), time1.day(), 
      time1.hour(), time1.minute(), time1.second());
  logfile.timestamp(T_ACCESS, time1.year(), time1.month(), time1.day(), 
      time1.hour(), time1.minute(), time1.second());
  logfile.close(); // force the data to be written to the file by closing it
  Serial.print(F("Ouput file: "));
  Serial.println(filename);
} // end of initFileName function
