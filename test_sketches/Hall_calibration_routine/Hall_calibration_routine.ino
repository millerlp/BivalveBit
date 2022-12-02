/* Hall_calibration_routine.ino
 *  BivalveBit data logger program to calibration Hall effect sensor
 *  of a KD Scientific Model 200 syringe dispenser.
 *  
 *  Compile with MegaCoreX: ATmega4808, 32 pin, BOD 2.6V, Internal 8MHz, Optiboot (UART0 default pins)
 *  

 *  Meant to work with a BivalveBit board. Connect
 *  to a KD Scientific Model 200 syringe dispenser to move the 
 *  dispenser a known distances, causing the 
 *  magnetic field of a Hall effect sensor to change in response.
 *  If you don't have a Model 200 syringe dispenser sitting around,
 *  don't bother running this program.
 *  
 *  Connect Syringe Trigger pin to MusselTracker pin PD2
 *  Connect Syringe GND to MusselTracker GND
 *  On the KD200 dispenser, set the syringe DIA to 50.00mm
 *  Set the Vol to 0.623ml
 *  Set the Rate to 40 ml/m
 *  
 *  To start a calibration run, open the Serial Monitor and type CALIB and 
 *  hit return. The program will step the syringe pump through the various 
 *  distances, with the first distance makred as "0.0" mm. 
 *  
 *  If the device just slowly flashes red after startup, the real time clock
 *  needs to be reset. Open the serial monitor and enter the correct date and
 *  time (in the UTC time zone preferably) using the command format:
 *  SETDATE YYYY-MM-DD HH:MM:SS
 *  
 */


#include "SdFat.h" // https://github.com/greiman/SdFat (you must be using ver. 2.1.2 or higher)
#include <Wire.h>  // built in library, for I2C communications
#include <EEPROM.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/cpufunc.h>
//#include "SSD1306Ascii.h" // https://github.com/greiman/SSD1306Ascii
//#include "SSD1306AsciiWire.h" // https://github.com/greiman/SSD1306Ascii
#include "MCP7940.h"  // https://github.com/Zanduino/MCP7940  Real time clock
//#include "Adafruit_VCNL4040.h" // https://github.com/adafruit/Adafruit_VCNL4040 & https://github.com/adafruit/Adafruit_BusIO
//#include "SparkFun_TMP117.h" // https://github.com/sparkfun/SparkFun_TMP117_Arduino_Library
#include "BivalveBit_lib.h" // https://github.com/millerlp/BivalveBit_lib


//*************************************
float moveDist = 0.3175; // units mm, if pump set to dispense 0.1mL
float moveLimit = 12.5; // units millimeters
float distanceVal = 0; // keep track of distance moved
uint8_t TrialNum = 0;   // Trial number counter
//*************

// -------------------------------------------
// ***** STATE MACHINE TYPE DEFINITIONS *****
typedef enum STATE
{
  STATE_IDLE,         // Idle and wait for user input
  STATE_CALIB     // Go into Hall effect calibration mode
} mainState_t;
// main state machine variable, this takes on the various
// values defined for the STATE typedef above. 
mainState_t mainState;

// Type definition for a case structure to write to SD card or skip writing
typedef enum WRITE_SD
{
  WRITE_HALL_TEMP_VOLTS, // collect Hall sensor data and battery voltage data
  WRITE_HEART, // collect heart rate data at sub-second intervals
  WRITE_NOTHING // don't write anything
  
} writeState_t;
writeState_t writeState;

/*******************************************************
 * SD card objects
 *******************************************************/
// SPI pins for SD card
const uint8_t SD_CHIP_SELECT = 7;
#define MOSI 4
#define MISO 5
#define SCK 6
SdFat sd; 
SdFile GAPEFile; //SD card object
bool SDfailFlag = false;
// Placeholder serialNumber
char serialNumber[] = "SN000";
bool serialValid = false;
// Declare initial name for output files written to SD card
char gapefilename[] =   "SN000_YYYYMMDD_HHMM_CALIB.csv";

/************************************************************
 * Hall effect sensor definitions
 ************************************************************/
#define ANALOG_IN A0  // Analog input pin for Hall sensor
#define HALL_SLEEP 10 // Hall sensor sleep pin, pull high to wake
unsigned int HallValue = 0; // Variable for Hall sensor reading

/***************************************************************************************************
** Declare global variables and instantiate classes for MCP79400 real time clock                                              
*   This library is written for the MCP7940, but works with the BivalveBit MCP79400
***************************************************************************************************/
MCP7940_Class MCP7940;                           // Create an instance of the MCP7940

const uint32_t SERIAL_SPEED{57600};     // Set the baud rate for Serial I/O
const uint8_t  SPRINTF_BUFFER_SIZE{32};  // Buffer size for sprintf()
char          inputBuffer[SPRINTF_BUFFER_SIZE];  // Buffer for sprintf()/sscanf()
DateTime now; // Variable to hold current time
DateTime firsttimestamp; // Variable to track fast sampling start 
DateTime lasttimestamp; // Variable to track fast sampling end time
byte oldday1; // Used to track midnight change
byte oldday2; // Used to track midnight change
/*! ///< Enumeration of MCP7940 alarm types */
enum alarmTypes {
  matchSeconds,
  matchMinutes,
  matchHours,
  matchDayOfWeek,
  matchDayOfMonth,
  Unused1,
  Unused2,
  matchAll,
  Unknown
};

//------------------------------------------------------------
//  Battery monitor 
#define BATT_MONITOR_EN  9 // digital output channel to turn on battery voltage check
#define BATT_MONITOR  A1  // analog input channel to sense battery voltage
float dividerRatio = 2; // Ratio of voltage divider (47k + 47k) / 47k = 2
float refVoltage = 3.00; // Voltage at AREF pin on ATmega microcontroller, measured per board
float batteryVolts = 0; // Estimated battery voltage returned from readBatteryVoltage function
float minimumVoltage = 3.4; // Minimum safe voltage for a Li-Ion battery
unsigned int lowVoltageCount = 0; // Count how many times voltage is too low
unsigned int lowVoltageCountLimit = 10; // Number of loops after which the program should be shutdown
#define REDLED 11   // Red LED pin
#define GRNLED 8    // Green LED pin
#define VREG_EN 24  // voltage regulator enable
#define TRIGGER 14 // External trigger to syringe pump, PD2 pin, Arduino pin 14(A2)


void test_pin_init(void){
    /* Make High (OFF) */
    PORTC.OUT |= PIN0_bm; // BivalveBit green led on pin PC0 (arduino pin 8)
    /* Make output */
    PORTC.DIR |= PIN0_bm;

    /* Make High (OFF) */
    PORTC.OUT |= PIN3_bm; // BivalveBit red led on pin PC3 (arduino pin 11)
    /* Make output */
    PORTC.DIR |= PIN3_bm;
    
    /* Make High (OFF) */
    PORTD.OUT |= PIN2_bm; // Pin PD2 (probe with scope/analyzer) - used to activate syringe pump
    /* Make output */
    PORTD.DIR |= PIN2_bm;
}

ISR(RTC_PIT_vect)
{
    /* You must clear PIT interrupt flag by writing '1': */
    RTC.PITINTFLAGS = RTC_PI_bm;
//    PORTD.OUTTGL |= PIN2_bm; // Toggle PD2 (probe with scope/analyzer)
//    PORTC.OUTTGL |= PIN0_bm; // BivalveBit green led on pin PC0 
}



//---------------------------------------------------------------
//-------- Setup
//---------------------------------------------------------------
void setup() {
  Serial.begin(57600);
  Serial.println("Hello");
  analogReference(EXTERNAL);
  setUnusedPins(); // in BivalveBit_lib
  disableUnusedPeripherals(); // in BivalveBit_lib
  pinMode(VREG_EN, OUTPUT);   // Voltage regulator pin
  digitalWrite(VREG_EN, LOW); // set low to turn off, high to turn on (~150usec to wake)
  pinMode(20, INPUT_PULLUP); // pin PF0, attached to RTC multi-function pin
  // Battery monitor pins
  pinMode(BATT_MONITOR, INPUT); // Battery voltage input channel
  pinMode(BATT_MONITOR_EN, OUTPUT); // Battery monitor enable pin
  digitalWrite(BATT_MONITOR_EN, LOW); // pull low to turn off battery monitor circuit
  test_pin_init(); // Set up the LEDs and PD2 output pin
  //---------------------------------
  // Retrieve board serial number
  EEPROM.get(0, serialNumber);
  if (serialNumber[0] == 'S') {
    serialValid = true; // set flag   
    Serial.print("Serial number: "); Serial.println(serialNumber); 
  } else {
    serialValid = false;
    Serial.print("No serial number");
  }
  //----------------------------------------------------------
  // SD card initialization
  pinMode(SD_CHIP_SELECT, OUTPUT); // SD card chip select pin
  if (!sd.begin(SD_CHIP_SELECT)) {
    Serial.println("Card failed, or not present");
    SDfailFlag = true;
    for (int i = 0; i < 50; i++){
      digitalWrite(REDLED, !digitalRead(REDLED));
      delay(100);
      digitalWrite(GRNLED, !digitalRead(GRNLED));
      delay(100);
    }
    return;
  } else {
    Serial.println("SD Card initialized.");
  }
  digitalWrite(GRNLED,LOW); // set low to turn on
  delay(500);
  digitalWrite(GRNLED,HIGH); // set high to turn off
  //---------------------------------------------------------------
  digitalWrite(VREG_EN, HIGH); // set low to turn off, high to turn on (~150usec to wake)

  //--------------------------------------------------------------
  MCP7940setup();
  // Turn on battery backup, default is off
  MCP7940.setBattery(true); // enable battery backup mode
  Serial.print("Battery Backup mode is ");
  if (MCP7940.getBattery()) {
   Serial.println("enabled.");
  } else {
   Serial.println("disabled.");
  }
  now = MCP7940.now();  // get the current time
  // Use sprintf() to pretty print date/time with leading zeroes
  sprintf(inputBuffer, "%04d-%02d-%02d %02d:%02d:%02d", now.year(), now.month(), now.day(),
          now.hour(), now.minute(), now.second());
  Serial.print("Date/Time: "); Serial.println(inputBuffer);
  
  bool clockErrorFlag = true;
  while (clockErrorFlag){
    // Check that real time clock has a reasonable time value
    if ( (now.year() < 2022) | (now.year() > 2035) ) {
        Serial.println("Please set clock to current UTC time");
        Serial.println("Use format SETDATE YYYY-MM-DD HH:MM:SS");
       // Error, clock isn't set
       while(clockErrorFlag){
        digitalWrite(REDLED, !digitalRead(REDLED));
        delay(500);
        readCommand();
        now = MCP7940.now();  // get the updated time
        if ( (now.year() >= 2022) & (now.year() <= 2035) ){
          // If the year is now within bounds, break out of this while statement
          clockErrorFlag = false;
          sprintf(inputBuffer, "%04d-%02d-%02d %02d:%02d:%02d", now.year(), now.month(), now.day(),
            now.hour(), now.minute(), now.second());
          Serial.print("Date/Time: "); Serial.println(inputBuffer);
          break;
        }
        
       }
    } else {
      // If the time's year was in bound, continue with the program
      clockErrorFlag = false;
    }
  }
  oldday1 = oldday2 = now.day(); // Store current day's value
  // Initialize data file
//  initGapeFileName(sd, GAPEFile, now, gapefilename, serialValid, serialNumber);
  digitalWrite(REDLED, HIGH); // turn off
//  Serial.print("Heart sample interval: "); Serial.print(heartMinute); Serial.println(" minutes");
//  MCP7940Alarm1Minute(now); // Set RTC multifunction pin to alarm when new minute hits
  attachInterrupt(digitalPinToInterrupt(20),RTC1MinuteInterrupt, CHANGE); // pin 20 to RTC
  
  mainState = STATE_IDLE;
  writeState = WRITE_NOTHING;
//  SLPCTRL_init(); // in BivalveBitlib - sets up sleep mode
//  sleep_cpu();    // put the cpu to sleep

}     // end setup


//--------------------------------------------------------------------
//----------------- Main loop
//--------------------------------------------------------------------
void loop() {
//        unsigned long m1 = millis();  

  switch(mainState) {
    case STATE_IDLE:
    {
      // Wait for user to start a calibration routine
      digitalWrite(VREG_EN, HIGH); // set high to enable voltage regulator
      HallValue = readWakeHall(ANALOG_IN, HALL_SLEEP);  
      digitalWrite(VREG_EN, LOW); // set low to turn off after sampling
      Serial.print(HallValue); 
      Serial.println("\t \t Type CALIB to start Hall sensor calibration");
      readCommand();
      delay(250);
    }
    break; 

    case STATE_CALIB:
    {
      TrialNum = TrialNum + 1; // Increment the counter before starting
      distanceVal = 0.0;
      // Take an initial reading at zero
      while(distanceVal < moveLimit){
        digitalWrite(VREG_EN, HIGH); // set high to enable voltage regulator
        delay(50);
        HallValue = readWakeHall(ANALOG_IN, HALL_SLEEP);    
        digitalWrite(VREG_EN, LOW); // set low to turn off after sampling
  
        Serial.print(F("\t Trial: "));
        Serial.print(TrialNum);
        Serial.print(F("\t Distance "));
        Serial.print(distanceVal,4);
        Serial.print(F(" mm, Value: "));
        Serial.println(HallValue);
        // Write the reading to the output file
        // Reopen logfile. If opening fails, notify the user
        if (!GAPEFile.isOpen()) {
          if (!GAPEFile.open(gapefilename, O_RDWR | O_CREAT | O_AT_END)) {
            digitalWrite(REDLED, HIGH); // turn on error LED
          }
        }
        GAPEFile.print(serialNumber);
        GAPEFile.print(F(","));
        GAPEFile.print(TrialNum);
        GAPEFile.print(F(","));
        GAPEFile.print(distanceVal,5);
        GAPEFile.print(F(","));
        GAPEFile.println(HallValue);
        GAPEFile.close();
  
        // Actuate the Trigger pin to make the syringe dispenser
        // run
        digitalWrite(TRIGGER, LOW); // Transition High to Low to trigger movement
        digitalWrite(GRNLED, LOW); // turn on
        delay(1200); // Give dispenser time to move
        digitalWrite(TRIGGER, HIGH); // Reset Trigger line
        digitalWrite(GRNLED, HIGH); // turn off
        delay(10);
        // Increment the distanceVal
        distanceVal = distanceVal + moveDist;  // units mm
      }
      Serial.print("Finished Trial ");
      Serial.println(TrialNum);
      Serial.print("Current file: ");
      Serial.println(gapefilename);
      Serial.println(F("Reset carriage and type CALIB again to run another trial"));
      mainState = STATE_IDLE;
    }
    break;
    
  }   // end of mainState switch block



}     // end main loop



//-------------- initFileName --------------------------------------------------
// initFileName - a function to create a filename for the SD card. 
// The character array 'filename' was defined as a global array 
// at the top of the sketch in the form "SN000_YYYYMMDD_HHMM_CALIB.csv"
void initFileName(DateTime time1) {

// Modify the filename to hold the current serial number, date, and time
  sprintf(gapefilename, "%5s_%d%02d%02d_%02d%02d_CALIB.csv", serialNumber, time1.year(),time1.month(),time1.day(),time1.hour(),time1.minute());
  

//  for (uint16_t i = 0; i < 10; i++) {
//    gapefilename[31] = i; // 1's digit
//    // Check and see if this filename is already on the card
//    // and if so, repeat the for loop with a value 1 digit higher
//    // until you find a non-existent filename.
    if (!sd.exists(gapefilename)) {
      // when sd.exists() returns false, this block
      // of code will be executed to open the file
      if (!GAPEFile.open(gapefilename, O_RDWR | O_CREAT | O_AT_END)) {
        // If there is an error opening the file, notify the
        // user. Otherwise, the file is open and ready for writing
        // Turn both indicator LEDs on to indicate a failure
        // to create the log file
        digitalWrite(REDLED, !digitalRead(REDLED)); // Toggle error led 
        digitalWrite(GRNLED, !digitalRead(GRNLED)); // Toggle indicator led 
        delay(5);
      }
//      break; // Break out of the for loop when the statement if(!sd.exists(gapefilename))
      // is finally false (i.e. you found a new file name to use).
    } // end of if(!sd.exists())
//
//  } // end of file-naming for loop
  //------------------------------------------------------------

  // write a header line to the SD file
  GAPEFile.println(F("Serial,Trial,Distance.mm,Reading"));
  // Update the file's creation date, modify date, and access date.
  GAPEFile.timestamp(T_CREATE, time1.year(), time1.month(), time1.day(), 
      time1.hour(), time1.minute(), time1.second());
  GAPEFile.timestamp(T_WRITE, time1.year(), time1.month(), time1.day(), 
      time1.hour(), time1.minute(), time1.second());
  GAPEFile.timestamp(T_ACCESS, time1.year(), time1.month(), time1.day(), 
      time1.hour(), time1.minute(), time1.second());
  GAPEFile.close(); // force the data to be written to the file by closing it
//  Serial.print(F("Ouput file: "));
//  Serial.println(gapefilename);
} // end of initFileName function


//---------------------------------------------------------
// Interrupt for pin 20, connected to RTC MFP
// Used for the 1-minute alarms
void RTC1MinuteInterrupt() {
  // Do nothing, this just wakes the microcontroller from sleep
}


//------------------------------------------------------
void MCP7940setup() {
   while (!MCP7940.begin())  // Loop until the RTC communications are established
  {
    Serial.println(F("Unable to find MCP7940. Checking again in 3s."));
    delay(3000);
  }  // of loop until device is located
  Serial.println(F("MCP7940 initialized."));
  while (!MCP7940.deviceStatus())  // Turn oscillator on if necessary
  {
    Serial.println(F("Oscillator is off, turning it on."));
    bool deviceStatus = MCP7940.deviceStart();  // Start oscillator and return state
    if (!deviceStatus)                          // If it didn't start
    {
      Serial.println(F("Oscillator did not start, trying again."));
      delay(1000);
    }  // of if-then oscillator didn't start
  }    // of while the oscillator is of
  MCP7940.setSQWState(false); // turn off square wave output if currently on

}

//---------------------------------------------------------
void MCP7940Alarm1Minute(DateTime currentTime){
  MCP7940.setSQWState(false); // turn off square wave output if currently on
  MCP7940.setAlarmPolarity(false); // pin goes low when alarm is triggered
  Serial.println("Setting alarm for every minute at 0 seconds.");
  MCP7940.setAlarm(0, matchSeconds, currentTime - TimeSpan(0, 0, 0, currentTime.second()),
                   true);  // Match once a minute at 0 seconds
  delay(10);
}

//--------------------------------------------
// Set RTC to generate a 32.768kHz signal on its multifunction pin
void MCP7940sqw32768(){
//  MCP7940.setAlarm(0,0, DateTime(2020,1,1,1,1,1)); // Deactivate alarm
  MCP7940.setAlarmState(0, false); // Deactivate the alarm
  // Turn on the square wave output pin of the RTC chip
  MCP7940.setSQWSpeed(3); // set SQW frequency (0=1Hz, 1 = 4.096kHz, 2 = 8.192kHz, 3 = 32.768kHz)
  MCP7940.setSQWState(true); // turn on the square wave output pin
}


/***************************************************************************************************
** Method readCommand(). This function checks the serial port to see if there has been any input. **
** If there is data it is read until a terminator is discovered and then the command is parsed    **
** and acted upon                                                                                 **
***************************************************************************************************/
void readCommand() {
  static uint8_t inputBytes = 0;              // Variable for buffer position
  while (Serial.available()) {                // Loop while incoming serial data
    inputBuffer[inputBytes] = Serial.read();  // Get the next byte of data
    if (inputBuffer[inputBytes] != '\n' &&
        inputBytes < SPRINTF_BUFFER_SIZE)  // keep on reading until a newline
      inputBytes++;                        // shows up or the buffer is full
    else {
      inputBuffer[inputBytes] = 0;                 // Add the termination character
      for (uint8_t i = 0; i < inputBytes; i++)     // Convert the whole input buffer
        inputBuffer[i] = toupper(inputBuffer[i]);  // to uppercase characters
      Serial.print(F("\nCommand \""));
      Serial.write(inputBuffer);
      Serial.print(F("\" received.\n"));
      /**********************************************************************************************
      ** Parse the single-line command and perform the appropriate action. The current list of **
      ** commands understood are as follows: **
      ** **
      ** SETDATE      - Set the device time **
      ** CALIB        - Calibrate Hall effect sensor **
      ** **
      **********************************************************************************************/
      enum commands { SetDate, Calib, Unknown_Command };  // of commands enumerated type
      commands command;                                     // declare enumerated type
      char     workBuffer[10];                              // Buffer to hold string compare
      sscanf(inputBuffer, "%s %*s", workBuffer);            // Parse the string for first word
      if (!strcmp(workBuffer, "SETDATE"))
        command = SetDate;  // Set command number when found
      else if (!strcmp(workBuffer, "CALIB"))
        command = Calib;  // Set command number when found
      else
        command = Unknown_Command;                              // Otherwise set to not found
      uint16_t tokens, year, month, day, hour, minute, second;  // Variables to hold parsed dt/tm
      switch (command) {                                        // Action depending upon command
        /*******************************************************************************************
        ** Set the device time and date                                                           **
        *******************************************************************************************/
        case SetDate:  // Set the RTC date/time
          tokens = sscanf(inputBuffer, "%*s %hu-%hu-%hu %hu:%hu:%hu;", &year, &month, &day, &hour,
                          &minute, &second);
          if (tokens != 6)  // Check to see if it was parsed
            Serial.print(F("Unable to parse date/time\n"));
          else {
            MCP7940.adjust(
                DateTime(year, month, day, hour, minute, second));  // Adjust the RTC date/time
            Serial.println(F("Date has been set."));
          }       // of if-then-else the date could be parsed
          mainState = STATE_IDLE; // Return to idle status
          break;  //
        /*******************************************************************************************
        ** Calibrate the Hall effect sensor
        *******************************************************************************************/
        case Calib:  // Calibrate the Hall sensor
            Serial.println(F("Starting Hall sensor calibration"));
            now = MCP7940.now();
            if (TrialNum < 1){
              initFileName(now);  
            }
            Serial.print(F("Data saved in "));
            Serial.println(gapefilename);
            delay(10);
            mainState = STATE_CALIB;  // Change the mainState to start the calibration
          break;
        /*******************************************************************************************
        ** Unknown command                                                                        **
        *******************************************************************************************/
        case Unknown_Command:  // Show options on bad command
        default:
          Serial.println(F("Unknown command. Valid commands are:"));
          Serial.println(F("SETDATE yyyy-mm-dd hh:mm:ss"));
          Serial.println(F("CALIB to start calibration"));
      }                // of switch statement to execute commands
      inputBytes = 0;  // reset the counter
    }                  // of if-then-else we've received full command
  }                    // of if-then there is something in our input buffer
}  // of method readCommand
