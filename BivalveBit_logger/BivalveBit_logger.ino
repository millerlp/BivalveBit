/* BivalveBit_logger.ino
 *  Draft of a BivalveBit data logger program with sleep modes and
 *  state machine to run things
 *  TODO: Figure out why the thing is in a reboot loop when the OLED
 *  isn't present. Maybe a capacitance/voltage droop issue on the I2C bus
 *  Seems to work on battery, but not on FTDI alone
 *  
 */


#include "SdFat.h" // https://github.com/greiman/SdFat-beta (you must be using ver. 2.1.0 or higher)
#include <Wire.h>  // built in library, for I2C communications
#include <EEPROM.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/cpufunc.h>
//#include "SSD1306Ascii.h" // https://github.com/greiman/SSD1306Ascii
//#include "SSD1306AsciiWire.h" // https://github.com/greiman/SSD1306Ascii
#include "MCP7940.h"  // https://github.com/Zanduino/MCP7940  Real time clock
#include "Adafruit_VCNL4040.h" // https://github.com/adafruit/Adafruit_VCNL4040 & https://github.com/adafruit/Adafruit_BusIO
#include "SparkFun_TMP117.h" // https://github.com/sparkfun/SparkFun_TMP117_Arduino_Library
#include "BivalveBit_lib.h" // https://github.com/millerlp/BivalveBit_lib

/* Define heartMinute, to be used to set which minutes of the hour to take
 *  heart rate samples. This value will be divided into the current minute
 *  value, and if the modulo (remainder) is 0, then the heart rate sampling
 *  will be activated. 
 *  A value of 1 would sample every minute
 *  A value of 2 would sample every even-numbered minute
 *  A value of 5 would sample every 5 minutes
 */
unsigned int heartMinute = 2; 
unsigned int heartSampleLength = 240; // Number of heart samples to take in 1 minute
unsigned int heartCount = 0; // Current number of heart samples taken in this minute
uint16_t heartBuffer[240] = {0}; // 



// -------------------------------------------
// ***** STATE MACHINE TYPE DEFINITIONS *****
typedef enum STATE
{
  STATE_1MINUTE_SLEEP, // sleep until new minute turns over based on RTC alarm
  STATE_FAST_SAMPLE,  // sleep for short interval based on RTC wakeup signal (8Hz)
  STATE_SHUTDOWN // close data file, shut down data collection due to low battery
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
File IRFile; //SD card object 1 (IR heart rate data)
File GAPEFile; //SD card object 2 (Gape, Temp, Battery voltage data)
bool SDfailFlag = false;
// Placeholder serialNumber
char serialNumber[] = "SN000";
bool serialValid = false;
// Declare initial name for output files written to SD card
char heartfilename[] =  "YYYYMMDD_HHMM_00_SN000_IR.csv";
char gapefilename[] =   "YYYYMMDD_HHMM_00_SN000_GAPE.csv";

/**********************************************************
 * Create VCNL4040 object (heart rate sensor)
********************************************************* */
Adafruit_VCNL4040 vcnl4040 = Adafruit_VCNL4040();


/**************************************************
 *  Create TMP117 temperature sensor object
 *   The default address of the device is 0x48 = (GND)
 *****************************************************/
TMP117 TMP117sensor; // Initalize sensor
float tempC;         // output variable for temperature
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
float minimumVoltage = 3.5; // Minimum safe voltage for a Li-Ion battery (3.2)
unsigned int lowVoltageCount = 0; // Count how many times voltage is too low
unsigned int lowVoltageCountLimit = 10; // Number of loops after which the program should be shutdown
#define REDLED 11   // Red LED pin
#define GRNLED 8    // Green LED pin
#define VREG_EN 24  // voltage regulator enable


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
    PORTD.OUT |= PIN2_bm; // Pin PD2 (probe with scope/analyzer)
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
  delayMicroseconds(250);
  
  // Start up VCNL4040 proximity sensor (heart sensor)
  if (!vcnl4040.begin()) {
    Serial.println("Couldn't find VCNL4040 chip");
    digitalWrite(REDLED, LOW);
    delay(500);
    digitalWrite(REDLED, HIGH);
    delay(100);
//    oled.println("Heart sensor fail");
  } else {
//    oled.println("Heart sensor on");
    digitalWrite(GRNLED,LOW);
    delay(100);
    digitalWrite(GRNLED,HIGH);
    delay(100);
  }

  setupVCNL4040(); // See function at bottom of program, takes approx 60ms
  vcnl4040.enableProximity(false);
    
  //------------------------------------------------------------------
  if (TMP117sensor.begin() == true) // Check if the sensor will correctly self-identify with the proper Device ID/Address
  {
    Serial.println("TMP117 Begin");
    TMP117sensor.setConversionAverageMode(1); // set to 1 for averaging 8 readings, should take ~124ms
    TMP117sensor.setShutdownMode(); // put into low power mode
  }  else {
    Serial.println("TMP117 device failed to setup- Freezing code.");
    while (1){
      digitalWrite(REDLED,!digitalRead(REDLED));
      delay(500);
      // Runs forever
    }
  }

  
  
  digitalWrite(VREG_EN, LOW); // set low to turn off voltage regulator
  //--------------------------------------------------------------
  MCP7940setup();
  now = MCP7940.now();  // get the current time
  // Use sprintf() to pretty print date/time with leading zeroes
  sprintf(inputBuffer, "%04d-%02d-%02d %02d:%02d:%02d", now.year(), now.month(), now.day(),
          now.hour(), now.minute(), now.second());
  Serial.print("Date/Time: "); Serial.println(inputBuffer);
  // Check that real time clock has a reasonable time value
  if ( (now.year() < 2022) | (now.year() > 2035) ) {
     // Error, clock isn't set
     while(1){
      digitalWrite(REDLED, !digitalRead(REDLED));
      delay(100);
     }
  }
  oldday1 = oldday2 = now.day(); // Store current day's value
  // Initialize two data files
  initHeartFileName(sd, IRFile, now, heartfilename, serialValid, serialNumber);
  initGapeFileName(sd, GAPEFile, now, gapefilename, serialValid, serialNumber);
  digitalWrite(REDLED, HIGH); // turn off
  MCP7940Alarm1Minute(now); // Set RTC multifunction pin to alarm when new minute hits
  attachInterrupt(digitalPinToInterrupt(20),RTC1MinuteInterrupt, CHANGE); // pin 20 to RTC
  
  mainState = STATE_1MINUTE_SLEEP;
  writeState = WRITE_NOTHING;
  SLPCTRL_init(); // in BivalveBitlib - sets up sleep mode
  sleep_cpu();    // put the cpu to sleep

}     // end setup


//--------------------------------------------------------------------
//----------------- Main loop
//--------------------------------------------------------------------
void loop() {
//        unsigned long m1 = millis();  

  switch(mainState) {
    case STATE_1MINUTE_SLEEP:
    {
        /* You arrived here because you're in the 1-minute sleep cycle
         *  
         */
        now = MCP7940.now();  // get the current time
        // Start by clearing the RTC alarm
        MCP7940.clearAlarm(0); // clear the alarm pin (reset the pin to high)
        Serial.println("Woke from 1 minute alarm");
        sprintf(inputBuffer, "%04d-%02d-%02d %02d:%02d:%02d", now.year(), now.month(), now.day(),
          now.hour(), now.minute(), now.second());
        Serial.println(inputBuffer);        delay(10);

        // Since a new minute just started, sample Hall, temperature, battery, and 
        // then decide whether to switch to 8Hz fast sampling or stay in 1-minute sleep cycle
          unsigned long firstmillis = millis();
          digitalWrite(VREG_EN, HIGH); // set high to enable voltage regulator
          TMP117sensor.setOneShotMode(); // start a temperature reading
          batteryVolts = readBatteryVoltage(BATT_MONITOR_EN, BATT_MONITOR,\
                                              dividerRatio, refVoltage);
          HallValue = readWakeHall(ANALOG_IN, HALL_SLEEP);                                              
//          Serial.print("Battery: ");Serial.print(batteryVolts,2);Serial.println("V");delay(10);
          if (batteryVolts < minimumVoltage) {
            ++lowVoltageCount; // Increment the counter
            digitalWrite(REDLED, LOW); delay(10); digitalWrite(REDLED, HIGH); // testing, comment out
          }
          while (!TMP117sensor.dataReady()){
            // do nothing during conversion
          }
          tempC = TMP117sensor.readTempC(); // retrieve the temperature value
          TMP117sensor.setShutdownMode(); // put into low power mode
          digitalWrite(VREG_EN, LOW); // set low to turn off after sampling
//          Serial.print("Temp sampling time ms: "); Serial.println(millis() - firstmillis);delay(10);
        
        // Check time, decide what kind of wake interval and sampling to proceed with
        if ( now.minute() % heartMinute == 0) {
          /* If it's a heart-sampling minute change mainState to STATE_FAST_SAMPLE
           * to switch to 8Hz wake interval
          */
          if (lowVoltageCount < lowVoltageCountLimit) {
            // Continue normal sampling routine
            mainState = STATE_FAST_SAMPLE; // switch to fast sampling next  
            writeState = WRITE_HALL_TEMP_VOLTS; // Save the gape/temp/volts data at start of fast sampling
          } else {
            // The battery voltage has been low for too long, shutdown
            mainState = STATE_SHUTDOWN;
            writeState = WRITE_HALL_TEMP_VOLTS;
          }
        } else {
          /* If it's a non-heart minute, update 1 minute wake interval,
          * and remain in STATE_1MINUTE_SLEEP
          */    
          if (lowVoltageCount < lowVoltageCountLimit) {
            mainState = STATE_1MINUTE_SLEEP; // Remain in 1-minute sleep mode  
            writeState = WRITE_HALL_TEMP_VOLTS; // Write gape, temperature, voltage to SD
          } else {
            // The battery voltage has been low for too long, shutdown
            mainState = STATE_SHUTDOWN;
            writeState = WRITE_HALL_TEMP_VOLTS;
          }
//          Serial.println("Staying in 1 minute sleep loop"); delay(10);
          // Re-enable 1-minute wakeup interrupt
          attachInterrupt(digitalPinToInterrupt(20),RTC1MinuteInterrupt, CHANGE); // pin 20 to RTC          
        }
    }
    break;

    case STATE_FAST_SAMPLE:
    {
        // If you arrive here from STATE_1MINUTE_SLEEP, the 8Hz wakeup signal should
        // be in effect, and the Hall/temp sensors will already have been read, so just
        // take a heart sensor reading and add it to the buffer
        // Keep count of how many times you visit this section, and revert to 
        // STATE_1MINUTE_SLEEP after you've taken enough heart readings (30 seconds) 

        // Take heart sensor sample, add it to the buffer
        digitalWrite(VREG_EN, HIGH); // set high to enable voltage regulator
        bitWrite(PORTC.OUT, 0, 0); // Set PC0 low to turn on green LED
        delayMicroseconds(150); // Give voltage regulator time to stabilize
        bitWrite(PORTC.OUT, 0, 1); // Turn off PC0 by setting pin high
        setupVCNL4040(); // consumes about 60ms of time, so only usable at 8Hz or slower sample rate
        heartBuffer[heartCount] = vcnl4040.getProximity();        
        ++heartCount; // Increment heart sample counter
        digitalWrite(VREG_EN, LOW); // set low to disable voltage regulator
//        vcnl4040.enableProximity(false); // turn off the VCNL4040 heart sensor
        
//        Serial.println(heartBuffer[heartCount-1]);
//        delay(5);
        

        
        // If enough samples have been taken, revert to 1 minute intervals
        if ( heartCount > (heartSampleLength-1) ) {
          heartCount = 0; // reset for next sampling round        
          writeState = WRITE_HEART; // switch to write SD state 
          lasttimestamp = MCP7940.now(); // Store last time stamp of fast sampling run        
          digitalWrite(VREG_EN, LOW); // set low to disable voltage regulator
        }  
    }
    break;


    case STATE_SHUTDOWN:
    {
        // If you arrive here due to too many low battery voltage readings, 
        // turn off the RTC alarm, detach the interrupt, close all files, and let the 
        // device go into sleep mode one more time (permanently)
        Serial.println("Oops, shutdown"); delay(10);
        digitalWrite(VREG_EN, LOW); // set low to disable voltage regulator
        detachInterrupt(digitalPinToInterrupt(20)); // Remove the current 1 minute interrupt
        MCP7940.setSQWState(false); // turn off square wave output if currently on
        RTC.PITINTCTRL &= ~RTC_PI_bm; /* Periodic Interrupt: disabled, cancels 8Hz wakeup */
        MCP7940.setAlarmState(0, false); // Deactivate alarm
        // Now allow cpu to re-enter sleep at the end of the main loop
    }
    break;
  }   // end of mainState switch block


  switch (writeState){
    case WRITE_NOTHING:
        // Do nothing here, it's not time to write any data to SD
    break;
    case WRITE_HALL_TEMP_VOLTS:
      // You arrive here because the hall sensor, temperature and battery
      // voltage have all been sampled. Write them to SD
//      Serial.println("Write gape, temp, volts"); delay(8);
      // Check if a new day has started, start a new data file if so.
      if (now.day() != oldday1) {
        GAPEFile.close(); // close old file
        // Open new file
        initGapeFileName(sd, GAPEFile, now, gapefilename, serialValid, serialNumber);
        oldday1 = now.day(); // update oldday1
      }
      // Write hall, temperature, battery voltage to SD
      writeGapeToSD(now); // See function at bottom of program
      
      // Reset to write-nothing state 
      writeState = WRITE_NOTHING;
      // The main state could be going back to 1-minute intervals or switching
      // to fast sampling heart rate, depending on what happened in the 
      // mainState switch above. Check which state we're in to decide what to do next 
      switch(mainState){
        // If we've arrived here and are switching to fast sample, activate the PIT timer
        case (STATE_FAST_SAMPLE):
          firsttimestamp = MCP7940.now(); // store value for start of IR samples
          detachInterrupt(digitalPinToInterrupt(20)); // Remove the current 1 minute interrupt
//          digitalWrite(VREG_EN, HIGH); // set high to enable voltage regulator here if you wanted to 
                                        // make sure it's running at the very start of the fast sampling
//          setupVCNL4040(); // reset the IR sensor after power-up of the voltage regulator
          MCP7940sqw32768(); // Reactivate the RTC's 32.768kHz clock output
          PIT_init(); // set up the Periodic Interrupt Timer to wake at 8Hz (BivalveBitlib)
          sei();  // enable global interrupts
          // head to end of main loop to go to sleep
        break;
        
        case (STATE_1MINUTE_SLEEP):
          // If we've arrived here and it's remaining a 1-minute sleep interval, do nothing      
        break;

        case (STATE_SHUTDOWN):
          // If the main state is now STATE_SHUTDOWN, do nothing here
          Serial.println("Time to shutdown");
        break;
        default:
          // Do nothing in the default case
        break;
      }
    break;

    case WRITE_HEART:
      // You arrive here after doing the fast heart rate samples and filling the
      // buffer. Write those data to the SD card heart rate file, and then 
      // reset the mainState to 1-minute intervals
//      Serial.println("Write heart data"); delay(8);
      // Check if a new day has started, if so open a new file
      if (firsttimestamp.day() != oldday2){
        IRFile.close();
        initHeartFileName(sd, IRFile, firsttimestamp, heartfilename, serialValid, serialNumber);
        oldday2 = firsttimestamp.day(); // update oldday2
      }
      
      writeHeartToSD(firsttimestamp,lasttimestamp); // write IR data to SD card
      // Revert to 1-minute wake interrupts, disable 8Hz PIT interrupts
      mainState = STATE_1MINUTE_SLEEP; // reset to 1 minute sleep interval
      writeState = WRITE_NOTHING; // reset to write-nothing state 
      RTC.PITINTCTRL &= ~RTC_PI_bm; /* Periodic Interrupt: disabled, cancels 8Hz wakeup */ 
      MCP7940Alarm1Minute(MCP7940.now()); // Reset 1 minute alarm
      attachInterrupt(digitalPinToInterrupt(20),RTC1MinuteInterrupt, CHANGE); // pin 20 to RTC
    break;

    default:
      // Nothing in default case
    break;
  } // end of writeState switch statement

  /* At the end of the main loop, put the cpu to sleep. When it
   *  wakes up, detach the RTC's alarm pin interrupt and go back 
   *  to the top of the main loop to check the time and decide what
   *  to do next in the mainState chunk
   */
//  Serial.print( (millis() - m1) ); Serial.println(" <- cycle time"); delay(8);
  Serial.println("Sleep"); delay(10);  // testing, comment out
  sleep_cpu(); 
  Serial.println("Awake"); delay(10);   // testing, comment out
//  detachInterrupt(digitalPinToInterrupt(20));
}     // end main loop


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
  Serial.println("Setting alarm 0 for every minute at 0 seconds.");
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


//------------- writeGapeToSD -----------------------------------------------
// writeGapeToSD function. This formats the available data in the
// data arrays and writes them to the SD card file in a
// comma-separated value format.
void writeGapeToSD (DateTime timestamp) {
  // Reopen logfile. If opening fails, notify the user
  if (!GAPEFile.isOpen()) {
    if (!GAPEFile.open(gapefilename, O_RDWR | O_CREAT | O_AT_END)) {
      digitalWrite(REDLED, HIGH); // turn on error LED
    }
  }
  // Write the unixtime
  GAPEFile.print(timestamp.unixtime(), DEC);GAPEFile.print(F(",")); // POSIX time value
  printTimeToSD(GAPEFile, timestamp); GAPEFile.print(F(","));// human-readable time stamp
  GAPEFile.print(serialNumber); GAPEFile.print(F(",")); // Serial number
  GAPEFile.print(HallValue); GAPEFile.print(F(","));  // Hall sensor value
  GAPEFile.print(tempC); GAPEFile.print(F(","));      // Temperature sensor value
  GAPEFile.print(batteryVolts,3); GAPEFile.print(F(",")); // Battery voltage
  GAPEFile.println();
  // GAPEFile.close(); // force the buffer to empty
  GAPEFile.timestamp(T_WRITE, timestamp.year(),timestamp.month(), \
      timestamp.day(),timestamp.hour(),timestamp.minute(),timestamp.second());
  
}


//------------- writeHeartToSD -----------------------------------------------
// writeHeartToSD function. This formats the available data in the
// data arrays and writes them to the SD card file in a
// comma-separated value format.
void writeHeartToSD (DateTime firsttimestamp, DateTime lasttimestamp) {
  // Reopen logfile. If opening fails, notify the user
  if (!IRFile.isOpen()) {
    if (!IRFile.open(heartfilename, O_RDWR | O_CREAT | O_AT_END)) {
      digitalWrite(REDLED, HIGH); // turn on error LED
    }
  }
  for (unsigned int bufferIndex = 0; bufferIndex < heartSampleLength; bufferIndex++){
      if (bufferIndex == 0) {
        // For first value in buffer, write first time stamp
        IRFile.print(firsttimestamp.unixtime(), DEC); IRFile.print(F(","));// POSIX time value
        printTimeToSD(IRFile, firsttimestamp); IRFile.print(F(",")); // Human readable date time
        IRFile.print(serialNumber); IRFile.print(F(",")); // Serial number
        IRFile.println(heartBuffer[bufferIndex]);         // Heart IR value
      } else if ( (bufferIndex > 0) & (bufferIndex < (heartSampleLength - 1)) ){
        IRFile.print(",,,"); // write empty time values for most heart values
        IRFile.println(heartBuffer[bufferIndex]);         // Heart IR value
      } else if (bufferIndex == (heartSampleLength - 1) ){
        // For last value in buffer, write last time stamp
        IRFile.print(lasttimestamp.unixtime(), DEC); IRFile.print(F(","));// POSIX time value
        printTimeToSD(IRFile, lasttimestamp); IRFile.print(F(",")); // Human readable date time
        IRFile.print(serialNumber); IRFile.print(F(",")); // Serial number
        IRFile.println(heartBuffer[bufferIndex]);
      } 
    
  }
  // IRFile.close(); // force the buffer to empty

  IRFile.timestamp(T_WRITE, lasttimestamp.year(),lasttimestamp.month(), \
      lasttimestamp.day(),lasttimestamp.hour(),lasttimestamp.minute(),lasttimestamp.second());
  
}



//-------------------------------------------------------
// (re)initialize VCNL4040 IR sensor
//------------------------------------------------------
void setupVCNL4040(){
  vcnl4040.enableAmbientLight(false);  // disable ambient light sensor
  vcnl4040.enableAmbientLightInterrupts(false); // disable ambient light interrupt
  vcnl4040.enableWhiteLight(false); // disable white light sensor
  vcnl4040.enableProximityInterrupts(VCNL4040_PROXIMITY_INT_DISABLE); // disable proximity sensor interrupt
  
  vcnl4040.setProximityHighResolution(true); // set to 16bit resolution (vs. 12bit)
  // Setting VCNL4040_LED_DUTY_1_40 gives shortest proximity measurement time (about 4.85ms)
  vcnl4040.setProximityLEDDutyCycle(VCNL4040_LED_DUTY_1_40); // 1_40, 1_80,1_160,1_320
  vcnl4040.setProximityLEDCurrent(VCNL4040_LED_CURRENT_50MA); // 50,75,100,120,140,160,180,200
  // Setting VCNL4040_PROXIMITY_INTEGRATION_TIME_1T gives the shortest pulse (lowest LED output)
  // in combination with the LED_CURRENT setting above. A longer integration time like
  // VCNL4040_PROXIMITY_INTEGRATION_TIME_8T raises the pulse length (higher LED output) in
  // combination with the LED_CURRENT setting.
  vcnl4040.setProximityIntegrationTime(VCNL4040_PROXIMITY_INTEGRATION_TIME_8T); // 1T,1_5T,2T,2_5T,3T,3_5T,4T,8T
  vcnl4040.enableProximity(true); // turn on IR sensor for right now
}
