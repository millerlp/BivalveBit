/* BivalveBit_logger2.ino
 *  Draft of a BivalveBit data logger program with sleep modes and
 *  state machine to run things
 *  
 */


#include "SdFat.h" // https://github.com/greiman/SdFat-beta (you must be using ver. 2.1.0 or higher)
#include <Wire.h>  // built in library, for I2C communications
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/cpufunc.h>
//#include "SSD1306Ascii.h" // https://github.com/greiman/SSD1306Ascii
//#include "SSD1306AsciiWire.h" // https://github.com/greiman/SSD1306Ascii
#include "MCP7940.h"  // https://github.com/Zanduino/MCP7940  Real time clock
#include <Adafruit_VCNL4040.h> // https://github.com/adafruit/Adafruit_VCNL4040 & https://github.com/adafruit/Adafruit_BusIO
#include <SparkFun_TMP117.h> // https://github.com/sparkfun/SparkFun_TMP117_Arduino_Library
#include "BivalveBit_lib.h" // https://github.com/millerlp/BivalveBit_lib


/* Define heartMinute, to be used to set which minutes of the hour to take
 *  heart rate samples. This value will be divided into the current minute
 *  value, and if the modulo (remainder) is 0, then the heart rate sampling
 *  will be activated. 
 *  A value of 1 would sample every minute
 *  A value of 2 would sample every even-numbered minute
 *  A value of 5 would sample every 5 minutes
 */
int heartMinute = 2; 
int heartSampleLength = 240; // Number of heart samples to take in 1 minute
int heartCount = 0; // Current number of heart samples taken in this minute

// ***** TYPE DEFINITIONS *****
typedef enum STATE
{
  STATE_1MINUTE_SLEEP, // sleep until new minute turns over based on RTC alarm
  STATE_FAST_SAMPLE,  // sleep for short interval based on RTC wakeup signal (8Hz)
  STATE_SHUTDOWN, // close data file, shut down data collection due to low battery
} mainState_t;
// main state machine variable, this takes on the various
// values defined for the STATE typedef above. 
mainState_t mainState;


typedef enum SAMPLE_STATE
{
  SAMPLE_HALL_ONLY, // collect Hall sensor data and battery voltage data
  SAMPLE_HEART // collect heart rate data at sub-second intervals
  
} sampleState_t;
sampleState_t sampleState;

/***************************************************************************************************
** Declare global variables and instantiate classes                                               **
***************************************************************************************************/
MCP7940_Class MCP7940;                           // Create an instance of the MCP7940
const uint32_t SERIAL_SPEED{57600};     // Set the baud rate for Serial I/O
const uint8_t  SPRINTF_BUFFER_SIZE{32};  // Buffer size for sprintf()
char          inputBuffer[SPRINTF_BUFFER_SIZE];  // Buffer for sprintf()/sscanf()
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


void setup() {
  Serial.begin(57600);
  Serial.println("Hello");
  pinMode(20, INPUT_PULLUP); // pin PF0, attached to RTC multi-function pin
  MCP79400setup();
  Serial.print("Date/Time set to ");
  DateTime now = MCP7940.now();  // get the current time
  // Use sprintf() to pretty print date/time with leading zeroes
  sprintf(inputBuffer, "%04d-%02d-%02d %02d:%02d:%02d", now.year(), now.month(), now.day(),
          now.hour(), now.minute(), now.second());
  Serial.println(inputBuffer);

  MCP79400Alarm1Minute(now); // Set RTC multifunction pin to alarm when new minute hits
  
  attachInterrupt(digitalPinToInterrupt(20),RTC1MinuteInterrupt, CHANGE); // pin 20 to RTC

  mainState = STATE_1MINUTE_SLEEP;
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // Set sleep mode to POWER DOWN mode 
  sleep_enable();
  sleep_cpu();
  detachInterrupt(digitalPinToInterrupt(20));
}     //end setup

void loop() {
  DateTime       now = MCP7940.now();  // get the current time first

  switch(mainState) {
    case STATE_1MINUTE_SLEEP:
        // Start by clearing the RTC alarm
        MCP7940.clearAlarm(0); // clear the alarm pin (reset the pin to high)
        
        
        // Check time, 
        if ( now.minute() % heartMinute == 0) {
          // If it's a heart minute, sample Hall, temperature, and battery, then 
          // switch to 8Hz wake interval and change mainState to STATE_FAST_SAMPLE
          // TODO: Take Hall, temperature, battery voltage samples

          // TODO: Write values to SD card (start new file if it's a new day)

          // TODO: Test if battery voltage is too low, go to STATE_SHUTDOWN if so

          // Switch to fast sample state
          detachInterrupt(digitalPinToInterrupt(20)); // Remove the current 1 minute interrupt
          mainState = STATE_FAST_SAMPLE;
          PIT_init(); // set up the Periodic Interrupt Timer to wake at 8Hz
          
        } else {
          // If it's a non-heart minute, only sample Hall, temperature, battery
          // and update 1 minute wake interval, remain in STATE_1MINUTE_SLEEP

          // TODO: Take Hall, temperature, battery voltage samples

          // TODO: Write values to SD card (start new file if it's a new day)

          // TODO: Test if battery voltage is too low, go to STATE_SHUTDOWN if so

          // Re-enable 1-minute wakeup interrupt
          attachInterrupt(digitalPinToInterrupt(20),RTC1MinuteInterrupt, CHANGE); // pin 20 to RTC          
        }
    break;

    case STATE_FAST_SAMPLE:
        // If you arrive here from STATE_1MINUTE_SLEEP, the 8Hz wakeup signal should
        // be in effect, and the Hall/temp sensors will already have been read, so just
        // take a heart sensor reading and add it to the buffer
        // Keep count of how many times you visit this section, and revert to 
        // STATE_1MINUTE_SLEEP after you've taken enough heart readings (30 seconds)
        // Write the heart data to file, reset the wakeup alarm to 1 minute intervals 

        // TODO: Take heart sensor sample, add it to the buffer

        
        ++heartCount; // Increment heart sample counter


        // If enough samples have been taken, revert to 1 minute intervals
        if ( heartCount >= (heartSampleLength-1) ) {
          heartCount = 0; // reset for next sampling round

          // TODO: Write heart buffer to SD card. Start new file if it's a new day

          
          mainState = STATE_1MINUTE_SAMPLE; // reset to 1 minute sample interval
          RTC.PITINTCTRL &= ~RTC_PI_bm; /* Periodic Interrupt: disabled, cancels 8Hz wakeup */ 
          MCP79400Alarm1Minute(MCP7940.now()); // Reset 1 minute alarm
          attachInterrupt(digitalPinToInterrupt(20),RTC1MinuteInterrupt, CHANGE); // pin 20 to RTC
        }

        
    break;

    case STATE_SHUTDOWN:
        // If you arrive here due to too many low battery voltage readings, 
        // turn off the RTC alarm, detach the interrupt, close all files, and let the 
        // device go into sleep mode one more time (permanently)
        detachInterrupt(digitalPinToInterrupt(20)); // Remove the current 1 minute interrupt
        MCP7940.setSQWState(false); // turn off square wave output if currently on
        RTC.PITINTCTRL &= ~RTC_PI_bm; /* Periodic Interrupt: disabled, cancels 8Hz wakeup */
        MCP7940.setAlarm(0,0, DateTime(2020,1,1,1,1,1)); // Deactivate alarm
        // Now allow cpu to re-enter sleep at the end of the main loop
    break;
  }   // end of mainState switch block

  /* At the end of the main loop, put the cpu to sleep. When it
   *  wakes up, detach the RTC's alarm pin interrupt and go back 
   *  to the top of the main loop to check the time and decide what
   *  to do next in the mainState chunk
   */
  sleep_cpu(); 
  detachInterrupt(digitalPinToInterrupt(20));
}     // end main loop

//---------------------------------------------------------
// Interrupt for pin 20, connected to RTC MFP
void RTC1MinuteInterrupt() {
  PORTC_OUTTGL |= PIN0_bm; // Toggle LED on pin PC0 (Arduino pin 8, GRNLED)
}


//------------------------------------------------------
void MCP79400setup() {
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
void MCP79400Alarm1Minute(DateTime currentTime){
  MCP7940.setSQWState(false); // turn off square wave output if currently on
  MCP7940.setAlarmPolarity(false); // pin goes low when alarm is triggered
  Serial.println("Setting alarm 0 for every minute at 0 seconds.");
  MCP7940.setAlarm(0, matchSeconds, currentTime - TimeSpan(0, 0, 0, currentTime.second()),
                   true);  // Match once a minute at 0 seconds
}

//--------------------------------------------
// Set RTC to generate a 32.768kHz signal on its multifunction pin
void MCP79400sqw32768(){
  MCP7940.setAlarm(0,0, DateTime(2020,1,1,1,1,1)); // Deactivate alarm
  // Turn on the square wave output pin of the RTC chip
  MCP7940.setSQWSpeed(3); // set SQW frequency (0=1Hz, 1 = 4.096kHz, 2 = 8.192kHz, 3 = 32.768kHz)
  MCP7940.setSQWState(true); // turn on the square wave output pin
}

//--------------------------------------------------------
// Set up the ATmega to run the Periodic Interrupt Timer at 8Hz
// assuming the RTC's multifunction pin is outputing a 32.768kHz clock signal
void PIT_init(void)
{
    // Activate the RTC's 32.768kHz clock output
    MCP79400sqw32768();
    
    uint8_t temp;
    
    /* Initialize 32.768kHz Oscillator: */
    /* Disable oscillator by writing 0 to the ENABLE bit in the XOSC32KCTRLA register: */
    temp = CLKCTRL.XOSC32KCTRLA; // read register contents
    temp &= ~CLKCTRL_ENABLE_bm; // modify register, write 0 to ENABLE bit to allow changing setting
    /* Writing to protected register */
//    ccp_write_io((void*)&CLKCTRL.XOSC32KCTRLA, temp);
    _PROTECTED_WRITE(CLKCTRL.XOSC32KCTRLA, temp);  // Arduino version of ccp_write_io
    
    while(CLKCTRL.MCLKSTATUS & CLKCTRL_XOSC32KS_bm)
    {
        ; /* Wait until XOSC32KS becomes 0 */
    }

    /* SEL = 0 (Use External Crystal): */
    // Now you can actually change the register values safely
    temp = CLKCTRL.XOSC32KCTRLA;
//    temp &= ~CLKCTRL_SEL_bm; // would set up for external crystal on TOSC1 & TOSC2
    temp |= CLKCTRL_SEL_bm; // Set up for external clock on TOSC1 pin only
    /* Writing to protected register */
//    ccp_write_io((void*)&CLKCTRL.XOSC32KCTRLA, temp);
     _PROTECTED_WRITE(CLKCTRL.XOSC32KCTRLA, temp);  // Arduino version of ccp_write_io
     // We're not yet set up to actually read a clock input on TOSC1

    /* Enable oscillator: */
    temp = CLKCTRL.XOSC32KCTRLA;
    temp |= CLKCTRL_ENABLE_bm; // This will update TOSC1's function
    /* Writing to protected register */
//    ccp_write_io((void*)&CLKCTRL.XOSC32KCTRLA, temp);
    _PROTECTED_WRITE(CLKCTRL.XOSC32KCTRLA, temp);  // Arduino version of ccp_write_io

    /* Initialize RTC: */
    while (RTC.STATUS > 0)
    {
        ; /* Wait for all register to be synchronized */
    }

    /* Set RTC peripheral to read a 32.768kHz external clock signal from TOSC1 */
    RTC.CLKSEL = RTC_CLKSEL_TOSC32K_gc; // external crystal on TOSC1

    /* Run in debug: enabled */
//    RTC.DBGCTRL = RTC_DBGRUN_bm;
    
    RTC.PITINTCTRL = RTC_PI_bm; /* Periodic Interrupt: enabled */
    // Define the prescalar value for the periodic interrupt timer. This will divide the
    // 32.768kHz input by the chosen prescaler. A prescaler of 32768 will cause the 
    // interrupt to only fire once per second.
//    RTC.PITCTRLA = RTC_PERIOD_CYC32768_gc /* RTC Clock Cycles 32768 */
//                 | RTC_PITEN_bm; /* Enable: enabled by writing 1*/

    // Prescaler 4096 gives 8 interrupts per second (period from high to high = 250ms)
    RTC.PITCTRLA = RTC_PERIOD_CYC4096_gc /* RTC Clock Cycles 4096 */
             | RTC_PITEN_bm; /* Enable: enabled by writing 1*/                 
}

ISR(RTC_PIT_vect)
{
    /* Clear flag by writing '1': */
    RTC.PITINTFLAGS = RTC_PI_bm;
}
