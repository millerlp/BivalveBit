/*  timer_counter_8Hz_test
 *   
 *   This program uses the 32.768kHz clock signal from the MCP79400 real time clock
 *   to run the Atmega's Period Interrupt Timer and wake it from a Power Down sleep
 *   8 times per second. 

*/
#include "Arduino.h"
#include <MCP7940.h>  // https://github.com/Zanduino/MCP7940  Address 0x6F, supports MCP79400
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/cpufunc.h>

#define REDLED 11
#define GRNLED 8
#define VREG_EN 24  // voltage regulator enable
#define CLKPIN 20 // From multifunction pin of MCP79400 clock (pin PF0)
const uint32_t SERIAL_SPEED{57600};     // Set the baud rate for Serial I/O
const uint8_t  SPRINTF_BUFFER_SIZE{32};  // Buffer size for sprintf()
/***************************************************************************************************
** Declare global variables and instantiate classes                                               **
***************************************************************************************************/
MCP7940_Class MCP7940;                           // Create an instance of the MCP7940
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


volatile bool CLCKPIN_flag = false; 
volatile bool PD6PIN_flag = false;
volatile byte INT_count = 0;

void RTC_init(void);
void LED0_init(void);
inline void LED0_toggle(void);
void SLPCTRL_init(void);

void RTC_init(void)
{
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

void LED0_init(void)
{
    /* Make High (OFF) */
    PORTC.OUT |= PIN0_bm; // BivalveBit green led on pin PC0 (arduino pin 8)
    /* Make output */
    PORTC.DIR |= PIN0_bm;
    
    /* Make High (OFF) */
    PORTD.OUT |= PIN2_bm; // Pin PD2
    /* Make output */
    PORTD.DIR |= PIN2_bm;
    
}

inline void LED0_toggle(void)
{
    PORTD.OUTTGL |= PIN2_bm; // Toggle PD2
    PORTC.OUTTGL |= PIN0_bm; // BivalveBit green led on pin PC0 (arduino pin 8)
}

ISR(RTC_PIT_vect)
{
    /* Clear flag by writing '1': */
    RTC.PITINTFLAGS = RTC_PI_bm;
    
    LED0_toggle();
}

void SLPCTRL_init(void)
{
    SLPCTRL.CTRLA |= SLPCTRL_SMODE_PDOWN_gc;
    SLPCTRL.CTRLA |= SLPCTRL_SEN_bm;
}



void setup(void)
{
  pinMode(REDLED, OUTPUT);
  pinMode(GRNLED, OUTPUT);

  digitalWrite(REDLED, HIGH); // set high to turn OFF
  digitalWrite(GRNLED, HIGH); // set high to turn OFF
    Wire.begin();
  Serial.begin(57600);
  delay(100);
  Serial.println("Clock check");
  delay(20);
  while (!MCP7940.begin()) {  // Initialize RTC communications
    Serial.println(F("Unable to find MCP7940. Checking again in 3s."));  // Show error text
    delay(3000);                                                          // wait a second
  }  // of loop until device is located
  Serial.println(F("MCP7940 initialized."));
  while (!MCP7940.deviceStatus()) {  // Turn oscillator on if necessary
    Serial.println(F("Oscillator is off, turning it on."));
    bool deviceStatus = MCP7940.deviceStart();  // Start oscillator and return state
    if (!deviceStatus) {                        // If it didn't start
      Serial.println(F("Oscillator did not start, trying again."));  // Show error and
      delay(1000);                                                   // wait for a second
    }                // of if-then oscillator didn't start
  }                  // of while the oscillator is off

  // Turn on battery backup, default is off
  MCP7940.setBattery(true); // enable battery backup mode
  Serial.print("Battery Backup mode is ");
  if (MCP7940.getBattery()) {
   Serial.println("enabled.");
  } else {
   Serial.println("disabled.");
  }
  delay(5);
  DateTime now = MCP7940.now();
// Testing - turn on alarm setting first, then try to shut if off and go to 32768
//  MCP7940.setSQWState(false); // turn off square wave output if currently on
//  MCP7940.setAlarmPolarity(false); // pin goes low when alarm is triggered
//  Serial.println("Setting alarm 0 for every minute at 0 seconds.");
//  MCP7940.setAlarm(0, matchSeconds, now - TimeSpan(0, 0, 0, now.second()),
//                   true);  // Match once a minute at 0 seconds
//  delay(10);
//  // now try to disable it
//  MCP7940.setAlarm(0,0, DateTime(2020,1,1,1,1,1)); // Deactivate alarm
//  attachInterrupt(digitalPinToInterrupt(20),RTC1MinuteInterrupt, CHANGE); // pin 20 to RTC
//  delay(10);
//  detachInterrupt(digitalPinToInterrupt(20));
  
  // Turn on the square wave output pin of the RTC chip
  MCP7940.setSQWSpeed(3); // set SQW frequency (0=1Hz, 1 = 4.096kHz, 2 = 8.192kHz, 3 = 32.768kHz)
  MCP7940.setSQWState(true); // turn on the square wave output pin

  LED0_init();
  RTC_init();
  SLPCTRL_init();
    
    /* Enable Global Interrupts */
    sei();
    
}

//*********************************************************
void loop(void)
{

    static uint8_t secs;                 // store the seconds value
  DateTime       now = MCP7940.now();  // get the current time
  if (secs != now.second()) {          // Output if seconds have changed
    sprintf(inputBuffer, "%04d-%02d-%02d %02d:%02d:%02d",
            now.year(),  // Use sprintf() to pretty print
            now.month(), now.day(), now.hour(), now.minute(),
            now.second());                         // date/time with leading zeros
    Serial.println(inputBuffer);                   // Display the current date/time
    secs = now.second();                           // Set the counter variable
//    digitalWrite(REDLED, !(digitalRead(REDLED)));   // Toggle the LED
//    digitalWrite(GRNLED, !(digitalRead(GRNLED)));   // Toggle the LED
  }                                                // of if the seconds have changed
  
  readCommand();  
  delay(10); // give time for Serial to print
  /* Put the CPU in sleep */
  sleep_cpu();
  /* The PIT interrupt will wake the CPU */
}


// Interrupt for pin 20, connected to RTC MFP
void RTC1MinuteInterrupt() {
//  PORTC_OUTTGL |= PIN0_bm; // Toggle LED on pin PC0 (Arduino pin 8, GRNLED)
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
      ** CALDATE      - Calibrate device time **
      ** **
      **********************************************************************************************/
      enum commands { SetDate, CalDate, Unknown_Command };  // of commands enumerated type
      commands command;                                     // declare enumerated type
      char     workBuffer[10];                              // Buffer to hold string compare
      sscanf(inputBuffer, "%s %*s", workBuffer);            // Parse the string for first word
      if (!strcmp(workBuffer, "SETDATE"))
        command = SetDate;  // Set command number when found
      else if (!strcmp(workBuffer, "CALDATE"))
        command = CalDate;  // Set command number when found
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
            Serial.print(F("Date has been set."));
          }       // of if-then-else the date could be parsed
          break;  //
        /*******************************************************************************************
        ** Calibrate the RTC and reset the time                                                   **
        *******************************************************************************************/
        case CalDate:  // Calibrate the RTC
          tokens = sscanf(inputBuffer,
                          "%*s %hu-%hu-%hu %hu:%hu:%hu;",  // Use sscanf() to parse the date/
                          &year, &month, &day, &hour, &minute, &second);  // time into variables
          if (tokens != 6)  // Check to see if it was parsed
            Serial.print(F("Unable to parse date/time\n"));
          else {
            int8_t trim =
                MCP7940.calibrate(DateTime(year, month, day,  // Calibrate the crystal and return
                                           hour, minute, second));  // the new trim offset value
            Serial.print(F("Trim value set to "));
            Serial.print(trim * 2);  // Each trim tick is 2 cycles
            Serial.println(F(" clock cycles every minute"));
          }  // of if-then-else the date could be parsed
          break;
        /*******************************************************************************************
        ** Unknown command                                                                        **
        *******************************************************************************************/
        case Unknown_Command:  // Show options on bad command
        default:
          Serial.println(F("Unknown command. Valid commands are:"));
          Serial.println(F("SETDATE yyyy-mm-dd hh:mm:ss"));
          Serial.println(F("CALDATE yyyy-mm-dd hh:mm:ss"));
      }                // of switch statement to execute commands
      inputBytes = 0;  // reset the counter
    }                  // of if-then-else we've received full command
  }                    // of if-then there is something in our input buffer
}  // of method readCommand
