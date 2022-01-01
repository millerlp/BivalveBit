/*  Basic_RTC_test
 *   
 *   For RevB version of BivalveBit (Atmega4808)
 *   Tested working (RevB needs the RX and TX pins swapped for upload)
 *   
 *   PC0 = GRNLED  (Pin D8)
 *   PC3 = REDLED  (Pin D11)
 *   
 *   You will need MegaCoreX board definitions installed, available
 *   at https://github.com/MCUdude/MegaCoreX
 *   
 *   Reminder for burning bootloader with microUPDI programmer:
 *   Reference https://github.com/MCUdude/microUPDI
 *   The microUPDI will show up in Arduino as Arduino UNO WiFi Rev2
 *   Select that device in the Tools>Port menu
 *   Under Tools>Programmer choose Atmel mEDBG (ATmega32U4)
 *   Select Tools>Board>MegaCoreX>Atmega4808
 *   Clock internal 8MHz, BOD 2.6V, Pinout 32 pin standard,
 *   Reset pin "Reset", Bootloader "Optiboot (UART0 default pins)"
 *   Then hit burn bootloader, it should upload 512k bootloader
 * 
 */

#include "Arduino.h"
#include <MCP7940.h>  // https://github.com/Zanduino/MCP7940  Address 0x6F

#define REDLED 11
#define GRNLED 8
#define VREG_EN 24  // voltage regulator enable
const uint32_t SERIAL_SPEED{57600};     // Set the baud rate for Serial I/O
const uint8_t  SPRINTF_BUFFER_SIZE{32};  // Buffer size for sprintf()

/***************************************************************************************************
** Declare global variables and instantiate classes                                               **
***************************************************************************************************/
MCP7940_Class MCP7940;                           // Create an instance of the MCP7940
char          inputBuffer[SPRINTF_BUFFER_SIZE];  // Buffer for sprintf()/sscanf()


//-------------------------------------- 
void setup() {
  pinMode(REDLED, OUTPUT);
  pinMode(GRNLED, OUTPUT);
  digitalWrite(REDLED, HIGH); // set high to turn OFF
  digitalWrite(GRNLED, HIGH); // set high to turn OFF
//  pinMode(2, OUTPUT);
//  digitalWrite(2, HIGH); // reset SDA pin 
//  pinMode(VREG_EN, OUTPUT);
//  digitalWrite(VREG_EN, HIGH); // set low to turn off, high to turn on (~150usec to wake)
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
  
  // Turn on the square wave output pin of the RTC chip
  MCP7940.setSQWSpeed(3); // set SQW frequency to 32768 Hz (0=1Hz, 1 = 4.096kHz, 2 = 8.192kHz, 3 = 32.768kHz)
  MCP7940.setSQWState(true); // turn on the square wave output pin

//  digitalWrite(VREG_EN, LOW); // turn off

}

void loop() {


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
    digitalWrite(GRNLED, !(digitalRead(GRNLED)));   // Toggle the LED
  }                                                // of if the seconds have changed
  
  readCommand();  

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
