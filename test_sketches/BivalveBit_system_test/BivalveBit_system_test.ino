/* BivalveBit_system_test.ino
 *  Check basic functions of a newly assembled BivalveBit logger
 *  
 *  You can set the real time clock using a command in the serial
 *  monitor in the format:
 *  SETDATE yyyy-mm-dd hh:mm:ss
 *  
 */

#include "SdFat.h" // https://github.com/greiman/SdFat-beta (you must be using ver. 2.1.0 or higher)
#include <Wire.h>  // built in library, for I2C communications
#include "SSD1306Ascii.h" // https://github.com/greiman/SSD1306Ascii
#include "SSD1306AsciiWire.h" // https://github.com/greiman/SSD1306Ascii
#include "MCP7940.h"  // https://github.com/Zanduino/MCP7940  Real time clock
#include <Adafruit_VCNL4040.h> // https://github.com/adafruit/Adafruit_VCNL4040 & https://github.com/adafruit/Adafruit_BusIO
#include <SparkFun_TMP117.h> // https://github.com/sparkfun/SparkFun_TMP117_Arduino_Library
#include "BivalveBit_lib.h" // https://github.com/millerlp/BivalveBit_lib

/*******************************************************
 * SD card objects
 *******************************************************/
// SPI pins for SD card
const uint8_t SD_CHIP_SELECT = 7;
#define MOSI 4
#define MISO 5
#define SCK 6
SdFat sd;  // sd card object
bool SDfailFlag = false;
SdFile logfile;  // for sd card, this is the file object to be written to
char filename[] = "YYYYMMDD_HHMM_00_SN00.csv";

/**********************************************************
 * Create VCNL4040 object
********************************************************* */
Adafruit_VCNL4040 vcnl4040 = Adafruit_VCNL4040();

/**************************************************
 *  Create TMP117 temperature sensor object
 *   The default address of the device is 0x48 = (GND)
 *****************************************************/
TMP117 TMP117sensor; // Initalize sensor


/************************************************************
 * Hall effect sensor definitions
 ************************************************************/
#define ANALOG_IN A0  // Analog input pin for Hall sensor
#define HALL_SLEEP 10 // Hall sensor sleep pin, pull high to wake
unsigned int HallValue = 0; // Variable for Hall sensor reading

/***************************************************************************************************
**  MCP7940 real time clock chip                                            **
***************************************************************************************************/
const uint32_t SERIAL_SPEED{57600};     // Set the baud rate for Serial I/O
const uint8_t  SPRINTF_BUFFER_SIZE{32};  // Buffer size for sprintf()
MCP7940_Class MCP7940;                           // Create an instance of the MCP7940
char          inputBuffer[SPRINTF_BUFFER_SIZE];  // Buffer for sprintf()/sscanf()
DateTime newtime;
DateTime oldtime;
//*******************************************

/* ***************************************
 *  OLED display objects ****
**********************************************/
SSD1306AsciiWire oled; // create OLED display object, using I2C Wire1 port
#define I2C_ADDRESS1 0x3C // for OLED. DIY mall units list 0x78 as address, but need 0x3C to work here

/********************************************************
 * Miscellaneous definitions
 *******************************************************/
#define REDLED 11   // Red LED pin
#define GRNLED 8    // Green LED pin
#define VREG_EN 24  // voltage regulator enable


/************************************************************
 *  Battery monitor variables
 ***********************************************************/
const byte BATT_MONITOR_EN = 9; // digital output channel to turn on battery voltage check
const byte BATT_MONITOR = A1;  // analog input channel to sense battery voltage

const float dividerRatio = 2; // Ratio of voltage divider (47k + 47k) / 47k = 2
const float refVoltage = 3.00; // Voltage at AREF pin on ATmega microcontroller, measured per board
//float refVoltage = 2.5; // Internal 2.5V voltage reference value (INTERNAL2V5)
float batteryVolts = 0; // Estimated battery voltage returned from readBatteryVoltage function

/**************************************************************************
 * Setup loop
 **************************************************************************/
void setup() {
  pinMode(REDLED, OUTPUT);
  pinMode(GRNLED, OUTPUT);
  digitalWrite(REDLED, HIGH); // set high to turn OFF
  digitalWrite(GRNLED, HIGH); // set high to turn OFF
  pinMode(VREG_EN, OUTPUT);
  digitalWrite(VREG_EN, HIGH); // set low to turn off, high to turn on (~150usec to wake)
  pinMode(HALL_SLEEP, OUTPUT);
  digitalWrite(HALL_SLEEP, LOW); // set high to wake, set low to sleep (~60usec to wake)
  analogReference(EXTERNAL); // using voltage regulator value on external pin
  pinMode(ANALOG_IN, INPUT); // Hall sensor input channel
  pinMode(BATT_MONITOR, INPUT); // Battery voltage input channel
  pinMode(BATT_MONITOR_EN, OUTPUT); // Battery monitor enable pin
  digitalWrite(BATT_MONITOR_EN, LOW); // pull low to turn off battery monitor circuit

  // Flash both LEDs to show that they work
  for (int i = 0; i++; i<5){
    digitalWrite(REDLED, !digitalRead(REDLED));
    delay(100);
    digitalWrite(GRNLED, !digitalRead(GRNLED));
    delay(100);
  }
  digitalWrite(REDLED, HIGH); // turn off
  digitalWrite(GRNLED, HIGH); // turn off
  
  Serial.begin(57600);
//  while (!Serial) { delay(1); } // Wait until serial port is opened
  Serial.println("Hi");

  //---------------------------------------------------------
  // OLED display setup
  Wire.begin();
  Wire.setClock(400000L);
  oled.begin(&Adafruit128x32, I2C_ADDRESS1);  
  oled.setFont(Adafruit5x7);    
  oled.clear(); 
  oled.home();
//  oled.println("Hello");

  
  //--------------------------------------------------------------------------
  // RTC initialization
  while (!MCP7940.begin()) {  // Initialize RTC communications
    Serial.println(F("Unable to find MCP7940M real time clock. Checking again in 3s."));  // Show error text
    delay(3000);                                                          // wait a second
  }  // of loop until device is located
  Serial.println(F("MCP7940 real time clock initialized."));
  // Turn on battery backup, default is off
  MCP7940.setBattery(true); // enable battery backup mode
  Serial.print("Battery Backup mode is ");
  if (MCP7940.getBattery()) {
   Serial.println("enabled.");
  } else {
   Serial.println("disabled.");
  }
  // Turn on the square wave output pin of the RTC chip
  MCP7940.setSQWSpeed(3); // set SQW frequency to 32768 Hz
  MCP7940.setSQWState(true); // turn on the square wave output pin
  newtime = MCP7940.now();
  Serial.print("Current clock time: ");
  printTimeSerial(newtime);
  Serial.println();
  // TODO: Print date and time to OLED at startup
  printTimeOLED(newtime, oled);
  oled.println();
  delay(2000);
  
  //----------------------------------------------------------
  // SD card initialization
  if (!sd.begin(SD_CHIP_SELECT)) {
    Serial.println("Card failed, or not present");
    oled.println("SD card fail");
    SDfailFlag = true;
    digitalWrite(REDLED, LOW);
    delay(100);
    digitalWrite(REDLED, HIGH);
    delay(100);
    
//    return;  // this would restart the setup loop
  } else {
    Serial.println("SD card initialized.");
    oled.println("SD card found");
  }
  digitalWrite(GRNLED,LOW);
  delay(100);
  digitalWrite(GRNLED,HIGH);
  delay(500);

  //------------------------------------------------------------------
  // Enable 3.0V voltage regulator so that you can talk to the heart and gape sensors
  digitalWrite(VREG_EN, HIGH); // Set high to turn on
  delayMicroseconds(250); // Give at least 150us for regulator to turn on
  
  oled.home();
  oled.clear();
  //------------------------------------------------------------------
  // Start up VCNL4040 proximity sensor (heart sensor)
  if (!vcnl4040.begin()) {
    Serial.println("Couldn't find VCNL4040 heart sensor chip");
    digitalWrite(REDLED, LOW);
    delay(500);
    digitalWrite(REDLED, HIGH);
    delay(100);

    oled.println("Heart sensor fail");
  } else {
    oled.println("Heart sensor on");
    Serial.println("VCNL4040 Heart sensor activated");
    digitalWrite(GRNLED,LOW);
    delay(100);
    digitalWrite(GRNLED,HIGH);
    delay(100);
    vcnl4040.enableAmbientLight(false);
    vcnl4040.enableWhiteLight(false);
    vcnl4040.enableProximity(true);
    vcnl4040.setProximityHighResolution(true);
    // Setting VCNL4040_LED_DUTY_1_40 gives shortest proximity measurement time (about 4.85ms)
    vcnl4040.setProximityLEDDutyCycle(VCNL4040_LED_DUTY_1_40); // 1_40, 1_80,1_160,1_320
    vcnl4040.setProximityLEDCurrent(VCNL4040_LED_CURRENT_50MA); // 50,75,100,120,140,160,180,200
    // Setting VCNL4040_PROXIMITY_INTEGRATION_TIME_1T gives the shortest pulse (lowest LED output)
    // in combination with the LED_CURRENT setting above. A longer integration time like
    // VCNL4040_PROXIMITY_INTEGRATION_TIME_8T raises the pulse length (higher LED output) in
    // combination with the LED_CURRENT setting.
    vcnl4040.setProximityIntegrationTime(VCNL4040_PROXIMITY_INTEGRATION_TIME_1T); // 1T,1_5T,2T,2_5T,3T,3_5T,4T,8T
    delay(50);
  }

  //----------------------------------------------------

  //--------------------------------------------------------------------------
  // Initialize TMP117 temperature sensor on the heart sensor board
  if (TMP117sensor.begin() == true) // Function to check if the sensor will correctly self-identify with the proper Device ID/Address
    {
      Serial.println("Begin TMP117 temperature sensor");
      oled.println("Temp sensor on");
      digitalWrite(GRNLED, LOW);
      delay(100);
      digitalWrite(GRNLED, HIGH);
      delay(100);
    }  else {
      Serial.println("TMP117 temperature sensor failed to setup");
      oled.println("Temp sensor fail");
      
      digitalWrite(REDLED,LOW);
      delay(100);
      digitalWrite(REDLED,HIGH);
      delay(100);
    }
    delay(400);
    //-------------------------------------------------------------------
    // Initialize Hall sensor
    digitalWrite(HALL_SLEEP, HIGH); // turn on hall effect sensor
    delayMicroseconds(50);
    HallValue = readHall(ANALOG_IN); // Function in BivalveBit_lib 
    digitalWrite(HALL_SLEEP, LOW); // put hall sensor to sleep
    if ( (HallValue > 0) & (HallValue < 1023) ) {
      Serial.print("Hall sensor: ");
      Serial.println(HallValue);
      oled.print("Hall: ");
      oled.println(HallValue);
      digitalWrite(GRNLED, LOW);
      delay(100);
      digitalWrite(GRNLED, HIGH);
      delay(100);
    } else {
      Serial.println("Hall sensor problem");
      oled.println("Hall sensor fail");
      digitalWrite(REDLED, LOW);
      delay(200);
      digitalWrite(REDLED, HIGH);
      delay(100);
    }
    delay(500);

    //------------------------------------------------------------
    // Battery voltage circuit test
    batteryVolts = readBatteryVoltage(BATT_MONITOR_EN,BATT_MONITOR,dividerRatio,refVoltage);
    oled.print("Battery: ");
    oled.print(batteryVolts,3);
    oled.println("V");
    Serial.print("Battery: ");
    Serial.print(batteryVolts,3);
    Serial.println("V");
    delay(500);

    //---------------------------------------
    // Voltage regulator can be turned off again
//    digitalWrite(VREG_EN, LOW); // set low to turn off


}   // end of setup loop


//-------------------------------------------------------------
void loop() {

  
  digitalWrite(GRNLED,!digitalRead(GRNLED));
  delay(1000);
  newtime = MCP7940.now(); // read the clock
  printTimeSerial(newtime);
  Serial.print(" ");
//  digitalWrite(HALL_SLEEP, HIGH); // turn on hall effect sensor
//  delayMicroseconds(50);
//  HallValue = readHall(ANALOG_IN); // Function in BivalveBit_lib 
  HallValue = readWakeHall(ANALOG_IN, HALL_SLEEP); // Function in BivalveBit_lib 
//  digitalWrite(HALL_SLEEP, LOW); // put hall sensor to sleep
  Serial.print("Hall: ");
  Serial.print(HallValue);
  Serial.print("  VCNL: ");
  unsigned int Prox = vcnl4040.getProximity();
  Serial.print(Prox);
  Serial.print("  temp: ");
  float tempC = TMP117sensor.readTempC();
  // Print temperature in °C
  Serial.print(tempC);
  Serial.print("C");
  batteryVolts = readBatteryVoltage(BATT_MONITOR_EN,BATT_MONITOR,dividerRatio,refVoltage);
  Serial.print("  Battery: ");
  Serial.print(batteryVolts,3);
  Serial.println("V");
  Serial.println();
  
  oled.home();
  oled.clear();
  oled.print("Hall: ");
  oled.println(HallValue);
  oled.print("VCNL: ");
  oled.println(Prox);
  oled.print("temp: ");
  oled.println(tempC,2);
  oled.print("Battery: ");
  oled.println(batteryVolts,3);
  
  readCommand();  // Look for a clock setting command (optional)

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
