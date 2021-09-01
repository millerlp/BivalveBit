/* BivalveBit_logger.ino
 *  Initial draft of a BivalveBit data logger program
 *  
 */

#include "SdFat.h" // https://github.com/greiman/SdFat-beta (you must be using ver. 2.1.0 or higher)
#include <Wire.h>  // built in library, for I2C communications
#include "SSD1306Ascii.h" // https://github.com/greiman/SSD1306Ascii
#include "SSD1306AsciiWire.h" // https://github.com/greiman/SSD1306Ascii
#include "MCP7940.h"  // https://github.com/Zanduino/MCP7940
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
** Declare global variables and instantiate classes for MCP7940 clock chip                                            **
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
  digitalWrite(VREG_EN, LOW); // set low to turn off, high to turn on (~150usec to wake)
  pinMode(HALL_SLEEP, OUTPUT);
  digitalWrite(HALL_SLEEP, LOW); // set high to wake, set low to sleep (~60usec to wake)
  analogReference(EXTERNAL); // using voltage regulator value on external pin
  pinMode(ANALOG_IN, INPUT); // Hall sensor input channel
  pinMode(BATT_MONITOR, INPUT); // Battery voltage input channel
  pinMode(BATT_MONITOR_EN, OUTPUT); // Battery monitor enable pin
  digitalWrite(BATT_MONITOR_EN, LOW); // pull low to turn off battery monitor circuit

  Serial.begin(57600);
  while (!Serial) { delay(1); } // Wait until serial port is opened

  //---------------------------------------------------------
  // OLED display setup
  Wire.begin();
  Wire.setClock(400000L);
  oled.begin(&Adafruit128x32, I2C_ADDRESS1);  
  oled.setFont(Adafruit5x7);    
  oled.clear(); 
  oled.home();
  oled.println("Hello");

  
  //--------------------------------------------------------------------------
  // RTC initialization
  while (!MCP7940.begin()) {  // Initialize RTC communications
    Serial.println(F("Unable to find MCP7940M. Checking again in 3s."));  // Show error text
    delay(3000);                                                          // wait a second
  }  // of loop until device is located
  Serial.println(F("MCP7940 initialized."));
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
  // TODO: Print date and time to OLED at startup
  
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
    
    return;
  }
  Serial.println("card initialized.");
  oled.println("SD card found");
  digitalWrite(GRNLED,LOW);
  delay(100);
  digitalWrite(GRNLED,HIGH);
  delay(100);

  //------------------------------------------------------------------
  // Enable 3.0V voltage regulator so that you can talk to the heart and gape sensors
  digitalWrite(VREG_EN, HIGH); // Set high to turn on
  delayMicroseconds(250); // Give at least 150us for regulator to turn on

  //------------------------------------------------------------------
  // Start up VCNL4040 proximity sensor (heart sensor)
  if (!vcnl4040.begin()) {
    Serial.println("Couldn't find VCNL4040 chip");
    digitalWrite(REDLED, LOW);
    delay(500);
    digitalWrite(REDLED, HIGH);
    delay(100);
    oled.home();
    oled.clear();
    oled.println("Heart sensor fail");
  } else {
    oled.println("Heart sensor on");
    digitalWrite(GRNLED,LOW);
    delay(100);
    digitalWrite(GRNLED,HIGH);
    delay(100);
  }
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
      Serial.println("Device failed to setup- Freezing code.");
      oled.println("Temp sensor fail");
      
      digitalWrite(REDLED,LOW);
      delay(100);
      digitalWrite(REDLED,HIGH);
      delay(100);
    }

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


    //------------------------------------------------------------
    // Battery voltage circuit test
    batteryVolts = readBatteryVoltage(BATT_MONITOR_EN,BATT_MONITOR,dividerRatio,refVoltage);
    oled.print("Battery: ");
    oled.print(batteryVolts,3);
    oled.println("V");

    //---------------------------------------
    // Voltage regulator can be turned off again
    digitalWrite(VREG_EN, LOW); // set low to turn off


}   // end of setup loop


//-------------------------------------------------------------
void loop() {




}
