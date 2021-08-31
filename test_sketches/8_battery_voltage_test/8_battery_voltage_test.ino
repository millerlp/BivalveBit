/* 8_battery_voltage_test.ino
 *  
 *  A sketch to test the battery voltage reading on 
 *  RevB BivalveBit hardware. 
 *  Using 47k resistors for the voltage divider
 *  
 *  We'll use the OLED display to show the voltage so
 *  that the device can be solely battery powered for this test
 */

#include <Wire.h>
#include "SSD1306Ascii.h" // https://github.com/greiman/SSD1306Ascii
#include "SSD1306AsciiWire.h" // https://github.com/greiman/SSD1306Ascii

SSD1306AsciiWire oled; // create OLED display object, using I2C Wire1 port
#define I2C_ADDRESS1 0x3C // for OLED. DIY mall units list 0x78 as address, but need 0x3C to work here

#define REDLED 11   // red LED
#define GRNLED 8    // green LED
#define VREG_EN 24  // voltage regulator enable

//------------------------------------------------------------
//  Battery monitor 
byte BATT_MONITOR_EN = 9; // digital output channel to turn on battery voltage check
byte BATT_MONITOR = A1;  // analog input channel to sense battery voltage

float dividerRatio = 2; // Ratio of voltage divider (47k + 47k) / 47k = 2
float refVoltage = 3.16; // Voltage at AREF pin on ATmega microcontroller, measured per board
float batteryVolts = 0; // Estimated battery voltage returned from readBatteryVoltage function


//------------readBatteryVoltage-------------------
// readBatteryVoltage function. This will read the AD convertor
// and calculate the approximate battery voltage (before the
// voltage regulator). Returns a floating point value for
// voltage.
float readBatteryVoltage(byte BATT_MONITOR_EN,byte BATT_MONITOR,float dividerRatio,float refVoltage){
    // Turn on the battery voltage monitor circuit
    digitalWrite(BATT_MONITOR_EN, HIGH);
    delay(1);
    // Read the analog input pin
    unsigned int rawAnalog = 0;
    analogRead(BATT_MONITOR); // This initial value is ignored
    delay(3); // Give the ADC time to stablize
    // Take 4 readings
    for (byte i = 0; i<4; i++){
        rawAnalog = rawAnalog + analogRead(BATT_MONITOR);
        delay(2);
    }
    // Do a 2-bit right shift to divide rawAnalog
    // by 4 to get the average of the 4 readings
    rawAnalog = rawAnalog >> 2;
    // Shut off the battery voltage sense circuit
    digitalWrite(BATT_MONITOR_EN, LOW);
    // Convert the rawAnalog count value (0-1023) into a voltage
    // Relies on variables dividerRatio and refVoltage
    float reading = (rawAnalog  * (refVoltage / 1023.0)) * dividerRatio;
    return reading; // return voltage result
}

void setup() {
  pinMode(REDLED, OUTPUT);
  pinMode(GRNLED, OUTPUT);
  digitalWrite(REDLED, HIGH); // set high to turn OFF
  digitalWrite(GRNLED, HIGH); // set high to turn OFF
  // Voltage regulator pins
  pinMode(VREG_EN, OUTPUT);
  digitalWrite(VREG_EN, HIGH); // set low to turn off, high to turn on (~150usec to wake)
  analogReference(EXTERNAL); // using voltage regulator value on external pin (will this fail if vreg is off?)
  // Battery monitor pins
  pinMode(BATT_MONITOR, INPUT); // Battery voltage input channel
  pinMode(BATT_MONITOR_EN, OUTPUT); // Battery monitor enable pin
  digitalWrite(BATT_MONITOR_EN, LOW); // pull low to turn off battery monitor circuit
  // Set up OLED display
  Wire.begin();
  Wire.setClock(400000L);
  oled.begin(&Adafruit128x32, I2C_ADDRESS1);  
  oled.setFont(Adafruit5x7);    
  oled.clear(); 
  oled.home();
//  oled.set2X();
  oled.print("Hello");
  oled.println();
  delay(1000);

}

void loop() {
  batteryVolts = readBatteryVoltage(BATT_MONITOR_EN,BATT_MONITOR,dividerRatio,refVoltage);
  oled.clear();
  oled.home();
  oled.print(batteryVolts,3);
  oled.print(" V");
  delay(1000);

}
