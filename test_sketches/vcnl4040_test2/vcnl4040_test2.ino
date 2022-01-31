/*  vcnl4040_test2.ino
 *   Using this to test different modes for the VCNL4040 and 
 *   timing
 *  Tested working with BivalveBit RevB board
 *  This script can be run with the Arduino Serial Plotter window to
 *  directly show the signal trace
 *  
 */

// Use this with the serial plotter to just see the proximity channel output
#include <Adafruit_VCNL4040.h> // https://github.com/adafruit/Adafruit_VCNL4040 & https://github.com/adafruit/Adafruit_BusIO

Adafruit_VCNL4040 vcnl4040 = Adafruit_VCNL4040();

#define REDLED 11
#define GRNLED 8
#define VREG_EN 24  // voltage regulator enable

unsigned long oldmillis;
unsigned long interval = 125; // interval in milliseconds for each new sample
    
void setup() {
  pinMode(REDLED, OUTPUT);
  pinMode(GRNLED, OUTPUT);
  digitalWrite(REDLED, HIGH); // set high to turn OFF
  digitalWrite(GRNLED, HIGH); // set high to turn OFF
  pinMode(VREG_EN, OUTPUT);
  digitalWrite(VREG_EN, HIGH); // set low to turn off, high to turn on (~150usec to wake)
  
  Serial.begin(57600);
  // Wait until serial port is opened
  while (!Serial) { delay(1); }
  if (!vcnl4040.begin()) {
    Serial.println("Couldn't find VCNL4040 chip");
    while (1) {
      digitalWrite(REDLED, !(digitalRead(REDLED)));
      delay(200);
    };
  }

  // Flash green led to signal that sensor was found
  digitalWrite(GRNLED, LOW);
  delay(1000);
  digitalWrite(GRNLED, HIGH); // turn off
  

  vcnl4040.enableAmbientLight(false);
  vcnl4040.enableWhiteLight(false);
  vcnl4040.enableProximity(true);
  vcnl4040.setProximityHighResolution(true);
  // Setting VCNL4040_LED_DUTY_1_40 gives shortest proximity measurement time (about 4.85ms)
  vcnl4040.setProximityLEDDutyCycle(VCNL4040_LED_DUTY_1_320); // 1_40, 1_80,1_160,1_320
  vcnl4040.setProximityLEDCurrent(VCNL4040_LED_CURRENT_50MA); // 50,75,100,120,140,160,180,200
  // Setting VCNL4040_PROXIMITY_INTEGRATION_TIME_1T gives the shortest pulse (lowest LED output)
  // in combination with the LED_CURRENT setting above. A longer integration time like
  // VCNL4040_PROXIMITY_INTEGRATION_TIME_8T raises the pulse length (higher LED output) in
  // combination with the LED_CURRENT setting.
  vcnl4040.setProximityIntegrationTime(VCNL4040_PROXIMITY_INTEGRATION_TIME_8T); // 1T,1_5T,2T,2_5T,3T,3_5T,4T,8T
  
  // Testing notes:
  // 1_40, 8T, 50mA gives a pulse range of about 50 counts on a thumb, around 6000-7000 on scale
  // 1_40, 8T, 100mA gives a pulse range around 40-50 counts, at around 9600 on scale
  // 1_40, 8T, 160mA gives a pulse range around 50-60 counts, at around 7500 on scale
  // 1_40, 4T, 50mA gives range of 20 counts, around 2500 on scale
  // 1_320, 4T, 50mA gives range of 20 counts, around 2500 on scale, poor quality signal (slow?)
  // 1_320, 1T, 50mA gives range of 4 counts, around 700 on scale, poor quality signal (slow?)
  // 1_40, 1T, 50mA, gives range of 5-6 counts, around 700 on scale

  //vcnl4040.setProximityLEDCurrent(VCNL4040_LED_CURRENT_200MA);
//  Serial.print("Proximity LED current set to: ");
//  switch(vcnl4040.getProximityLEDCurrent()) {
//    case VCNL4040_LED_CURRENT_50MA: Serial.println("50 mA"); break;
//    case VCNL4040_LED_CURRENT_75MA: Serial.println("75 mA"); break;
//    case VCNL4040_LED_CURRENT_100MA: Serial.println("100 mA"); break;
//    case VCNL4040_LED_CURRENT_120MA: Serial.println("120 mA"); break;
//    case VCNL4040_LED_CURRENT_140MA: Serial.println("140 mA"); break;
//    case VCNL4040_LED_CURRENT_160MA: Serial.println("160 mA"); break;
//    case VCNL4040_LED_CURRENT_180MA: Serial.println("180 mA"); break;
//    case VCNL4040_LED_CURRENT_200MA: Serial.println("200 mA"); break;
//  }
  
  //vcnl4040.setProximityLEDDutyCycle(VCNL4040_LED_DUTY_1_40);
//  Serial.print("Proximity LED duty cycle set to: ");
//  switch(vcnl4040.getProximityLEDDutyCycle()) {
//    case VCNL4040_LED_DUTY_1_40: Serial.println("1/40"); break; // 1 measurement every 4.85ms, about 200 samples per second
//    case VCNL4040_LED_DUTY_1_80: Serial.println("1/80"); break;
//    case VCNL4040_LED_DUTY_1_160: Serial.println("1/160"); break;
//    case VCNL4040_LED_DUTY_1_320: Serial.println("1/320"); break; // 1 measurement every 40ms, about 25 samples per second
//  }

  //vcnl4040.setAmbientIntegrationTime(VCNL4040_AMBIENT_INTEGRATION_TIME_80MS);
//  Serial.print("Ambient light integration time set to: ");
//  switch(vcnl4040.getAmbientIntegrationTime()) {
//    case VCNL4040_AMBIENT_INTEGRATION_TIME_80MS: Serial.println("80 ms"); break;
//    case VCNL4040_AMBIENT_INTEGRATION_TIME_160MS: Serial.println("160 ms"); break;
//    case VCNL4040_AMBIENT_INTEGRATION_TIME_320MS: Serial.println("320 ms"); break;
//    case VCNL4040_AMBIENT_INTEGRATION_TIME_640MS: Serial.println("640 ms"); break;
//  }


  //vcnl4040.setProximityIntegrationTime(VCNL4040_PROXIMITY_INTEGRATION_TIME_8T);
//  Serial.print("Proximity integration time set to: ");
//  switch(vcnl4040.getProximityIntegrationTime()) {
//    case VCNL4040_PROXIMITY_INTEGRATION_TIME_1T: Serial.println("1T"); break; // 1T = 125usec
//    case VCNL4040_PROXIMITY_INTEGRATION_TIME_1_5T: Serial.println("1.5T"); break;
//    case VCNL4040_PROXIMITY_INTEGRATION_TIME_2T: Serial.println("2T"); break;
//    case VCNL4040_PROXIMITY_INTEGRATION_TIME_2_5T: Serial.println("2.5T"); break;
//    case VCNL4040_PROXIMITY_INTEGRATION_TIME_3T: Serial.println("3T"); break;
//    case VCNL4040_PROXIMITY_INTEGRATION_TIME_3_5T: Serial.println("3.5T"); break;
//    case VCNL4040_PROXIMITY_INTEGRATION_TIME_4T: Serial.println("4T"); break;
//    case VCNL4040_PROXIMITY_INTEGRATION_TIME_8T: Serial.println("8T"); break; // 8T = 800usec
//  }

  //vcnl4040.setProximityHighResolution(false);
//  Serial.print("Proximity measurement high resolution? ");
//  Serial.println(vcnl4040.getProximityHighResolution() ? "True" : "False");

//  Serial.println("");


  oldmillis = millis();
  interval = 125;
}                       // end of setup loop

void loop() {
  
  if ( millis() - oldmillis > interval){
    oldmillis = millis();
    
//    digitalWrite(VREG_EN, HIGH); // high to turn on 
    vcnl4040.enableProximity(true); // turn on the VCNL4040 heart sensor
//    delay(114); // seems to take at least 113ms for the VCNL to return a valid value after power-on
    int16_t vcnlValue = vcnl4040.getProximity();
    vcnl4040.enableProximity(false); // turn off the VCNL4040 heart sensor
//    digitalWrite(VREG_EN, LOW); // low to turn off 
    Serial.println(vcnlValue);
  }
  
//  Serial.print("Proximity:"); 
//  Serial.println(vcnl4040.getProximity());
//  Serial.print("Ambient light:"); Serial.println(vcnl4040.getLux());
//  Serial.print("Raw white light:"); Serial.println(vcnl4040.getWhiteLight());
//  Serial.println("");
 
//  delay(50);
}
