/* 6_Hall_sensor_test.ino
 *  
 *  
 *  
 *  Tested with RevB hardware, on 3V3 supply and 3.9V LiIon battery
 *  
 */

//#include "MusselGapeTrackerlib.h" // https://github.com/millerlp/MusselGapeTrackerlib

#define ANALOG_IN A0  // Input for Hall sensor
#define REDLED 11   // red LED
#define GRNLED 8    // green LED
#define VREG_EN 24  // voltage regulator enable
#define HALL_SLEEP 10 // Hall sensor sleep pin, pull high to wake
 
void setup() {
  pinMode(REDLED, OUTPUT);
  pinMode(GRNLED, OUTPUT);
  digitalWrite(REDLED, HIGH); // set high to turn OFF
  digitalWrite(GRNLED, HIGH); // set high to turn OFF
  pinMode(VREG_EN, OUTPUT);
  digitalWrite(VREG_EN, HIGH); // set low to turn off, high to turn on (~150usec to wake)
  pinMode(HALL_SLEEP, OUTPUT);
  digitalWrite(HALL_SLEEP, LOW); // set high to wake, set low to sleep (~60usec to wake)
  analogReference(EXTERNAL); // using voltage regulator value on external pin (will this fail if vreg is off?)
  pinMode(ANALOG_IN, INPUT); // Hall sensor input channel
  Serial.begin(57600);
  // Wait until serial port is opened
  while (!Serial) { delay(1); }
//  Serial.print("Hello");

}


// Function to take a few readings from hall sensor and average them
unsigned int readHall(byte ANALOG_IN){
  unsigned int rawAnalog = 0;
  analogRead(ANALOG_IN); // throw away 1st reading
  for (byte i = 0; i<4; i++){
    rawAnalog = rawAnalog + analogRead(ANALOG_IN);
    delay(1);
  }
  // Do a 2-bit right shift to divide rawAnalog
  // by 4 to get the average of the 4 readings
  rawAnalog = rawAnalog >> 2;   
  return rawAnalog;
}

//-----------------------------------------------
// Main loop
void loop() {
  digitalWrite(VREG_EN, HIGH); // turn on voltage regulator
  digitalWrite(HALL_SLEEP, HIGH); // turn on hall effect sensor
  delayMicroseconds(250); // give voltage regulator time to turn on
  unsigned int HallValue = readHall(ANALOG_IN);
  digitalWrite(VREG_EN, LOW); // turn voltage regulator off again
  digitalWrite(HALL_SLEEP, LOW); // put hall sensor to sleep
  Serial.println(HallValue);
  digitalWrite(GRNLED,!digitalRead(GRNLED));
  delay(100);


}
