/*  7_OLED_test
 *   
 *   Bivalve bit has room for a 128x32 I2C OLED display, commonly sold 
 *   on Amazon.com. The displays normally run on a SSD1306 controller
 *   chip, and so we use the SSD1306Ascii library
 *  
 */


#include <Wire.h>
#include "SSD1306Ascii.h" // https://github.com/greiman/SSD1306Ascii
#include "SSD1306AsciiWire.h" // https://github.com/greiman/SSD1306Ascii


SSD1306AsciiWire oled; // create OLED display object, using I2C Wire1 port
#define I2C_ADDRESS1 0x3C // for OLED. DIY mall units list 0x78 as address, but need 0x3C to work here


void setup() {
  Wire.begin();
  Wire.setClock(400000L);
  oled.begin(&Adafruit128x32, I2C_ADDRESS1);  
  oled.setFont(Adafruit5x7);    
  oled.clear(); 
  oled.home();
//  oled.set2X();
  oled.print("Hello");
  oled.println();
  oled.println(F("Did this work?"));
  oled.println(F("Third line"));
  oled.println(F("Fourth line"));
  delay(1000);
  

}

void loop() {


}
