/*  Basic_blink
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
 *   Clock internal 16MHz, BOD 2.6V, Pinout 32 pin standard,
 *   Reset pin "Reset", Bootloader "Optiboot (UART0 default pins)"
 *   Then hit burn bootloader, it should upload 512k bootloader
 * 
 */

#include "Arduino.h"
#define REDLED 11
#define GRNLED 8
 
void setup() {
  pinMode(REDLED, OUTPUT);
  pinMode(GRNLED, OUTPUT);
  digitalWrite(REDLED, LOW); // set low to turn ON
  digitalWrite(GRNLED, LOW); // set low to turn OFF

  Serial.begin(57600);
  delay(100);
  Serial.println("Hello");

}

void loop() {
  digitalWrite(REDLED, !(digitalRead(REDLED)));
  digitalWrite(GRNLED, !(digitalRead(GRNLED)));
  delay(500);

}
