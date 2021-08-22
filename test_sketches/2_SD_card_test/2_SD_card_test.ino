/*  SD_card_test.ino
 *   Tested on BivalveBit RevB
 *  using the "Standard" 48-pin pinout for the 4808 chip as defined in the MegaCoreX
 *  boards file (https://github.com/MCUdude/MegaCoreX) using version 1.0.8 (2021-05),
 *  with the Optiboot (UART0 default pins) bootloader option,
 *  and using SdFat-beta version=2.1.0-beta.1 (https://github.com/greiman/SdFat-beta)
 *  The bootloader was burned with an Atmel-ICE via UPDI, after upgrading the
 *  programmer to firmware v1.29 from the shipped v1.0. Arduino v1.8.13
 *  
 *  This will flash green if the card is found, and just flash red
 *  if no card is found.
 */


#include "SdFat.h" // https://github.com/greiman/SdFat-beta (you must be using ver. 2.1.0 or higher)

// SPI pins for SD card
const uint8_t SD_CHIP_SELECT = 7;
#define MOSI 4
#define MISO 5
#define SCK 6

#define REDLED 11
#define GRNLED 8


SdFat sd;

bool failFlag = false;

void setup() {
  // LED setup
  pinMode(REDLED, OUTPUT);
  pinMode(GRNLED, OUTPUT);
  digitalWrite(REDLED, HIGH); // set high to turn OFF
  digitalWrite(GRNLED, HIGH); // set high to turn OFF
  // Serial start
  Serial.begin(57600);
  Serial.println("Hello");


  // see if the card is present and can be initialized:
  if (!sd.begin(SD_CHIP_SELECT)) {
    Serial.println("Card failed, or not present");
    failFlag = true;
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");

  
}

void loop() {
  // If failFlag is false, the SD card was detected, flash green LED
  if (failFlag == false){
    for (byte i = 0; i < 5; i++){
      digitalWrite(GRNLED, LOW);
      delay(500);
      digitalWrite(GRNLED, HIGH);
      delay(500);  
    }
  } else if (failFlag) {
    // Flash red LED if SD card detection fails
      digitalWrite(REDLED, LOW);
      delay(50);
      digitalWrite(REDLED, HIGH);
      delay(50); 
  }
}
