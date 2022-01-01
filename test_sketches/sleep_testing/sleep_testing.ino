/*
  Nano Every: Cycle of Blink 5 times then Sleep for 8 seconds
  Configuration option: No ATmega328P Register emulation

*/

#include <avr/sleep.h>
#define GRNLED 8

uint8_t loopCount = 0 ;


void RTC_init(void)
{
  while (RTC.STATUS > 0) ;    /* Wait for all register to be synchronized */
 
  RTC.CLKSEL = RTC_CLKSEL_INT1K_gc;        // Run low power oscillator (OSCULP32K) at 1024Hz for long term sleep
  RTC.PITINTCTRL = RTC_PI_bm;              // PIT Interrupt: enabled */

  RTC.PITCTRLA = RTC_PERIOD_CYC8192_gc | RTC_PITEN_bm;     // Set period 8 seconds (see data sheet) and enable PIC                      
}

ISR(RTC_PIT_vect)
{
  RTC.PITINTFLAGS = RTC_PI_bm;          // Clear interrupt flag by writing '1' (required) 
}

void setup() {
  pinMode(GRNLED, OUTPUT);
  RTC_init();   
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // Set sleep mode to POWER DOWN mode 
  sleep_enable();                       // Enable sleep mode, but not going to sleep yet 
}


void loop() {
  digitalWrite(GRNLED, LOW);   // turn the LED on (LOW is the voltage level)
  delay(200);                       // wait for a second
  digitalWrite(GRNLED, HIGH);    // turn the LED off by making the voltage HIGH
  delay(200);                       // wait for a second

  if( ++ loopCount == 5 ) {
    loopCount = 0 ;
    sleep_cpu();                     // Sleep the device and wait for an interrupt to continue
  }
}
