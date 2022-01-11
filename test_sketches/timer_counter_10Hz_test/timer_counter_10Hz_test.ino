 // TODO this isn't close to working currently
 
#include "EveryTimerB.h" https://github.com/Kees-van-der-Oord/Arduino-Nano-Every-Timer-Controller-B

#define GRNLED_ON   PORTC |=(1<<PORTC0)
#define GRNLED_OFF  PORTC &= ~(1<<PORTC0)
#define GRNLED_TOGGLE PORTC_OUTTGL |= PIN0_bm
#define REDLED 11
#define GRNLED 8

void setup() {
  pinMode(REDLED, OUTPUT);
  pinMode(GRNLED, OUTPUT);
  digitalWrite(REDLED, HIGH); // high to shut off
  digitalWrite(GRNLED, HIGH); // high to shut off

  TimerB2.initialize();
  TimerB2.attachInterrupt(timerInt);
  TimerB2.setPeriod(1000000);
}

void timerInt(){
  GRNLED_TOGGLE;
}



void loop() {


}
