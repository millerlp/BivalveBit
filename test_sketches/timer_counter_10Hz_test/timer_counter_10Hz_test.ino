 // TODO this isn't close to working currently. 
 /* Trying to clock it off of TCA, which arduino should be setting
  *  to 125kHz (8MHz / 64) (need to confirm this for 8MHz mode)
  *  And then if you let TCB count to 65536 it should roll over at 1.9Hz
  *  but the green led output is clearly not doing that. 
  *  Try having it toggle one of the spare breakout pins (PD2) and scope it.
  */

 
 // Don't mess with timer counter B3 (TCB3), which is the arduino millis()/micros() counter
 // Timer counter A is used to generate the clock for TCB3
 // Example code from https://github.com/microchip-pic-avr-examples/atmega4809-getting-started-with-tcb-studio/blob/master/Using_TCB_in_Sleep_Mode/main.c
 
//#include "EveryTimerB.h" // https://github.com/Kees-van-der-Oord/Arduino-Nano-Every-Timer-Controller-B


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/cpufunc.h>

//#define GRNLED_ON   PORTC |=(1<<PORTC0)
//#define GRNLED_OFF  PORTC &= ~(1<<PORTC0)
//#define GRNLED_TOGGLE PORTC_OUTTGL |= PIN0_bm
#define REDLED 11
#define GRNLED 8



#define TCB_CMP_EXAMPLE_VALUE   (0xffff)  // (0x7fff) = 32768

void CLOCK_init (void);
void SLPCTRL_init (void);
void PORT_init (void);
void TCB0_init (void);

ISR(TCB0_INT_vect)
{
    TCB0.INTFLAGS = TCB_CAPT_bm; /* Clear the interrupt flag */
    digitalWrite(GRNLED, !digitalRead(GRNLED));
//    PORTC.IN = PIN0_bm; /* Toggle PC0 GPIO */
//    PORTC_OUTTGL |= PIN0_bm;
}

void CLOCK_init (void)
{
    /* Disable CLK_PER Prescaler */
//    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, (0 << CLKCTRL_PEN_bp) );
//    ccp_write_io( (void *) &CLKCTRL.MCLKCTRLB , (0 << CLKCTRL_PEN_bp)); 
    
    /* Select 32KHz Internal Ultra Low Power Oscillator (OSCULP32K) */
//    ccp_write_io( (void *) &CLKCTRL.MCLKCTRLA, (CLKCTRL_CLKSEL_OSCULP32K_gc));
//    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, (CLKCTRL_CLKSEL_OSCULP32K_gc));
    
    /* Wait for system oscillator changing to finish */
    while (CLKCTRL.MCLKSTATUS & CLKCTRL_SOSC_bm)
    {
        ;
    }
}

void SLPCTRL_init (void)
{
    /* Enable sleep mode and select Standby mode */
    SLPCTRL.CTRLA = SLPCTRL_SMODE_gm | SLPCTRL_SMODE_STDBY_gc;
}

//void PORT_init (void)
//{
//    PORTC.DIR |= PIN0_bm; /* Configure PC0 as digital output */
//    PORTC.OUT |= PIN0_bm; /* Set initial level of PC0 */
//}

void TCB0_init (void)
{
    /* Load the Compare or Capture register with the timeout value*/
    TCB0.CCMP = TCB_CMP_EXAMPLE_VALUE;
   
    /* Enable TCB and set CLK_PER divider to 1 (No Prescaling) */
//    TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm | TCB_RUNSTDBY_bm;
    TCB0.CTRLA = TCB_CLKSEL_CLKTCA_gc | TCB_ENABLE_bm | TCB_RUNSTDBY_bm;
    /* Enable Capture or Timeout interrupt */
    TCB0.INTCTRL = TCB_CAPT_bm;
}




void setup() {
  pinMode(REDLED, OUTPUT);
  pinMode(GRNLED, OUTPUT);
  Serial.begin(57600);



  digitalWrite(REDLED, HIGH); // high to shut off
  digitalWrite(GRNLED, LOW); // high to shut off
  delay(3000);
  digitalWrite(GRNLED, HIGH); // high to shut off
    delay(2000);
    Serial.println("Starting");
        SLPCTRL_init();
    Serial.println("Sleep adjusted");
//    PORT_init();
//    Serial.println("LED PC0 configured");
    delay(10);
//    CLOCK_init();
//    Serial.println("Clock adjusted");

    TCB0_init();
    Serial.println("TCB0_init");
    sei(); /* Enable Global Interrupts */

}



void loop() {
        sleep_mode();
        Serial.println("Awake");
        delay(10);
}
