/* This assumes you have a 32.768kHz clock signal hooked up
 *  to the RTC pin on the Atmega4808. 
 *  
 *  
 *  Based on code from Microchip bulletin TB3213
 *  http://ww1.microchip.com/downloads/en/AppNotes/TB3213-Getting-Started-with-RTC-90003213A.pdf
 *  
 *  
 *  Set up the RTC on the 4808 to take an external clock signal
 *  which will be the 32.768kHz signal from the separate real time
 *  clock chip. Have it fire an interrupt once per second, which will 
 *  be counted. Also take in a 1 pulse-per-second signal on a digital
 *  pin from a GPS module, firing an interrupt each time.
 *  
 *  After a set number of GPS interrupts has fired, read the count of the 
 *  RTC interrupts that had fired, and read the current
 *  value of the RTC.CNT register to get its value (which should be very 
 *  near another full 1-second tick when the GPS tick happens). 
 *  
 *  
 *  TODO: RTC interrupt counter is running at 1/2 speed (2 seconds)
 */

#include <MCP7940.h>  // https://github.com/Zanduino/MCP7940  Address 0x6F


volatile uint16_t RTCticks = 0; // Used to count RTC interrupts
volatile uint16_t GPSticks = 0; // Used to count GPSticks
int8_t trimValue = 20; // Signed crystal frequency trimming value -127 to +127
#define GPSinput 14 // Take input on PD2, digital pin 14
#define GRNLED 8 

#define RTC_EXAMPLE_PERIOD (32767) // Set how high the RTC will run before overflow (max 32767)

MCP7940_Class MCP7940;                           // Create an instance of the MCP7940
const uint8_t  SPRINTF_BUFFER_SIZE{32};  // Buffer size for sprintf()
char          inputBuffer[SPRINTF_BUFFER_SIZE];  // Buffer for sprintf()/sscanf()

void setup() {
  Serial.begin(57600);
  Serial.println("Hello");

  Wire.begin();
  pinMode(GPSinput, INPUT);
  pinMode(GRNLED, OUTPUT);
  digitalWrite(GRNLED, HIGH);
  pinMode(17, OUTPUT);  // PD5 = D17
  Serial.println("Clock check");
  delay(20);
  while (!MCP7940.begin()) {  // Initialize RTC communications
    Serial.println(F("Unable to find MCP7940. Checking again in 3s."));  // Show error text
    delay(3000);                                                          // wait a second
  }  // of loop until device is located
  Serial.println(F("MCP7940 initialized."));
    while (!MCP7940.deviceStatus()) {  // Turn oscillator on if necessary
    Serial.println(F("Oscillator is off, turning it on."));
    bool deviceStatus = MCP7940.deviceStart();  // Start oscillator and return state
    if (!deviceStatus) {                        // If it didn't start
      Serial.println(F("Oscillator did not start, trying again."));  // Show error and
      delay(1000);                                                   // wait for a second
    }                // of if-then oscillator didn't start
  }                  // of while the oscillator is off

  // Turn on battery backup, default is off
  MCP7940.setBattery(true); // enable battery backup mode
  // Turn on the square wave output pin of the RTC chip
  MCP7940.setSQWSpeed(3); // set SQW frequency to 32768 Hz
  MCP7940.setSQWState(true); // turn on the square wave output pin
  Serial.println("32.768kHz clock started");

  DateTime       now = MCP7940.now();  // get the current time
  sprintf(inputBuffer, "%04d-%02d-%02d %02d:%02d:%02d",
            now.year(),  // Use sprintf() to pretty print
            now.month(), now.day(), now.hour(), now.minute(),
            now.second());                         // date/time with leading zeros
  Serial.println(inputBuffer);                   // Display the current date/time


  MCP7940.calibrate(trimValue); 
  int8_t newTrimValue = MCP7940.getCalibrationTrim();
  Serial.print("Trim value: ");
  Serial.println(newTrimValue);
  
  
  Serial.println("Waiting for GPS");
  RTC_init();
  delay(150);
  // Enable interrupt on the GPS input pin
  attachInterrupt(digitalPinToInterrupt(GPSinput), GPS_ISR, RISING);
 
  // also enable global interrupts
  sei();
  while (GPSticks < 0) {
    // kill time here until the GPS ticks over
  }  
//  RTC.CNT = 0; // write all zeros to the count register. Note that it will take the processor at least
            // 2 RTC clock cycles to update this register (Datasheet section 22.13.9)
  RTCticks = 0; // reset
  GPSticks = 0; // reset
  
} // end of setup


//---------------------------------------------------------------------------------------
void loop() {
  // Most action should be in the interrupt service routines, until GPSticks gets to 10
 
  if (GPSticks >= 10){
   uint16_t rtcRawCount = RTC.CNT; // Read the current count value in the RTC counter 

    Serial.print("GPS: ");
    Serial.print(GPSticks);
    Serial.print("\t RTC ticks: ");
    Serial.print(RTCticks);
    Serial.print("\t RTC residual counts: ");
    Serial.println(rtcRawCount);

   GPSticks = 0; // Reset
   RTCticks = 0; // Reset
   rtcRawCount = 0; // Reset
   while (GPSticks < 0){
    // kill time here until GPS ticks again
   }
//   RTC.CNT = 0; // reset RTC counter
   RTCticks = 0;
   GPSticks = 0; // reset again
  } // end of if(GPSticks == 10)
  
}  // end of main loop


//-------------------------------------------------------------------
// Interrupt service routine when RTC counter overflows
ISR(RTC_CNT_vect)
  {
    RTC.INTFLAGS = RTC_OVF_bm; // Clear the interrupt by writing '1' to the overflow bit in RTC.INTFLAGS
    RTCticks++; // increment RTCticks counter
    PORTC.IN |= PIN0_bm;  // Toggle pin PC0 attached to green led
    PORTD.IN |= PIN5_bm; // Toggle pin PD5 also

  }

//-------------------------------------------------------------------------
// Interrupt service routine for counting GPS ticks
void GPS_ISR(void){
//  detachInterrupt(digitalPinToInterrupt(GPSinput));
  GPSticks++;
//  attachInterrupt(digitalPinToInterrupt(GPSinput), GPS_ISR, RISING);
}


//-------------------------------------------------------------
// Function to enable RTC, take in 32.768kHz clock signal on TOSC1 pin
void RTC_init(void){
  // Set up the Atmega's RTC to take an external clock signal
  // Start by writing a 0 to the ENABLE bit
  uint8_t temp;
  temp = CLKCTRL.XOSC32KCTRLA;
  temp &= ~CLKCTRL_ENABLE_bm; // turns the enable bit to 0
  CPU_CCP = CCP_IOREG_gc; // Write a 0xD8 to the Configuration Change Protection register to allow writing
                          // to the CLKCTRL.XOSC32KCTRLA register in the next step
  CLKCTRL.XOSC32KCTRLA = temp; // Write the new value to the register
  // Wait for status to show '0', signifying the change has taken place
  while(CLKCTRL.MCLKSTATUS & CLKCTRL_XOSC32KS_bm)
  {
    ;
  }
  // Select external oscillator by setting the SEL bit to 1 in the 
  // CLKCTRL.XOSC32KCTRLA register. Setting it to 1 specifies an 
  // external clock source on TOSC1 pin (not a crystal). Note that the
  // technical bulletin example uses 0 here for a crystal
  temp = CLKCTRL.XOSC32KCTRLA;
//  temp &= ~CLKCTRL_SEL_bm; // sets SEL bit to 0
  temp |= CLKCTRL_SEL_bm;   // Should set SEL bit to 1  (hex value of bitmask is 0x04, 0b00000100)
  CPU_CCP = CCP_IOREG_gc; // Write 0xD8 to the configuration control protection register to allow writing
                          // to CLKCTRL.XOSC32KCTRLA on the next line
  CLKCTRL.XOSC32KCTRLA = temp;
  // Then enable the oscillator
  temp = CLKCTRL.XOSC32KCTRLA;
  temp |= CLKCTRL_ENABLE_bm;  // set bitmask to set ENABLE bit to 1
  CPU_CCP = CCP_IOREG_gc; // Again turn off the configuration control protection register for the next write
                          // operation
  CLKCTRL.XOSC32KCTRLA = temp;
  // Wait while the RTC status bits update
  while (RTC.STATUS > 0)
  {
    ;
  }
  /* Set period */
  RTC.PER = RTC_EXAMPLE_PERIOD;
  // Select the clock source in the CLKSEL register 
  RTC.CLKSEL = RTC_CLKSEL_TOSC32K_gc; // 0x02 selects external clock on TOSC1
  // Set the RTC prescaler divider, and set it to run in Standby mode, and Enable the RTC
//    RTC.CTRLA = RTC_PRESCALER_DIV32_gc | RTC_RTCEN_bm | RTC_RUNSTDBY_bm; // prescale by 32
  // RTC.CTRLA = RTC_PRESCALER_DIV32768_gc | RTC_RTCEN_bm | RTC_RUNSTDBY_bm; // prescale by 32768
  RTC.CTRLA = RTC_PRESCALER_DIV1_gc | RTC_RTCEN_bm | RTC_RUNSTDBY_bm; // don't prescale 32.768kHZ signal

  // Enable overflow interrupt
  RTC.INTCTRL |= RTC_OVF_bm;
  
}
