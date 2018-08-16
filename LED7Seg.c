/* driver code for a pair of 74HC595s driving two 7 segment LED displays */


/* 
 * based on:
 * https://www.arduino.cc/en/tutorial/ShiftOut 
 * http://processors.wiki.ti.com/index.php/MSP430_Launchpad_Shift_Register
 */

/*
Using 2 7-segment displays with the 74HC595 shift registers
CC by-sa-nc 3.0
http://tronixstuff.wordpress.com
 */
#include "mcc_generated_files/mcc.h"
#include "LED7Seg.h"

/* in PIC world, these pins are irrelevant */
const int latchpin = 16; // connect to pin 12 on the '595
const int clockpin = 17; // connect to pin 11 on the '595
const int datapin = 18; // connect to pin 14 on the '595
float b = 0;
unsigned int c = 0;
float d = 0;
int e = 0;
const unsigned int speed = 50; // used to control speed of 
const unsigned int twoK = 10000;
const unsigned int toggleDly=10;
int segdisp[10] = {
 3,159,37,13,153,73,65,27,1,9 };
void LED7SegSetup()
{
 //pinMode(latchpin, OUTPUT);
 IO_RC5_LED_LATCH_RCK_SetDigitalMode();
 IO_RC5_LED_LATCH_RCK_SetDigitalOutput();
 
 //pinMode(clockpin, OUTPUT);
 IO_RC6_LED_SCLOCK_SetDigitalMode();
 IO_RC6_LED_SCLOCK_SetDigitalOutput();
 
 //pinMode(datapin, OUTPUT);
 IO_RC7_LED_DATA_SetDigitalMode();
 IO_RC7_LED_DATA_SetDigitalOutput();
}
// Writes a value to the specified bitmask/pin. Use built in defines
// when calling this, as the shiftOut() function does.
// All nonzero values are treated as "high" and zero is "low"
void LED7SegPinWrite(unsigned char val )
{
  if (val){
    //P1OUT |= bit;
    IO_RC7_LED_DATA_SetHigh();
  } else {
    //P1OUT &= ~bit;
    IO_RC7_LED_DATA_SetLow();
  }
}
 
// Pulse the clock pin
void LED7SegPulseClock( void )
{
  //P1OUT |= CLOCK;
  IO_RC6_LED_SCLOCK_SetHigh();
  delay(1);
  //P1OUT ^= CLOCK;
  IO_RC6_LED_SCLOCK_SetLow();
}
// Take the given 8-bit value and shift it out, LSB to MSB
void LED7SegShiftOutPulseClock(unsigned char val)
{
  char i;
 
  // Iterate over each bit, set data pin, and pulse the clock to send it
  // to the shift register
  for (i = 0; i < 8; i++)  {
      LED7SegPinWrite((val & (1 << i)));
      LED7SegPulseClock();
  }
}
void LED7SegDisplayValue(unsigned int iTwoDigitNumber){
    unsigned int iTensDigit;     // 0 - 9
    unsigned int iUnitsDigit;    // 0 - 9
    unsigned int iTensCode;      // the code for the tens digit, so that we can blank it
    
    if(0>iTwoDigitNumber)  iTwoDigitNumber=0;   // guard the range
    if(99<iTwoDigitNumber) iTwoDigitNumber=99;  //
    
    d=iTwoDigitNumber%10; // find the remainder of dividing z by 10, this will be the right-hand digit
    iUnitsDigit=d;        // the right hand digit, make it an integer
    b=iTwoDigitNumber/10; // divide z by 10 - the whole number value will be the left-hand digit
    iTensDigit=b;         // the left hand digit
    iTensCode=segdisp[iTensDigit];
    if(0==iTensDigit)     iTensCode=255;         // blank leading zeroes

    LED7SegShiftOutPulseClock( segdisp[iUnitsDigit]); // sends the digit down the serial path
    LED7SegShiftOutPulseClock( iTensCode ); // sends a blank down the serial path to push the digit to the right  
    // Pulse the latch pin to write the values into the storage register
    IO_RC5_LED_LATCH_RCK_SetHigh();
    IO_RC5_LED_LATCH_RCK_SetLow();
    
}
void LED7SegBlankDisplay() {
 //IO_RC5_LED_LATCH_RCK_SetLow() ;
 LED7SegShiftOutPulseClock(255); // clears the right display
 LED7SegShiftOutPulseClock(255); // clears the left display
 // Pulse the latch pin to write the values into the storage register
 IO_RC5_LED_LATCH_RCK_SetHigh();
 //delay(1);
 IO_RC5_LED_LATCH_RCK_SetLow(); 
}
void LED7SegLoop()
{
 LED7SegBlankDisplay();  

 // wait a second
 for(int d=0;d<200;d++){
    delay(speed);
 }

 //  Count up
 for (int z=0; z<100; z++)
 {
   LED7SegDisplayValue(z);
   for(int d=0;d<25;d++){
        delay(speed);
    }
 }

 // wait a second
 for(int d=0;d<200;d++){
    delay(speed);
 }
 
  LED7SegBlankDisplay(); 

}
