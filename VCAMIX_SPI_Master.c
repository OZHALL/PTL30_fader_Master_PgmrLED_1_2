
#include "VCAMIX_SPI_Master.h"


#define _XTAL_FREQ 32000000
#include <xc.h>
//   __delay_us(10);
//   __delay_ms(10);

// these constants are to adjust the settling and clock times
const int lineSettleMicroTime = 400; // min 400
const int clockMicroTime = 200;  // min 200

void putByte(uint8_t data,int clockPin) {
  uint8_t i = 8;
  uint8_t mask;
  while(i > 0) {
    mask = 0x01 << (i - 1);      // get bitmask
    //digitalWrite( clockPin, LOW);   // tick
    DACCLK_RB6_LOW ;   // hardcode DAC clock
    //delayMicroseconds(clockMicroTime);
    __delay_us(clockMicroTime);
    if (data & mask){            // choose bit
      //digitalWrite(SPIDATA, HIGH);// send 1
      SPIDATA_RB7_HIGH ;
    }else{
      //digitalWrite(SPIDATA, LOW); // send 0
      SPIDATA_RB7_LOW ;
    }
    //delayMicroseconds(clockMicroTime);
    __delay_us(clockMicroTime);
    //digitalWrite(clockPin, HIGH);   // tock
    DACCLK_RB6_HIGH ;   // hardcode DAC clock
    //delayMicroseconds(clockMicroTime);
    __delay_us(clockMicroTime);
    --i;                         // move to lesser bit
  }
  //delayMicroseconds(lineSettleMicroTime);
//  __delay_us(lineSettleMicroTime);  
}

/* Begin MAX528 Octal 8 bit SPI DAC Support */   

void initDAC528() {
  // note: data line is shared with LED Display, so CS4 must be on to write to MAX528 also use separate clock
  //       putByte is a bit banging routine from the LED code.  reuse it.
  // set CS4 low
  //digitalWrite(CS4,LOW); 
  CS4_RA7_LOW ;
  // ADDR byte 
  putByte(0x00,0); //DACCLK);  // set buffer mode   
  // DATA byte
  putByte(0xFF,0); //DACCLK);  // set all to full buffer mode
  // set CS4 high (disable)
  //digitalWrite(CS4,HIGH); 
  CS4_RA7_HIGH ;
}

void writeDAC528(int dacNumber, uint8_t dacData) {
  uint8_t wkDacNumber=0;
  wkDacNumber=cPowerOfTwo[dacNumber];  // look up the power of 2 to set the bits
  #if defined(OZH702_DBG_DAC_WRITES)
  Serial.print("DAC data: "); 
  Serial.print(dacData);
  Serial.print("   DAC #: "); 
  Serial.print(dacNumber);
  Serial.print("   wkDacNumber: ");
  Serial.println(wkDacNumber);
  #endif
  // set CS4 low
  //digitalWrite(CS4,LOW); 
  CS4_RA7_LOW ;
  // ADDR byte 
  putByte(wkDacNumber,0); //DACCLK);  // set dacNumber
  // DATA byte
  putByte(dacData,0); //DACCLK);  // set all to full buffer mode
  // set CS4 high (disable)
  //digitalWrite(CS4,HIGH); 
  CS4_RA7_HIGH ;
}

/* End MAX528 Octal 8 bit SPI DAC Support */
