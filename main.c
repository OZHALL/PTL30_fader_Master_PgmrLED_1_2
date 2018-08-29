/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs 

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs  - 1.45
        Device            :  PIC16F18855
        Driver Version    :  2.00
    The generated drivers are tested against the following:
        Compiler          :  XC8 1.35
        MPLAB             :  MPLAB X 3.40
*/

/*
    (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/
/*
 2017-06-29 ozh this is working:  read fader 0 & change the level of all 8 fader LEDs
 2017-07-04 ozh this is working:  Master fader controls speed.  Count goes a full 16 bits.
 2017-07-04 ozh this is working:  I2C 100KHz.  uP clockspeed 16Mhz on Master & 4 Mhz on Slave
 2017-07-09 ozh I upped the clockspeed to 32Mhz.  Had to juggle some times in the delay routine
 *              with a 32x multiplier the counts were overrunning the "int" type variables
 *              as a result, the total blinkyloop was finishing before the slave, and they did not sync
 2017-07-09 ozh upped the I2C 400KHz.  It's working (with slave @ 16 MHz). 
 2017-07-11 ozh The read fader functionality seems to be working, but something is funky with the values.
 *              the symptom is that adjacent values seem to affect each other.
 2018-07-07 ozh add LED support - drive LED # (0-9) from fader 1 position.
 2018-07-08 ozh hardware test (count 0 - 99) has been added/debugged
 *              also, reading fader 1 and displaying the 0-99 value is working.
 2018-07-18 ozh morph this into a fader accuracy test 
 2018-08-16 ozh Programmer/VCA/Mix OS (for MU single width module)
 2018-08-20 ozh Display works (but is dim)
 *              the SPI comms to the VCA/Mix board seems to work
 *              faders are being read, but there are issues
 *              1) there is some interaction between faders (config issue?)
 *              2) vca/mix channels 3 & 4 do not work
 2018-08-22 ozh Added the read for the LOAD/SAVE switch, but LEDs are now off
 *              most likely because of the change in the config
 * 
 todo: 
 *  there is a little DC bleed in the attenuators (may be a circuit issue or DAC output issue)
 *  debug VCA/Mix board issue (done - floating ground on top half of TL074)
 *  verify switch input (debounce) for load/save
 *  add model for two satellite Dual EGs
 *  add eeprom code
 *  add I2C code to read/write fader values on two satellite Dual EGs
 * 
 RA0-RA3 - Faders for "volume" level
 RA4     - pot for program # units digit 
 RA5     - pot for program # tens digit 
 RA6     - unused but available at header
 RA7     - /CS for SPI
 RB0-RB3 - Fader LED control
 RB4     - Load Toggle
 RB5     - Save Toggle
 RB6     - SPI CLK to VCA/Mix board
 RB7     - SPI DAT to VCA/Mix board
 RC0     - unused but available at header JPI2CPWR pin 7
 RC1     - unused but available at header JPI2CPWR pin 8
 RC2     - unused but available at header JPI2CPWR pin 6
 RC3     - I2CDAT - JPI2CPWR pin 4
 RC4     - I2CDAT - JPI2CPWR pin 5
 RC5     - LED_LATCH_RCK (/CS) to 74HC595 for 7-Seg LED display
 RC6     - LED_SCLOCK (/CS) to 74HC595 for 7-Seg LED display
 RC7     - LED_DATA_IN to 74HC595 for 7-Seg LED display
 RE3     - /MCLR
 * 
 */
#include "mcc_generated_files/mcc.h"
#include "VCAMIX_SPI_Master.h"
#include "LED7Seg.h"

/* BEGIN
 copied from PLT30_fader_led_test1 
 */
typedef uint16_t adc_result_t;

typedef enum
{
    FADER0 = 0x0,
    FADER1 = 0x1,
    FADER2 = 0x2,
    FADER3 = 0x3,
    FADER4 = 0x4,
    POT0 =  0x4,
    FADER5 = 0x5,
    POT1 =  0x5,
//    FADER6 = 0x6,
//    FADER7 = 0x7,      
} adcc_channel_t;

#define SET_Fader1_LED_LOW  LATB  &= 0b10011101 // RB1
#define SET_Fader0_LED_HIGH LATB  |= 0b00000001 // RB0
#define SET_Fader1_LED_HIGH LATB  |= 0b00000010 // RB1

    /* Dual ADSR module I2C messages
     * I2C slave messages:
        Pointer Byte:
       D7:D4 ? Mode Bits                D3:D0 - Address Comments
       0 0 0 0 ? Configuration Mode                     Unused at the moment
       0 0 0 1 ? Write LED mode                         Always use Address 0 with 2 data bytes
       0 0 1 0 ? Read  ADC (fader) mode 0000 to 0111    Only 0-7 address for this module
       0 0 1 1 ? Write ADC (fader) mode 0000 to 0111    Only 0-7 address for this module 8 data bytes
     */
// these work for both the internal faders as well as the dualADSR!
const uint8_t cPointerByteWriteFaders = 0b00110000; 
const uint8_t cPointerByteReadFaders  = 0b00100000; 
const uint8_t cPointerByteWriteLED    = 0b00010000; 
/* The Model All Data written to the EEPROM */

/* EEPROM header contains constants related to storage memory */
#include "EEPROM_24LC256.h"

/* Dual EG module support */
// model for fader submodule 0 
uint8_t iLEDs0_MSB;
uint8_t iLEDs0_LSB;

// model for fader submodule 1
uint8_t iLEDs1_MSB;
uint8_t iLEDs1_LSB;

int mvcPatchNumber; // 0-99 controlled by Pot0 and Pot1

// dual ADSR
const uint8_t i2c_addr_dualADSR0=0x68;
const uint8_t i2c_addr_dualADSR1=0x69;
/* end Dual EG module support */
extern uint8_t aPatchMemory[cPatchSize];  // the current Patch at read/write time


uint8_t wkFaderValue=0;
volatile uint8_t iI2CFaderValue[cADSRdataBytes]; // storage for sending/receiving
// these are for the internal faders on the mixer/programmer
uint8_t faderActiveFlag[cMixFaderCount]; // this is controls "takeover" mode for faders
//uint8_t byteMixFaderValue[cMixFaderCount]; // this is the Model for the Panel fader values
//uint8_t prevByteMixFaderValue[cMixFaderCount]; // these are the previous fader values

//MIX_CHANNEL MidiInChannels[cOutChannelCount];  // array of 8 output channels from MIDI
uint8_t PanelInChannels[cMixFaderCount]; // array of output channels from Panel
uint8_t prevPanelInChannels[cMixFaderCount]; // array of output channels from Panel
uint8_t mvcChannels[cMixFaderCount];     // array of output channels
uint8_t dualADSRdata0[cADSRdataBytes]; // local storage for one dualADSR
uint8_t dualADSRdata1[cADSRdataBytes]; // local storage for second dualADSRconst int scale7bitTO8bit = 2;   // multiplier to scale 7 bit value to 8 bit value (e.g. from MIDI to digitalPot)


void InitPanelInChannels() {       // initialize the array
  for(int i=0; i<cMixFaderCount;i++) {
    PanelInChannels[i]=0;
    faderActiveFlag[i]=0; // active at first
  }
}
void InitmvcChannels() {       // initialize the MIDI array
  for(int i=0; i<cMixFaderCount;i++) {
    mvcChannels[i]=0;
  }
}


/* End - The Model All Data written to the EEPROM */

void delay(int);   
 // master clock frequency adjust.  1x = 1MHz   8x = 8MHz  16=16MHz
const int cMstrClkAdjust = 32;
void delay(int delaytime) {    
    long counter = 0;
    long adjustedDelaytime=delaytime*cMstrClkAdjust;

    if (0<delaytime)
        for (counter = 0; counter<adjustedDelaytime; counter++);  
 //   __delay_ms(adjustedDelaytime);     // this should be the proper way to do this
 //     __delay_us(delaytime);
 } 


/* END
 copied from PLT30_fader_led_test1 
 */
I2C1_MESSAGE_STATUS MYI2C_ReadFaders(uint8_t slaveDeviceAddress);
void MYI2C_WriteFaders(uint8_t slaveDeviceAddress);
void MYI2C_Write2LEDBytes(uint8_t slaveDeviceAddress, uint8_t MSBWriteByte,uint8_t LSBWriteByte);
void UpdateLEDsFromValue(uint8_t inFaderNum,uint8_t inValue);

void savePatch(int PatchNumber);
void loadPatch(int PatchNumber);


void blinkyLoop (int maxLoops){
    int loopCount=maxLoops;
    int delayTime;
    
    ODCONB &= 0xF0;  // not open drain bottom 4 bits
    ODCONC &= 0x1F;  // not open drain top 3 bits

    while(loopCount>0){
        delayTime=loopCount*400;    

        // brightest
        LATB |= 0x0F; // bottom 4 bits of PORTB
       // PORTC |= 0xE0; // top 3 bits of PORTC all led's on activate all
        MYI2C_Write2LEDBytes(i2c_addr_dualADSR0,0xFF,0xFF); // all on bright
        delay(delayTime);
        // deactivate all led's
        LATB &= 0xF0; // deactivate all led's 
        //PORTC &= 0x1F; // deactivate all led's 
        MYI2C_Write2LEDBytes(i2c_addr_dualADSR0,0x0,0x0); // all off
        delay(delayTime);
        
        loopCount--;
    }
    LATB |= 0x0F; // bottom 4 bits of PORTB
    //LATC |= 0xE0; // top 3 bits of PORTC all led's on activate all
    delay(5000); // extra delay on the Master side - this appears to be required!
    MYI2C_Write2LEDBytes(i2c_addr_dualADSR0,0xFF,0xFF); // all on bright
    return;
}
/*
                         Main application
 */

void main(void)
{
    uint16_t iCounter=0; // counter
    uint16_t iMSByte;
    uint16_t iLSByte;
    const uint16_t iResetValue=100; // at what point do we reset the change count?
    uint16_t iLoopCounter=0;  // used to reset the count 
    uint8_t  iChangeCount=0;  // count the number of times the value changes
    int faderValue;
    uint8_t fader8bitValue;
    uint8_t prevFader8bitValue;
    uint8_t POT0Value;
    uint8_t POT1Value;
    uint8_t DACDataValue;
    double  dFader8bitValue;
    int LoadDebounceCount=0;
    int SaveDebounceCount=0;
    
    I2C1_MESSAGE_STATUS readStatus=0;
    
    // initialize the device
    SYSTEM_Initialize();
    initDAC528();
    LED7SegSetup();
    
    InitPanelInChannels();
    InitmvcChannels();
    // When using interrupts, you need to set the Global and Peripheral Interrupt Enable bits
    // Use the following macros to:

    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();
    
    // get value from fader
#define INIT_FROM_FADERS
#ifdef INIT_FROM_FADERS
    for(int fx=0;fx<4;fx++){
        faderValue=ADCC_GetSingleConversion(fx);  // not POT
        PanelInChannels[fx]=255-(faderValue>>2);   // invert and convert 10 bit to 8 bit 
        prevPanelInChannels[fx]= PanelInChannels[fx];
    }
#endif
    blinkyLoop(10);
    //Clear_WDT(); // clear watchdog timer, until i figure  out how to shut it off 
    
   //LED7SegLoop(); 
    // all 4 LEDs on
    PORTB |= 0x0F; // bottom 4 bits of PORTB
    PORTC |= 0xE0; // top 3 bits of PORTC all led's on activate all
    ODCONB &= 0xF0;  // not open drain
    ODCONC &= 0x1F;  // not open drain  
    
    while (1) {
            iLoopCounter++;
            if (iResetValue<=iLoopCounter) iLoopCounter=0;
            // get value from fader
            //if((0==iCounter%4)) // only get it once every 4 loops
            {
                /* service faders */
                for(int fx=0;fx<cMixFaderCount;fx++){
                    faderValue=ADCC_GetSingleConversion(fx);  // 
                    PanelInChannels[fx]=255-(faderValue>>2);   // invert and convert 10 bit to 8 bit 
                    //fader8bitValue=(PanelInChannels[fx]+prevPanelInChannels[fx])/2;  // smooth it for DAC
                    fader8bitValue=PanelInChannels[fx];
                    if( (PanelInChannels[fx]<(mvcChannels[fx]+4))
                    &&  (PanelInChannels[fx]>(mvcChannels[fx]-4)) ) {
                            faderActiveFlag[fx]=1; // fader is w/in range, activate it
                            SET_Fader1_LED_HIGH ;
                    }
                    if(faderActiveFlag[fx]>0){
                        mvcChannels[fx]=PanelInChannels[fx]; // update model
                        writeDAC528(fx+4,fader8bitValue); // +4 because the VCA/Mix is connected to DAC channels 5-8
                    }                     
                    prevPanelInChannels[fx]=PanelInChannels[fx];
                    //__delay_ms(10);
                    delay(10);
                }
                
                /* service pots */
                delay(10);              
                faderValue=ADCC_GetSingleConversion(POT0);  
                faderValue=1023-faderValue;  // must invert the value for prototype - pots wired backwards
                //fader8bitValue=faderValue>>2;
                POT0Value=faderValue/102.3;   // change to 0-9
                delay(10);
                faderValue=ADCC_GetSingleConversion(POT1);  
                faderValue=1023-faderValue;  // must invert the value
                POT1Value=faderValue/102.3;   // change to 0-9
            } 
            
                        
            // hardware test VCA/MIX board (int dacNumber, uint8_t dacData)
            /* this code works
            DACDataValue=4* (iCounter%32);
            writeDAC528(4,DACDataValue); // this should ramp up Chnl 4 (0) volumee at some rate
            writeDAC528(5,DACDataValue); // this should ramp up Chnl 4 (0) volumee at some rate
            writeDAC528(6,DACDataValue); // this should ramp up Chnl 4 (0) volumee at some rate
            writeDAC528(7,DACDataValue); // this should ramp up Chnl 4 (0) volumee at some rate
            */ 
            // test only - show the POT0 value from fader submodule
            //faderValue=byteFaderValue[POT0];
            
//            // show change locally:  this will NOT be in the Teensy code
//            if (faderValue> 170) { //640){  // test only - 170
//                // brightest
//                PORTB |= 0x0F; // bottom 4 bits of PORTB
//                PORTC |= 0xE0; // top 3 bits of PORTC all led's on activate all
//                ODCONB &= 0xF0;  // not open drain
//                ODCONC &= 0x1F;  // not open drain
//            }else{
//                if (faderValue > 80) { //320) { // test only 80
//                    // Pullups are NOT active on OUTPUT pins!!!
//                    // activate pullup
//                    //WPUC = 0xFF;
//                    PORTB |= 0x0F; // bottom 4 bits of PORTB
//                    PORTC |= 0xE0; // top 3 bits of PORTC all led's on activate all
//                    ODCONB |= 0x0F; //  open drain combines with external pullup
//                    ODCONC |= 0xE0; //  open drain combines with external pullup
//                }else{
//                    // deactivate pullup
//                    //WPUC = 0x00;
//                    PORTB &= 0xF0; // deactivate all led's 
//                    PORTC &= 0x1F; // deactivate all led's 
//                    ODCONB &= 0xF0;  // not open drain
//                    ODCONC &= 0x1F;  // not open drain
//                }
//            }
            
            // get fader values
        /* this is all slave code
            readStatus = MYI2C_ReadFaders(i2c_addr_dualADSR0);
            
            if(readStatus==I2C1_MESSAGE_COMPLETE)
            {
                // parse them & convert to LED values
                for(int ndx=0;ndx<8;ndx++){
                    UpdateLEDsFromValue(ndx,byteFaderValue[ndx]);
                }
                // now send to slave
                MYI2C_Write2LEDBytes(i2c_addr_dualADSR0,iLEDs0_MSB,iLEDs0_LSB);
            }
        */
            /*  count up routine for test only
            //if (prevFader8bitValue != fader8bitValue)
            //{
            iMSByte = iCounter;
            iMSByte =  iMSByte>>8;
            iLSByte = iCounter&0xFF;
            MYI2C_Write2LEDBytes(i2c_addr_dualADSR0,iMSByte,iLSByte);
            //}
            */
            if (prevFader8bitValue!=fader8bitValue) {
                iChangeCount++;
                //if (iResetValue<iChangeCount) iChangeCount=0;
            }
            prevFader8bitValue=fader8bitValue;
            //delay(); 
            //Clear_WDT(); // clear watchdog timer, until i figure  out how to shut it off
            iCounter++;    // 16 bits auto rollover
            if (fader8bitValue<252)
                delay(1*(255-fader8bitValue)); // invert so that higher is faster
            
            
            //LED7SegDisplayValue(fader8bitValue/2.55);
            //LED7SegDisplayValue(iChangeCount/2.55);
            //LED7SegDisplayValue(fader8bitValue/2.55);
            mvcPatchNumber=(POT1Value*10)+POT0Value;
            LED7SegDisplayValueByDigit(POT1Value,POT0Value);  
          
            // check switches (normally pulled up)
// If you're doing a read-modify-write (for example a BSF), you generally want to use the LATCH.

            if (PORTBbits.RB4 == 0){
                // Load
                LoadDebounceCount++;
                if(5<LoadDebounceCount) { //do the load 
                    SET_Fader0_LED_LOW ;
                    // do I2C EEProm write
                    loadPatch(mvcPatchNumber);  // see EEPROM_24LC256
                    
                    LoadDebounceCount=1; // reset to 1 so that the LED will toggle back
                    __delay_ms(500);
                }
            }else{
                if(0<LoadDebounceCount){
                    LoadDebounceCount=0;
                    SET_Fader0_LED_HIGH ;
                }
            }
            if (PORTBbits.RB5 == 0){
                // Save
                SaveDebounceCount++;
                if(5<SaveDebounceCount) { //do the save 
                    SET_Fader1_LED_LOW ;
                    // get all system data from the ElectricDruid I2C network (e.g. DUALADSR)
#ifdef DO_ADSR_I2C
                    readStatus = MYI2C_ReadFaders(i2c_addr_dualADSR0);
                    if(readStatus==I2C1_MESSAGE_COMPLETE)
                    {
                        for(int x=0;x<cADSRdataBytes;x++) {
                            // copy from iI2CFaderValue array to mvc location
                            dualADSRdata0[x]=iI2CFaderValue[x];
                        }
                        readStatus = MYI2C_ReadFaders(i2c_addr_dualADSR1);
                        if(readStatus==I2C1_MESSAGE_COMPLETE)
                        {
                            for(int x=0;x<cADSRdataBytes;x++) {
                                // copy from iI2CFaderValue array to mvc location
                                dualADSRdata0[x]=iI2CFaderValue[x];
                            }
#endif                           
                            // do I2C EEProm write
                            savePatch(mvcPatchNumber);  // see EEPROM_24LC256
#ifdef DO_ADSR_I2C
                        }
                    }
#endif  
                    SaveDebounceCount=1; // reset to 1 so that the LED will toggle back
                    __delay_ms(500);
                }
            }else{
                if(0<SaveDebounceCount){
                    SaveDebounceCount=0;
                    SET_Fader1_LED_HIGH;
                }
            }
    } 
 }
void UpdateLEDsFromValue(uint8_t inFaderNum,uint8_t inValue)
{
    bool    wkOnFlag = 0;
    bool    wkBrightFlag = 0;
    
    // parse value
    if (inValue > 80)  wkOnFlag=1;
    if (inValue > 170) wkBrightFlag=1;
             
    switch(inFaderNum)
    {
        case 0 : {
            if(wkOnFlag) {  iLEDs0_LSB   |= 0b00000010;
                if(wkBrightFlag) {iLEDs0_LSB  |= 0b00000001;}else{iLEDs0_LSB &= 0b11111110;};
            }else{          iLEDs0_LSB   &= 0b11111100;}
            break;
        }
        case 1 : {
            if(wkOnFlag) {  iLEDs0_LSB   |= 0b00001000;
                if(wkBrightFlag) {iLEDs0_LSB  |= 0b00000100;}else{iLEDs0_LSB &= 0b11111011;};
            }else{          iLEDs0_LSB   &= 0b11110011;}
            break;
        }
        case 2 : {
            if(wkOnFlag) {  iLEDs0_LSB   |= 0b00100000;
                if(wkBrightFlag) {iLEDs0_LSB  |= 0b00010000;}else{iLEDs0_LSB &= 0b11101111;};
            }else{          iLEDs0_LSB   &= 0b11001111;}
            break;
        }
        case 3 : {
            if(wkOnFlag) {  iLEDs0_LSB   |= 0b10000000;
                if(wkBrightFlag) {iLEDs0_LSB  |= 0b01000000;}else{iLEDs0_LSB &= 0b10111111;};
            }else{          iLEDs0_LSB   &= 0b00111111;}
            break;
        }
        case 4 : {
            if(wkOnFlag) {  iLEDs0_MSB   |= 0b00000010;
                if(wkBrightFlag) {iLEDs0_MSB  |= 0b00000001;}else{iLEDs0_MSB &= 0b11111110;};
            }else{          iLEDs0_MSB   &= 0b11111100;}
            break;
        }
        case 5 : {
            if(wkOnFlag) {  iLEDs0_MSB   |= 0b00001000;
                if(wkBrightFlag) {iLEDs0_MSB  |= 0b00000100;}else{iLEDs0_MSB &= 0b11111011;};
            }else{          iLEDs0_MSB   &= 0b11110011;}
            break;
        }
        case 6 : {
            if(wkOnFlag) {  iLEDs0_MSB   |= 0b00100000;
                if(wkBrightFlag) {iLEDs0_MSB  |= 0b00010000;}else{iLEDs0_MSB &= 0b11101111;};
            }else{          iLEDs0_MSB   &= 0b11001111;}
            break;
        }
        case 7 : {
            if(wkOnFlag) {  iLEDs0_MSB   |= 0b10000000;
                if(wkBrightFlag) {iLEDs0_MSB  |= 0b01000000;}else{iLEDs0_MSB &= 0b10111111;};
            }else{          iLEDs0_MSB   &= 0b00111111;}
        }
    }
}
I2C1_MESSAGE_STATUS MYI2C_ReadFaders(uint8_t slaveDeviceAddress)
{
    // read faders on a dual EG or other ZebraSynth module that has two sets of 4 faders

    int pflag;
    int dataAddress=0;  // start at fader 0
    uint8_t pointerByte=cPointerByteReadFaders;  // address 0
    I2C1_MESSAGE_STATUS status;
    I2C1_TRANSACTION_REQUEST_BLOCK readTRB[2];
    uint8_t     writeBuffer[3];
    uint16_t    timeOut;
    
    #define SLAVE_I2C_GENERIC_RETRY_MAX     100
    // this initial value is important
    status = I2C1_MESSAGE_PENDING;
    
    // build the write buffer first
    // starting address of the EEPROM memory
    /* address is in the pointerByte (and always zero)
    writeBuffer[0] = (dataAddress >> 8);                        // high address
    writeBuffer[1] = (uint8_t)(dataAddress);                    // low low address
    */
    timeOut = 0;

    while(status != I2C1_MESSAGE_FAIL)
    {
        
        // we need to create the TRBs for a random read sequence to the EEPROM
        // Build TRB for sending address
        I2C1_MasterWriteTRBBuild(   &readTRB[0],
                                        &pointerByte,
                                        1, // address to be read
                                        slaveDeviceAddress);
        // Build TRB for receiving data
        I2C1_MasterReadTRBBuild(    &readTRB[1],
                                       iI2CFaderValue, // pointer to where to write
                                       8, // should match cADSRdataBytes
                                       slaveDeviceAddress);
        
        // now send the transaction read TRB
        I2C1_MasterTRBInsert(2, readTRB, &status);
        
        // wait for the message to be sent or status has changed.
        while(status == I2C1_MESSAGE_PENDING);

        if (status == I2C1_MESSAGE_COMPLETE)
            break;

        // if status is  I2C1_MESSAGE_ADDRESS_NO_ACK,
        //               or I2C1_DATA_NO_ACK,
        // The device may be busy and needs more time for the last
        // write so we can retry writing the data, this is why we
        // use a while loop here

        // check for max retry and skip this byte
        if (timeOut == SLAVE_I2C_GENERIC_RETRY_MAX)
            break;
        else
            timeOut++;

    }

    if (I2C1_MESSAGE_FAIL==status) { 
        //"Houston we have a problem!!!";
    }

    return status;
}
/* sample code for emulated EEPROM read
        <code>

  
            void EMULATED_EEPROM_Read(
                                           uint16_t slaveDeviceAddress,
                                           uint16_t dataAddress,
                                           uint8_t *pData,
                                           uint16_t nCount)
            {
                I2C1_MESSAGE_STATUS status;
                I2C1_TRANSACTION_REQUEST_BLOCK readTRB[2];
                uint8_t     writeBuffer[3];
                uint16_t    timeOut;

                // this initial value is important
                status = I2C1_MESSAGE_PENDING;

                // build the write buffer first
                // starting address of the EEPROM memory
                writeBuffer[0] = (dataAddress >> 8);                        // high address
                writeBuffer[1] = (uint8_t)(dataAddress);                    // low low address

                // we need to create the TRBs for a random read sequence to the EEPROM
                // Build TRB for sending address
                I2C1_MasterWriteTRBBuild(   &readTRB[0],
                                                writeBuffer,
                                                2,
                                                slaveDeviceAddress);
                // Build TRB for receiving data
                I2C1_MasterReadTRBBuild(    &readTRB[1],
                                                pData,
                                                nCount,
                                                slaveDeviceAddress);

                timeOut = 0;

                while(status != I2C1_MESSAGE_FAIL)
                {
                    // now send the transactions
                    I2C1_MasterTRBInsert(2, readTRB, &status);

                    // wait for the message to be sent or status has changed.
                    while(status == I2C1_MESSAGE_PENDING);

                    if (status == I2C1_MESSAGE_COMPLETE)
                        break;

                    // if status is  I2C1_MESSAGE_ADDRESS_NO_ACK,
                    //               or I2C1_DATA_NO_ACK,
                    // The device may be busy and needs more time for the last
                    // write so we can retry writing the data, this is why we
                    // use a while loop here

                    // check for max retry and skip this byte
                    if (timeOut == SLAVE_I2C_GENERIC_RETRY_MAX)
                        break;
                    else
                        timeOut++;

                }

            }   
  
        </code>
*/
void MYI2C_WriteFaders(uint8_t slaveDeviceAddress)
{
        #define SLAVE_I2C_GENERIC_RETRY_MAX     100
           
        // write to Fader Device
        
        //uint16_t        dataAddress;
        uint8_t         *pData;
        uint16_t        nCount;

        uint8_t         writeBuffer[9];
        uint8_t         *pD;
        uint16_t        counter, timeOut;
        uint8_t         pointerByte = cPointerByteWriteFaders;    

        I2C1_MESSAGE_STATUS status = I2C1_MESSAGE_PENDING;

        //dataAddress = 0x00;             // starting Fader Submodule address 
        //pD = sourceData;                // initialize the source of the data
        nCount = 1;                     // number of byte pairs to write
        
        for (counter = 0; counter < nCount; counter++)
        {
            // build the write buffer first
            // starting address of the EEPROM memory
            writeBuffer[0] = pointerByte;                        // pointer byte
            // data to be written ( use iI2CFaderValue array)
            for(int x=1;x<cADSRdataBytes;x++){
                writeBuffer[x+1] = iI2CFaderValue[x];
            }

            // Now it is possible that the slave device will be slow.
            // As a work around on these slaves, the application can
            // retry sending the transaction
            timeOut = 0;
            while(status != I2C1_MESSAGE_FAIL)
            {
                // write one byte to EEPROM (3 is the number of bytes to write)
                I2C1_MasterWrite(  writeBuffer,
                                        cADSRdataBytes+1,
                                        slaveDeviceAddress,
                                        &status);

                // wait for the message to be sent or status has changed.
                while(status == I2C1_MESSAGE_PENDING);

                if (status == I2C1_MESSAGE_COMPLETE)
                    break;

                // if status is  I2C1_MESSAGE_ADDRESS_NO_ACK,
                //               or I2C1_DATA_NO_ACK,
                // The device may be busy and needs more time for the last
                // write so we can retry writing the data, this is why we
                // use a while loop here

                // check for max retry and skip this byte
                if (timeOut == SLAVE_I2C_GENERIC_RETRY_MAX)
                    break;
                else
                    timeOut++;
            }

            if (status == I2C1_MESSAGE_FAIL)
            {
                break;
            }
            //dataAddress++;
        }
}
void MYI2C_Write2LEDBytes(uint8_t slaveDeviceAddress,uint8_t MSBWriteByte,uint8_t LSBWriteByte)
{
        // write to Fader/LED Device  
        //uint16_t        dataAddress;
        uint8_t         *pData;
        uint16_t        nCount;

        uint8_t         writeBuffer[3];
        //uint8_t         *pD;
        uint16_t        counter, timeOut;
        uint8_t         pointerByte = cPointerByteWriteLED; 
        
        I2C1_MESSAGE_STATUS status = I2C1_MESSAGE_PENDING;

        //dataAddress = 0x00;             // starting Fader Submodule address 
        //pD = sourceData;                // initialize the source of the data
        nCount = 1;                     // number of byte pairs to write
        
        for (counter = 0; counter < nCount; counter++)
        {
            // build the write buffer first
            // starting address of the EEPROM memory
            writeBuffer[0] = pointerByte;                        // pointer byte
            // data to be written
            writeBuffer[1] = (MSBWriteByte);            // high address
            writeBuffer[2] = (LSBWriteByte);            // low address

            // Now it is possible that the slave device will be slow.
            // As a work around on these slaves, the application can
            // retry sending the transaction
            timeOut = 0;
            while(status != I2C1_MESSAGE_FAIL)
            {
                // write one byte to EEPROM (3 is the number of bytes to write)
                I2C1_MasterWrite(  writeBuffer,
                                        3,
                                        slaveDeviceAddress,
                                        &status);

                // wait for the message to be sent or status has changed.
                while(status == I2C1_MESSAGE_PENDING);

                if (status == I2C1_MESSAGE_COMPLETE)
                    break;

                // if status is  I2C1_MESSAGE_ADDRESS_NO_ACK,
                //               or I2C1_DATA_NO_ACK,
                // The device may be busy and needs more time for the last
                // write so we can retry writing the data, this is why we
                // use a while loop here

                // check for max retry and skip this byte
                if (timeOut == SLAVE_I2C_GENERIC_RETRY_MAX)
                    break;
                else
                    timeOut++;
            }

            if (status == I2C1_MESSAGE_FAIL)
            {
                break;
            }
         //   dataAddress++;
        }  
}

void savePatch(int PatchNumber){ // patch number = 0 to 99  (vs 0 to 11 for banks)
  int iPatchAddress=PatchNumber*cPatchSize;  // calc starting addr for patch
  uint8_t iPatchAddressHigh=iPatchAddress>>8;
  uint8_t iPatchAddressLow=iPatchAddress&0xFF;    // mask lower byte

    // marshall the data into aPatchMemory
    marshallPatchMemory(mvcChannels,dualADSRdata0,dualADSRdata1); 

    // write "cPatchSize" bytes, from address "iPatchAddress" in batches
    uint8_t *pData;
    int idx;
    //pData=&mvcChannels[0].scInChannel; //point to first byte in first struct
    
    for(int batchNum=0;(batchNum*ciBatchSize)<cPatchSize;batchNum++){
        // pointer to 16 bytes of data to write
        idx=batchNum*ciBatchSize;
        pData=&aPatchMemory[idx];
        MYI2C_Write16EEPROMBytes(ciI2C_EEPROM_DEVICE_ADDRESS,iPatchAddress,pData); 
        // update local variables for the next batch
        iPatchAddress+=ciBatchSize;
        /* NOTE: pData gets incremented w/in the write routine! */
        __delay_ms(5);  // 5 ms write time for 24LC256!!!
    }

    // this will happen in the service routine for the timer
    //updateLEDfromModel(cMode1to8,true); // model,redisplay
    //updateLEDfromModel(cMode9to16,true);  
}

// send updated model data out to all supported submodules
void writeI2CSystemData(){
    MYI2C_WriteFaders(i2c_addr_dualADSR0); // dual EG module 0  
    MYI2C_WriteFaders(i2c_addr_dualADSR1); // dual EG module 0 
}
// declare this here for use by loadPatch
uint8_t wkMIDICC=0;
uint8_t wkByte=0;
float wkFloat=0;  // used to multiply
  
void loadPatch(int PatchNumber){ // patch number = 0 to 35  (vs 0 to 11 for banks)
    int iPatchAddress=PatchNumber*cPatchSize;  // calc starting addr for patch
    
    // read "cPatchSize" bytes, from address "iPatchAddress"
    uint8_t *pData;
    int idx;
    //pData=&mvcChannels[0].scInChannel; //point to first byte in the first struct
    
  #ifdef DEF_I2C_EEPROM
    pData=&aPatchMemory; //point to first byte in the first struct

    MYI2C_ReadPatch(ciI2C_EEPROM_DEVICE_ADDRESS,iPatchAddress,pData);
    
  #endif

    // I know the program values did not come from MIDI, but imagine they did - :)
    // MIDI is the "priority channel"

    updateModelFromPatchMemory(mvcChannels,dualADSRdata0,dualADSRdata1);

    // update DAC and Matrix Switch from model
    for(int i=0;i<cMixFaderCount;i++){
       writeDAC528(i+4,mvcChannels[i]);
     }

  #ifdef DUALADSR 
     writeI2CSystemData();
  #endif      
}
void initModelRelatedVariables(){
}   
/**
 End of File
*/