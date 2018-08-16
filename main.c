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
 * 
 */
#include "mcc_generated_files/mcc.h"
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
    POT =  0x4,
    FADER5 = 0x5,
    FADER6 = 0x6,
    FADER7 = 0x7,      
} adcc_channel_t;
// model for fader submodule 0 
uint8_t iLEDs0_MSB;
uint8_t iLEDs0_LSB;
// model for fader submodule 1
uint8_t iLEDs1_MSB;
uint8_t iLEDs1_LSB;


uint8_t wkFaderValue=0;
volatile uint8_t byteFaderValue[8]; // this is the Model for the fader values

void delay(int);   
 // master clock frequency adjust.  1x = 1MHz   8x = 8MHz  16=16MHz
const int cMstrClkAdjust = 32;
void delay(int delaytime) {     
    long counter = 0;
    long adjustedDelaytime=delaytime*cMstrClkAdjust;
    if (0<delaytime)
        for (counter = 0; counter<adjustedDelaytime; counter++);   
    //__delay_ms(delaytime);     // this should be the proper way to do this
 } 

void blinkyLoop (int maxLoops){
    int loopCount=maxLoops;
    int delayTime;
    
    ODCONB &= 0xE0;  // not open drain
    ODCONC &= 0x1F;  // not open drain    
    while(loopCount>0){
        delayTime=loopCount*400;    

        // brightest
        PORTB |= 0x1F; // bottom 5 bits of PORTB
        PORTC |= 0xE0; // top 3 bits of PORTC all led's on activate all

        delay(delayTime);
        // deactivate all led's
        PORTB &= 0xE0; // deactivate all led's 
        PORTC &= 0x1F; // deactivate all led's 

        delay(delayTime);
        
        loopCount--;
    }
    PORTB |= 0x1F; // bottom 5 bits of PORTB
    PORTC |= 0xE0; // top 3 bits of PORTC all led's on activate all
    delay(5000); // extra delay on the Master side - this appears to be required!
    return;
}

/* END
 copied from PLT30_fader_led_test1 
 */
I2C1_MESSAGE_STATUS MYI2C_ReadFaders(uint8_t slaveDeviceAddress);
void MYI2C_Write2LEDBytes(uint8_t slaveDeviceAddress, uint8_t MSBWriteByte,uint8_t LSBWriteByte);
void UpdateLEDsFromValue(uint8_t inFaderNum,uint8_t inValue);
/*
                         Main application
 */

void main(void)
{
    uint8_t I2C_ADDRESS_FADELED0 =  0x10;  // assume only 1 device for now
    uint16_t iCounter=0; // counter
    uint16_t iMSByte;
    uint16_t iLSByte;
    const uint16_t iResetValue=100; // at what point do we reset the change count?
    uint16_t iLoopCounter=0;  // used to reset the count 
    uint8_t  iChangeCount=0;  // count the number of times the value changes
    int faderValue;
    uint8_t fader8bitValue;
    uint8_t prevFader8bitValue;
    
    I2C1_MESSAGE_STATUS readStatus=0;
    
    // initialize the device
    SYSTEM_Initialize();

    LED7SegSetup();
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
    faderValue=ADCC_GetSingleConversion(FADER0);  // not POT
    prevFader8bitValue=faderValue>>2; // convert 10 bit to 8 bit   
    
    blinkyLoop(10);
    //Clear_WDT(); // clear watchdog timer, until i figure  out how to shut it off 
    
    LED7SegLoop(); 
    
    while (1) {
            iLoopCounter++;
            if (iResetValue<=iLoopCounter) iLoopCounter=0;
            // get value from fader
            //if((0==iCounter%4)) // only get it once every 4 loops
            {
                faderValue=ADCC_GetSingleConversion(FADER0);  // not POT
                fader8bitValue=faderValue>>2;
            }   
            // test only - show the Fader0 value from fader submodule
            faderValue=byteFaderValue[0];
            
//            // show change locally:  this will NOT be in the Teensy code
//            if (faderValue> 170) { //640){  // test only - 170
//                // brightest
//                PORTB |= 0x1F; // bottom 5 bits of PORTB
//                PORTC |= 0xE0; // top 3 bits of PORTC all led's on activate all
//                ODCONB &= 0xE0;  // not open drain
//                ODCONC &= 0x1F;  // not open drain
//            }else{
//                if (faderValue > 80) { //320) { // test only 80
//                    // Pullups are NOT active on OUTPUT pins!!!
//                    // activate pullup
//                    //WPUC = 0xFF;
//                    PORTB |= 0x1F; // bottom 5 bits of PORTB
//                    PORTC |= 0xE0; // top 3 bits of PORTC all led's on activate all
//                    ODCONB |= 0x1F; //  open drain combines with external pullup
//                    ODCONC |= 0xE0; //  open drain combines with external pullup
//                }else{
//                    // deactivate pullup
//                    //WPUC = 0x00;
//                    PORTB &= 0xE0; // deactivate all led's 
//                    PORTC &= 0x1F; // deactivate all led's 
//                    ODCONB &= 0xE0;  // not open drain
//                    ODCONC &= 0x1F;  // not open drain
//                }
//            }
            
            // get fader values
            readStatus = MYI2C_ReadFaders(I2C_ADDRESS_FADELED0);
            
            if(readStatus==I2C1_MESSAGE_COMPLETE)
            {
                // parse them & convert to LED values
                for(int ndx=0;ndx<8;ndx++){
                    UpdateLEDsFromValue(ndx,byteFaderValue[ndx]);
                }
                // now send to slave
                MYI2C_Write2LEDBytes(I2C_ADDRESS_FADELED0,iLEDs0_MSB,iLEDs0_LSB);
            }
            /*  count up routine for test only
            //if (prevFader8bitValue != fader8bitValue)
            //{
            iMSByte = iCounter;
            iMSByte =  iMSByte>>8;
            iLSByte = iCounter&0xFF;
            MYI2C_Write2LEDBytes(I2C_ADDRESS_FADELED0,iMSByte,iLSByte);
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
            LED7SegDisplayValue(iChangeCount/2.55);
            
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
    // read faders, one at a time
    int pflag;
    int dataAddress=0;  // start at fader 0
    uint8_t pointerByte=0b00100000;  // address 0
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
                                        1,
                                        slaveDeviceAddress);
        // Build TRB for receiving data
        I2C1_MasterReadTRBBuild(    &readTRB[1],
                                       byteFaderValue,
                                       8,
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
void MYI2C_Write2LEDBytes(uint8_t slaveDeviceAddress,uint8_t MSBWriteByte,uint8_t LSBWriteByte)
{
   //<code>
        #define SLAVE_I2C_GENERIC_RETRY_MAX     100
        
        // initialize the module
        //I2C1_Initialize();  done in system_initialize())
           

        // write to Fader/LED Device
        
        uint16_t        dataAddress;
        uint8_t         sourceData[16] = {  0xA0, 0xA1, 0xA2, 0xA3, 
                                            0xA4, 0xA5, 0xA6, 0xA7, 
                                            0xA8, 0xA9, 0xAA, 0xAB, 
                                            0xAC, 0xAD, 0xAE, 0xAF }; 
        uint8_t         *pData;
        uint16_t        nCount;

        uint8_t         writeBuffer[3];
        uint8_t         *pD;
        uint16_t        counter, timeOut;
        uint8_t         pointerByte = 0b00100000;    // 7 bit address + 0 for Write
        
        I2C1_MESSAGE_STATUS status = I2C1_MESSAGE_PENDING;

        dataAddress = 0x00;             // starting Fader Submodule address 
        pD = sourceData;                // initialize the source of the data
        nCount = 1;                     // number of byte pairs to write

        // signal I2C traffic
        // TODO convert RC1 back to Analog In for the slave module
        PORTC |= 0x02; // RC1 bit of PORTC led activate
        
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
            dataAddress++;
        }
        PORTC &= 0xFD; // deactivate all led's 
        //delay(1000);

    //</code>    
}
        
/**
 End of File
*/