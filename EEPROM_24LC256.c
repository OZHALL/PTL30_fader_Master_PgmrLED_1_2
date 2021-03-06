/* code to support onboard 24LC256 EEPROM read/write */
#include "EEPROM_24LC256.h"
  
#ifndef DEF_I2C_EEPROM
    #define DEF_I2C_EEPROM
#endif

#ifndef DUALADSR
    #define DUALADSR
#endif
volatile extern uint8_t iI2CFaderValue[cADSRdataBytes]; // storage for sending/receiving

extern uint8_t faderActiveFlag[cMixFaderCount]; // this is controls "takeover" mode for faders
extern uint8_t mvcChannels[cMixFaderCount];     // array of output channels
extern uint8_t dualADSRdata0[cADSRdataBytes]; // local storage for one dualADSR
extern uint8_t dualADSRdata1[cADSRdataBytes]; // local storage for second dualADSRconst int scale7bitTO8bit = 2;   // multiplier to scale 7 bit value to 8 bit value (e.g. from MIDI to digitalPot)

/* this struct is created to hold a single Patch (Program) */
uint8_t aPatchMemory[cPatchSize];  // the current Patch at read/write time


void marshallPatchMemory(uint8_t mvcChannels[],uint8_t dualADSRdata0[],uint8_t dualADSRdata1[]){
    int iPatchMemoryNDX=0;
    for(int i=0; i<cMixFaderCount;i++) {
        aPatchMemory[iPatchMemoryNDX++]=mvcChannels[i];
    }
    for(int i=0; i<cADSRdataBytes;i++) {
        aPatchMemory[iPatchMemoryNDX++]=dualADSRdata0[i];
    }
    for(int i=0; i<cADSRdataBytes;i++) {
        aPatchMemory[iPatchMemoryNDX++]=dualADSRdata1[i];
    }
    // zero out the rest of it
    for(;iPatchMemoryNDX<cPatchSize;iPatchMemoryNDX++){
        aPatchMemory[iPatchMemoryNDX]=0;
    }
}

void updateModelFromPatchMemory(uint8_t mvcChannels[],uint8_t dualADSRdata0[],uint8_t dualADSRdata1[]){
    int iPatchMemoryNDX=0;
    for(int i=0; i<cMixFaderCount;i++) {
        mvcChannels[i]=aPatchMemory[iPatchMemoryNDX++];
        faderActiveFlag[i]=0;  // deactivate fader
    }
    for(int i=0; i<cADSRdataBytes;i++) {
        dualADSRdata0[i]=aPatchMemory[iPatchMemoryNDX++];
    }
    for(int i=0; i<cADSRdataBytes;i++) {
        dualADSRdata1[i]=aPatchMemory[iPatchMemoryNDX++];
    }    
}

/* BEGIN - EEPROM patch storage support */
const int MvcMinValue = 0;
const int MvcMaxValue = 1023;
const int maxPatches = 100; 
const int maxBanks = 1;     // vs 3 for banks

// logic is a bit different with a single switch which toggles up for write and down for load
/*
int WriteButtonPressedFlag = LOW; // pressed then released = 0 (i.e. commit write), released then pressed = 1; Teensy 2 analog input
int prevWriteButtonPressedFlag = LOW;
int LoadButtonPressedFlag = LOW;  // pressed then released = 0 (i.e. commit load),  released then pressed = 1; Teensy 2 analog input
int prevLoadButtonPressedFlag = LOW;

const int PgmWriteButtonInChannel = 7; // select the input pin for the Write switch
int PgmSelectValue = 0;
int PgmWriteButtonValue = 0; // variable to store the value coming from the switch

int PgmWriteButtonChannel = 7;
*/

void MYI2C_Write16EEPROMBytes(uint8_t slaveDeviceAddress,int writeAddress,uint8_t *pData) // pointer to 16 bytes of data to write
{
    uint8_t iPatchAddressHigh=writeAddress>>8;
    uint8_t iPatchAddressLow=writeAddress&0xFF;    // mask lower byte

    #define SLAVE_I2C_GENERIC_RETRY_MAX     100
    // write to 24LC256 EEPROM Device

    //uint16_t        dataAddress;
    uint16_t        nCount;
    uint8_t         writeBuffer[ciBufferSize];
    uint16_t        counter, timeOut;
    // 24LC256 - 4 bit cmd (always 1010) + 3 bit device select + 0 for Write
    uint8_t         pointerByte = 0b10100000; 

    I2C1_MESSAGE_STATUS status = I2C1_MESSAGE_PENDING;


    //dataAddress = writeAddress;     // starting EEPROM memory address 
    nCount = 2;                     // number of byte pairs to write

// write patch data to 24C256 eeprom (I2C address 80 = 0x50) at the correct address 
    for (counter = 0; counter < nCount; counter++)
    {
        // build the write buffer first
        // starting address of the EEPROM memory
        writeBuffer[0] = pointerByte;                        // pointer byte

        if(0==counter) { //initial message w/data address
  
            writeBuffer[1] = iPatchAddressHigh; // address high byte 
            writeBuffer[2] = iPatchAddressHigh; // address low byte  
        }
        else
        {
            // data to be written (start at index 1 cuz pointer byte is at 0)
            for(int i=1;i<ciBatchSize;i++){
                writeBuffer[i]=(*pData++); // dereference & increment the pointer
            }
        }  

        // Now it is possible that the slave device will be slow.
        // As a work around on these slaves, the application can
        // retry sending the transaction
        timeOut = 0;
        while(status != I2C1_MESSAGE_FAIL)
        {   
            if(0==counter) { 
                //initial message w/data address (eeprom memory location)
                I2C1_MasterWrite(  writeBuffer,
                                        3,
                                        slaveDeviceAddress,
                                        &status);               
            } else {         
                // send the actual data
                I2C1_MasterWrite(  writeBuffer,
                                        ciBatchSize+1,
                                        slaveDeviceAddress,
                                        &status);                
            }

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

        if (status == I2C1_MESSAGE_FAIL) { break; }
        //dataAddress++;
    }   
}

I2C1_MESSAGE_STATUS MYI2C_ReadPatch(uint8_t slaveDeviceAddress,int readAddress,uint8_t *pData)
{
    uint8_t iPatchAddressHigh=readAddress>>8;
    uint8_t iPatchAddressLow=readAddress&0xFF;    // mask lower byte
    //int pflag;
    // 24LC256 - 4 bit cmd (always 1010) + 3 bit device select + 1 for Read
    uint8_t pointerByte=0b10100001;  // address 0
    I2C1_MESSAGE_STATUS status;
    I2C1_TRANSACTION_REQUEST_BLOCK readTRB[2];
    uint8_t     writeBuffer[3];
    uint16_t    timeOut;
    
    #define SLAVE_I2C_GENERIC_RETRY_MAX     100
    // this initial value is important
    status = I2C1_MESSAGE_PENDING;
    
    // build the write buffer first
    // starting address of the EEPROM memory
    writeBuffer[0] = iPatchAddressHigh;              
    writeBuffer[1] = iPatchAddressLow;                    
    
    timeOut = 0;

    while(status != I2C1_MESSAGE_FAIL)
    {
        
        // we need to create the TRBs for a random read sequence to the EEPROM
        // Build TRB for sending address
        I2C1_MasterWriteTRBBuild(   &readTRB[0],
                                        &pointerByte,
                                        2,
                                        slaveDeviceAddress);
        // Build TRB for receiving data
        I2C1_MasterReadTRBBuild(    &readTRB[1],
                                       pData,
                                       cPatchSize, // bytes to read
                                       slaveDeviceAddress);
        
        // now send the transaction read TRB
        I2C1_MasterTRBInsert(2, readTRB, &status); // two TRBlocks
        
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
void readI2CEEPROM(int readAddress,int batchSize,uint8_t *pData){ // pointer to data to be read
    uint8_t iPatchAddressHigh=readAddress>>8;
    uint8_t iPatchAddressLow=readAddress&0xFF;    // mask lower byte

/*
    Wire.beginTransmission(ciI2C_EEPROM_DEVICE_ADDRESS);
    Wire.send(iPatchAddressHigh);   // address high byte  Note: the order of address is backwards compared to the PJRC example
    Wire.send(iPatchAddressLow);    // address low byte         but it matches the datasheet (and works!)
    Wire.endTransmission();

    Wire.requestFrom(ciI2C_EEPROM_DEVICE_ADDRESS, batchSize);
    #ifdef DEF_DEBUG_I2C_EEPROM
      Serial.print("Requested bytes: ");
      Serial.println(batchSize);
    #endif
*/
/*
    while(Wire.available()) {
      *pData = Wire.receive();
    #ifdef DEF_DEBUG_I2C_EEPROM
      Serial.print("Load pData: ");
      Serial.println(*pData);
    #endif
      pData++;
    }
    Wire.endTransmission();
 */
}




/* END - EEPROM patch storage support */