/* 
 * File:   EEPROM_24LC256.h
 * Author: Professor Beatz
 *
 * Created on August 23, 2018, 6:21 AM
 */
#include "mcc_generated_files/i2c1.h"
#include "VCAMIX_SPI_Master.h"

/* add this type for Teensy code which uses uint8_t for a 0-255 value (a byte) */
typedef unsigned char uint8_t;
typedef unsigned int  uint16_t;

#ifndef EEPROM_24LC256_H
#define	EEPROM_24LC256_H

enum {
    cPatchSize = 128,
    cMixFaderCount=4,
    cADSRdataBytes=8
} MY_CONSTS;

#ifdef	__cplusplus
extern "C" {
#endif



//void EEPROM_get();
//void EEPROM_put();
void marshallPatchMemory(uint8_t mvcChannels[],uint8_t dualADSRdata0[],uint8_t dualADSRdata1[]);
void MYI2C_Write16EEPROMBytes(uint8_t slaveDeviceAddress,int writeAddress,uint8_t *pData);

void readI2CEEPROM(int readAddress,int batchSize,uint8_t *pData);
I2C1_MESSAGE_STATUS MYI2C_ReadPatch(uint8_t slaveDeviceAddress,int readAddress,uint8_t *pData);

void savePatch(int PatchNumber);
void loadPatch(int PatchNumber);

void initModelRelatedVariables(); // to be called after loadPatch
#ifdef	__cplusplus
}
#endif

#endif	/* EEPROM_24LC256_H */

