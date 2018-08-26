/* 
 * File:   VCAMIX_SPI_Master.h
 * Author: Professor Beatz
 *
 * Created on August 19, 2018, 9:39 AM
 */

#include "mcc_generated_files/mcc.h"
#define _XTAL_FREQ 32000000

#ifndef VCAMIX_SPI_MASTER_H
#define	VCAMIX_SPI_MASTER_H
const int cPowerOfTwo[8] = {0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80};
//NOTE: these pins will have to be addressed as individual bits in PORTA/PORTB in the routines which use these constants.
#define CS4_RA7_HIGH PORTA     |= 0x80 //b'10000000'; // RA7    - /CS for SPI
#define DACCLK_RB6_HIGH PORTB  |= 0x40 //b'01000000'; // RB6    - SPI CLK to VCA/Mix board
#define SPIDATA_RB7_HIGH PORTB |= 0x80 //b'10000000'; // RB7    - SPI DAT to VCA/Mix board 
#define CS4_RA7_LOW PORTA     &= 0x7F //b'01111111';  // RA7    - /CS for SPI
#define DACCLK_RB6_LOW PORTB  &= 0xBF //b'10111111';  // RB6    - SPI CLK to VCA/Mix board
#define SPIDATA_RB7_LOW PORTB &= 0x7F //b'01111111';  // RB7    - SPI DAT to VCA/Mix board 

void putByte(uint8_t data,int clockPin);
void initDAC528();
void writeDAC528(int dacNumber, uint8_t dacData);

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* VCAMIX_SPI_MASTER_H */

