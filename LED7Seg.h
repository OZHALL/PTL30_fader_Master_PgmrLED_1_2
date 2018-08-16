/* 
 * File:   LED7Seg.h
 * Author: Professor Beatz
 *
 * Created on July 8, 2018, 7:27 AM
 */


#ifndef LED7SEG_H
#define	LED7SEG_H

#ifdef	__cplusplus
extern "C" {
#endif

    void LED7SegLoop();
    void LED7SegSetup();
    void delay(int delaytime);
    void LED7SegBlankDisplay();
    void LED7SegDisplayValue(unsigned int iTwoDigitNumber);


#ifdef	__cplusplus
}
#endif

#endif	/* LED7SEG_H */

