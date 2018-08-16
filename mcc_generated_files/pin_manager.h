/**
  @Generated Pin Manager Header File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.h

  @Summary:
    This is the Pin Manager file generated using MPLAB(c) Code Configurator

  @Description:
    This header file provides implementations for pin APIs for all pins selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB(c) Code Configurator - 4.26.2
        Device            :  PIC16F18855
        Version           :  1.01
    The generated drivers are tested against the following:
        Compiler          :  XC8 1.35
        MPLAB             :  MPLAB X 3.40

    Copyright (c) 2013 - 2015 released Microchip Technology Inc.  All rights reserved.

    Microchip licenses to you the right to use, modify, copy and distribute
    Software only when embedded on a Microchip microcontroller or digital signal
    controller that is integrated into your product or third party product
    (pursuant to the sublicense terms in the accompanying license agreement).

    You should refer to the license agreement accompanying this Software for
    additional information regarding your rights and obligations.

    SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
    EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
    MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
    IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
    CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
    OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
    INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
    CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
    SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
    (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

*/


#ifndef PIN_MANAGER_H
#define PIN_MANAGER_H

#define INPUT   1
#define OUTPUT  0

#define HIGH    1
#define LOW     0

#define ANALOG      1
#define DIGITAL     0

#define PULL_UP_ENABLED      1
#define PULL_UP_DISABLED     0

// get/set channel_ANA0 aliases
#define channel_ANA0_TRIS               TRISAbits.TRISA0
#define channel_ANA0_LAT                LATAbits.LATA0
#define channel_ANA0_PORT               PORTAbits.RA0
#define channel_ANA0_WPU                WPUAbits.WPUA0
#define channel_ANA0_OD                ODCONAbits.ODCA0
#define channel_ANA0_ANS                ANSELAbits.ANSA0
#define channel_ANA0_SetHigh()            do { LATAbits.LATA0 = 1; } while(0)
#define channel_ANA0_SetLow()             do { LATAbits.LATA0 = 0; } while(0)
#define channel_ANA0_Toggle()             do { LATAbits.LATA0 = ~LATAbits.LATA0; } while(0)
#define channel_ANA0_GetValue()           PORTAbits.RA0
#define channel_ANA0_SetDigitalInput()    do { TRISAbits.TRISA0 = 1; } while(0)
#define channel_ANA0_SetDigitalOutput()   do { TRISAbits.TRISA0 = 0; } while(0)
#define channel_ANA0_SetPullup()      do { WPUAbits.WPUA0 = 1; } while(0)
#define channel_ANA0_ResetPullup()    do { WPUAbits.WPUA0 = 0; } while(0)
#define channel_ANA0_SetPushPull()    do { ODCONAbits.ODCA0 = 1; } while(0)
#define channel_ANA0_SetOpenDrain()   do { ODCONAbits.ODCA0 = 0; } while(0)
#define channel_ANA0_SetAnalogMode()  do { ANSELAbits.ANSA0 = 1; } while(0)
#define channel_ANA0_SetDigitalMode() do { ANSELAbits.ANSA0 = 0; } while(0)

// get/set channel_ANA1 aliases
#define channel_ANA1_TRIS               TRISAbits.TRISA1
#define channel_ANA1_LAT                LATAbits.LATA1
#define channel_ANA1_PORT               PORTAbits.RA1
#define channel_ANA1_WPU                WPUAbits.WPUA1
#define channel_ANA1_OD                ODCONAbits.ODCA1
#define channel_ANA1_ANS                ANSELAbits.ANSA1
#define channel_ANA1_SetHigh()            do { LATAbits.LATA1 = 1; } while(0)
#define channel_ANA1_SetLow()             do { LATAbits.LATA1 = 0; } while(0)
#define channel_ANA1_Toggle()             do { LATAbits.LATA1 = ~LATAbits.LATA1; } while(0)
#define channel_ANA1_GetValue()           PORTAbits.RA1
#define channel_ANA1_SetDigitalInput()    do { TRISAbits.TRISA1 = 1; } while(0)
#define channel_ANA1_SetDigitalOutput()   do { TRISAbits.TRISA1 = 0; } while(0)
#define channel_ANA1_SetPullup()      do { WPUAbits.WPUA1 = 1; } while(0)
#define channel_ANA1_ResetPullup()    do { WPUAbits.WPUA1 = 0; } while(0)
#define channel_ANA1_SetPushPull()    do { ODCONAbits.ODCA1 = 1; } while(0)
#define channel_ANA1_SetOpenDrain()   do { ODCONAbits.ODCA1 = 0; } while(0)
#define channel_ANA1_SetAnalogMode()  do { ANSELAbits.ANSA1 = 1; } while(0)
#define channel_ANA1_SetDigitalMode() do { ANSELAbits.ANSA1 = 0; } while(0)

// get/set channel_ANA2 aliases
#define channel_ANA2_TRIS               TRISAbits.TRISA2
#define channel_ANA2_LAT                LATAbits.LATA2
#define channel_ANA2_PORT               PORTAbits.RA2
#define channel_ANA2_WPU                WPUAbits.WPUA2
#define channel_ANA2_OD                ODCONAbits.ODCA2
#define channel_ANA2_ANS                ANSELAbits.ANSA2
#define channel_ANA2_SetHigh()            do { LATAbits.LATA2 = 1; } while(0)
#define channel_ANA2_SetLow()             do { LATAbits.LATA2 = 0; } while(0)
#define channel_ANA2_Toggle()             do { LATAbits.LATA2 = ~LATAbits.LATA2; } while(0)
#define channel_ANA2_GetValue()           PORTAbits.RA2
#define channel_ANA2_SetDigitalInput()    do { TRISAbits.TRISA2 = 1; } while(0)
#define channel_ANA2_SetDigitalOutput()   do { TRISAbits.TRISA2 = 0; } while(0)
#define channel_ANA2_SetPullup()      do { WPUAbits.WPUA2 = 1; } while(0)
#define channel_ANA2_ResetPullup()    do { WPUAbits.WPUA2 = 0; } while(0)
#define channel_ANA2_SetPushPull()    do { ODCONAbits.ODCA2 = 1; } while(0)
#define channel_ANA2_SetOpenDrain()   do { ODCONAbits.ODCA2 = 0; } while(0)
#define channel_ANA2_SetAnalogMode()  do { ANSELAbits.ANSA2 = 1; } while(0)
#define channel_ANA2_SetDigitalMode() do { ANSELAbits.ANSA2 = 0; } while(0)

// get/set channel_ANA3 aliases
#define channel_ANA3_TRIS               TRISAbits.TRISA3
#define channel_ANA3_LAT                LATAbits.LATA3
#define channel_ANA3_PORT               PORTAbits.RA3
#define channel_ANA3_WPU                WPUAbits.WPUA3
#define channel_ANA3_OD                ODCONAbits.ODCA3
#define channel_ANA3_ANS                ANSELAbits.ANSA3
#define channel_ANA3_SetHigh()            do { LATAbits.LATA3 = 1; } while(0)
#define channel_ANA3_SetLow()             do { LATAbits.LATA3 = 0; } while(0)
#define channel_ANA3_Toggle()             do { LATAbits.LATA3 = ~LATAbits.LATA3; } while(0)
#define channel_ANA3_GetValue()           PORTAbits.RA3
#define channel_ANA3_SetDigitalInput()    do { TRISAbits.TRISA3 = 1; } while(0)
#define channel_ANA3_SetDigitalOutput()   do { TRISAbits.TRISA3 = 0; } while(0)
#define channel_ANA3_SetPullup()      do { WPUAbits.WPUA3 = 1; } while(0)
#define channel_ANA3_ResetPullup()    do { WPUAbits.WPUA3 = 0; } while(0)
#define channel_ANA3_SetPushPull()    do { ODCONAbits.ODCA3 = 1; } while(0)
#define channel_ANA3_SetOpenDrain()   do { ODCONAbits.ODCA3 = 0; } while(0)
#define channel_ANA3_SetAnalogMode()  do { ANSELAbits.ANSA3 = 1; } while(0)
#define channel_ANA3_SetDigitalMode() do { ANSELAbits.ANSA3 = 0; } while(0)

// get/set channel_ANA4 aliases
#define channel_ANA4_TRIS               TRISAbits.TRISA4
#define channel_ANA4_LAT                LATAbits.LATA4
#define channel_ANA4_PORT               PORTAbits.RA4
#define channel_ANA4_WPU                WPUAbits.WPUA4
#define channel_ANA4_OD                ODCONAbits.ODCA4
#define channel_ANA4_ANS                ANSELAbits.ANSA4
#define channel_ANA4_SetHigh()            do { LATAbits.LATA4 = 1; } while(0)
#define channel_ANA4_SetLow()             do { LATAbits.LATA4 = 0; } while(0)
#define channel_ANA4_Toggle()             do { LATAbits.LATA4 = ~LATAbits.LATA4; } while(0)
#define channel_ANA4_GetValue()           PORTAbits.RA4
#define channel_ANA4_SetDigitalInput()    do { TRISAbits.TRISA4 = 1; } while(0)
#define channel_ANA4_SetDigitalOutput()   do { TRISAbits.TRISA4 = 0; } while(0)
#define channel_ANA4_SetPullup()      do { WPUAbits.WPUA4 = 1; } while(0)
#define channel_ANA4_ResetPullup()    do { WPUAbits.WPUA4 = 0; } while(0)
#define channel_ANA4_SetPushPull()    do { ODCONAbits.ODCA4 = 1; } while(0)
#define channel_ANA4_SetOpenDrain()   do { ODCONAbits.ODCA4 = 0; } while(0)
#define channel_ANA4_SetAnalogMode()  do { ANSELAbits.ANSA4 = 1; } while(0)
#define channel_ANA4_SetDigitalMode() do { ANSELAbits.ANSA4 = 0; } while(0)

// get/set channel_ANA5 aliases
#define channel_ANA5_TRIS               TRISAbits.TRISA5
#define channel_ANA5_LAT                LATAbits.LATA5
#define channel_ANA5_PORT               PORTAbits.RA5
#define channel_ANA5_WPU                WPUAbits.WPUA5
#define channel_ANA5_OD                ODCONAbits.ODCA5
#define channel_ANA5_ANS                ANSELAbits.ANSA5
#define channel_ANA5_SetHigh()            do { LATAbits.LATA5 = 1; } while(0)
#define channel_ANA5_SetLow()             do { LATAbits.LATA5 = 0; } while(0)
#define channel_ANA5_Toggle()             do { LATAbits.LATA5 = ~LATAbits.LATA5; } while(0)
#define channel_ANA5_GetValue()           PORTAbits.RA5
#define channel_ANA5_SetDigitalInput()    do { TRISAbits.TRISA5 = 1; } while(0)
#define channel_ANA5_SetDigitalOutput()   do { TRISAbits.TRISA5 = 0; } while(0)
#define channel_ANA5_SetPullup()      do { WPUAbits.WPUA5 = 1; } while(0)
#define channel_ANA5_ResetPullup()    do { WPUAbits.WPUA5 = 0; } while(0)
#define channel_ANA5_SetPushPull()    do { ODCONAbits.ODCA5 = 1; } while(0)
#define channel_ANA5_SetOpenDrain()   do { ODCONAbits.ODCA5 = 0; } while(0)
#define channel_ANA5_SetAnalogMode()  do { ANSELAbits.ANSA5 = 1; } while(0)
#define channel_ANA5_SetDigitalMode() do { ANSELAbits.ANSA5 = 0; } while(0)

// get/set channel_ANA6 aliases
#define channel_ANA6_TRIS               TRISAbits.TRISA6
#define channel_ANA6_LAT                LATAbits.LATA6
#define channel_ANA6_PORT               PORTAbits.RA6
#define channel_ANA6_WPU                WPUAbits.WPUA6
#define channel_ANA6_OD                ODCONAbits.ODCA6
#define channel_ANA6_ANS                ANSELAbits.ANSA6
#define channel_ANA6_SetHigh()            do { LATAbits.LATA6 = 1; } while(0)
#define channel_ANA6_SetLow()             do { LATAbits.LATA6 = 0; } while(0)
#define channel_ANA6_Toggle()             do { LATAbits.LATA6 = ~LATAbits.LATA6; } while(0)
#define channel_ANA6_GetValue()           PORTAbits.RA6
#define channel_ANA6_SetDigitalInput()    do { TRISAbits.TRISA6 = 1; } while(0)
#define channel_ANA6_SetDigitalOutput()   do { TRISAbits.TRISA6 = 0; } while(0)
#define channel_ANA6_SetPullup()      do { WPUAbits.WPUA6 = 1; } while(0)
#define channel_ANA6_ResetPullup()    do { WPUAbits.WPUA6 = 0; } while(0)
#define channel_ANA6_SetPushPull()    do { ODCONAbits.ODCA6 = 1; } while(0)
#define channel_ANA6_SetOpenDrain()   do { ODCONAbits.ODCA6 = 0; } while(0)
#define channel_ANA6_SetAnalogMode()  do { ANSELAbits.ANSA6 = 1; } while(0)
#define channel_ANA6_SetDigitalMode() do { ANSELAbits.ANSA6 = 0; } while(0)

// get/set channel_ANA7 aliases
#define channel_ANA7_TRIS               TRISAbits.TRISA7
#define channel_ANA7_LAT                LATAbits.LATA7
#define channel_ANA7_PORT               PORTAbits.RA7
#define channel_ANA7_WPU                WPUAbits.WPUA7
#define channel_ANA7_OD                ODCONAbits.ODCA7
#define channel_ANA7_ANS                ANSELAbits.ANSA7
#define channel_ANA7_SetHigh()            do { LATAbits.LATA7 = 1; } while(0)
#define channel_ANA7_SetLow()             do { LATAbits.LATA7 = 0; } while(0)
#define channel_ANA7_Toggle()             do { LATAbits.LATA7 = ~LATAbits.LATA7; } while(0)
#define channel_ANA7_GetValue()           PORTAbits.RA7
#define channel_ANA7_SetDigitalInput()    do { TRISAbits.TRISA7 = 1; } while(0)
#define channel_ANA7_SetDigitalOutput()   do { TRISAbits.TRISA7 = 0; } while(0)
#define channel_ANA7_SetPullup()      do { WPUAbits.WPUA7 = 1; } while(0)
#define channel_ANA7_ResetPullup()    do { WPUAbits.WPUA7 = 0; } while(0)
#define channel_ANA7_SetPushPull()    do { ODCONAbits.ODCA7 = 1; } while(0)
#define channel_ANA7_SetOpenDrain()   do { ODCONAbits.ODCA7 = 0; } while(0)
#define channel_ANA7_SetAnalogMode()  do { ANSELAbits.ANSA7 = 1; } while(0)
#define channel_ANA7_SetDigitalMode() do { ANSELAbits.ANSA7 = 0; } while(0)

// get/set IO_RB0 aliases
#define IO_RB0_TRIS               TRISBbits.TRISB0
#define IO_RB0_LAT                LATBbits.LATB0
#define IO_RB0_PORT               PORTBbits.RB0
#define IO_RB0_WPU                WPUBbits.WPUB0
#define IO_RB0_OD                ODCONBbits.ODCB0
#define IO_RB0_ANS                ANSELBbits.ANSB0
#define IO_RB0_SetHigh()            do { LATBbits.LATB0 = 1; } while(0)
#define IO_RB0_SetLow()             do { LATBbits.LATB0 = 0; } while(0)
#define IO_RB0_Toggle()             do { LATBbits.LATB0 = ~LATBbits.LATB0; } while(0)
#define IO_RB0_GetValue()           PORTBbits.RB0
#define IO_RB0_SetDigitalInput()    do { TRISBbits.TRISB0 = 1; } while(0)
#define IO_RB0_SetDigitalOutput()   do { TRISBbits.TRISB0 = 0; } while(0)
#define IO_RB0_SetPullup()      do { WPUBbits.WPUB0 = 1; } while(0)
#define IO_RB0_ResetPullup()    do { WPUBbits.WPUB0 = 0; } while(0)
#define IO_RB0_SetPushPull()    do { ODCONBbits.ODCB0 = 1; } while(0)
#define IO_RB0_SetOpenDrain()   do { ODCONBbits.ODCB0 = 0; } while(0)
#define IO_RB0_SetAnalogMode()  do { ANSELBbits.ANSB0 = 1; } while(0)
#define IO_RB0_SetDigitalMode() do { ANSELBbits.ANSB0 = 0; } while(0)

// get/set IO_RB1 aliases
#define IO_RB1_TRIS               TRISBbits.TRISB1
#define IO_RB1_LAT                LATBbits.LATB1
#define IO_RB1_PORT               PORTBbits.RB1
#define IO_RB1_WPU                WPUBbits.WPUB1
#define IO_RB1_OD                ODCONBbits.ODCB1
#define IO_RB1_ANS                ANSELBbits.ANSB1
#define IO_RB1_SetHigh()            do { LATBbits.LATB1 = 1; } while(0)
#define IO_RB1_SetLow()             do { LATBbits.LATB1 = 0; } while(0)
#define IO_RB1_Toggle()             do { LATBbits.LATB1 = ~LATBbits.LATB1; } while(0)
#define IO_RB1_GetValue()           PORTBbits.RB1
#define IO_RB1_SetDigitalInput()    do { TRISBbits.TRISB1 = 1; } while(0)
#define IO_RB1_SetDigitalOutput()   do { TRISBbits.TRISB1 = 0; } while(0)
#define IO_RB1_SetPullup()      do { WPUBbits.WPUB1 = 1; } while(0)
#define IO_RB1_ResetPullup()    do { WPUBbits.WPUB1 = 0; } while(0)
#define IO_RB1_SetPushPull()    do { ODCONBbits.ODCB1 = 1; } while(0)
#define IO_RB1_SetOpenDrain()   do { ODCONBbits.ODCB1 = 0; } while(0)
#define IO_RB1_SetAnalogMode()  do { ANSELBbits.ANSB1 = 1; } while(0)
#define IO_RB1_SetDigitalMode() do { ANSELBbits.ANSB1 = 0; } while(0)

// get/set IO_RB2 aliases
#define IO_RB2_TRIS               TRISBbits.TRISB2
#define IO_RB2_LAT                LATBbits.LATB2
#define IO_RB2_PORT               PORTBbits.RB2
#define IO_RB2_WPU                WPUBbits.WPUB2
#define IO_RB2_OD                ODCONBbits.ODCB2
#define IO_RB2_ANS                ANSELBbits.ANSB2
#define IO_RB2_SetHigh()            do { LATBbits.LATB2 = 1; } while(0)
#define IO_RB2_SetLow()             do { LATBbits.LATB2 = 0; } while(0)
#define IO_RB2_Toggle()             do { LATBbits.LATB2 = ~LATBbits.LATB2; } while(0)
#define IO_RB2_GetValue()           PORTBbits.RB2
#define IO_RB2_SetDigitalInput()    do { TRISBbits.TRISB2 = 1; } while(0)
#define IO_RB2_SetDigitalOutput()   do { TRISBbits.TRISB2 = 0; } while(0)
#define IO_RB2_SetPullup()      do { WPUBbits.WPUB2 = 1; } while(0)
#define IO_RB2_ResetPullup()    do { WPUBbits.WPUB2 = 0; } while(0)
#define IO_RB2_SetPushPull()    do { ODCONBbits.ODCB2 = 1; } while(0)
#define IO_RB2_SetOpenDrain()   do { ODCONBbits.ODCB2 = 0; } while(0)
#define IO_RB2_SetAnalogMode()  do { ANSELBbits.ANSB2 = 1; } while(0)
#define IO_RB2_SetDigitalMode() do { ANSELBbits.ANSB2 = 0; } while(0)

// get/set IO_RB3 aliases
#define IO_RB3_TRIS               TRISBbits.TRISB3
#define IO_RB3_LAT                LATBbits.LATB3
#define IO_RB3_PORT               PORTBbits.RB3
#define IO_RB3_WPU                WPUBbits.WPUB3
#define IO_RB3_OD                ODCONBbits.ODCB3
#define IO_RB3_ANS                ANSELBbits.ANSB3
#define IO_RB3_SetHigh()            do { LATBbits.LATB3 = 1; } while(0)
#define IO_RB3_SetLow()             do { LATBbits.LATB3 = 0; } while(0)
#define IO_RB3_Toggle()             do { LATBbits.LATB3 = ~LATBbits.LATB3; } while(0)
#define IO_RB3_GetValue()           PORTBbits.RB3
#define IO_RB3_SetDigitalInput()    do { TRISBbits.TRISB3 = 1; } while(0)
#define IO_RB3_SetDigitalOutput()   do { TRISBbits.TRISB3 = 0; } while(0)
#define IO_RB3_SetPullup()      do { WPUBbits.WPUB3 = 1; } while(0)
#define IO_RB3_ResetPullup()    do { WPUBbits.WPUB3 = 0; } while(0)
#define IO_RB3_SetPushPull()    do { ODCONBbits.ODCB3 = 1; } while(0)
#define IO_RB3_SetOpenDrain()   do { ODCONBbits.ODCB3 = 0; } while(0)
#define IO_RB3_SetAnalogMode()  do { ANSELBbits.ANSB3 = 1; } while(0)
#define IO_RB3_SetDigitalMode() do { ANSELBbits.ANSB3 = 0; } while(0)

// get/set IO_RB4 aliases
#define IO_RB4_TRIS               TRISBbits.TRISB4
#define IO_RB4_LAT                LATBbits.LATB4
#define IO_RB4_PORT               PORTBbits.RB4
#define IO_RB4_WPU                WPUBbits.WPUB4
#define IO_RB4_OD                ODCONBbits.ODCB4
#define IO_RB4_ANS                ANSELBbits.ANSB4
#define IO_RB4_SetHigh()            do { LATBbits.LATB4 = 1; } while(0)
#define IO_RB4_SetLow()             do { LATBbits.LATB4 = 0; } while(0)
#define IO_RB4_Toggle()             do { LATBbits.LATB4 = ~LATBbits.LATB4; } while(0)
#define IO_RB4_GetValue()           PORTBbits.RB4
#define IO_RB4_SetDigitalInput()    do { TRISBbits.TRISB4 = 1; } while(0)
#define IO_RB4_SetDigitalOutput()   do { TRISBbits.TRISB4 = 0; } while(0)
#define IO_RB4_SetPullup()      do { WPUBbits.WPUB4 = 1; } while(0)
#define IO_RB4_ResetPullup()    do { WPUBbits.WPUB4 = 0; } while(0)
#define IO_RB4_SetPushPull()    do { ODCONBbits.ODCB4 = 1; } while(0)
#define IO_RB4_SetOpenDrain()   do { ODCONBbits.ODCB4 = 0; } while(0)
#define IO_RB4_SetAnalogMode()  do { ANSELBbits.ANSB4 = 1; } while(0)
#define IO_RB4_SetDigitalMode() do { ANSELBbits.ANSB4 = 0; } while(0)

// get/set IO_RB5 aliases
#define IO_RB5_TRIS               TRISBbits.TRISB5
#define IO_RB5_LAT                LATBbits.LATB5
#define IO_RB5_PORT               PORTBbits.RB5
#define IO_RB5_WPU                WPUBbits.WPUB5
#define IO_RB5_OD                ODCONBbits.ODCB5
#define IO_RB5_ANS                ANSELBbits.ANSB5
#define IO_RB5_SetHigh()            do { LATBbits.LATB5 = 1; } while(0)
#define IO_RB5_SetLow()             do { LATBbits.LATB5 = 0; } while(0)
#define IO_RB5_Toggle()             do { LATBbits.LATB5 = ~LATBbits.LATB5; } while(0)
#define IO_RB5_GetValue()           PORTBbits.RB5
#define IO_RB5_SetDigitalInput()    do { TRISBbits.TRISB5 = 1; } while(0)
#define IO_RB5_SetDigitalOutput()   do { TRISBbits.TRISB5 = 0; } while(0)
#define IO_RB5_SetPullup()      do { WPUBbits.WPUB5 = 1; } while(0)
#define IO_RB5_ResetPullup()    do { WPUBbits.WPUB5 = 0; } while(0)
#define IO_RB5_SetPushPull()    do { ODCONBbits.ODCB5 = 1; } while(0)
#define IO_RB5_SetOpenDrain()   do { ODCONBbits.ODCB5 = 0; } while(0)
#define IO_RB5_SetAnalogMode()  do { ANSELBbits.ANSB5 = 1; } while(0)
#define IO_RB5_SetDigitalMode() do { ANSELBbits.ANSB5 = 0; } while(0)

// get/set IO_RB6 aliases
#define IO_RB6_TRIS               TRISBbits.TRISB6
#define IO_RB6_LAT                LATBbits.LATB6
#define IO_RB6_PORT               PORTBbits.RB6
#define IO_RB6_WPU                WPUBbits.WPUB6
#define IO_RB6_OD                ODCONBbits.ODCB6
#define IO_RB6_ANS                ANSELBbits.ANSB6
#define IO_RB6_SetHigh()            do { LATBbits.LATB6 = 1; } while(0)
#define IO_RB6_SetLow()             do { LATBbits.LATB6 = 0; } while(0)
#define IO_RB6_Toggle()             do { LATBbits.LATB6 = ~LATBbits.LATB6; } while(0)
#define IO_RB6_GetValue()           PORTBbits.RB6
#define IO_RB6_SetDigitalInput()    do { TRISBbits.TRISB6 = 1; } while(0)
#define IO_RB6_SetDigitalOutput()   do { TRISBbits.TRISB6 = 0; } while(0)
#define IO_RB6_SetPullup()      do { WPUBbits.WPUB6 = 1; } while(0)
#define IO_RB6_ResetPullup()    do { WPUBbits.WPUB6 = 0; } while(0)
#define IO_RB6_SetPushPull()    do { ODCONBbits.ODCB6 = 1; } while(0)
#define IO_RB6_SetOpenDrain()   do { ODCONBbits.ODCB6 = 0; } while(0)
#define IO_RB6_SetAnalogMode()  do { ANSELBbits.ANSB6 = 1; } while(0)
#define IO_RB6_SetDigitalMode() do { ANSELBbits.ANSB6 = 0; } while(0)

// get/set IO_RB7 aliases
#define IO_RB7_TRIS               TRISBbits.TRISB7
#define IO_RB7_LAT                LATBbits.LATB7
#define IO_RB7_PORT               PORTBbits.RB7
#define IO_RB7_WPU                WPUBbits.WPUB7
#define IO_RB7_OD                ODCONBbits.ODCB7
#define IO_RB7_ANS                ANSELBbits.ANSB7
#define IO_RB7_SetHigh()            do { LATBbits.LATB7 = 1; } while(0)
#define IO_RB7_SetLow()             do { LATBbits.LATB7 = 0; } while(0)
#define IO_RB7_Toggle()             do { LATBbits.LATB7 = ~LATBbits.LATB7; } while(0)
#define IO_RB7_GetValue()           PORTBbits.RB7
#define IO_RB7_SetDigitalInput()    do { TRISBbits.TRISB7 = 1; } while(0)
#define IO_RB7_SetDigitalOutput()   do { TRISBbits.TRISB7 = 0; } while(0)
#define IO_RB7_SetPullup()      do { WPUBbits.WPUB7 = 1; } while(0)
#define IO_RB7_ResetPullup()    do { WPUBbits.WPUB7 = 0; } while(0)
#define IO_RB7_SetPushPull()    do { ODCONBbits.ODCB7 = 1; } while(0)
#define IO_RB7_SetOpenDrain()   do { ODCONBbits.ODCB7 = 0; } while(0)
#define IO_RB7_SetAnalogMode()  do { ANSELBbits.ANSB7 = 1; } while(0)
#define IO_RB7_SetDigitalMode() do { ANSELBbits.ANSB7 = 0; } while(0)

// get/set channel_ANC0 aliases
#define channel_ANC0_TRIS               TRISCbits.TRISC0
#define channel_ANC0_LAT                LATCbits.LATC0
#define channel_ANC0_PORT               PORTCbits.RC0
#define channel_ANC0_WPU                WPUCbits.WPUC0
#define channel_ANC0_OD                ODCONCbits.ODCC0
#define channel_ANC0_ANS                ANSELCbits.ANSC0
#define channel_ANC0_SetHigh()            do { LATCbits.LATC0 = 1; } while(0)
#define channel_ANC0_SetLow()             do { LATCbits.LATC0 = 0; } while(0)
#define channel_ANC0_Toggle()             do { LATCbits.LATC0 = ~LATCbits.LATC0; } while(0)
#define channel_ANC0_GetValue()           PORTCbits.RC0
#define channel_ANC0_SetDigitalInput()    do { TRISCbits.TRISC0 = 1; } while(0)
#define channel_ANC0_SetDigitalOutput()   do { TRISCbits.TRISC0 = 0; } while(0)
#define channel_ANC0_SetPullup()      do { WPUCbits.WPUC0 = 1; } while(0)
#define channel_ANC0_ResetPullup()    do { WPUCbits.WPUC0 = 0; } while(0)
#define channel_ANC0_SetPushPull()    do { ODCONCbits.ODCC0 = 1; } while(0)
#define channel_ANC0_SetOpenDrain()   do { ODCONCbits.ODCC0 = 0; } while(0)
#define channel_ANC0_SetAnalogMode()  do { ANSELCbits.ANSC0 = 1; } while(0)
#define channel_ANC0_SetDigitalMode() do { ANSELCbits.ANSC0 = 0; } while(0)

// get/set IO_RC1 aliases
#define IO_RC1_TRIS               TRISCbits.TRISC1
#define IO_RC1_LAT                LATCbits.LATC1
#define IO_RC1_PORT               PORTCbits.RC1
#define IO_RC1_WPU                WPUCbits.WPUC1
#define IO_RC1_OD                ODCONCbits.ODCC1
#define IO_RC1_ANS                ANSELCbits.ANSC1
#define IO_RC1_SetHigh()            do { LATCbits.LATC1 = 1; } while(0)
#define IO_RC1_SetLow()             do { LATCbits.LATC1 = 0; } while(0)
#define IO_RC1_Toggle()             do { LATCbits.LATC1 = ~LATCbits.LATC1; } while(0)
#define IO_RC1_GetValue()           PORTCbits.RC1
#define IO_RC1_SetDigitalInput()    do { TRISCbits.TRISC1 = 1; } while(0)
#define IO_RC1_SetDigitalOutput()   do { TRISCbits.TRISC1 = 0; } while(0)
#define IO_RC1_SetPullup()      do { WPUCbits.WPUC1 = 1; } while(0)
#define IO_RC1_ResetPullup()    do { WPUCbits.WPUC1 = 0; } while(0)
#define IO_RC1_SetPushPull()    do { ODCONCbits.ODCC1 = 1; } while(0)
#define IO_RC1_SetOpenDrain()   do { ODCONCbits.ODCC1 = 0; } while(0)
#define IO_RC1_SetAnalogMode()  do { ANSELCbits.ANSC1 = 1; } while(0)
#define IO_RC1_SetDigitalMode() do { ANSELCbits.ANSC1 = 0; } while(0)

// get/set SDA1 aliases
#define SDA1_TRIS               TRISCbits.TRISC3
#define SDA1_LAT                LATCbits.LATC3
#define SDA1_PORT               PORTCbits.RC3
#define SDA1_WPU                WPUCbits.WPUC3
#define SDA1_OD                ODCONCbits.ODCC3
#define SDA1_ANS                ANSELCbits.ANSC3
#define SDA1_SetHigh()            do { LATCbits.LATC3 = 1; } while(0)
#define SDA1_SetLow()             do { LATCbits.LATC3 = 0; } while(0)
#define SDA1_Toggle()             do { LATCbits.LATC3 = ~LATCbits.LATC3; } while(0)
#define SDA1_GetValue()           PORTCbits.RC3
#define SDA1_SetDigitalInput()    do { TRISCbits.TRISC3 = 1; } while(0)
#define SDA1_SetDigitalOutput()   do { TRISCbits.TRISC3 = 0; } while(0)
#define SDA1_SetPullup()      do { WPUCbits.WPUC3 = 1; } while(0)
#define SDA1_ResetPullup()    do { WPUCbits.WPUC3 = 0; } while(0)
#define SDA1_SetPushPull()    do { ODCONCbits.ODCC3 = 1; } while(0)
#define SDA1_SetOpenDrain()   do { ODCONCbits.ODCC3 = 0; } while(0)
#define SDA1_SetAnalogMode()  do { ANSELCbits.ANSC3 = 1; } while(0)
#define SDA1_SetDigitalMode() do { ANSELCbits.ANSC3 = 0; } while(0)

// get/set SCL1 aliases
#define SCL1_TRIS               TRISCbits.TRISC4
#define SCL1_LAT                LATCbits.LATC4
#define SCL1_PORT               PORTCbits.RC4
#define SCL1_WPU                WPUCbits.WPUC4
#define SCL1_OD                ODCONCbits.ODCC4
#define SCL1_ANS                ANSELCbits.ANSC4
#define SCL1_SetHigh()            do { LATCbits.LATC4 = 1; } while(0)
#define SCL1_SetLow()             do { LATCbits.LATC4 = 0; } while(0)
#define SCL1_Toggle()             do { LATCbits.LATC4 = ~LATCbits.LATC4; } while(0)
#define SCL1_GetValue()           PORTCbits.RC4
#define SCL1_SetDigitalInput()    do { TRISCbits.TRISC4 = 1; } while(0)
#define SCL1_SetDigitalOutput()   do { TRISCbits.TRISC4 = 0; } while(0)
#define SCL1_SetPullup()      do { WPUCbits.WPUC4 = 1; } while(0)
#define SCL1_ResetPullup()    do { WPUCbits.WPUC4 = 0; } while(0)
#define SCL1_SetPushPull()    do { ODCONCbits.ODCC4 = 1; } while(0)
#define SCL1_SetOpenDrain()   do { ODCONCbits.ODCC4 = 0; } while(0)
#define SCL1_SetAnalogMode()  do { ANSELCbits.ANSC4 = 1; } while(0)
#define SCL1_SetDigitalMode() do { ANSELCbits.ANSC4 = 0; } while(0)

// get/set IO_RC5_LED_LATCH_RCK aliases
#define IO_RC5_LED_LATCH_RCK_TRIS               TRISCbits.TRISC5
#define IO_RC5_LED_LATCH_RCK_LAT                LATCbits.LATC5
#define IO_RC5_LED_LATCH_RCK_PORT               PORTCbits.RC5
#define IO_RC5_LED_LATCH_RCK_WPU                WPUCbits.WPUC5
#define IO_RC5_LED_LATCH_RCK_OD                ODCONCbits.ODCC5
#define IO_RC5_LED_LATCH_RCK_ANS                ANSELCbits.ANSC5
#define IO_RC5_LED_LATCH_RCK_SetHigh()            do { LATCbits.LATC5 = 1; } while(0)
#define IO_RC5_LED_LATCH_RCK_SetLow()             do { LATCbits.LATC5 = 0; } while(0)
#define IO_RC5_LED_LATCH_RCK_Toggle()             do { LATCbits.LATC5 = ~LATCbits.LATC5; } while(0)
#define IO_RC5_LED_LATCH_RCK_GetValue()           PORTCbits.RC5
#define IO_RC5_LED_LATCH_RCK_SetDigitalInput()    do { TRISCbits.TRISC5 = 1; } while(0)
#define IO_RC5_LED_LATCH_RCK_SetDigitalOutput()   do { TRISCbits.TRISC5 = 0; } while(0)
#define IO_RC5_LED_LATCH_RCK_SetPullup()      do { WPUCbits.WPUC5 = 1; } while(0)
#define IO_RC5_LED_LATCH_RCK_ResetPullup()    do { WPUCbits.WPUC5 = 0; } while(0)
#define IO_RC5_LED_LATCH_RCK_SetPushPull()    do { ODCONCbits.ODCC5 = 1; } while(0)
#define IO_RC5_LED_LATCH_RCK_SetOpenDrain()   do { ODCONCbits.ODCC5 = 0; } while(0)
#define IO_RC5_LED_LATCH_RCK_SetAnalogMode()  do { ANSELCbits.ANSC5 = 1; } while(0)
#define IO_RC5_LED_LATCH_RCK_SetDigitalMode() do { ANSELCbits.ANSC5 = 0; } while(0)

// get/set IO_RC6_LED_SCLOCK aliases
#define IO_RC6_LED_SCLOCK_TRIS               TRISCbits.TRISC6
#define IO_RC6_LED_SCLOCK_LAT                LATCbits.LATC6
#define IO_RC6_LED_SCLOCK_PORT               PORTCbits.RC6
#define IO_RC6_LED_SCLOCK_WPU                WPUCbits.WPUC6
#define IO_RC6_LED_SCLOCK_OD                ODCONCbits.ODCC6
#define IO_RC6_LED_SCLOCK_ANS                ANSELCbits.ANSC6
#define IO_RC6_LED_SCLOCK_SetHigh()            do { LATCbits.LATC6 = 1; } while(0)
#define IO_RC6_LED_SCLOCK_SetLow()             do { LATCbits.LATC6 = 0; } while(0)
#define IO_RC6_LED_SCLOCK_Toggle()             do { LATCbits.LATC6 = ~LATCbits.LATC6; } while(0)
#define IO_RC6_LED_SCLOCK_GetValue()           PORTCbits.RC6
#define IO_RC6_LED_SCLOCK_SetDigitalInput()    do { TRISCbits.TRISC6 = 1; } while(0)
#define IO_RC6_LED_SCLOCK_SetDigitalOutput()   do { TRISCbits.TRISC6 = 0; } while(0)
#define IO_RC6_LED_SCLOCK_SetPullup()      do { WPUCbits.WPUC6 = 1; } while(0)
#define IO_RC6_LED_SCLOCK_ResetPullup()    do { WPUCbits.WPUC6 = 0; } while(0)
#define IO_RC6_LED_SCLOCK_SetPushPull()    do { ODCONCbits.ODCC6 = 1; } while(0)
#define IO_RC6_LED_SCLOCK_SetOpenDrain()   do { ODCONCbits.ODCC6 = 0; } while(0)
#define IO_RC6_LED_SCLOCK_SetAnalogMode()  do { ANSELCbits.ANSC6 = 1; } while(0)
#define IO_RC6_LED_SCLOCK_SetDigitalMode() do { ANSELCbits.ANSC6 = 0; } while(0)

// get/set IO_RC7_LED_DATA aliases
#define IO_RC7_LED_DATA_TRIS               TRISCbits.TRISC7
#define IO_RC7_LED_DATA_LAT                LATCbits.LATC7
#define IO_RC7_LED_DATA_PORT               PORTCbits.RC7
#define IO_RC7_LED_DATA_WPU                WPUCbits.WPUC7
#define IO_RC7_LED_DATA_OD                ODCONCbits.ODCC7
#define IO_RC7_LED_DATA_ANS                ANSELCbits.ANSC7
#define IO_RC7_LED_DATA_SetHigh()            do { LATCbits.LATC7 = 1; } while(0)
#define IO_RC7_LED_DATA_SetLow()             do { LATCbits.LATC7 = 0; } while(0)
#define IO_RC7_LED_DATA_Toggle()             do { LATCbits.LATC7 = ~LATCbits.LATC7; } while(0)
#define IO_RC7_LED_DATA_GetValue()           PORTCbits.RC7
#define IO_RC7_LED_DATA_SetDigitalInput()    do { TRISCbits.TRISC7 = 1; } while(0)
#define IO_RC7_LED_DATA_SetDigitalOutput()   do { TRISCbits.TRISC7 = 0; } while(0)
#define IO_RC7_LED_DATA_SetPullup()      do { WPUCbits.WPUC7 = 1; } while(0)
#define IO_RC7_LED_DATA_ResetPullup()    do { WPUCbits.WPUC7 = 0; } while(0)
#define IO_RC7_LED_DATA_SetPushPull()    do { ODCONCbits.ODCC7 = 1; } while(0)
#define IO_RC7_LED_DATA_SetOpenDrain()   do { ODCONCbits.ODCC7 = 0; } while(0)
#define IO_RC7_LED_DATA_SetAnalogMode()  do { ANSELCbits.ANSC7 = 1; } while(0)
#define IO_RC7_LED_DATA_SetDigitalMode() do { ANSELCbits.ANSC7 = 0; } while(0)

/**
   @Param
    none
   @Returns
    none
   @Description
    GPIO and peripheral I/O initialization
   @Example
    PIN_MANAGER_Initialize();
 */
void PIN_MANAGER_Initialize (void);

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    Interrupt on Change Handling routine
 * @Example
    PIN_MANAGER_IOC();
 */
void PIN_MANAGER_IOC(void);



#endif // PIN_MANAGER_H
/**
 End of File
*/