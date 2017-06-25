/********************************************************************
 FileName:     	HardwareProfile - PIC24FJ64GB004 PIM.h
 Dependencies:  See INCLUDES section
 Processor:     PIC24FJ64GB004
 Hardware:      PIC24FJ64GB004 PIM
 Compiler:      Microchip C30
 Company:       Microchip Technology, Inc.

 Software License Agreement:

 The software supplied herewith by Microchip Technology Incorporated
 (the �Company�) for its PIC� Microcontroller is intended and
 supplied to you, the Company�s customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.

 THIS SOFTWARE IS PROVIDED IN AN �AS IS� CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

********************************************************************
 File Description:

 Change History:
  Rev   Date         Description
  1.0   11/19/2004   Initial release
  2.1   02/26/2007   Updated for simplicity and to use common
                     coding style
  2.3   09/15/2008   Broke out each hardware platform into its own
                     "HardwareProfile - xxx.h" file
  2.4b  04/08/2009   Initial support for PIC24FJ64GB004 family
********************************************************************/

#ifndef HARDWARE_PROFILE_PIC24FJ64GB004_PIM_H
#define HARDWARE_PROFILE_PIC24FJ64GB004_PIM_H

    /*******************************************************************/
    /******** USB stack hardware selection options *********************/
    /*******************************************************************/
    //This section is the set of definitions required by the MCHPFSUSB
    //  framework.  These definitions tell the firmware what mode it is
    //  running in, and where it can find the results to some information
    //  that the stack needs.
    //These definitions are required by every application developed with
    //  this revision of the MCHPFSUSB framework.  Please review each
    //  option carefully and determine which options are desired/required
    //  for your application.

    //#define USE_SELF_POWER_SENSE_IO
    #define tris_self_power     TRISAbits.TRISA2    // Input
    #define self_power          1

    //#define USE_USB_BUS_SENSE_IO
    #define tris_usb_bus_sense  TRISBbits.TRISB5    // Input
    #define USB_BUS_SENSE       1 
   
    //Uncomment this to make the output HEX of this project 
    //   to be able to be bootloaded using the HID bootloader
    #define PROGRAMMABLE_WITH_USB_HID_BOOTLOADER	

    //If the application is going to be used with the HID bootloader
    //  then this will provide a function for the application to 
    //  enter the bootloader from the application (optional)
    #if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)
        #define EnterBootloader() __asm__("goto 0x400")
    #endif   

    /*******************************************************************/
    /*******************************************************************/
    /*******************************************************************/
    /******** Application specific definitions *************************/
    /*******************************************************************/
    /*******************************************************************/
    /*******************************************************************/

    /** Board definition ***********************************************/
    //These defintions will tell the main() function which board is
    //  currently selected.  This will allow the application to add
    //  the correct configuration bits as wells use the correct
    //  initialization functions for the board.  These defitions are only
    //  required in the stack provided demos.  They are not required in
    //  final application design.
    #define EXPLORER_16
    #define PIC24FJ64GB004_PIM
    #define CLOCK_FREQ 32000000
    //#define GetSystemClock() CLOCK_FREQ
    //#define GetPeripheralClock() CLOCK_FREQ
    
    /** LED ************************************************************/
    #define mInitAllLEDs()      LATA = 0x18; LATB = 0x10; TRISA = 0x7; TRISB = 0xFFEF;
    
    #define mLED_1              LATAbits.LATA4
    #define mLED_2              LATBbits.LATB4
    #define mLED_3              LATAbits.LATA3

    #define mGetLED_1()         mLED_1
    #define mGetLED_2()         mLED_2
    #define mGetLED_3()         mLED_3
    
    #define mLED_1_On()         mLED_1 = 0;
    #define mLED_2_On()         mLED_2 = 0;
    #define mLED_3_On()         mLED_3 = 0;
    
    #define mLED_1_Off()        mLED_1 = 1;
    #define mLED_2_Off()        mLED_2 = 1;
    #define mLED_3_Off()        mLED_3 = 1;
    
    #define mLED_1_Toggle()     mLED_1 = !mLED_1;
    #define mLED_2_Toggle()     mLED_2 = !mLED_2;
    #define mLED_3_Toggle()     mLED_3 = !mLED_3;

    /** SWITCH *********************************************************/
//    #define mInitAllSwitches()  mInitSwitch2();
//    #define mInitSwitch2()      TRISBbits.TRISB11=1;
//    #define sw2                 PORTBbits.RB11

    /** Programmer pins *************************************************/
    #define VPP LATBbits.LATB14
    #define TRISVPP TRISBbits.TRISB14
    #define VPP_RST LATBbits.LATB15
    #define TRISVPP_RST TRISBbits.TRISB15
    #define PGD LATBbits.LATB7
    #define TRISPGD TRISBbits.TRISB7
    #define PGD_READ PORTBbits.RB7
    #define PGC LATBbits.LATB8
    #define TRISPGC TRISBbits.TRISB8
    #define VDD LATBbits.LATB13
    #define TRISVDD TRISBbits.TRISB13
    #define VPP_RUN LATAbits.LATA0
    #define TRISVPP_RUN TRISAbits.TRISA0
    #define PGD_LOW LATBbits.LATB2
    #define TRISPGD_LOW TRISBbits.TRISB2
    #define PGC_LOW LATBbits.LATB3
    #define TRISPGC_LOW TRISBbits.TRISB3

    /** Manipulate Programmer pins **************************************/
    #define setPGDinput()	do { TRISPGD=1;TRISPGD_LOW=1; } while(0)
    #define setPGDoutput()	do { if( is3_3V() ) TRISPGD_LOW=0; TRISPGD=0;} while(0)
    #define enablePGC_D()	do { if( is3_3V() ) { TRISPGC_LOW=0; TRISPGD_LOW=0; } TRISPGC=0;TRISPGD=0;} while(0)
    #define disablePGC_D()	do { TRISPGC_LOW=1; TRISPGD_LOW=1; TRISPGC=1;TRISPGD=1;} while(0)
    //#define enablePGC_LOW()	TRISPGC_LOW=0
    //#define trisPGC_LOW()	TRISPGC_LOW=1
    //#define PGC_LOWoff()	PGC_LOW=1
    //#define PGC_LOWon()	PGC_LOW=0
    //#define enablePGD_LOW()	TRISPGD_LOW=0
    //#define trisPGD_LOW()	TRISPGD_LOW=1
    //#define PGD_LOWoff()	PGD_LOW=1
    //#define PGD_LOWon()	PGD_LOW=0

    #define enableI2CPullup do { TRISPGD_LOW=0; PGD_LOW=1; } while(0)
    #define disableI2CPullup do { TRISPGD_LOW=1; PGD_LOW=0; } while(0)

    #define enableVPP_RUN()	TRISVPP_RUN=0		//FIXME: should rename VPP and VPP_RUN - perhaps VPP_HI, VPP_5V
    #define trisVPP_RUN()	TRISVPP_RUN=1
    #define VPP_RUNoff()	VPP_RUN=0
    #define VPP_RUNon()     VPP_RUN=1
    #define enableVDD()     TRISVDD=0
    #define trisVDD()       TRISVDD=1
    #define VDDoff()        VDD=1
    #define VDDon()         if(ConfigDisableVDD==0)VDD=0
    #define enablePGC()     TRISPGC=0
    //#define trisPGC()     TRISPGC=1
    #define PGClow()        PGC=0
    #define PGChigh()       PGC=1
    //#define enablePGD()	TRISPGD=0
    //#define trisPGD()     TRISPGD=1
    #define PGDlow()        PGD=0
    #define PGDhigh()       PGD=1
    #define enableVPP_RST()	TRISVPP_RST=0
    #define trisVPP_RST()	TRISVPP_RST=1
    #define VPP_RSToff()	VPP_RST=0
    #define VPP_RSTon()     VPP_RST=1
    #define enableVPP()     TRISVPP=0
    #define trisVPP()       TRISVPP=1
    #define VPPoff()        {if(ConfigLimitVPP==0)VPP=1;else{VPP_RUNoff();}}
    #define VPPon()         {if(ConfigLimitVPP==0)VPP=0;else{VPP_RUNon();}}

    /** I/O pin definitions ********************************************/
    #define INPUT_PIN 1
    #define OUTPUT_PIN 0

    //These definitions are only relevant if the respective functions are enabled
    //in the usb_config.h file.
    //Make sure these definitions match the GPIO pins being used for your hardware
    //setup.
    #define UART_DSR PORTAbits.RA10
    #define UART_DTR LATBbits.LATB2
    #define UART_RTS LATCbits.LATC7
    #define UART_CTS PORTBbits.RB14
    
    #define mInitRTSPin() {TRISCbits.TRISC7 = 0;}   //Configure RTS as a digital output.  
    #define mInitCTSPin() {TRISBbits.TRISB14 = 1;}   //Configure CTS as a digital input.  (Make sure pin is digital if ANxx functions is present on the pin)
    #define mInitDSRPin() {TRISAbits.TRISA10 = 1;}   //Configure DTS as a digital input.  (Make sure pin is digital if ANxx functions is present on the pin)
    #define mInitDTRPin() {TRISBbits.TRISB2 = 0;}   //Configure DTR as a digital output.
#endif  //HARDWARE_PROFILE_PIC24FJ64GB004_PIM_H
