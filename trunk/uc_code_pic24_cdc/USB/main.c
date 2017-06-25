/********************************************************************
 FileName:      main.c
 Dependencies:  See INCLUDES section
 Processor:     PIC18, PIC24, dsPIC, and PIC32 USB Microcontrollers
 Hardware:      This demo is natively intended to be used on Microchip USB demo
                boards supported by the MCHPFSUSB stack.  See release notes for
                support matrix.  This demo can be modified for use on other 
                hardware platforms.
 Complier:      Microchip C18 (for PIC18), XC16 (for PIC24/dsPIC), XC32 (for PIC32)
 Company:       Microchip Technology, Inc.

 Software License Agreement:

 The software supplied herewith by Microchip Technology Incorporated
 (the "Company") for its PIC(R) Microcontroller is intended and
 supplied to you, the Company's customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.

 THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

********************************************************************
 File Description:

 Change History:
  Rev   Description
  ----  -----------------------------------------
  1.0   Initial release
  2.1   Updated for simplicity and to use common coding style
  2.8   Improvements to USBCBSendResume(), to make it easier to use.
        Added runtime check to avoid buffer overflow possibility if 
        the USB IN data rate is somehow slower than the UART RX rate.
  2.9b  Added support for optional hardware flow control.
  2.9f  Adding new part support   
  2.9j  Updates to support new bootloader features (ex: app version 
        fetching).
********************************************************************/

/** INCLUDES *******************************************************/
#include "./USB/usb.h"
#include "./USB/usb_function_cdc.h"

#include "HardwareProfile.h"

/** CONFIGURATION **************************************************/

// CONFIG4
#pragma config DSWDTPS = DSWDTPS0       // DSWDT Postscale Select (1:2 (2.1 ms))
#pragma config DSWDTOSC = LPRC          // Deep Sleep Watchdog Timer Oscillator Select (DSWDT uses Low Power RC Oscillator (LPRC))
#pragma config RTCOSC = LPRC            // RTCC Reference Oscillator Select (RTCC uses Low Power RC Oscillator (LPRC))
#pragma config DSBOREN = OFF            // Deep Sleep BOR Enable bit (BOR disabled in Deep Sleep)
#pragma config DSWDTEN = OFF            // Deep Sleep Watchdog Timer (DSWDT disabled)

// CONFIG3
#pragma config WPFP = WPFP0             // Write Protection Flash Page Segment Boundary (Page 0 (0x0))
#pragma config SOSCSEL = IO             // Secondary Oscillator Pin Mode Select (SOSC pins have digital I/O functions (RA4, RB4))
#pragma config WUTSEL = LEG             // Voltage Regulator Wake-up Time Select (Default regulator start-up time used)
#pragma config WPDIS = WPDIS            // Segment Write Protection Disable (Segmented code protection disabled)
#pragma config WPCFG = WPCFGDIS         // Write Protect Configuration Page Select (Last page and Flash Configuration words are unprotected)
#pragma config WPEND = WPSTARTMEM       // Segment Write Protection End Page Select (Write Protect from page 0 to WPFP)

// CONFIG2
#pragma config POSCMOD = NONE           // Primary Oscillator Select (Primary Oscillator disabled)
#pragma config I2C1SEL = PRI            // I2C1 Pin Select bit (Use default SCL1/SDA1 pins for I2C1 )
#pragma config IOL1WAY = OFF            // IOLOCK One-Way Set Enable (The IOLOCK bit can be set and cleared using the unlock sequence)
#pragma config OSCIOFNC = ON            // OSCO Pin Configuration (OSCO pin functions as port I/O (RA3))
#pragma config FCKSM = CSECME           // Clock Switching and Fail-Safe Clock Monitor (Sw Enabled, Mon Enabled)
#pragma config FNOSC = FRCPLL           // Initial Oscillator Select (Fast RC Oscillator with Postscaler and PLL module (FRCPLL))
#pragma config PLL96MHZ = ON            // 96MHz PLL Startup Select (96 MHz PLL Startup is enabled automatically on start-up)
#pragma config PLLDIV = NODIV           // USB 96 MHz PLL Prescaler Select (Oscillator input divided by 1 (4 MHz input))
#pragma config IESO = OFF               // Internal External Switchover (IESO mode (Two-Speed Start-up) disabled)

// CONFIG1
#pragma config WDTPS = PS8192           // Watchdog Timer Postscaler (1:8,192)
#pragma config FWPSA = PR32             // WDT Prescaler (Prescaler ratio of 1:32)
#pragma config WINDIS = OFF             // Windowed WDT (Standard Watchdog Timer enabled,(Windowed-mode is disabled))
#pragma config FWDTEN = OFF             // Watchdog Timer (Watchdog Timer is disabled)
#pragma config ICS = PGx2               // Emulator Pin Placement Select bits (Emulator functions are shared with PGEC2/PGED2)
#pragma config GWRP = OFF               // General Segment Write Protect (Writes to program memory are allowed)
#pragma config GCP = OFF                // General Segment Code Protect (Code protection is disabled)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG port is disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

/** I N C L U D E S **********************************************************/
#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "usb_config.h"
#include "USB/usb_device.h"
#include "USB/usb.h"
#include "PicProg/upp.h"

//UsbPicProg Includes
#include "PicProg/device.h"
#include "PicProg/prog_lolvl.h"
#include "PicProg/svn_revision.h"

#include "HardwareProfile.h"

/** V A R I A B L E S ********************************************************/
unsigned char output_buffer[CDC_DATA_OUT_EP_SIZE];
unsigned char input_buffer[CDC_DATA_IN_EP_SIZE];

extern unsigned char ConfigDisableVDD;
extern unsigned char ConfigLimitVPP;
extern unsigned char ConfigLimitPGDPGC;

USB_HANDLE lastTransmission;

//Interrupt variables
long timerCnt;
unsigned char timerRunning;

char upp_version[] = "usbpicprog 1.0.0";

/** P R I V A T E  P R O T O T Y P E S ***************************************/
static void InitializeSystem(void);
void ProcessIO(void);
void USBDeviceTasks(void);
void USBCBSendResume(void);
void BlinkUSBStatus(void);
void setLeds( char n );
void UserInit(void);
void InitAdc(void);
void ReadAdc(unsigned char* data);
void timer1Init(void);
void timer3Init(void);
unsigned char set_pictype(unsigned char pt);


/** VECTOR REMAPPING ***********************************************/
    #if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)
        /*
         *	ISR JUMP TABLE
         *
         *	It is necessary to define jump table as a function because C30 will
         *	not store 24-bit wide values in program memory as variables.
         *
         *	This function should be stored at an address where the goto instructions 
         *	line up with the remapped vectors from the bootloader's linker script.
         *  
         *  For more information about how to remap the interrupt vectors,
         *  please refer to AN1157.  An example is provided below for the T2
         *  interrupt with a bootloader ending at address 0x1400
         */
//        void __attribute__ ((address(0x1404))) ISRTable(){
//        
//        	asm("reset"); //reset instruction to prevent runaway code
//        	asm("goto %0"::"i"(&_T2Interrupt));  //T2Interrupt's address
//        }

    #endif
    
    void __attribute__((__interrupt__, __auto_psv__)) _T3Interrupt(void)
    {    
    	if( timerRunning && --timerCnt <= 0 )
            timerRunning = 0;
        
        PR3 = 16000;

        IFS0bits.T3IF = 0;
    }
 
/******************************************************************************
 * Function:        void main(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Main program entry point.
 *
 * Note:            None
 *****************************************************************************/
int main(void)
{   
    InitializeSystem();

    while(1)
    {
        #if defined(USB_POLLING)
		// Check bus status and service USB interrupts.
        USBDeviceTasks(); // Interrupt or polling method.  If using polling, must call
        				  // this function periodically.  This function will take care
        				  // of processing and responding to SETUP transactions 
        				  // (such as during the enumeration process when you first
        				  // plug in).  USB hosts require that USB devices should accept
        				  // and process SETUP packets in a timely fashion.  Therefore,
        				  // when using polling, this function should be called 
        				  // regularly (such as once every 1.8ms or faster** [see 
        				  // inline code comments in usb_device.c for explanation when
        				  // "or faster" applies])  In most cases, the USBDeviceTasks() 
        				  // function does not take very long to execute (ex: <100 
        				  // instruction cycles) before it returns.
        #endif
    				  

		// Application-specific tasks.
		// Application related code may be added here, or in the ProcessIO() function.
        ProcessIO();        
    }//end while
}//end main


/********************************************************************
 * Function:        static void InitializeSystem(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        InitializeSystem is a centralize initialization
 *                  routine. All required USB initialization routines
 *                  are called from here.
 *
 *                  User application initialization routine should
 *                  also be called from here.                  
 *
 * Note:            None
 *******************************************************************/
static void InitializeSystem(void)
{
    AD1PCFGL = 0xFFFF; //Turn off analogue functions on all pins;

    //On the PIC24FJ64GB004 Family of USB microcontrollers, the PLL will not power up and be enabled
    //by default, even if a PLL enabled oscillator configuration is selected (such as HS+PLL).
	//This allows the device to power up at a lower initial operating frequency, which can be
	//advantageous when powered from a source which is not gauranteed to be adequate for 32MHz
	//operation.  On these devices, user firmware needs to manually set the CLKDIV<PLLEN> bit to
	//power up the PLL.
    {
        unsigned int pll_startup_counter = 600;
        CLKDIVbits.PLLEN = 1;
        while(pll_startup_counter--);
    }
    
    //Device switches over automatically to PLL output after PLL is locked and ready.

//	The USB specifications require that USB peripheral devices must never source
//	current onto the Vbus pin.  Additionally, USB peripherals should not source
//	current on D+ or D- when the host/hub is not actively powering the Vbus line.
//	When designing a self powered (as opposed to bus powered) USB peripheral
//	device, the firmware should make sure not to turn on the USB module and D+
//	or D- pull up resistor unless Vbus is actively powered.  Therefore, the
//	firmware needs some means to detect when Vbus is being powered by the host.
//	A 5V tolerant I/O pin can be connected to Vbus (through a resistor), and
// 	can be used to detect when Vbus is high (host actively powering), or low
//	(host is shut down or otherwise not supplying power).  The USB firmware
// 	can then periodically poll this I/O pin to know when it is okay to turn on
//	the USB module/D+/D- pull up resistor.  When designing a purely bus powered
//	peripheral device, it is not possible to source current on D+ or D- when the
//	host is not actively providing power on Vbus. Therefore, implementing this
//	bus sense feature is optional.  This firmware can be made to use this bus
//	sense feature by making sure "USE_USB_BUS_SENSE_IO" has been defined in the
//	HardwareProfile.h file.    
    #if defined(USE_USB_BUS_SENSE_IO)
    tris_usb_bus_sense = INPUT_PIN; // See HardwareProfile.h
    #endif
    
//	If the host PC sends a GetStatus (device) request, the firmware must respond
//	and let the host know if the USB peripheral device is currently bus powered
//	or self powered.  See chapter 9 in the official USB specifications for details
//	regarding this request.  If the peripheral device is capable of being both
//	self and bus powered, it should not return a hard coded value for this request.
//	Instead, firmware should check if it is currently self or bus powered, and
//	respond accordingly.  If the hardware has been configured like demonstrated
//	on the PICDEM FS USB Demo Board, an I/O pin can be polled to determine the
//	currently selected power source.  On the PICDEM FS USB Demo Board, "RA2" 
//	is used for	this purpose.  If using this feature, make sure "USE_SELF_POWER_SENSE_IO"
//	has been defined in HardwareProfile - (platform).h, and that an appropriate I/O pin 
//  has been mapped	to it.
    #if defined(USE_SELF_POWER_SENSE_IO)
    tris_self_power = INPUT_PIN;	// See HardwareProfile.h
    #endif
    
    UserInit();

    USBDeviceInit();	//usb_device.c.  Initializes USB module SFRs and firmware
    					//variables to known states.
}//end InitializeSystem



/******************************************************************************
 * Function:        void UserInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine should take care of all of the demo code
 *                  initialization that is required.
 *
 * Note:            
 *
 *****************************************************************************/
void UserInit(void)
{
    unsigned char i;
// 	 Initialize the arrays
	for (i = 0; i < sizeof(output_buffer); i++)
    {
		output_buffer[i] = 0;
    }

	lastTransmission = 0;

	mInitAllLEDs();
    InitAdc();
    
    TRISVPP = 0; //output
    TRISVPP_RST = 0; //output
	TRISPGD = 0;
	TRISPGC = 0;
	TRISVDD = 0;
	TRISVPP_RUN = 0;
	VPP_RUN = 0; //run = off
	PGD_LOW = 0;	// always set to 0 -except for I2C EEproms
	TRISPGD_LOW = 1; //LV devices disabled, high impedance / input
	PGC_LOW = 0;	// always set to 0
	TRISPGC_LOW = 1; //LV devices disabled, high impedance / input
	VPP = 1; //VPP is low (inverted)
	VPP_RST = 0; //No hard reset (inverted
	PGD = 0;
	PGC = 0;

    timer1Init();
    timer3Init();
}//end UserInit

void timer1Init( void ) 
{
    IEC0bits.T1IE = 1; //enable timer1 interrupt
	IFS0bits.T1IF = 0; //clear interrupt flag
	IPC0bits.T1IP0 = 1; //Interrupt priority 1
    T1CON |= 0x0000;    //timer1 off, prescaler 1:1, internal clock
}

void timer3Init( void )
{
	IEC0bits.T3IE = 1; //enable timer3 interrupt
	IFS0bits.T3IF = 0; //clear interrupt flag
	IPC2bits.T3IP0 = 1; //interrupt priority 1
	PR3 = 16000; //1ms PRESET
	T3CON |= 0x8000; //timer3 on, prescaler 1;1, internal clock
}

 /******************************************************************************
 * This function set up timerCnt
 * at the moment timer1 is a 1 ms timer, if it is changes cnt needs to be adjusted
 */
void startTimerMs( unsigned int cnt )
{
	timerRunning = 0; // no surprises from timer interrupt
	if( cnt ) {
		timerCnt = cnt + 1;
		timerRunning = 1;
	}
}

void DelayMs( unsigned cnt )
{
	startTimerMs( cnt );
	while( timerRunning )
		continue;
}

void DelayUs(unsigned cnt) {
    PR1 = cnt * 16; // Number of ticks per microsecond
    IFS0bits.T1IF = 0; // Reset interrupt flag
    T1CONbits.TON = 1;
    while (!IFS0bits.T1IF) // Wait here for timeout
        Nop();
    T1CONbits.TON = 0;
}

void InitAdc( void )
{
    AD1CON1 = 0x00E0; //SSRC = 111 => internal counter ends sampling and starts converting
    AD1CON3 = 0x1F02; //31 Tad, Tad = 3Tcy 
    AD1CHS = 0x0001; //Input on AN1
    AD1PCFG = 0xFFFD; //Pin AN1 to analogue 
    TRISAbits.TRISA1 = 1; //Input;
    AD1CON1bits.ADON = 1;   //Turn on ADC
}

void ReadAdc( unsigned char* data )
{
	AD1CON1bits.SAMP = 1; //start sampling input
    while(!AD1CON1bits.DONE){};
    
    data[0] = ADC1BUF0 & 0xFF;
    data[1] = ADC1BUF0 >> 8;
//	AD1PCFG = 0xFFFF; //back to digital...
}

/******************************************************************************
 * Function:        void mySetLineCodingHandler(void)
 *
 * PreCondition:    USB_CDC_SET_LINE_CODING_HANDLER is defined
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function gets called when a SetLineCoding command
 *                  is sent on the bus.  This function will evaluate the request
 *                  and determine if the application should update the baudrate
 *                  or not.
 *
 * Note:            
 *
 *****************************************************************************/
#if defined(USB_CDC_SET_LINE_CODING_HANDLER)
void mySetLineCodingHandler(void)
{
    //If the request is not in a valid range
    if(cdc_notice.GetLineCoding.dwDTERate.Val > 115200)
    {
        //NOTE: There are two ways that an unsupported baud rate could be
        //handled.  The first is just to ignore the request and don't change
        //the values.  That is what is currently implemented in this function.
        //The second possible method is to stall the STATUS stage of the request.
        //STALLing the STATUS stage will cause an exception to be thrown in the 
        //requesting application.  Some programs, like HyperTerminal, handle the
        //exception properly and give a pop-up box indicating that the request
        //settings are not valid.  Any application that does not handle the
        //exception correctly will likely crash when this requiest fails.  For
        //the sake of example the code required to STALL the status stage of the
        //request is provided below.  It has been left out so that this demo
        //does not cause applications without the required exception handling
        //to crash.
        //---------------------------------------
        //USBStallEndpoint(0,1);
    }
    else
    {
        //Update the baudrate info in the CDC driver
        CDCSetBaudRate(cdc_notice.GetLineCoding.dwDTERate.Val);
    }
}
#endif

/********************************************************************
 * Function:        void ProcessIO(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is a place holder for other user
 *                  routines. It is a mixture of both USB and
 *                  non-USB tasks.
 *
 * Note:            None
 *******************************************************************/
void ProcessIO(void)
{   
    static BYTE counter = 0;
	int nBytes;
	unsigned long address;
	static int isReading = 0;
    
    // User Application USB tasks
    if((USBDeviceState < CONFIGURED_STATE)||(USBSuspendControl==1)) 
    {
        //Blink the LEDs according to the USB device status
        BlinkUSBStatus();
        return;
    }
	
    nBytes = getsUSBUSART((unsigned char*)input_buffer, 64); //until the buffer is free.
    
    if( nBytes == 0 && USBUSARTIsTxTrfReady() && isReading || nBytes > 0 )
	{	
        switch( input_buffer[0] ) 
        {
            case CMD_GET_PROTOCOL_VERSION:
                output_buffer[0] = PROT_UPP;		// see upp.h
                counter = 1;
                break;
                
            case CMD_EXIT_TO_BOOTLOADER:
                //exitToBootloader( 1 );		// if this returns -> bad bootloader version, return error
                for( counter = 0; counter < 10; counter++ )
                {
                    setLeds(7);
                    DelayMs( 200 );
                    setLeds( 0 );
                    DelayMs( 200 );
                }
                output_buffer[0] = 3;
                counter = 1;
                break;
            
            case CMD_ERASE:
                setLeds( LEDS_ON | LEDS_WR );
                output_buffer[0] = bulk_erase( input_buffer[1] );
                counter = 1;
                setLeds( LEDS_ON );
                break;
		
            case CMD_READ_ID:
                setLeds( LEDS_ON | LEDS_RD );
                switch( picfamily ) {
                    case PIC24:
                    case dsPIC30:
                        read_code( 0xFF0000, (unsigned char*) output_buffer, 2, BLOCKTYPE_FIRST|BLOCKTYPE_LAST|BLOCKTYPE_CONFIG );
                        break;
                    case PIC18:
                    case PIC18J:
                    case PIC18K:
                        //devid is at location 0x3ffffe   for PIC18 devices
                        read_code( 0x3FFFFE, (unsigned char*) output_buffer, 2, BLOCKTYPE_FIRST|BLOCKTYPE_LAST|BLOCKTYPE_CONFIG );
                        break;
                    case PIC16:
                        exit_ISCP();
                        //devid is at location 0x2006  for PIC16 devices
                        read_code( 0x2006, (unsigned char*) output_buffer, 2, BLOCKTYPE_FIRST|BLOCKTYPE_LAST|BLOCKTYPE_CONFIG );
                        break;
                }
                counter = 2;
                setLeds( LEDS_ON );
                break;
            
            case CMD_WRITE_CODE:
                setLeds( LEDS_ON | LEDS_WR );
                address = ((unsigned long) input_buffer[2]) << 16 | ((unsigned long) input_buffer[3]) << 8
					| ((unsigned long) input_buffer[4]);
                output_buffer[0] = write_code( address,
					(unsigned char*) (input_buffer + 6), input_buffer[1], input_buffer[5] );
                counter = 1;
                setLeds( LEDS_ON );
                break;
                
            case CMD_READ_CONFIG:
                input_buffer[5] |= BLOCKTYPE_CONFIG;
                // no break
                
            case CMD_READ_CODE_OLD:
                if( input_buffer[1] <= 8 )		// this is correct for all but PIC18F4321(?) where it doesn't matter
                    input_buffer[5] |= BLOCKTYPE_CONFIG;
                
            case CMD_READ_CODE:
                setLeds( LEDS_ON | LEDS_RD );

                address = ((unsigned long) input_buffer[2]) << 16 | ((unsigned long) input_buffer[3]) << 8
                        | ((unsigned long) input_buffer[4]);
                read_code( address, (unsigned char*) output_buffer, input_buffer[1], input_buffer[5] );
                counter = input_buffer[1];
                setLeds( LEDS_ON );
                break;
                
            case CMD_MREAD_CODE:
                setLeds( LEDS_ON | LEDS_RD );

                address = ((unsigned long) input_buffer[2]) << 16 | ((unsigned long) input_buffer[3]) << 8
                        | ((unsigned long) input_buffer[4]);
                read_code( address, (unsigned char*) output_buffer, input_buffer[1], input_buffer[5] );
                if( --input_buffer[7] == 255 )
                    --input_buffer[6];
                if( input_buffer[6] == 0 && input_buffer[7] == 0 )
                    isReading = 0;
                else
                    isReading = 1;
                if( picfamily == PIC10
                 || picfamily == PIC12
                 || picfamily == PIC16 )
                    address += input_buffer[1]/2;
                else
                    address += input_buffer[1];
                input_buffer[4] = address;
                input_buffer[3] = address >> 8;
                input_buffer[2] = address >>16;
                counter = input_buffer[1];
                setLeds( LEDS_ON );
                break;
                
            case CMD_WRITE_DATA:
                setLeds( LEDS_ON | LEDS_WR );
                address = ((unsigned long) input_buffer[2]) << 16 | ((unsigned long) input_buffer[3]) << 8
                        | ((unsigned long) input_buffer[4]);
                output_buffer[0] = write_data( address,
                        (unsigned char*) (input_buffer + 6), input_buffer[1], input_buffer[5] );
                counter = 1;
                setLeds( LEDS_ON );
                break;
                
            case CMD_READ_DATA:
                setLeds( LEDS_ON | LEDS_RD );
                address = ((unsigned long) input_buffer[2]) << 16 | ((unsigned long) input_buffer[3]) << 8
                    	| ((unsigned long) input_buffer[4]);
                read_data( address, (unsigned char*) output_buffer, input_buffer[1],
                    	input_buffer[5] );
                counter = input_buffer[1];
                setLeds( LEDS_ON );
                break;
                
            case CMD_WRITE_CONFIG:
                setLeds( LEDS_ON | LEDS_WR );
                address = ((unsigned long) input_buffer[2]) << 16 | ((unsigned long) input_buffer[3]) << 8
                    	| ((unsigned long) input_buffer[4]);
                output_buffer[0] = write_config_bits( address,
                    	(unsigned char*) (input_buffer + 6), input_buffer[1], input_buffer[5] );
                counter = 1;
                setLeds( LEDS_ON );
                break;
                
            case CMD_SET_PICTYPE:
            {
                int i;

                output_buffer[0] = set_pictype( input_buffer[1] );
                counter = 1;
                setLeds( LEDS_ON );
                break;
            }
            
            case CMD_FIRMWARE_VERSION:
                //strcpypgm2ram((char*)output_buffer,(const far rom char*)upp_version);
                strcpypgm2ram((char*)output_buffer,(const char*)upp_version);
                counter = 18;
                setLeds( LEDS_ON );
                break;
            
            case CMD_DEBUG:
                setLeds( LEDS_ON | LEDS_WR | LEDS_RD );
                switch( input_buffer[1] ) {
                    case 0:
                        set_pictype( dsP30F );
                        enter_ISCP();
                        output_buffer[0] = 1;
                        counter = 1;
                        break;
                    case 1:
                        exit_ISCP();
                        output_buffer[0] = 1;
                        counter = 1;
                        break;
                    case 2:
                        dspic_send_24_bits( ((unsigned long) input_buffer[2])
                                          | ((unsigned long) input_buffer[3]) << 8
                                          | ((unsigned long) input_buffer[4]) << 16 );
                        output_buffer[0] = 1;
                        counter = 1;
                        break;
                    case 3:
                        nBytes = dspic_read_16_bits( 1 );
                        output_buffer[0] = (unsigned char) nBytes;
                        output_buffer[1] = (unsigned char) (nBytes >> 8);
                        counter = 2;
                        break;
                }
                break;
                
            case CMD_GET_PIN_STATUS:
                switch( input_buffer[1] ) {
                    case SUBCMD_PIN_PGC:
                        if( PGC == 0 )
                            output_buffer[0] = PIN_STATE_0V;
                        else if( (!TRISPGC_LOW) ) //3.3V levels
                            output_buffer[0] = PIN_STATE_3_3V;
                        else
                            output_buffer[0] = PIN_STATE_5V;
                    counter = 1;
                    break;
                    
                    case SUBCMD_PIN_PGD:
                        if( PGD == 0 )
                            output_buffer[0] = PIN_STATE_0V;
                        else if( (!TRISPGD_LOW) ) //3.3V levels
                            output_buffer[0] = PIN_STATE_3_3V;
                        else
                            output_buffer[0] = PIN_STATE_5V;
                        counter = 1;
                        break;
                    case SUBCMD_PIN_VDD:
                        if( VDD )
                            output_buffer[0] = PIN_STATE_FLOAT;
                        else
                            output_buffer[0] = PIN_STATE_5V;
                        counter = 1;
                        break;
                    case SUBCMD_PIN_VPP:
                        counter = 1;
                        if( !VPP )
                        {
                            output_buffer[0] = PIN_STATE_12V;
                            break;
                        }
                        if( VPP_RST )
                        {
                            output_buffer[0] = PIN_STATE_0V;
                            break;
                        }
                        if( VPP_RUN )
                        {
                            output_buffer[0] = PIN_STATE_5V;
                            break;
                        }
                        output_buffer[0] = PIN_STATE_FLOAT;
                        break;
                    case SUBCMD_PIN_VPP_VOLTAGE:
                        ReadAdc( output_buffer );
                        counter = 2;
                        break;
                    default:
                        output_buffer[0] = 3;
                        counter = 1;
                        break;
                }
                break;
                
            case CMD_SET_PIN_STATUS:
                switch( input_buffer[1] ) {
                    case SUBCMD_PIN_PGC:
                        switch( input_buffer[2] ) {
                            case PIN_STATE_0V:
                                TRISPGC = 0;
                                PGC = 0;
                                TRISPGC_LOW = 1;
                                PGC_LOW = 0;
                                output_buffer[0] = 1;//ok
                                break;
                            case PIN_STATE_3_3V:
                                TRISPGC = 0;
                                PGC = 1;
                                TRISPGC_LOW = 0;
                                PGC_LOW = 0;
                                output_buffer[0] = 1;//ok
                                break;
                            case PIN_STATE_5V:
                                TRISPGC = 0;
                                PGC = 1;
                                TRISPGC_LOW = 1;
                                PGC_LOW = 0;
                                output_buffer[0] = 1;//ok
                                break;
                            default:
                                output_buffer[0] = 3;
                                break;
                        }
                        break;
                    case SUBCMD_PIN_PGD:
                        switch( input_buffer[2] ) {
                            case PIN_STATE_0V:
                                TRISPGD = 0;
                                PGD = 0;
                                TRISPGD_LOW = 1;
                                PGD_LOW = 0;
                                output_buffer[0] = 1;//ok
                                break;
                            case PIN_STATE_3_3V:
                                TRISPGD = 0;
                                PGD = 1;
                                TRISPGD_LOW = 0;
                                PGD_LOW = 0;
                                output_buffer[0] = 1;//ok
                                break;
                            case PIN_STATE_5V:
                                TRISPGD = 0;
                                PGD = 1;
                                TRISPGD_LOW = 1;
                                PGD_LOW = 0;
                                output_buffer[0] = 1;//ok
                                break;
                            case PIN_STATE_INPUT:
                                TRISPGD_LOW = 1;
                                TRISPGD = 1;
                                output_buffer[0] = 1;//ok
                                break;
                            default:
                                output_buffer[0] = 3;
                                break;
                        }
                        break;
                    case SUBCMD_PIN_VDD:
                        switch( input_buffer[2] ) {
                            case PIN_STATE_5V:
                                VDD = 0;
                                output_buffer[0] = 1;
                                break;
                            case PIN_STATE_FLOAT:
                                VDD = 1;
                                output_buffer[0] = 1;
                                break;
                            default:
                                output_buffer[0] = 3;
                                break;
                        }
                        break;
                    case SUBCMD_PIN_VPP:
                        switch( input_buffer[2] ) {
                            case PIN_STATE_0V:
                                VPP = 1;
                                VPP_RST = 1;
                                VPP_RUN = 0;
                                output_buffer[0] = 1;//ok
                                break;
                            case PIN_STATE_5V:
                                VPP = 1;
                                VPP_RST = 0;
                                VPP_RUN = 1;
                                output_buffer[0] = 1;//ok
                                break;
                            case PIN_STATE_12V:
                                VPP = 0;
                                VPP_RST = 0;
                                VPP_RUN = 0;
                                output_buffer[0] = 1;//ok
                                break;
                            case PIN_STATE_FLOAT:
                                VPP = 1;
                                VPP_RST = 0;
                                VPP_RUN = 0;
                                output_buffer[0] = 1;//ok
                                break;
                            default:
                                output_buffer[0] = 3;
                                break;
                        }
                        break;
                    default:
                        output_buffer[0] = 3;
                }
                counter = 1;
                break;
                
            case CMD_APPLY_SETTINGS:
                if(input_buffer[1]&CONFIG_DISABLE_VDD_MASK)
                    ConfigDisableVDD=1;
                else
                    ConfigDisableVDD=0;
                if(input_buffer[1]&CONFIG_LIMIT_VPP_MASK)
                    ConfigLimitVPP=1;
                else
                	ConfigLimitVPP=0;
                if(input_buffer[1]&CONFIG_LIMIT_PGDPGC_MASK)
                	ConfigLimitPGDPGC=1;
                else
                    ConfigLimitPGDPGC=0;
                output_buffer[0]=1;
                counter=1;
                break;
            
            default:
                output_buffer[0] = 3;			// unrecognized command
                counter = 1;        
        }
	}
   
    //Check if any bytes are waiting in the queue to send to the USB host.
    //If any bytes are waiting, and the endpoint is available, prepare to
    //send the USB packet to the host.
    if (counter != 0) 
    {
        if(mUSBUSARTIsTxTrfReady()) {
            putUSBUSART((unsigned char *) &output_buffer[0], counter);
            counter = 0;
        }
    }
	
    CDCTxService();
}//end ProcessIO

unsigned char set_pictype( unsigned char pt )
{
	unsigned char i;

	pictype = pt;
	for( i = 0; i < UPP_INVALID_PICTYPE; i++ )
	{
		if( devices[i].flags.type == pt )
		{
			currDevice = devices[i];
			break;
		}
	}

	if( i < UPP_INVALID_PICTYPE && currDevice.flags.family != UPP_INVALID_PICFAMILY )
	{
		picfamily = currDevice.flags.family;
		return (1);
	}
	pictype = P18F2XXX;
	for( i = 0; i < UPP_INVALID_PICTYPE; i++ )
	{
		if( devices[i].flags.type == pictype )
		{
			currDevice = devices[i];
			break;
		}
	}
	picfamily = currDevice.flags.family;
	return( 3 );
}

/********************************************************************
 * Function:        void BlinkUSBStatus(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        BlinkUSBStatus turns on and off LEDs 
 *                  corresponding to the USB device state.
 *
 * Note:            mLED macros can be found in HardwareProfile.h
 *                  USBDeviceState is declared and updated in
 *                  usb_device.c.
 *******************************************************************/
void BlinkUSBStatus(void)
{
    static WORD led_count=0;
    static char startup_state = 0xFF;
    
    if(led_count == 0)
        led_count = 10000U;
    led_count--;

    #define mLED_Both_Off()         {mLED_1_Off();mLED_2_Off();}
    #define mLED_Both_On()          {mLED_1_On();mLED_2_On();}
    #define mLED_Only_1_On()        {mLED_1_On();mLED_2_Off();}
    #define mLED_Only_2_On()        {mLED_1_Off();mLED_2_On();}

    if(USBSuspendControl == 1)
    {
        if(led_count==0)
        {
            mLED_1_Off();
            mLED_2_Off();
            mLED_3_Toggle();
        }//end if
    }
    else
    {
        if(USBDeviceState == DETACHED_STATE)
        {
            setLeds( 1 );
        }
        else if(USBDeviceState == ATTACHED_STATE)
        {
            setLeds( 2 );
        }
        else if(USBDeviceState == POWERED_STATE)
        {
            setLeds( 4 );
        }
        else if(USBDeviceState == DEFAULT_STATE)
        {
            setLeds( 2 );
        }
        else if(USBDeviceState == ADDRESS_STATE)
        {
            setLeds( LEDS_ON );
        }
        else if(USBDeviceState == CONFIGURED_STATE)
        {
            startup_state = 0;
        }//end if(...)
    }//end if(UCONbits.SUSPND...)
}//end BlinkUSBStatus

void setLeds( char n )
{
	mLED_1 = ~(n & 1);
	mLED_2 = ~((n & 2) >> 1);
	mLED_3 = ~((n & 4) >> 2);
}

// ******************************************************************************************************
// ************** USB Callback Functions ****************************************************************
// ******************************************************************************************************
// The USB firmware stack will call the callback functions USBCBxxx() in response to certain USB related
// events.  For example, if the host PC is powering down, it will stop sending out Start of Frame (SOF)
// packets to your device.  In response to this, all USB devices are supposed to decrease their power
// consumption from the USB Vbus to <2.5mA* each.  The USB module detects this condition (which according
// to the USB specifications is 3+ms of no bus activity/SOF packets) and then calls the USBCBSuspend()
// function.  You should modify these callback functions to take appropriate actions for each of these
// conditions.  For example, in the USBCBSuspend(), you may wish to add code that will decrease power
// consumption from Vbus to <2.5mA (such as by clock switching, turning off LEDs, putting the
// microcontroller to sleep, etc.).  Then, in the USBCBWakeFromSuspend() function, you may then wish to
// add code that undoes the power saving things done in the USBCBSuspend() function.

// The USBCBSendResume() function is special, in that the USB stack will not automatically call this
// function.  This function is meant to be called from the application firmware instead.  See the
// additional comments near the function.

// Note *: The "usb_20.pdf" specs indicate 500uA or 2.5mA, depending upon device classification. However,
// the USB-IF has officially issued an ECN (engineering change notice) changing this to 2.5mA for all 
// devices.  Make sure to re-download the latest specifications to get all of the newest ECNs.

/******************************************************************************
 * Function:        void USBCBSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Call back that is invoked when a USB suspend is detected
 *
 * Note:            None
 *****************************************************************************/
void USBCBSuspend(void)
{
	//Example power saving code.  Insert appropriate code here for the desired
	//application behavior.  If the microcontroller will be put to sleep, a
	//process similar to that shown below may be used:
	
	//ConfigureIOPinsForLowPower();
	//SaveStateOfAllInterruptEnableBits();
	//DisableAllInterruptEnableBits();
	//EnableOnlyTheInterruptsWhichWillBeUsedToWakeTheMicro();	//should enable at least USBActivityIF as a wake source
	//Sleep();
	//RestoreStateOfAllPreviouslySavedInterruptEnableBits();	//Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.
	//RestoreIOPinsToNormal();									//Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.

	//IMPORTANT NOTE: Do not clear the USBActivityIF (ACTVIF) bit here.  This bit is 
	//cleared inside the usb_device.c file.  Clearing USBActivityIF here will cause 
	//things to not work as intended.	
	

 
}

/******************************************************************************
 * Function:        void USBCBWakeFromSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The host may put USB peripheral devices in low power
 *					suspend mode (by "sending" 3+ms of idle).  Once in suspend
 *					mode, the host may wake the device back up by sending non-
 *					idle state signalling.
 *					
 *					This call back is invoked when a wakeup from USB suspend 
 *					is detected.
 *
 * Note:            None
 *****************************************************************************/
void USBCBWakeFromSuspend(void)
{
	// If clock switching or other power savings measures were taken when
	// executing the USBCBSuspend() function, now would be a good time to
	// switch back to normal full power run mode conditions.  The host allows
	// 10+ milliseconds of wakeup time, after which the device must be 
	// fully back to normal, and capable of receiving and processing USB
	// packets.  In order to do this, the USB module must receive proper
	// clocking (IE: 48MHz clock must be available to SIE for full speed USB
	// operation).  
	// Make sure the selected oscillator settings are consistent with USB 
    // operation before returning from this function.
}

/********************************************************************
 * Function:        void USBCB_SOF_Handler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB host sends out a SOF packet to full-speed
 *                  devices every 1 ms. This interrupt may be useful
 *                  for isochronous pipes. End designers should
 *                  implement callback routine as necessary.
 *
 * Note:            None
 *******************************************************************/
void USBCB_SOF_Handler(void)
{
    // No need to clear UIRbits.SOFIF to 0 here.
    // Callback caller is already doing that.
}

/*******************************************************************
 * Function:        void USBCBErrorHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The purpose of this callback is mainly for
 *                  debugging during development. Check UEIR to see
 *                  which error causes the interrupt.
 *
 * Note:            None
 *******************************************************************/
void USBCBErrorHandler(void)
{
    // No need to clear UEIR to 0 here.
    // Callback caller is already doing that.

	// Typically, user firmware does not need to do anything special
	// if a USB error occurs.  For example, if the host sends an OUT
	// packet to your device, but the packet gets corrupted (ex:
	// because of a bad connection, or the user unplugs the
	// USB cable during the transmission) this will typically set
	// one or more USB error interrupt flags.  Nothing specific
	// needs to be done however, since the SIE will automatically
	// send a "NAK" packet to the host.  In response to this, the
	// host will normally retry to send the packet again, and no
	// data loss occurs.  The system will typically recover
	// automatically, without the need for application firmware
	// intervention.
	
	// Nevertheless, this callback function is provided, such as
	// for debugging purposes.
}


/*******************************************************************
 * Function:        void USBCBCheckOtherReq(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        When SETUP packets arrive from the host, some
 * 					firmware must process the request and respond
 *					appropriately to fulfill the request.  Some of
 *					the SETUP packets will be for standard
 *					USB "chapter 9" (as in, fulfilling chapter 9 of
 *					the official USB specifications) requests, while
 *					others may be specific to the USB device class
 *					that is being implemented.  For example, a HID
 *					class device needs to be able to respond to
 *					"GET REPORT" type of requests.  This
 *					is not a standard USB chapter 9 request, and 
 *					therefore not handled by usb_device.c.  Instead
 *					this request should be handled by class specific 
 *					firmware, such as that contained in usb_function_hid.c.
 *
 * Note:            None
 *******************************************************************/
void USBCBCheckOtherReq(void)
{
    USBCheckCDCRequest();
}//end


/*******************************************************************
 * Function:        void USBCBStdSetDscHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USBCBStdSetDscHandler() callback function is
 *					called when a SETUP, bRequest: SET_DESCRIPTOR request
 *					arrives.  Typically SET_DESCRIPTOR requests are
 *					not used in most applications, and it is
 *					optional to support this type of request.
 *
 * Note:            None
 *******************************************************************/
void USBCBStdSetDscHandler(void)
{
    // Must claim session ownership if supporting this request
}//end


/*******************************************************************
 * Function:        void USBCBInitEP(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called when the device becomes
 *                  initialized, which occurs after the host sends a
 * 					SET_CONFIGURATION (wValue not = 0) request.  This 
 *					callback function should initialize the endpoints 
 *					for the device's usage according to the current 
 *					configuration.
 *
 * Note:            None
 *******************************************************************/
void USBCBInitEP(void)
{
    //Enable the CDC data endpoints
    CDCInitEP();
}

/********************************************************************
 * Function:        void USBCBSendResume(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB specifications allow some types of USB
 * 					peripheral devices to wake up a host PC (such
 *					as if it is in a low power suspend to RAM state).
 *					This can be a very useful feature in some
 *					USB applications, such as an Infrared remote
 *					control	receiver.  If a user presses the "power"
 *					button on a remote control, it is nice that the
 *					IR receiver can detect this signalling, and then
 *					send a USB "command" to the PC to wake up.
 *					
 *					The USBCBSendResume() "callback" function is used
 *					to send this special USB signalling which wakes 
 *					up the PC.  This function may be called by
 *					application firmware to wake up the PC.  This
 *					function will only be able to wake up the host if
 *                  all of the below are true:
 *					
 *					1.  The USB driver used on the host PC supports
 *						the remote wakeup capability.
 *					2.  The USB configuration descriptor indicates
 *						the device is remote wakeup capable in the
 *						bmAttributes field.
 *					3.  The USB host PC is currently sleeping,
 *						and has previously sent your device a SET 
 *						FEATURE setup packet which "armed" the
 *						remote wakeup capability.   
 *
 *                  If the host has not armed the device to perform remote wakeup,
 *                  then this function will return without actually performing a
 *                  remote wakeup sequence.  This is the required behavior, 
 *                  as a USB device that has not been armed to perform remote 
 *                  wakeup must not drive remote wakeup signalling onto the bus;
 *                  doing so will cause USB compliance testing failure.
 *                  
 *					This callback should send a RESUME signal that
 *                  has the period of 1-15ms.
 *
 * Note:            This function does nothing and returns quickly, if the USB
 *                  bus and host are not in a suspended condition, or are 
 *                  otherwise not in a remote wakeup ready state.  Therefore, it
 *                  is safe to optionally call this function regularly, ex: 
 *                  anytime application stimulus occurs, as the function will
 *                  have no effect, until the bus really is in a state ready
 *                  to accept remote wakeup. 
 *
 *                  When this function executes, it may perform clock switching,
 *                  depending upon the application specific code in 
 *                  USBCBWakeFromSuspend().  This is needed, since the USB
 *                  bus will no longer be suspended by the time this function
 *                  returns.  Therefore, the USB module will need to be ready
 *                  to receive traffic from the host.
 *
 *                  The modifiable section in this routine may be changed
 *                  to meet the application needs. Current implementation
 *                  temporary blocks other functions from executing for a
 *                  period of ~3-15 ms depending on the core frequency.
 *
 *                  According to USB 2.0 specification section 7.1.7.7,
 *                  "The remote wakeup device must hold the resume signaling
 *                  for at least 1 ms but for no more than 15 ms."
 *                  The idea here is to use a delay counter loop, using a
 *                  common value that would work over a wide range of core
 *                  frequencies.
 *                  That value selected is 1800. See table below:
 *                  ==========================================================
 *                  Core Freq(MHz)      MIP         RESUME Signal Period (ms)
 *                  ==========================================================
 *                      48              12          1.05
 *                       4              1           12.6
 *                  ==========================================================
 *                  * These timing could be incorrect when using code
 *                    optimization or extended instruction mode,
 *                    or when having other interrupts enabled.
 *                    Make sure to verify using the MPLAB SIM's Stopwatch
 *                    and verify the actual signal on an oscilloscope.
 *******************************************************************/
void USBCBSendResume(void)
{
    static WORD delay_count;
    
    //First verify that the host has armed us to perform remote wakeup.
    //It does this by sending a SET_FEATURE request to enable remote wakeup,
    //usually just before the host goes to standby mode (note: it will only
    //send this SET_FEATURE request if the configuration descriptor declares
    //the device as remote wakeup capable, AND, if the feature is enabled
    //on the host (ex: on Windows based hosts, in the device manager 
    //properties page for the USB device, power management tab, the 
    //"Allow this device to bring the computer out of standby." checkbox 
    //should be checked).
    if(USBGetRemoteWakeupStatus() == TRUE) 
    {
        //Verify that the USB bus is in fact suspended, before we send
        //remote wakeup signalling.
        if(USBIsBusSuspended() == TRUE)
        {
            USBMaskInterrupts();
            
            //Clock switch to settings consistent with normal USB operation.
            USBCBWakeFromSuspend();
            USBSuspendControl = 0; 
            USBBusIsSuspended = FALSE;  //So we don't execute this code again, 
                                        //until a new suspend condition is detected.

            //Section 7.1.7.7 of the USB 2.0 specifications indicates a USB
            //device must continuously see 5ms+ of idle on the bus, before it sends
            //remote wakeup signalling.  One way to be certain that this parameter
            //gets met, is to add a 2ms+ blocking delay here (2ms plus at 
            //least 3ms from bus idle to USBIsBusSuspended() == TRUE, yeilds
            //5ms+ total delay since start of idle).
            delay_count = 3600U;        
            do
            {
                delay_count--;
            }while(delay_count);
            
            //Now drive the resume K-state signalling onto the USB bus.
            USBResumeControl = 1;       // Start RESUME signaling
            delay_count = 1800U;        // Set RESUME line for 1-13 ms
            do
            {
                delay_count--;
            }while(delay_count);
            USBResumeControl = 0;       //Finished driving resume signalling

            USBUnmaskInterrupts();
        }
    }
}


/*******************************************************************
 * Function:        void USBCBEP0DataReceived(void)
 *
 * PreCondition:    ENABLE_EP0_DATA_RECEIVED_CALLBACK must be
 *                  defined already (in usb_config.h)
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called whenever a EP0 data
 *                  packet is received.  This gives the user (and
 *                  thus the various class examples a way to get
 *                  data that is received via the control endpoint.
 *                  This function needs to be used in conjunction
 *                  with the USBCBCheckOtherReq() function since 
 *                  the USBCBCheckOtherReq() function is the apps
 *                  method for getting the initial control transfer
 *                  before the data arrives.
 *
 * Note:            None
 *******************************************************************/
#if defined(ENABLE_EP0_DATA_RECEIVED_CALLBACK)
void USBCBEP0DataReceived(void)
{
}
#endif

/*******************************************************************
 * Function:        BOOL USER_USB_CALLBACK_EVENT_HANDLER(
 *                        USB_EVENT event, void *pdata, WORD size)
 *
 * PreCondition:    None
 *
 * Input:           USB_EVENT event - the type of event
 *                  void *pdata - pointer to the event data
 *                  WORD size - size of the event data
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called from the USB stack to
 *                  notify a user application that a USB event
 *                  occured.  This callback is in interrupt context
 *                  when the USB_INTERRUPT option is selected.
 *
 * Note:            None
 *******************************************************************/
BOOL USER_USB_CALLBACK_EVENT_HANDLER(int event, void *pdata, WORD size)
{
    switch(event)
    {
        case EVENT_TRANSFER:
            //Add application specific callback task or callback function here if desired.
            break;
        case EVENT_SOF:
            USBCB_SOF_Handler();
            break;
        case EVENT_SUSPEND:
            USBCBSuspend();
            break;
        case EVENT_RESUME:
            USBCBWakeFromSuspend();
            break;
        case EVENT_CONFIGURED: 
            USBCBInitEP();
            break;
        case EVENT_SET_DESCRIPTOR:
            USBCBStdSetDscHandler();
            break;
        case EVENT_EP0_REQUEST:
            USBCBCheckOtherReq();
            break;
        case EVENT_BUS_ERROR:
            USBCBErrorHandler();
            break;
        case EVENT_TRANSFER_TERMINATED:
            //Add application specific callback task or callback function here if desired.
            //The EVENT_TRANSFER_TERMINATED event occurs when the host performs a CLEAR
            //FEATURE (endpoint halt) request on an application endpoint which was 
            //previously armed (UOWN was = 1).  Here would be a good place to:
            //1.  Determine which endpoint the transaction that just got terminated was 
            //      on, by checking the handle value in the *pdata.
            //2.  Re-arm the endpoint if desired (typically would be the case for OUT 
            //      endpoints).
            break;
        default:
            break;
    }      
    return TRUE; 
}

/** EOF main.c *************************************************/
