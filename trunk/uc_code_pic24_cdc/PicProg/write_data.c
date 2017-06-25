/**************************************************************************
 *   Copyright (C) 2008 by Frans Schreuder                                 *
 *   usbpicprog.sourceforge.net                                            *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 **************************************************************************/

#include "PicProg/upp.h"
#include "PicProg/write_data.h"
#include <p24Fxxxx.h>
#include "GenericTypeDefs.h"
#include "HardwareProfile.h"
#include "PicProg/prog.h"
#include "PicProg/prog_lolvl.h"


extern PICFAMILY picfamily;
extern PICTYPE pictype;

char write_data( unsigned long address, unsigned char* data, char blocksize, char lastblock )
{
	if( lastblock & BLOCKTYPE_FIRST )
	{
		if(blocksize == 0) return 1; //ok, nothing to write, empty data
		enter_ISCP();
	}
	
    if( currDevice.write_data )
		currDevice.write_data( address, data, blocksize, lastblock );
	else
	{
		exit_ISCP();
		return 3; //unknown pic type
	}
	
    if( lastblock & BLOCKTYPE_LAST )
	{
		exit_ISCP();
		return 1; //ok
	}
	else
	{
		return 2; //ask for next block
	}
}

void write_data_dsPIC30( unsigned long address, unsigned char* data, char blocksize, char lastblock )
{
	char blockcounter, i;

	//Step 1: Exit the Reset vector.
	dspic_send_24_bits( 0x000000 ); //NOP
	dspic_send_24_bits( 0x000000 ); //NOP
	dspic_send_24_bits( 0x040100 ); //GOTO 0x100
	dspic_send_24_bits( 0x040100 ); //GOTO 0x100
	dspic_send_24_bits( 0x000000 ); //NOP
	//Step 2: Set the NVMCON to write 16 data words.
	dspic_send_24_bits( 0x24005A ); //MOV #0x4005, W10
	dspic_send_24_bits( 0x883B0A ); //MOV W10, NVMCON

	for( blockcounter = 0; blockcounter < blocksize; blockcounter += 8 )
	{
		//Step 3: Initialize the write pointer (W7) for TBLWT instruction.
		//dspic_send_24_bits(0x200000|((blockcounter+address&0xFF0000)>>12));   //MOV #0x7F, W0
		dspic_send_24_bits( 0x2007F0 );//|((blockcounter+address&0xFF0000)>>12)); //MOV #0x7F, W0
		dspic_send_24_bits( 0x880190 ); //MOV W0, TBLPAG
		dspic_send_24_bits( 0x2F0007 | ((((unsigned long) blockcounter + address) & 0x0FFF) << 4) ); //MOV #<DestinationAddress15:0>, W7
		//Step 4: Load W0:W3 with the next 4 data words to program.
		dspic_send_24_bits( 0x200000 | (((unsigned long) data[blockcounter]) << 4)
				| (((unsigned long) data[blockcounter + 1]) << 12) );
		dspic_send_24_bits( 0x200001 | (((unsigned long) data[blockcounter + 2]) << 4)
				| (((unsigned long) data[blockcounter + 3]) << 12) );
		dspic_send_24_bits( 0x200002 | (((unsigned long) data[blockcounter + 4]) << 4)
				| (((unsigned long) data[blockcounter + 5]) << 12) );
		dspic_send_24_bits( 0x200003 | (((unsigned long) data[blockcounter + 6]) << 4)
				| (((unsigned long) data[blockcounter + 7]) << 12) );
		/*for(i=0;i<4;i++)
		 {
		 dspic_send_24_bits(0x200000|
		 (((unsigned long)data[blockcounter+(i*2)])<<4)|
		 (((unsigned long)data[blockcounter+(i*2)+1])<<12)|
		 ((unsigned long) i));
		 }*/
		//Step 5: Set the read pointer (W6) and load the (next set of) write latches.
		dspic_send_24_bits( 0xEB0300 ); //CLR W6
		dspic_send_24_bits( 0x000000 ); //NOP
		
        for( i = 0; i < 4; i++ )
		{
			dspic_send_24_bits( 0xBB1BB6 ); //TBLWTL [W6++], [W7++]
			dspic_send_24_bits( 0x000000 ); //NOP
			dspic_send_24_bits( 0x000000 ); //NOP
		}
	}//Step 6: Repeat steps 3-4 four times to load the write latches for 16 data words.
	//Step 7: Unlock the NVMCON for writing.
	dspic_send_24_bits( 0x200558 ); //MOV #0x55, W8
	dspic_send_24_bits( 0x883B38 ); //MOV W8, NVMKEY
	dspic_send_24_bits( 0x200AA9 ); //MOV #0xAA, W9
	dspic_send_24_bits( 0x883B39 ); //MOV W9, NVMKEY
	//Step 8: Initiate the write cycle.
	dspic_send_24_bits( 0xA8E761 ); //BSET NVMCON, #WR
	dspic_send_24_bits( 0x000000 ); //NOP
	dspic_send_24_bits( 0x000000 ); //NOP
	DelayMs( 2 ); //Externally time 2 msec
	dspic_send_24_bits( 0x000000 ); //NOP
	dspic_send_24_bits( 0x000000 ); //NOP
	dspic_send_24_bits( 0xA9E761 ); //BCLR NVMCON, #WR
	dspic_send_24_bits( 0x000000 ); //NOP
	dspic_send_24_bits( 0x000000 ); //NOP
	//Step 9: Reset device internal PC.
	dspic_send_24_bits( 0x040100 ); //GOTO 0x100
	dspic_send_24_bits( 0x000000 ); //NOP
}
void write_data_P24KA1( unsigned long address, unsigned char* data, char blocksize, char lastblock )
{
	write_config_bits_PIC24( address+0x7FFE00, data, blocksize, 0x4004 );
}

void write_data_P18FXX2( unsigned long address, unsigned char* data, char blocksize, char lastblock )
{
	char blockcounter;
	char receiveddata;

	pic_send( 4, 0x00, 0x9EA6 ); //BCF EECON1, EEPGD
	pic_send( 4, 0x00, 0x9CA6 ); //BCF EECON1, CFGS
	
    for( blockcounter = 0; blockcounter < blocksize; blockcounter++ )
	{
		pic_send( 4, 0x00, (0x0E00 | (address + (unsigned int) blockcounter) & 0xFF) ); //MOVLW Addr [7:0]
		pic_send( 4, 0x00, 0x6EA9 ); //MOVWF EEADR
		pic_send( 4, 0x00, (0x0E00 | ((address + (unsigned int) blockcounter) >> 8) & 0xFF) ); //MOVLW Addr [15:8]
		pic_send( 4, 0x00, 0x6EAA ); //MOVWF EEADRH
		pic_send( 4, 0x00, 0x0E00 | (unsigned int) data[blockcounter] ); //MOVLW data
		pic_send( 4, 0x00, 0x6eA8 ); //MOVWF EEDATA
		pic_send( 4, 0x00, 0x84A6 ); //BSF EECON1, WREN
		//Step 5: Perform required sequence.
		pic_send( 4, 0, 0x0E55 ); //MOVLW 0X55
		pic_send( 4, 0, 0x6EA7 ); //MOVWF EECON2
		pic_send( 4, 0, 0x0EAA ); //MOVLW 0XAA
		pic_send( 4, 0, 0x6EA7 ); //MOVWF EECON2
		pic_send( 4, 0x00, 0x82A6 ); //BSF EECON1, WR
		//pic_send(4,0x00,0x0000); //NOP, when not polling for the WR bit, the PIC still needs at least 4 clocks
		startTimerMs( P11A );
		do
		{
			pic_send( 4, 0x00, 0x50A6 ); //movf EECON1, W, 0
			pic_send( 4, 0x00, 0x6EF5 ); //movwf TABLAT
			pic_send( 4, 0x00, 0x0000 ); //nop
			receiveddata = pic_read_byte2( 4, 0x02 ); //Shift TABLAT register out
		} while( ((receiveddata & 0x02) == 0x02) && timerRunning ); //poll for WR bit to clear
		//PGClow();    //hold PGC low for P10 (100us)
		DelayMs( P10 );
		pic_send( 4, 0x00, 0x94A6 ); //BCF EECON1, WREN
	}
}
void write_data_P18F4XK22( unsigned long address, unsigned char* data, char blocksize, char lastblock )
{
	char blockcounter;
	char receiveddata;

	pic_send( 4, 0x00, 0x9EA6 ); //BCF EECON1, EEPGD
	pic_send( 4, 0x00, 0x9CA6 ); //BCF EECON1, CFGS
	
    for( blockcounter = 0; blockcounter < blocksize; blockcounter++ )
	{
		pic_send( 4, 0x00, (0x0E00 | (address + (unsigned int) blockcounter) & 0xFF) ); //MOVLW Addr [7:0]
		pic_send( 4, 0x00, 0x6EA9 ); //MOVWF EEADR
		pic_send( 4, 0x00, (0x0E00 | ((address + (unsigned int) blockcounter) >> 8) & 0xFF) ); //MOVLW Addr [15:8]
		pic_send( 4, 0x00, 0x6EAA ); //MOVWF EEADRH
		pic_send( 4, 0x00, 0x0E00 | (unsigned int) data[blockcounter] ); //MOVLW data
		pic_send( 4, 0x00, 0x6eA8 ); //MOVWF EEDATA
		pic_send( 4, 0x00, 0x84A6 ); //BSF EECON1, WREN
		pic_send( 4, 0x00, 0x82A6 ); //BSF EECON1, WR
		pic_send( 4, 0x00, 0x0000 ); //NOP, two NOPs required before polling
		pic_send( 4, 0x00, 0x0000 ); //NOP, write starts on 4th clock of this instruction
		//pic_send(4,0x00,0x0000); //NOP, when not polling for the WR bit, the PIC still needs at least 4 clocks
		startTimerMs( P11A );
		
        do
		{
			pic_send( 4, 0x00, 0x50A6 ); //movf EECON1, W, 0
			pic_send( 4, 0x00, 0x6EF5 ); //movwf TABLAT
			pic_send( 4, 0x00, 0x0000 ); //nop
			receiveddata = pic_read_byte2( 4, 0x02 ); //Shift TABLAT register out
		} while( ((receiveddata & 0x02) == 0x02) && timerRunning ); //poll for WR bit to clear
		
        //PGClow();    //hold PGC low for P10 (100us)
		DelayMs( P10 );
		pic_send( 4, 0x00, 0x94A6 ); //BCF EECON1, WREN
	}
}

void write_data_PIC18( unsigned long address, unsigned char* data, char blocksize, char lastblock )
{
	char blockcounter;
	char receiveddata;

	pic_send( 4, 0x00, 0x9EA6 ); //BCF EECON1, EEPGD
	pic_send( 4, 0x00, 0x9CA6 ); //BCF EECON1, CFGS
	
    for( blockcounter = 0; blockcounter < blocksize; blockcounter++ )
	{
		pic_send( 4, 0x00, (0x0E00 | (address + (unsigned int) blockcounter) & 0xFF) ); //MOVLW Addr [7:0]
		pic_send( 4, 0x00, 0x6EA9 ); //MOVWF EEADR
		pic_send( 4, 0x00, (0x0E00 | ((address + (unsigned int) blockcounter) >> 8) & 0xFF) ); //MOVLW Addr [15:8]
		pic_send( 4, 0x00, 0x6EAA ); //MOVWF EEADRH
		pic_send( 4, 0x00, 0x0E00 | (unsigned int) data[blockcounter] ); //MOVLW data
		pic_send( 4, 0x00, 0x6eA8 ); //MOVWF EEDATA
		pic_send( 4, 0x00, 0x84A6 ); //BSF EECON1, WREN
		pic_send( 4, 0x00, 0x82A6 ); //BSF EECON1, WR
		//pic_send(4,0x00,0x0000); //NOP, when not polling for the WR bit, the PIC still needs at least 4 clocks
		startTimerMs( P11A );
		
        do
		{
			pic_send( 4, 0x00, 0x50A6 ); //movf EECON1, W, 0
			pic_send( 4, 0x00, 0x6EF5 ); //movwf TABLAT
			pic_send( 4, 0x00, 0x0000 ); //nop
			receiveddata = pic_read_byte2( 4, 0x02 ); //Shift TABLAT register out
		} while( ((receiveddata & 0x02) == 0x02) && timerRunning ); //poll for WR bit to clear
		//PGClow();    //hold PGC low for P10 (100us)
		DelayMs( P10 );
		pic_send( 4, 0x00, 0x94A6 ); //BCF EECON1, WREN
	}
}

void write_data_PIC18K( unsigned long address, unsigned char* data, char blocksize, char lastblock )
{
	char blockcounter;
	char receiveddata;

	pic_send( 4, 0x00, 0x9E7F ); //BCF EECON1, EEPGD
	pic_send( 4, 0x00, 0x9C7F ); //BCF EECON1, CFGS
	
    for( blockcounter = 0; blockcounter < blocksize; blockcounter++ )
	{
		pic_send( 4, 0x00, (0x0E00 | (address + (unsigned int) blockcounter) & 0xFF) ); //MOVLW Addr [7:0]
		pic_send( 4, 0x00, 0x6E62 ); //MOVWF EEADR
		pic_send( 4, 0x00, (0x0E00 | ((address + (unsigned int) blockcounter) >> 8) & 0xFF) ); //MOVLW Addr [15:8]
		pic_send( 4, 0x00, 0x6E63 ); //MOVWF EEADRH
		pic_send( 4, 0x00, 0x0E00 | (unsigned int) data[blockcounter] ); //MOVLW data
		pic_send( 4, 0x00, 0x6e61 ); //MOVWF EEDATA
		pic_send( 4, 0x00, 0x847F ); //BSF EECON1, WREN
		pic_send( 4, 0x00, 0x827F ); //BSF EECON1, WR

		//pic_send(4,0x00,0x0000); //NOP, when not polling for the WR bit, the PIC still needs at least 4 clocks
		startTimerMs( P11A );
		do
		{
			pic_send( 4, 0x00, 0x507F ); //movf EECON1, W, 0
			pic_send( 4, 0x00, 0x6EF5 ); //movwf TABLAT
			pic_send( 4, 0x00, 0x0000 ); //nop
			receiveddata = pic_read_byte2( 4, 0x02 ); //Shift TABLAT register out
		} while( ((receiveddata & 0x02) == 0x02) && timerRunning ); //poll for WR bit to clear
		//PGClow();	//hold PGC low for P10 (100us)
		DelayMs( P10 );
		pic_send( 4, 0x00, 0x947F ); //BCF EECON1, WREN
	}
}

void write_data_P16F785( unsigned long address, unsigned char* data, char blocksize, char lastblock )
{

	char blockcounter;

	if( (lastblock & BLOCKTYPE_FIRST) && (address > 0) )
	{
		set_address_P16( address ); //set the initial address
	}
	
    for( blockcounter = 0; blockcounter < blocksize; blockcounter++ )
	{
		//load data
		pic_send_14_bits( 6, 0x03, ((unsigned int) data[blockcounter]) );//LSB only
		//begin programming command
		pic_send_n_bits( 6, 0x18 );// begin programming only cycle
		//wait Tprog
		// 8MS, should be 20 ms for other?
		DelayMs( Tdprog );
		pic_send_n_bits( 6, 0x0A );//end programming

		//read data from data memory (to verify) not yet impl...
		//increment address
		pic_send_n_bits( 6, 0x06 );
	}
}

void write_data_P16F87( unsigned long address, unsigned char* data, char blocksize, char lastblock )
{

	char blockcounter;

	if( (lastblock & BLOCKTYPE_FIRST) && (address > 0) )
	{
		set_address_P16( address ); //set the initial address
	}
	
    for( blockcounter = 0; blockcounter < blocksize; blockcounter++ )
	{
		//load data
		pic_send_14_bits( 6, 0x03, ((unsigned int) data[blockcounter]) );//LSB only
		//begin programming command
		pic_send_n_bits( 6, 0x18 );// begin programming only cycle
		//wait Tprog
		// 8MS, should be 20 ms for other?
		DelayMs( Tdprog );
		pic_send_n_bits( 6, 0x17 );//end programming

		//read data from data memory (to verify) not yet impl...
		//increment address
		pic_send_n_bits( 6, 0x06 );
	}
}
void write_data_P16F84A( unsigned long address, unsigned char* data, char blocksize, char lastblock )
{
	char blockcounter;

    if( (lastblock & BLOCKTYPE_FIRST) && (address > 0) )
	{
		set_address_P16( address ); //set the initial address
	}
	
    for( blockcounter = 0; blockcounter < blocksize; blockcounter++ )
	{
		//load data
		pic_send_14_bits( 6, 0x03, ((unsigned int) data[blockcounter]) );//LSB only
		//begin programming command
		pic_send_n_bits( 6, 0x18 );// begin programming only cycle
		//wait Tprog
		// 8MS, should be 20 ms for other?
		DelayMs( Tdprog );
		//read data from data memory (to verify) not yet impl...
		//increment address
		pic_send_n_bits( 6, 0x06 );
	}
}
void write_data_P16F18XX( unsigned long address, unsigned char* data, char blocksize, char lastblock )
{
	char blockcounter;
	
    if( (lastblock & BLOCKTYPE_FIRST) && (address > 0) )
	{
		pic_send_n_bits(6, 0x16); //reset address
		set_address_P16( address ); //set the initial address
	}
	
    for( blockcounter = 0; blockcounter < blocksize; blockcounter++ )
	{
		//load data
		pic_send_14_bits( 6, 0x03, ((unsigned int) data[blockcounter]) );//LSB only
		//begin programming command
		pic_send_n_bits( 6, 0x08 );// begin programming only cycle
		//wait Tprog
		// 8MS, should be 20 ms for other?
		DelayMs( Tdprog );
		//read data from data memory (to verify) not yet impl...
		//increment address
		pic_send_n_bits( 6, 0x06 );
	}
}
void write_data_PIC16( unsigned long address, unsigned char* data, char blocksize, char lastblock )
{
	char blockcounter;
	
    if( (lastblock & BLOCKTYPE_FIRST) && (address > 0) )
	{
		set_address_P16( address ); //set the initial address
	}
	
    for( blockcounter = 0; blockcounter < blocksize; blockcounter++ )
	{
		//load data
		pic_send_14_bits( 6, 0x03, ((unsigned int) data[blockcounter]) );//LSB only
		//begin programming command
		pic_send_n_bits( 6, 0x08 );// begin programming only cycle
		//wait Tprog
		// 8MS, should be 20 ms for other?
		DelayMs( Tdprog );
		//read data from data memory (to verify) not yet impl...
		//increment address
		pic_send_n_bits( 6, 0x06 );
	}
}
