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
#include "PicProg/write_code.h"
#include <p24Fxxxx.h>
#include "GenericTypeDefs.h"
#include "HardwareProfile.h"
#include "PicProg/prog.h"
#include "PicProg/prog_lolvl.h"
extern unsigned char ConfigLimitPGDPGC;
/**
 address points to the first byte of the block
 data contains the data MSB0, LSB0, MSB1, LSB1, etc...
 blocksize is the block syze in BYTES
 */
char write_code( unsigned long address, unsigned char* data, char blocksize, char lastblock )
{

	if( lastblock & BLOCKTYPE_FIRST )
		enter_ISCP();
	if( currDevice.write_code )
		currDevice.write_code( address, data, blocksize, lastblock );
	else
	{
		exit_ISCP();
		return 3;
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

void write_code_EE_1( unsigned long address, unsigned char* data, char blocksize, char lastblock )
{
//FIXME: This currently dowsn't work as the app sends too many bytes for the write cache (2-16bytes depending on model)

	unsigned char i, tries;
	char blockcounter;

	tries = 0;
restart:
	I2C_start();
	I2C_write( 0xA0 | ((int)address>>7)&0x0E );			//Device Address + 0=write

	I2C_write( (unsigned char) ((address & 0x00FF)) ); 	//LSB

	for( blockcounter = 0; blockcounter < blocksize; blockcounter++ )
	{
		i = (unsigned int) I2C_write( data[blockcounter] );
		if( i == 1 )   // received a NACK
		{
			for( i = 0; i < 4; i++ ) {
				setLeds( 7);
				DelayMs(125);
				setLeds(0);
				DelayMs( 125 );
			}
			I2C_stop();
			DelayMs(30);
			if( tries < 2 )
				goto restart;
			return; // what else to do??
		}
	}
	I2C_stop();
	DelayMs( 10 );
}

void write_code_EE_2( unsigned long address, unsigned char* data, char blocksize, char lastblock )
{
	unsigned char i, tries;
	char blockcounter;

	tries = 0;
restart:
	I2C_start();
	I2C_write( 0xA0 | (address >= 0x10000? 8: 0) ); 	//Device Address + 0=write

	I2C_write( (unsigned char) ((address & 0xFF00) >> 8) ); //MSB
	I2C_write( (unsigned char) ((address & 0x00FF)) );	//LSB

	for( blockcounter = 0; blockcounter < blocksize; blockcounter++ )
	{
		i = (unsigned int) I2C_write( data[blockcounter] );
		if( i == 1 )   // received a NACK
		{
			for( i = 0; i < 4; i++ ) {
				setLeds( 7);
				DelayMs(125);
				setLeds(0);
				DelayMs( 125 );
			}
			I2C_stop();
			DelayMs(30);
			if( tries < 2 )
				goto restart;
			return; // what else to do??
		}
	}
	I2C_stop();
	DelayMs( 30 );
}

void write_code_P24FXXKAXXX( unsigned long address, unsigned char* data, char blocksize, char lastblock )
{
	unsigned int i, payload;
	char blockcounter;
	dspic_send_24_bits( 0x000000 ); //NOP
	//Step 1: Exit the Reset vector.
	dspic_send_24_bits( 0x040200 ); //GOTO 0x200
	dspic_send_24_bits( 0x000000 ); //NOP
	//Step 2: Set the NVMCON to program 32 instruction words.
	dspic_send_24_bits( 0x24004A ); //MOV #0x4004, W10
	dspic_send_24_bits( 0x883B0A ); //MOV W10, NVMCON


	for( blockcounter = 0; blockcounter < blocksize; blockcounter += 12 )
	{
		//Step 3: Initialize the write pointer (W7) for TBLWT instruction.
		dspic_send_24_bits( 0x200000 | (((((blockcounter + address) * 2) / 3) & 0xFF0000) >> 12) ); //MOV #<DestinationAddress23:16>, W0
		dspic_send_24_bits( 0x880190 ); //MOV W0, TBLPAG
		dspic_send_24_bits( 0x200007 | (((((blockcounter + address) * 2) / 3) & 0x00FFFF) << 4) ); //MOV #<DestinationAddress15:0>, W7
		//Step 4: Initialize the read pointer (W6) and load W0:W5 with the next 4 instruction words to program.
		for( i = 0; i < 6; i++ )
		{
			dspic_send_24_bits( 0x200000 | (((unsigned long) data[blockcounter + (i * 2)]) << 4)
					| (((unsigned long) data[blockcounter + (i * 2) + 1]) << 12)
					| ((unsigned long) i) );
			/**
			 MOV #<LSW0>, W0
			 MOV #<MSB1:MSB0>, W1
			 MOV #<LSW1>, W2
			 MOV #<LSW2>, W3
			 MOV #<MSB3:MSB2>, W4
			 MOV #<LSW3>, W5
			 */
		}
		//Step 5: Set the read pointer (W6) and load the (next set of) write latches.
		dspic_send_24_bits( 0xEB0300 ); //CLR W6
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0xBB0BB6 ); //TBLWTL [W6++], [W7]
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0xBBDBB6 ); //TBLWTH.B [W6++], [W7++]
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0xBBEBB6 ); //TBLWTH.B [W6++], [++W7]
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0xBB1BB6 ); //TBLWTL [W6++], [W7++]
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0xBB0BB6 ); //TBLWTL [W6++], [W7]
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0xBBDBB6 ); //TBLWTH.B [W6++], [W7++]
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0xBBEBB6 ); //TBLWTH.B [W6++], [++W7]
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0xBB1BB6 ); //TBLWTL [W6++], [W7++]
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0x000000 ); //NOP
	}//Step 6: Repeat steps 3-5 eight times to load the write latches for 32 instructions.
	//if((address%96)==64)
	//{
	//Step 7: Unlock the NVMCON for writing.
	dspic_send_24_bits( 0xA8E761 ); //BSET NVMCON, #WR
	dspic_send_24_bits( 0x000000 ); //NOP
	dspic_send_24_bits( 0x000000 ); //NOP
	//Step 8: CHECK bit 15 of NVMCON


	for( i = 0; i < 20; i++ )
	{
		//step 5
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0x040200 ); //GOTO 0x200
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0x803B02 ); //MOV NVMCON, W2
		dspic_send_24_bits( 0x883C22 ); //MOV W2, VISI
		dspic_send_24_bits( 0x000000 ); //NOP
		payload = dspic_read_16_bits( is3_3V() );
		dspic_send_24_bits( 0x000000 ); //NOP
		if( (payload && 0x8000) == 0 )
			break; //programming completed
		DelayMs( 1 );

	}//step 8: repeat step 5-7
	//Step 9: Reset device internal PC.
	dspic_send_24_bits( 0x040200 ); //GOTO 0x200
	dspic_send_24_bits( 0x000000 ); //NOP
}

void write_code_dsP30F( unsigned long address, unsigned char* data, char blocksize, char lastblock )
{
	unsigned int i;
	char blockcounter;

	dspic_send_24_bits( 0x000000 ); //NOP
	dspic_send_24_bits( 0x000000 ); //NOP
	//Step 1: Exit the Reset vector.
	dspic_send_24_bits( 0x040100 ); //GOTO 0x100
	dspic_send_24_bits( 0x040100 ); //GOTO 0x100
	dspic_send_24_bits( 0x000000 ); //NOP
	//Step 2: Set the NVMCON to program 32 instruction words.
	dspic_send_24_bits( 0x24001A ); //MOV #0x4001, W10
	dspic_send_24_bits( 0x883B0A ); //MOV W10, NVMCON
	for( blockcounter = 0; blockcounter < blocksize; blockcounter += 12 )
	{
		//Step 3: Initialize the write pointer (W7) for TBLWT instruction.
		dspic_send_24_bits( 0x200000 | (((((blockcounter + address) * 2) / 3) & 0xFF0000) >> 12) ); //MOV #<DestinationAddress23:16>, W0
		dspic_send_24_bits( 0x880190 ); //MOV W0, TBLPAG
		dspic_send_24_bits( 0x200007 | (((((blockcounter + address) * 2) / 3) & 0x00FFFF) << 4) ); //MOV #<DestinationAddress15:0>, W7
		//Step 4: Initialize the read pointer (W6) and load W0:W5 with the next 4 instruction words to program.
		for( i = 0; i < 6; i++ )
		{
			dspic_send_24_bits( 0x200000 | (((unsigned long) data[blockcounter + (i * 2)]) << 4)
					| (((unsigned long) data[blockcounter + (i * 2) + 1]) << 12)
					| ((unsigned long) i) );
			/**
			 MOV #<LSW0>, W0
			 MOV #<MSB1:MSB0>, W1
			 MOV #<LSW1>, W2
			 MOV #<LSW2>, W3
			 MOV #<MSB3:MSB2>, W4
			 MOV #<LSW3>, W5
			 */
		}
		//Step 5: Set the read pointer (W6) and load the (next set of) write latches.
		dspic_send_24_bits( 0xEB0300 ); //CLR W6
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0xBB0BB6 ); //TBLWTL [W6++], [W7]
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0xBBDBB6 ); //TBLWTH.B [W6++], [W7++]
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0xBBEBB6 ); //TBLWTH.B [W6++], [++W7]
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0xBB1BB6 ); //TBLWTL [W6++], [W7++]
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0xBB0BB6 ); //TBLWTL [W6++], [W7]
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0xBBDBB6 ); //TBLWTH.B [W6++], [W7++]
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0xBBEBB6 ); //TBLWTH.B [W6++], [++W7]
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0xBB1BB6 ); //TBLWTL [W6++], [W7++]
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0x000000 ); //NOP
	}//Step 6: Repeat steps 3-5 eight times to load the write latches for 32 instructions.
	//if((address%96)==64)
	//{
	//Step 7: Unlock the NVMCON for writing.
	dspic_send_24_bits( 0x200558 ); //MOV #0x55, W8
	dspic_send_24_bits( 0x883B38 ); //MOV W8, NVMKEY
	dspic_send_24_bits( 0x200AA9 ); //MOV #0xAA, W9
	dspic_send_24_bits( 0x883B39 ); //MOV W9, NVMKEY
	//Step 8: Initiate the write cycle.
	dspic_send_24_bits( 0xA8E761 ); //BSET NVMCON, #WR
	dspic_send_24_bits( 0x000000 ); //NOP
	dspic_send_24_bits( 0x000000 ); //NOP
	DelayMs( 3 ); //Externally time 2 msec
	dspic_send_24_bits( 0x000000 ); //NOP
	dspic_send_24_bits( 0x000000 ); //NOP
	dspic_send_24_bits( 0xA9E761 ); //BCLR NVMCON, #WR
	dspic_send_24_bits( 0x000000 ); //NOP
	dspic_send_24_bits( 0x000000 ); //NOP
	//Step 9: Reset device internal PC.
	dspic_send_24_bits( 0x040100 ); //GOTO 0x100
	dspic_send_24_bits( 0x000000 ); //NOP
	//}
}
void write_code_PIC24( unsigned long address, unsigned char* data, char blocksize, char lastblock, int nv, unsigned char write_size )
{
	unsigned int i;
	unsigned char blockcounter;

	//Step 2: Set the NVMCON to program 32/64 instruction words.
	dspic_send_24_bits( 0x24000A|(nv<<4) ); //MOV #0x4004/#0x4001, W10
	dspic_send_24_bits( 0x883B0A ); 	//MOV W10, NVMCON

	//Step 6: Repeat steps 3-5 eight times to load the write latches for 32 instructions.
	//Step 7: do write	(construct is ugly, but works independent of how large blockcounter increment is (as long as it's a divisor of write_size)
	for( blockcounter = 0; blockcounter < blocksize;
	     blockcounter += 12,((address + blockcounter)%write_size) == 0? p16b_do_write(): 0 )
	{
		//Step 3: Initialize the write pointer (W7) for TBLWT instruction.
		dspic_send_24_bits( 0x200000 | (((((blockcounter + address) * 2) / 3) & 0xFF0000) >> 12) ); //MOV #<DestinationAddress23:16>, W0
		dspic_send_24_bits( 0x880190 ); //MOV W0, TBLPAG
		dspic_send_24_bits( 0x200007 | (((((blockcounter + address) * 2) / 3) & 0x00FFFF) << 4) ); //MOV #<DestinationAddress15:0>, W7

		//Step 4: Initialize the read pointer (W6) and load W0:W5 with the next 4 instruction words to program.
		for( i = 0; i < 6; i++ )
		{
			dspic_send_24_bits( 0x200000 | (((unsigned long) data[blockcounter + (i * 2)]) << 4)
					| (((unsigned long) data[blockcounter + (i * 2) + 1]) << 12)
					| ((unsigned long) i) );
			/**
			 MOV #<LSW0>, W0
			 MOV #<MSB1:MSB0>, W1
			 MOV #<LSW1>, W2
			 MOV #<LSW2>, W3
			 MOV #<MSB3:MSB2>, W4
			 MOV #<LSW3>, W5
			 */
		}
		//Step 5: Set the read pointer (W6) and load the (next set of) write latches.
		dspic_send_24_bits( 0xEB0300 ); //CLR W6
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0xBB0BB6 ); //TBLWTL [W6++], [W7]
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0xBBDBB6 ); //TBLWTH.B [W6++], [W7++]
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0xBBEBB6 ); //TBLWTH.B [W6++], [++W7]
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0xBB1BB6 ); //TBLWTL [W6++], [W7++]
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0xBB0BB6 ); //TBLWTL [W6++], [W7]
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0xBBDBB6 ); //TBLWTH.B [W6++], [W7++]
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0xBBEBB6 ); //TBLWTH.B [W6++], [++W7]
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0xBB1BB6 ); //TBLWTL [W6++], [W7++]
		dspic_send_24_bits( 0x000000 ); //NOP
		dspic_send_24_bits( 0x000000 ); //NOP
	}

	if( ((address + blockcounter)%write_size) != 0 && lastblock & BLOCKTYPE_LAST )  // do the write if BLOCKTYPE_LAST and we haven't already done so
		p16b_do_write();
}
void write_code_P24KA( unsigned long address, unsigned char* data, char blocksize, char lastblock )
{
	// write in 32 instruction (32x3 byte) blocks
	write_code_PIC24( address, data, blocksize, lastblock, 0x4004, 32 * 3 );
}
void write_code_P24FJ( unsigned long address, unsigned char* data, char blocksize, char lastblock )
{
	// write in 64 instruction (64*3 byte) blocks
	write_code_PIC24( address, data, blocksize, lastblock, 0x4001, 64 * 3 );
}
void write_code_P18F872X( unsigned long address, unsigned char* data, char blocksize, char lastblock )
{
	char blockcounter;
					//FIXME: this only needs to be done on FIRST_BLOCK
	if( (address & 0x20) == 0 ) //package must be 64 bytes, so only do this every two packages.
	{
		pic_send( 4, 0x00, 0x8EA6 ); //BSF EECON1, EEPGD
		pic_send( 4, 0x00, 0x8CA6 ); //BSF EECON1, CFGS
		pic_send( 4, 0x00, 0x88A6 ); //BSF EECON1, WREN
		pic_send( 4, 0x00, 0x8EA6 ); //BSF EECON1, EEPGD
		pic_send( 4, 0x00, 0x9CA6 ); //BCF EECON1, CFGS
		set_address_P18( address );
	}
	for( blockcounter = 0; blockcounter < (blocksize - 2); blockcounter += 2 )
	{
		//write 2 bytes and post increment by 2
		//				MSB				LSB
		pic_send( 4, 0x0D, ((unsigned int) *(data + blockcounter))
				| (((unsigned int) *(data + 1 + blockcounter)) << 8) );
	}
	if( (address & 0x20) == 0x20 || (lastblock & BLOCKTYPE_LAST) )
	{
		//write last 2 bytes of the block and start programming
		pic_send( 4, 0x0F, ((unsigned int) *(data + blockcounter))
				| (((unsigned int) *(data + 1 + blockcounter)) << 8) );
		pic_send_n_bits( 3, 0 );
		PGChigh(); //hold PGC high for P9 and low for P10
		DelayMs( P9 );
		PGClow();
		DelayMs( P10 );
		pic_send_word( 0x0000 );
	}
	else
		pic_send( 4, 0x0D, ((unsigned int) *(data + blockcounter))
				| (((unsigned int) *(data + 1 + blockcounter)) << 8) );
}
void write_code_P18F6XKXX( unsigned long address, unsigned char* data, char blocksize, char lastblock )
{
	char blockcounter;

	//FIXME: this only needs to be done on FIRST_BLOCK
	if( (address & 0x20) == 0 ) //package must be 64 bytes, so only do this every two packages.
	{
		pic_send( 4, 0x00, 0x8E7F ); //BSF EECON1, EEPGD
		pic_send( 4, 0x00, 0x9C7F ); //BSF EECON1, CFGS
		pic_send( 4, 0x00, 0x847F ); //BSF EECON1, WREN
		set_address_P18( address );
	}
	for( blockcounter = 0; blockcounter < (blocksize - 2); blockcounter += 2 )
	{
		//write 2 bytes and post increment by 2
		//				MSB				LSB
		pic_send( 4, 0x0D, ((unsigned int) *(data + blockcounter))
				| (((unsigned int) *(data + 1 + blockcounter)) << 8) );
	}
	if( (address & 0x20) == 0x20 || (lastblock & BLOCKTYPE_LAST) )
	{
		//write last 2 bytes of the block and start programming
		pic_send( 4, 0x0F, ((unsigned int) *(data + blockcounter))
				| (((unsigned int) *(data + 1 + blockcounter)) << 8) );
		pic_send_n_bits( 3, 0 );
		PGChigh(); //hold PGC high for P9 and low for P10
		DelayMs( P9 );
		PGClow();
		DelayMs( P10 );
		pic_send_word( 0x0000 );
	}
	else
		pic_send( 4, 0x0D, ((unsigned int) *(data + blockcounter))
				| (((unsigned int) *(data + 1 + blockcounter)) << 8) );
}

void write_code_P18F67KXX( unsigned long address, unsigned char* data, char blocksize, char lastblock )
{
	char blockcounter;

	//FIXME: this only needs to be done on FIRST_BLOCK
	if( (address & 0x60) == 0 ) //package must be 128 bytes, so only do this every four packages.
	{
		pic_send( 4, 0x00, 0x8E7F ); //BSF EECON1, EEPGD
		pic_send( 4, 0x00, 0x9C7F ); //BSF EECON1, CFGS
		pic_send( 4, 0x00, 0x847F ); //BSF EECON1, WREN
		set_address_P18( address );
	}
	for( blockcounter = 0; blockcounter < (blocksize - 2); blockcounter += 2 )
	{
		//write 2 bytes and post increment by 2
		//				MSB				LSB
		pic_send( 4, 0x0D, ((unsigned int) *(data + blockcounter))
				| (((unsigned int) *(data + 1 + blockcounter)) << 8) );
	}
	if( (address & 0x60) == 0x60 || (lastblock & BLOCKTYPE_LAST) )
	{
		//write last 2 bytes of the block and start programming
		pic_send( 4, 0x0F, ((unsigned int) *(data + blockcounter))
				| (((unsigned int) *(data + 1 + blockcounter)) << 8) );
		pic_send_n_bits( 3, 0 );
		PGChigh(); //hold PGC high for P9 and low for P10
		DelayMs( P9 );
		PGClow();
		DelayMs( P10 );
		pic_send_word( 0x0000 );
	}
	else
		pic_send( 4, 0x0D, ((unsigned int) *(data + blockcounter))
				| (((unsigned int) *(data + 1 + blockcounter)) << 8) );

}

void write_code_P18F2XXX( unsigned long address, unsigned char* data, char blocksize, char lastblock )
{
	char blockcounter;

	pic_send( 4, 0x00, 0x8EA6 ); //BSF EECON1, EEPGD
	pic_send( 4, 0x00, 0x9CA6 ); //BCF EECON1, CFGS
	set_address_P18( address );
	for( blockcounter = 0; blockcounter < (blocksize - 2); blockcounter += 2 )
	{
		//write 2 bytes and post increment by 2
		//				MSB				LSB
		pic_send( 4, 0x0D, ((unsigned int) *(data + blockcounter))
				| (((unsigned int) *(data + 1 + blockcounter)) << 8) );
	}
	//write last 2 bytes of the block and start programming
	pic_send( 4, 0x0F, ((unsigned int) *(data + blockcounter)) | (((unsigned int) *(data + 1 + blockcounter)) << 8) );
	pic_send_n_bits( 3, 0 );
	PGChigh(); //hold PGC high for P9 and low for P10
	DelayMs( P9 );
	PGClow();
	DelayMs( P10 );
	pic_send_word( 0x0000 );
}
void write_code_P18F4XK22( unsigned long address, unsigned char* data, char blocksize, char lastblock )
{
	char blockcounter;

	//FIXME: this only needs to be done on FIRST_BLOCK
	pic_send( 4, 0x00, 0x8EA6 ); //BSF EECON1, EEPGD
	pic_send( 4, 0x00, 0x9CA6 ); //BCF EECON1, CFGS
	pic_send( 4, 0x00, 0x84A6 ); //BSF EECON1, WREN
	set_address_P18( address );
	for( blockcounter = 0; blockcounter < (blocksize - 2); blockcounter += 2 )
	{
		//write 2 bytes and post increment by 2
		//				MSB				LSB
		pic_send( 4, 0x0D, ((unsigned int) *(data + blockcounter))
				| (((unsigned int) *(data + 1 + blockcounter)) << 8) );
	}
	//write last 2 bytes of the block and start programming
	pic_send( 4, 0x0F, ((unsigned int) *(data + blockcounter)) | (((unsigned int) *(data + 1 + blockcounter)) << 8) );
	pic_send_n_bits( 3, 0 );
	PGChigh(); //hold PGC high for P9 and low for P10
	DelayMs( P9 );
	PGClow();
	DelayMs( P10 );
	pic_send_word( 0x0000 );
}
void write_code_P18F14K22( unsigned long address, unsigned char* data, char blocksize, char lastblock )
{
	unsigned int i;
	char blockcounter;

	//FIXME: this only needs to be done on FIRST_BLOCK
	//direct access to code memory
	pic_send( 4, 0x00, 0x8EA6 ); //BSF EECON1, EEPGD
	pic_send( 4, 0x00, 0x9CA6 ); //BCF EECON1, CFGS
	pic_send( 4, 0x00, 0x84A6 ); //BSF EECON1, WREN
	set_address_P18( address );
	for( blockcounter = 0; blockcounter < (blocksize); blockcounter += 16 ) //blocks of 16 bytes
	{
		for( i = 0; i < 14; i += 2 )
		{
			//write 2 bytes and post increment by 2
			//				MSB				LSB
			pic_send( 4, 0x0D, ((unsigned int) *(data + blockcounter + i)) | (((unsigned int) *(data + 1
					+ blockcounter + i)) << 8) );
		}
		//write last 2 bytes of the block and start programming
		pic_send( 4, 0x0F, ((unsigned int) *(data + blockcounter + 14)) | (((unsigned int) *(data + 15
				+ blockcounter)) << 8) );
		pic_send_n_bits( 3, 0 );
		PGChigh(); //hold PGC high for P9 and low for P10
		DelayMs( P9 );
		PGClow();
		DelayMs( P10 );
		pic_send_word( 0x0000 );
		pic_read_byte2( 4, 0x09 ); //perform 2 reads to increase the address by 2
		pic_read_byte2( 4, 0x09 );
	}
}
void write_code_P18F13K22( unsigned long address, unsigned char* data, char blocksize, char lastblock )
{
	unsigned int i;
	char blockcounter;

	//FIXME: this only needs to be done on FIRST_BLOCK
	//direct access to code memory
	pic_send( 4, 0x00, 0x8EA6 ); //BSF EECON1, EEPGD
	pic_send( 4, 0x00, 0x9CA6 ); //BCF EECON1, CFGS
	pic_send( 4, 0x00, 0x84A6 ); //BSF EECON1, WREN
	set_address_P18( address );
	for( blockcounter = 0; blockcounter < (blocksize); blockcounter += 8 ) //blocks of 8 bytes
	{
		for( i = 0; i < 6; i += 2 )
		{
			//write 2 bytes and post increment by 2
			//				MSB				LSB
			pic_send( 4, 0x0D, ((unsigned int) *(data + blockcounter + i)) | (((unsigned int) *(data + 1
					+ blockcounter + i)) << 8) );
		}
		//write last 2 bytes of the block and start programming
		pic_send( 4, 0x0F, ((unsigned int) *(data + blockcounter + 6)) | (((unsigned int) *(data + 7
				+ blockcounter)) << 8) );
		pic_send_n_bits( 3, 0 );
		PGChigh(); //hold PGC high for P9 and low for P10
		DelayMs( P9 );
		PGClow();
		DelayMs( P10 );
		pic_send_word( 0x0000 );
		pic_read_byte2( 4, 0x09 ); //perform 2 reads to increase the address by 2
		pic_read_byte2( 4, 0x09 );
	}
}
void write_code_P18FXX20( unsigned long address, unsigned char* data, char blocksize, char lastblock )
{
	unsigned int i;
	char blockcounter;
	if( lastblock & BLOCKTYPE_FIRST )
	{
		pic_send( 4, 0x00, 0x8EA6);// BSF EECON1, EEPGD
		pic_send( 4, 0x00, 0x8CA6);// BSF EECON1, CFGS
		pic_send( 4, 0x00, 0x86A6);// BSF EECON1, WREN
		set_address_P18( 0x3C0006 );
		pic_send( 4, 0x0C, 0x0040 ); //Write 40h to 3C0006h to enable multi-panel writes.
		//direct access to code memory
		pic_send( 4, 0x00, 0x8EA6 ); //BSF EECON1, EEPGD
		pic_send( 4, 0x00, 0x9CA6 ); //BCF EECON1, CFGS

	}
	
	set_address_P18( address );
	for( blockcounter = 0; blockcounter < (blocksize); blockcounter += 8 ) //blocks of 8 bytes
	{
		for( i = 0; i < 6; i += 2 )
		{
			//write 2 bytes and post increment by 2
			//				MSB				LSB
			pic_send( 4, 0x0D, ((unsigned int) *(data + blockcounter + i)) | (((unsigned int) *(data + 1
					+ blockcounter + i)) << 8) );
		}
		if((lastblock & BLOCKTYPE_LAST)&& (blockcounter==(blocksize-8)))
		{
			//write last 2 bytes of the block and start programming
			pic_send( 4, 0x0F, ((unsigned int) *(data + blockcounter + 6)) | (((unsigned int) *(data + 7
					+ blockcounter)) << 8) );
		}
		else
		{
			//write last 2 bytes of the block and start programming
			pic_send( 4, 0x0C, ((unsigned int) *(data + blockcounter + 6)) | (((unsigned int) *(data + 7
					+ blockcounter)) << 8) );
			pic_send_n_bits( 3, 0 );
			PGChigh(); //hold PGC high for P9 and low for P10
			DelayMs( P9 );
			PGClow();
			DelayMs( P10 );
			pic_send_word( 0x0000 );
		}
		pic_read_byte2( 4, 0x09 ); //perform 2 reads to increase the address by 2
		pic_read_byte2( 4, 0x09 );
	}

}
void write_code_P18FX220( unsigned long address, unsigned char* data, char blocksize, char lastblock )
{
	unsigned int i;
	char blockcounter;

	//FIXME: this only needs to be done on FIRST_BLOCK
	//direct access to code memory
	pic_send( 4, 0x00, 0x8EA6 ); //BSF EECON1, EEPGD
	pic_send( 4, 0x00, 0x9CA6 ); //BCF EECON1, CFGS
	set_address_P18( address );
	for( blockcounter = 0; blockcounter < (blocksize); blockcounter += 8 ) //blocks of 8 bytes
	{
		for( i = 0; i < 6; i += 2 )
		{
			//write 2 bytes and post increment by 2
			//				MSB				LSB
			pic_send( 4, 0x0D, ((unsigned int) *(data + blockcounter + i)) | (((unsigned int) *(data + 1
					+ blockcounter + i)) << 8) );
		}
		//write last 2 bytes of the block and start programming
		pic_send( 4, 0x0F, ((unsigned int) *(data + blockcounter + 6)) | (((unsigned int) *(data + 7
				+ blockcounter)) << 8) );
		pic_send_n_bits( 3, 0 );
		PGChigh(); //hold PGC high for P9 and low for P10
		DelayMs( P9 );
		PGClow();
		DelayMs( P10 );
		pic_send_word( 0x0000 );
		pic_read_byte2( 4, 0x09 ); //perform 2 reads to increase the address by 2
		pic_read_byte2( 4, 0x09 );
	}

}
void write_code_P18FXX31( unsigned long address, unsigned char* data, char blocksize, char lastblock )
{
	unsigned int i;
	char blockcounter;

	//FIXME: this only needs to be done on FIRST_BLOCK
	//direct access to config memory
	pic_send( 4, 0x00, 0x8EA6 ); //BSF EECON1, EEPGD
	pic_send( 4, 0x00, 0x8CA6 ); //BSF EECON1, CFGS
	//configure the device for single panel writes
	set_address_P18( 0x3C0006 );
	pic_send( 4, 0x0C, 0x0000 ); //write 0x00 to the tblptr to disable multi-panel writes
	//direct access to code memory
	pic_send( 4, 0x00, 0x8EA6 ); //BSF EECON1, EEPGD
	pic_send( 4, 0x00, 0x9CA6 ); //BCF EECON1, CFGS
	set_address_P18( address );
	for( blockcounter = 0; blockcounter < (blocksize); blockcounter += 8 ) //blocks of 8 bytes
	{
		for( i = 0; i < 6; i += 2 )
		{
			//write 2 bytes and post increment by 2
			//				MSB				LSB
			pic_send( 4, 0x0D, ((unsigned int) *(data + blockcounter + i)) | (((unsigned int) *(data + 1
					+ blockcounter + i)) << 8) );
		}
		//write last 2 bytes of the block and start programming
		pic_send( 4, 0x0F, ((unsigned int) *(data + blockcounter + 6)) | (((unsigned int) *(data + 7
				+ blockcounter)) << 8) );
		pic_send_n_bits( 3, 0 );
		PGChigh(); //hold PGC high for P9 and low for P10
		DelayMs( P9 );
		PGClow();
		DelayMs( P10 );
		pic_send_word( 0x0000 );
		pic_read_byte2( 4, 0x09 ); //perform 2 reads to increase the address by 2
		pic_read_byte2( 4, 0x09 );
	}
}
void write_code_P18F45J10( unsigned long address, unsigned char* data, char blocksize, char lastblock )
{

	char blockcounter;

	if( !(address & 0x20) )
	{
		pic_send( 4, 0x00, 0x84A6 ); //BSF EECON1, WREN
		set_address_P18( address ); //blocks of 64 bytes, but divided into two chunks
	}
	for( blockcounter = 0; blockcounter < (blocksize - 2); blockcounter += 2 )
	{
		//write 2 bytes and post increment by 2
		//				MSB				LSB
		pic_send( 4, 0x0D, ((unsigned int) *(data + blockcounter))
				| (((unsigned int) *(data + 1 + blockcounter)) << 8) );
	}
	//write last 2 bytes of the block and start programming
	if( address & 0x20 )
	{
		pic_send( 4, 0x0F, ((unsigned int) *(data + blockcounter))
				| (((unsigned int) *(data + 1 + blockcounter)) << 8) );
		pic_send_n_bits( 3, 0 );
		PGChigh(); //hold PGC high for P9 and low for P10
		DelayMs( 10 );
		PGClow();
		DelayMs( 1 );
		pic_send_word( 0x0000 );
	}
	else
	{
		pic_send( 4, 0x0D, ((unsigned int) *(data + blockcounter))
				| (((unsigned int) *(data + 1 + blockcounter)) << 8) );
		if( lastblock & BLOCKTYPE_LAST ) //if the last block is the first half of 64 bytes, it needs to be finished with a dummy block to finish.
		{
			for( blockcounter = 0; blockcounter < 30; blockcounter += 2 )
				pic_send( 4, 0x0D, 0xFFFF );
			pic_send( 4, 0x0F, 0xFFFF );
			pic_send_n_bits( 3, 0 );
			PGChigh(); //hold PGC high for P9 and low for P10
			DelayMs( 10 );
			PGClow();
			DelayMs( 1 );
			pic_send_word( 0x0000 );
		}
	}
}

void write_code_P16F18XX( unsigned long address, unsigned char* data, char blocksize, char lastblock )
{

	char blockcounter;

	if( lastblock & BLOCKTYPE_FIRST )
	{
		pic_send_n_bits( 6, 0x16 ); //reset address
		set_address_P16( address ); //set the initial address
	}

	for( blockcounter = 0; blockcounter < blocksize; blockcounter += 2 )
	{
		pic_send_14_bits( 6, 0x02, (((unsigned int) data[blockcounter])) | //MSB
				(((unsigned int) data[blockcounter + 1]) << 8) );//LSB
		pic_send_n_bits( 6, 0x08 ); //begin programming
		DelayMs( Tprog );
		DelayMs( 10 );
		pic_send_n_bits( 6, 0x06 ); //increment address
	}
}

void write_code_P16F84A( unsigned long address, unsigned char* data, char blocksize, char lastblock )
{

	char blockcounter;

	if( lastblock & BLOCKTYPE_FIRST )
	{
		set_address_P16( address ); //set the initial address
	}

	for( blockcounter = 0; blockcounter < blocksize; blockcounter += 2 )
	{
		pic_send_14_bits( 6, 0x02, (((unsigned int) data[blockcounter])) | //MSB
				(((unsigned int) data[blockcounter + 1]) << 8) );//LSB
		pic_send_n_bits( 6, 0x08 ); //begin programming
		DelayMs( Tprog );
		DelayMs( 10 );
		pic_send_n_bits( 6, 0x06 ); //increment address
	}
	/*if(pictype==P12F629&&((lastblock&2)&&((address+blocksize)<0x3FF))) //restore osccal register
	 {
	 for(i=0;i<(0x3FF-(address+blocksize));i++)
	 pic_send_n_bits(6,0x06);	//increment address
	 pic_send_14_bits(6,0x02,osccal);
	 pic_send_n_bits(6,0x08);    //begin programming, internally timed
	 DelayMs(8);
	 //pic_send_n_bits(6,0x0A); 	//end programming
	 }*/
}
void write_code_P12F61X( unsigned long address, unsigned char* data, char blocksize, char lastblock )
{

	char blockcounter;

	if( (lastblock & BLOCKTYPE_FIRST) && (address > 0) )
	{
		set_address_P16( address ); //set the initial address
	}
	for( blockcounter = 0; blockcounter < blocksize; blockcounter += 2 )
	{
		pic_send_14_bits( 6, 0x02, (((unsigned int) data[blockcounter])) | //MSB
				(((unsigned int) data[blockcounter + 1]) << 8) );//LSB
		pic_send_n_bits( 6, 0x18 ); //begin programming
		DelayMs( Tprog );
		pic_send_n_bits( 6, 0x0A ); //end programming
		DelayMs( 10 );
		/*payload=pic_read_14_bits(6,0x04); //read code memory
		 if(payload!=((((unsigned int)data[blockcounter]))|(((unsigned int)data[blockcounter+1])<<8)))
		 {
		 set_vdd_vpp(pictype, picfamily,0);	//do a hard reset to target processor
		 set_vdd_vpp(pictype, picfamily,1);
		 DelayMs(10);		//wait a while
		 set_address(picfamily, address+(blockcounter>>1)); //go back to the address where it was

		 pic_send_14_bits(6,0x02,(((unsigned int)data[blockcounter]))|   //MSB
		 (((unsigned int)data[blockcounter+1])<<8));//LSB
		 switch(pictype)
		 {
		 case P12F61X:
		 pic_send_n_bits(6,0x18);    //begin programming
		 DelayMs(Tprog);
		 pic_send_n_bits(6,0x0A);    //end programming
		 break;
		 default:
		 pic_send_n_bits(6,0x08);    //begin programming
		 DelayMs(Tprog);
		 break;
		 }
		 payload=pic_read_14_bits(6,0x04); //read code memory
		 if(payload!=((((unsigned int)data[blockcounter]))|(((unsigned int)data[blockcounter+1])<<8)))
		 {
		 return 4; //verify error
		 }
		 }*/
		pic_send_n_bits( 6, 0x06 ); //increment address
	}
	/*if(pictype==P12F629&&((lastblock&2)&&((address+blocksize)<0x3FF))) //restore osccal register
	 {
	 for(i=0;i<(0x3FF-(address+blocksize));i++)
	 pic_send_n_bits(6,0x06);	//increment address
	 pic_send_14_bits(6,0x02,osccal);
	 pic_send_n_bits(6,0x08);    //begin programming, internally timed
	 DelayMs(8);
	 //pic_send_n_bits(6,0x0A); 	//end programming
	 }*/
}
void write_code_P16C6XX( unsigned long address, unsigned char* data, char blocksize, char lastblock )
{

	char blockcounter;
	char i;
	unsigned int payload;
	if( (lastblock & BLOCKTYPE_FIRST) && (address > 0) )
	{
		set_address_P16( address ); //set the initial address
	}
	for( blockcounter = 0; blockcounter < blocksize; blockcounter += 2 )
	{
		for(i=0;i<25;i++)
		{
			payload = (((unsigned int) data[blockcounter])) | //MSB
				(((unsigned int) data[blockcounter + 1]) << 8);
			pic_send_14_bits( 6, 0x02,  payload);//LSB
		
			pic_send_n_bits( 6, 0x08 ); //begin programming
			DelayUs( 100 );
			pic_send_n_bits( 6, 0x0E ); //end programming
			if(pic_read_14_bits( 6, 0x04 )==payload&&i<22)i=22; //correct? do 3 more programming cycles.
		}
		pic_send_n_bits( 6, 0x06 ); //increment address
	}
}
void write_code_P16F72( unsigned long address, unsigned char* data, char blocksize, char lastblock )
{
	unsigned int i;
	char blockcounter;
	if( (lastblock & BLOCKTYPE_FIRST) && (address > 0) )
	{
		set_address_P16( address ); //set the initial address
	}
	//2 word programming
	for( blockcounter = 0; blockcounter < blocksize; blockcounter += 4 ) //2 words of data = 4 bytes
	{
		for( i = 0; i < 4; i += 2 )
		{
			pic_send_14_bits( 6, 0x02, (((unsigned int) data[blockcounter + i])) | //MSB
					(((unsigned int) data[blockcounter + i + 1]) << 8) );//LSB
			if( i < 2 )
				pic_send_n_bits( 6, 0x06 ); //increment address
		}
		pic_send_n_bits( 6, 0x08 ); //begin programming only, externally timed
		DelayMs( 2 );
		pic_send_n_bits( 6, 0x0E ); //end programming
		//for(i=0;i<100;i++);		//wait Tdis
		pic_send_n_bits( 6, 0x06 ); //increment address
	}
}
void write_code_P16F785( unsigned long address, unsigned char* data, char blocksize, char lastblock )
{
	unsigned int i;
	char blockcounter;
	if( (lastblock & BLOCKTYPE_FIRST) && (address > 0) )
	{
		set_address_P16( address ); //set the initial address
	}
	//4 word programming
	for( blockcounter = 0; blockcounter < blocksize; blockcounter += 8 ) //4 words of data = 8 bytes
	{
		for( i = 0; i < 8; i += 2 )
		{
			pic_send_14_bits( 6, 0x02, (((unsigned int) data[blockcounter + i])) | //MSB
					(((unsigned int) data[blockcounter + i + 1]) << 8) );//LSB
			if( i < 6 )
				pic_send_n_bits( 6, 0x06 ); //increment address
		}
		pic_send_n_bits( 6, 0x18 ); //begin programming
		DelayMs( Tprog );
		pic_send_n_bits( 6, 0x0A ); //end programming
		//for(i=0;i<100;i++);		//wait Tdis
		pic_send_n_bits( 6, 0x06 ); //increment address
	}
}
void write_code_P16F716( unsigned long address, unsigned char* data, char blocksize, char lastblock )
{
	unsigned int i;
	char blockcounter;
	if( (lastblock & BLOCKTYPE_FIRST) && (address > 0) )
	{
		set_address_P16( address ); //set the initial address
	}
	//4 word programming
	for( blockcounter = 0; blockcounter < blocksize; blockcounter += 8 ) //4 words of data = 8 bytes
	{
		for( i = 0; i < 8; i += 2 )
		{
			pic_send_14_bits( 6, 0x02, (((unsigned int) data[blockcounter + i])) | //MSB
					(((unsigned int) data[blockcounter + i + 1]) << 8) );//LSB
			if( i < 6 )
				pic_send_n_bits( 6, 0x06 ); //increment address
		}
		pic_send_n_bits( 6, 0x18 ); //begin programming
		DelayMs( Tprog );
		pic_send_n_bits( 6, 0x0E ); //end programming
		//for(i=0;i<100;i++);		//wait Tdis
		pic_send_n_bits( 6, 0x06 ); //increment address
	}
}
void write_code_P12F6XX( unsigned long address, unsigned char* data, char blocksize, char lastblock )
{
	unsigned int i;
	char blockcounter;
	if( (lastblock & BLOCKTYPE_FIRST) && (address > 0) )
	{
		set_address_P16( address ); //set the initial address
	}
	//4 word programming
	for( blockcounter = 0; blockcounter < blocksize; blockcounter += 8 ) //4 words of data = 8 bytes
	{
		for( i = 0; i < 8; i += 2 )
		{
			pic_send_14_bits( 6, 0x02, (((unsigned int) data[blockcounter + i])) | //MSB
					(((unsigned int) data[blockcounter + i + 1]) << 8) );//LSB
			if( i < 6 )
				pic_send_n_bits( 6, 0x06 ); //increment address
		}
		pic_send_n_bits( 6, 0x8 ); //begin programming, externally timed
		DelayMs( Tprog );
		//for(i=0;i<100;i++);		//wait Tdis
		pic_send_n_bits( 6, 0x06 ); //increment address
	}
}
void write_code_P16F87( unsigned long address, unsigned char* data, char blocksize, char lastblock )
{
	unsigned int i;
	char blockcounter;
	if( (lastblock & BLOCKTYPE_FIRST) && (address > 0) )
	{
		set_address_P16( address ); //set the initial address
	}
	//4 word programming
	for( blockcounter = 0; blockcounter < blocksize; blockcounter += 8 ) //4 words of data = 8 bytes
	{
		for( i = 0; i < 8; i += 2 )
		{
			pic_send_14_bits( 6, 0x02, (((unsigned int) data[blockcounter + i])) | //MSB
					(((unsigned int) data[blockcounter + i + 1]) << 8) );//LSB
			if( i < 6 )
				pic_send_n_bits( 6, 0x06 ); //increment address
		}
		pic_send_n_bits( 6, 0x18 ); //begin programming only, externally timed
		DelayMs( 2 );
		pic_send_n_bits( 6, 0x17 ); //end programming
		//for(i=0;i<100;i++);		//wait Tdis
		pic_send_n_bits( 6, 0x06 ); //increment address
	}
}
void write_code_P16F87XA( unsigned long address, unsigned char* data, char blocksize, char lastblock )
{
	unsigned int i;
	char blockcounter;
	if( (lastblock & BLOCKTYPE_FIRST) && (address > 0) )
	{
		set_address_P16( address ); //set the initial address
	}
	for( blockcounter = 0; blockcounter < blocksize; blockcounter += 16 ) //8 words of data = 16 bytes
	{
		for( i = 0; i < 16; i += 2 )
		{
			pic_send_14_bits( 6, 0x02, (((unsigned int) data[blockcounter + i])) | //MSB
					(((unsigned int) data[blockcounter + i + 1]) << 8) );//LSB
			if( i < 14 )
				pic_send_n_bits( 6, 0x06 ); //increment address
		}
		pic_send_n_bits( 6, 0x08 ); //begin erase / programming
		DelayMs( 16 );
		//pic_send_n_bits(5,0x17);    //end programming
		pic_send_n_bits( 6, 0x06 ); //increment address
	}
}
void write_code_P16F54( unsigned long address, unsigned char* data, char blocksize, char lastblock )
{

	char blockcounter;
	if( lastblock & BLOCKTYPE_FIRST )
	{
		pic_send_n_bits( 6, 0x06 );//increment address to go from 1FF / 3FF to 0
		set_address_P16( address ); //set the initial address
		//pic_send_n_bits(6,0x09);//bulk erase, which is necessary anyway...
		//DelayMs(20);
	}
	for( blockcounter = 0; blockcounter < blocksize; blockcounter += 2 )
	{
		pic_send_14_bits( 6, 0x02, (((unsigned int) data[blockcounter])) | //MSB
				(((unsigned int) data[blockcounter + 1]) << 8) );//LSB
		DelayMs( 1 );
		pic_send_n_bits( 6, 0x08 ); //begin programming
		DelayMs( 3 );
		pic_send_n_bits( 6, 0x0E ); //end programming
		DelayMs( 1 );
		//payload=pic_read_14_bits(6,0x04); //read code memory
		//if(payload!=((((unsigned int)data[blockcounter]))|(((unsigned int)data[blockcounter+1])<<8)))
		//{
		//	set_vdd_vpp(pictype,picfamily,0);
		//	return 4;//verify error.
		//}
		pic_send_n_bits( 6, 0x06 ); //increment address
	}
}
