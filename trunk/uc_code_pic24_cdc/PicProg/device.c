#include "xc.h"
#include "PicProg/device.h"
#include "PicProg/prog_lolvl.h"
#include "PicProg/bulk_erase.h"
#include "PicProg/read_code.h"
#include "PicProg/read_data.h"
#include "PicProg/write_code.h"
#include "PicProg/write_data.h"
#include "PicProg/write_config_bits.h"

#define LIST(x) #x,

//rom char *rom picfamilyName[] =
char *picfamilyName[] =
{
#include "PicProg/picfamily.h"
};
//#undef LIST

//#define LIST( x, a0, a1, a2, a3, a4, a5, a6, a7, a8 ) #x,

//rom char *rom pictypeName[] =
char *pictypeName[] =
{
#include "PicProg/pictype.h"
	 "P24FJG"
};
#undef LIST

#define LIST DEVICE_ENTRY
#pragma romdata DEVICES

DEVICE_TABLE devices[]  =
{
#include "PicProg/device_table.h"
};

#undef LIST
#pragma romdata


DEVICE_t currDevice;
PICFAMILY picfamily = PIC18;
PICTYPE pictype = P18F2XXX;
