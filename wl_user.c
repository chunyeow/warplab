/******************************************************************************
*
* File   :	wl_transport.c
* Authors:	Chris Hunter (chunter [at] mangocomm.com)
*			Patrick Murphy (murphpo [at] mangocomm.com)
*           Erik Welsh (welsh [at] mangocomm.com)
* License:	Copyright 2013, Mango Communications. All rights reserved.
*			Distributed under the WARP license  (http://warpproject.org/license)
*
******************************************************************************/
/**
*
* @file wl_user.c
*
* Implements the WARPLab Transport user extensions.
*
* This implementation supports both WARP v2 and WARP v3 hardware
*
* <pre>
* MODIFICATION HISTORY:
*
* Ver   Who  Date     Changes
* ----- ---- -------- -------------------------------------------------------
*
* </pre>
*
******************************************************************************/


/***************************** Include Files *********************************/

#include "include/wl_user.h"
#include "include/wl_common.h"
#include <xparameters.h>
#include <stdlib.h>
#include <stdio.h>
#include <Xio.h>
#include <xil_io.h>
#include <string.h>
#include <xil_types.h>

#ifdef WARP_HW_VER_v3
#include <w3_iic_eeprom.h>
#endif

/*************************** Constant Definitions ****************************/



/*********************** Global Variable Definitions *************************/



/*************************** Variable Definitions ****************************/



/*************************** Function Prototypes *****************************/

#ifdef _DEBUG_
void print_buf(unsigned char *, int);
#endif

/******************************** Functions **********************************/


/*****************************************************************************/
/**
*
* This function is the User callback that allows WARPLab to process
* a received Ethernet packet
*
* @param	cmdHdr is a pointer to the WARPLab command header
* @param	cmdArgs is a pointer to the Transport command arguments
* @param	respHdr is a pointer to the WARPLab response header
* @param	respArgs is a pointer to the Transport response arguments
* @param	pktSrc is a pointer to the Ethenet packet source structure
* @param    eth_dev_num is an int that specifies the Ethernet interface to use
*
* @return	-NO_RESP_SENT to indicate no response has been sent
*		-RESP_SENT to indicate a response has been sent
*
******************************************************************************/

int user_processCmd(const wl_cmdHdr* cmdHdr, const void* cmdArgs, wl_respHdr* respHdr, void* respArgs, void* pktSrc, unsigned int eth_dev_num) {
	//IMPORTANT ENDIAN NOTES:
	// -cmdHdr is safe to access directly (pre-swapped if needed)
	// -cmdArgs is *not* pre-swapped, since the framework doesn't know what it is
	// -respHdr will be swapped by the framework; user code should fill it normally
	// -respArgs will *not* be swapped by the framework, since only user code knows what it is
	//    Any data added to respArgs by the code below must be endian-safe (swapped on AXI hardware)

	//Cast argument buffers into arrays for easy accessing
	unsigned int  respSent   = NO_RESP_SENT;
	unsigned int  respIndex  = 0;
	const u32    *cmdArgs32  = cmdArgs;
	u32          *respArgs32 = respArgs;
	unsigned int  cmdID      = WL_CMD_TO_CMDID(cmdHdr->cmd);

	respHdr->cmd      = cmdHdr->cmd;
	respHdr->length   = 0;
	respHdr->numArgs  = 0;

#ifdef WARP_HW_VER_v3
	u32 stringBuffer32[10];
	u8 *stringBuffer8 = (u8*)stringBuffer32;
	int k;
	u32 readLength;
	u32 eepromAddrOffset;
#endif

	switch(cmdID) {

#ifdef WARP_HW_VER_v3

		case USER_EEPROM_WRITE_STRING:
			eepromAddrOffset = Xil_Ntohl(cmdArgs32[0]);

			for( k=0; k<((cmdHdr->length)/sizeof(u32)); k++ ){
				stringBuffer32[k] = Xil_Ntohl(cmdArgs32[k+1]);
			}

			for( k=0; k<(cmdHdr->length); k++ ){
				iic_eeprom_writeByte(EEPROM_BASEADDR,k+eepromAddrOffset,stringBuffer8[k]);
			}

			xil_printf("Wrote '%s' to EEPROM\n",stringBuffer8);
		break;

		case USER_EEPROM_READ_STRING:
			eepromAddrOffset = Xil_Ntohl(cmdArgs32[0]);
			readLength = Xil_Ntohl(cmdArgs32[1]);

			for( k=0; k<readLength; k++ ){
				stringBuffer8[k] = iic_eeprom_readByte(EEPROM_BASEADDR,k+eepromAddrOffset);
			}

			xil_printf("Read '%s' from EEPROM\n",stringBuffer8);

			for( k=0; k<((readLength)/sizeof(u32)); k++ ){
				respArgs32[k] = Xil_Htonl(stringBuffer32[k]);
			}
			respHdr->length += ((k) * sizeof(respArgs32));
			respHdr->numArgs = (k);

		break;

#endif

/* Example Additional User Command
		case SOME_CMD_ID:                              <-- Needs to be defined as same number in both Matlab and C
			arg0   = Xil_Ntohl(cmdArgs[0]);
			arg1   = Xil_Ntohl(cmdArgs[1]);
			result = do_something_with_args(arg0, arg1);

			respArgs32[respIndex++] = Xil_Htonl(result);
			respHdr->length        += (respIndex * sizeof(respArgs32));
			respHdr->numArgs        = respIndex;
		break;
*/

		default:
			xil_printf("Unknown user command ID: %d\n", cmdID);
		break;
	}
	return respSent;
}



/*****************************************************************************/
/**
*
* This function will initialize the user functions
*
* @param    None
*
* @return	-SUCCESS to indicate that the transport was initialized
*		-FAILURE to indicated that the transport was not initialized
*
* @note		None.
*
******************************************************************************/
int user_init(){
	int status = SUCCESS;

	//User initialization goes here
	// Framework calls user_init when node is initialized
	//  (on boot and when node 'initialize' command is received)

	return status;
}




#ifdef _DEBUG_

/*****************************************************************************/
/**
*
* Debug printing functions
*
* @param    See function.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/

void print_buf(unsigned char *buf, int size)
{
	int i;

	xil_printf("Buffer: (0x%x bytes)\n", size);

	for (i=0; i<size; i++) {
        xil_printf("%2x ", buf[i]);
        if ( (((i + 1) % 16) == 0) && ((i + 1) != size) ) {
            xil_printf("\n");
        }
	}
	xil_printf("\n\n");
}

#endif


