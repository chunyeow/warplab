////////////////////////////////////////////////////////////////////////////////
// File   :	wl_common.h
// Authors:	Chris Hunter (chunter [at] mangocomm.com)
//			Patrick Murphy (murphpo [at] mangocomm.com)
//          Erik Welsh (welsh [at] mangocomm.com)
// License:	Copyright 2013, Mango Communications. All rights reserved.
//			Distributed under the WARP license  (http://warpproject.org/license)
////////////////////////////////////////////////////////////////////////////////

//Include xil_types here (instead of in .c) so function prototypes can use u8/u16/u32 data types
#include "xil_types.h"
#include "warp_hw_ver.h"

#ifdef WARP_HW_VER_v2
#include "sleep.h"
#endif

#ifndef WL_COMMON_H_
#define WL_COMMON_H_

// **********************************************************************
// The values below must match the corresponding values in wl_config.ini
// **********************************************************************

//Version info (MAJOR.MINOR.REV, all must be ints)
// m-code requires C code MAJOR.MINOR match values in wl_version.ini
#define WARPLAB_VER_MAJOR	7
#define WARPLAB_VER_MINOR	4
#define WARPLAB_VER_REV		0

#define REQ_HW_VER          (WARPLAB_VER_MAJOR<<16)|(WARPLAB_VER_MINOR<<8)|(WARPLAB_VER_REV)

//Default network info
// The base IP address should be a u32 with (at least) the last octet 0x00
#define NODE_IP_ADDR_BASE           0x0a000000 //10.0.0.0
#define BROADCAST_ID                0xFFFF

//Default ports- unicast ports are used for host-to-node, multicast for triggers and host-to-multinode
#define NODE_UDP_UNICAST_PORT_BASE	9000
#define NODE_UDP_MCAST_BASE			10000
// **********************************************************************

#define WARPLAB_CONFIG_4RF 1

#define PAYLOAD_PAD_NBYTES 2

#ifdef WARP_HW_VER_v3
#define USERIO_BASEADDR		XPAR_W3_USERIO_BASEADDR
#define CLK_BASEADDR 		XPAR_W3_CLOCK_CONTROLLER_0_BASEADDR
#define EEPROM_BASEADDR 	XPAR_W3_IIC_EEPROM_ONBOARD_BASEADDR
#define DRAM_BASEADDR 		XPAR_DDR3_2GB_SODIMM_MPMC_BASEADDR
#define RC_BASEADDR 		XPAR_RADIO_CONTROLLER_0_BASEADDR
#define AD_BASEADDR 		XPAR_W3_AD_CONTROLLER_0_BASEADDR
#endif

#ifdef WARP_HW_VER_v2
#define EEPROM_BASEADDR		XPAR_EEPROM_CONTROLLER_MEM0_BASEADDR
#define RC_BASEADDR 		XPAR_RADIO_CONTROLLER_0_BASEADDR
#define USERIO_BASEADDR		XPAR_USERIO_BASEADDR
#endif

#define WARPLAB_TXBUFF_RADIO1 XPAR_WARPLAB_BUFFERS_MEMMAP_TXBUFF_RADIO1
#define WARPLAB_TXBUFF_RADIO2 XPAR_WARPLAB_BUFFERS_MEMMAP_TXBUFF_RADIO2
#define WARPLAB_TXBUFF_RADIO3 XPAR_WARPLAB_BUFFERS_MEMMAP_TXBUFF_RADIO3
#define WARPLAB_TXBUFF_RADIO4 XPAR_WARPLAB_BUFFERS_MEMMAP_TXBUFF_RADIO4
#define WARPLAB_RXBUFF_RADIO1 XPAR_WARPLAB_BUFFERS_MEMMAP_RXBUFF_RADIO1
#define WARPLAB_RXBUFF_RADIO2 XPAR_WARPLAB_BUFFERS_MEMMAP_RXBUFF_RADIO2
#define WARPLAB_RXBUFF_RADIO3 XPAR_WARPLAB_BUFFERS_MEMMAP_RXBUFF_RADIO3
#define WARPLAB_RXBUFF_RADIO4 XPAR_WARPLAB_BUFFERS_MEMMAP_RXBUFF_RADIO4
#define WARPLAB_RSSIBUFF_RADIO1 XPAR_WARPLAB_BUFFERS_MEMMAP_RSSIBUFF_RADIO1
#define WARPLAB_RSSIBUFF_RADIO2 XPAR_WARPLAB_BUFFERS_MEMMAP_RSSIBUFF_RADIO2
#define WARPLAB_RSSIBUFF_RADIO3 XPAR_WARPLAB_BUFFERS_MEMMAP_RSSIBUFF_RADIO3
#define WARPLAB_RSSIBUFF_RADIO4 XPAR_WARPLAB_BUFFERS_MEMMAP_RSSIBUFF_RADIO4

#define TIMER_FREQ			XPAR_TMRCTR_0_CLOCK_FREQ_HZ
#define TMRCTR_DEVICE_ID	XPAR_TMRCTR_0_DEVICE_ID
#define TIMER_COUNTER_0	 0

#define RESP_SENT 1
#define NO_RESP_SENT 0

#define LINK_READY       0
#define LINK_NOT_READY  -1

#define SUCCESS          0
#define FAILURE         -1

#define WL_CMD_TO_GRP(x) ((x)>>24)
#define WL_CMD_TO_CMDID(x) ((x)&0xffffff)

//2: [cmdID,len]
//#define wl_getNumArgs(x) (x>>2)-2
//#define wl_getNumArgs(x) (x/sizeof(wl_arg))-2

//typedef unsigned int wl_arg;



typedef struct{
	u32 cmd;
	u16 length;
	u16 numArgs;
} wl_cmdHdr;

typedef struct{
	void *buffer;
	void *payload;
	u32 length;
} wl_host_message;


typedef wl_cmdHdr wl_respHdr;

int wl_gpio_debug_initialize();
inline void wl_setDebugGPIO(u8 mask);
inline void wl_clearDebugGPIO(u8 mask);
int wl_timer_initialize();
unsigned char sevenSegmentMap(unsigned char x);

#ifdef WARP_HW_VER_v3
void usleep(unsigned int duration);
void wl_set_timer();
void wl_setback_timer();
u32 get_timestamp();
#endif


#endif /* WL_COMMON_H_ */
