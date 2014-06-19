////////////////////////////////////////////////////////////////////////////////
// File   :	wl_transport.h
// Authors:	Chris Hunter (chunter [at] mangocomm.com)
//			Patrick Murphy (murphpo [at] mangocomm.com)
//          Erik Welsh (welsh [at] mangocomm.com)
// License:	Copyright 2013, Mango Communications. All rights reserved.
//			Distributed under the WARP license  (http://warpproject.org/license)
////////////////////////////////////////////////////////////////////////////////


/***************************** Include Files *********************************/
#include "wl_common.h"
#include <xilnet_udp.h>

#ifdef WARP_HW_VER_v3
#include <xaxiethernet.h>	// defines Axi Ethernet APIs
#endif


/*************************** Constant Definitions ****************************/
#ifndef WL_TRANSPORT_H_
#define WL_TRANSPORT_H_


// Define WARPLab Command IDs
#define TRANS_PING_CMDID 			1
#define TRANS_IP_CMDID 				2
#define TRANS_PORT_CMDID 			3
#define TRANS_PAYLOADSIZETEST_CMDID 4
#define TRANS_NODEGRPID_ADD			5
#define TRANS_NODEGRPID_CLEAR		6

#define WL_NUM_ETH_DEVICES          XILNET_NUM_ETH_DEVICES

#ifdef WARP_HW_VER_v2
#define WL_ETH_A                    TRIMODE_MAC_GMII
#define WL_ETH_A_MAC_DEVICE_ID		XPAR_LLTEMAC_0_DEVICE_ID
#define WL_ETH_A_SPEED              1000
#define WL_ETH_A_MDIO_PHYADDR       0x0

#define TEMAC_DEVICE_ID		        XPAR_LLTEMAC_0_DEVICE_ID
#define FIFO_DEVICE_ID		        XPAR_LLFIFO_0_DEVICE_ID
#endif

#ifdef WARP_HW_VER_v3
#define WL_ETH_A                    ETH_A_MAC
#define WL_ETH_A_MAC_DEVICE_ID      XPAR_ETH_A_MAC_DEVICE_ID
#define WL_ETH_A_SPEED              1000
#define WL_ETH_A_MDIO_PHYADDR       0x6

#define WL_ETH_B                    ETH_B_MAC
#define WL_ETH_B_MAC_DEVICE_ID      XPAR_ETH_B_MAC_DEVICE_ID
#define WL_ETH_B_SPEED              1000
#define WL_ETH_B_MDIO_PHYADDR       0x7
#endif

#define PAYLOAD_OFFSET              LINK_HDR_LEN+IP_HDR_LEN*4+UDP_HDR_LEN+PAYLOAD_PAD_NBYTES

#define TRANSPORT_ROBUST_MASK       0x1

#define wl_conv_eth_dev_num(x)      (char)(((int)'A') + x)

#define ETHPHYREG_17_0_LINKUP       0x0400

#define WAITDURATION_SEC            2

#define ETHERTYPE_IP                0x0800
#define IPPROTO_UDP                 0x11

#define PKTTYPE_TRIGGER             0
#define PKTTYPE_HTON_MSG            1
#define PKTTYPE_NTOH_MSG            2



/*********************** Global Structure Definitions ************************/

typedef struct {
	u16 destID;
	u16 srcID;
	u8 reserved;
	u8 pktType;
	u16 length;
	u16 seqNum;
	u16 flags;
} wl_transport_header;

// Over-the-wire structs
// Ethernet: 14 bytes
typedef struct{
	u8 dst_addr[6];
	u8 src_addr[6];
	u16 ethertype;
} ethernet_header;

// IPv4: 20 bytes
typedef struct{
	u8 ver_ihl;              // Version (4 bits) + Internet header length (4 bits)
	u8 tos;                  // Type of service
	u16 tlen;                // Total length
	u16 identification;      // Identification
	u16 flags_fo;            // Flags (3 bits) + Fragment offset (13 bits)
    u8 ttl;                  // Time to live
    u8 proto;                // Protocol
    u16 checksum;            // Header checksum
    u32 src_addr;            // Source address
    u32 dst_addr;            // Destination address
} ipv4_header;

// UDP: 4 bytes
typedef struct {
	u16 src_port;            // Source port
	u16 dst_port;            // Destination port
	u16 len;                 // Datagram length
	u16 checksum;            // Checksum
} udp_header;

// WARPLab Ethernet device structure
typedef struct  {

	// Ethernet instances
	unsigned int          mac_device_id;

#ifdef WARP_HW_VER_v2
	XLlTemac              mac_inst;
	XLlTemac_Config     * mac_cfg_ptr;
#endif

	#ifdef WARP_HW_VER_v3
	XAxiEthernet          mac_inst;
	XAxiEthernet_Config * mac_cfg_ptr;
#endif

	// Pointers to WARPLab specific offsets
	wl_transport_header * wl_header_tx;
	unsigned int        * wl_TX_payload_ptr32;

	// Other Ethernet device specific fields
	unsigned int          eth_mdio_phyaddr;
	unsigned int          eth_speed;

} wl_eth_device;




/*************************** Function Prototypes *****************************/

int  transport_init(unsigned int);
int  transport_linkStatus(unsigned int);
int  transport_processCmd(const wl_cmdHdr*, const void*, wl_respHdr*, void*, void*, unsigned int);
void transport_receiveCallback(unsigned char*, unsigned int, void*, unsigned int);
int  transport_setReceiveCallback(void(*handler));
void transport_poll(unsigned int);
void transport_send(wl_host_message*, pktSrcInfo*, unsigned int);
void transport_close(unsigned int);

int  transport_get_hw_info( unsigned int, unsigned char*, unsigned char*);
int  transport_config_sockets(unsigned int);

#endif /* WL_TRANSPORT_H_ */
