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
* @file wl_transport.c
*
* Implements the WARPLab Transport protocol layer for the embedded processor
* on the WARP Hardware.
*
* This implementation supports both WARP v2 and WARP v3 hardware and also
* supports both the use of the AXI FIFO as well as the AXI DMA in order
* to transport packets to/from the Ethernet controller.
*
* <pre>
* MODIFICATION HISTORY:
*
* Ver   Who  Date     Changes
* ----- ---- -------- -------------------------------------------------------
* 2.00a ejw  5/24/13  Updated for new Xilnet driver:  3.02.a
*
* </pre>
*
******************************************************************************/


// TODO:  Allow DMA initialization to different buffer location (such as warlab buffers)
// #ifdef _USE_WARPLAB_BUFFERS_
// unsigned int *recvBuffer = (unsigned int *) 0x730C0000; // WARPlab Buffer
// #endif



/***************************** Include Files *********************************/

#include "warp_hw_ver.h"
#include "include/wl_transport.h"
#include "include/wl_common.h"
#include "include/wl_trigger_manager.h"
#include "include/wl_node.h"

#include <stdlib.h>
#include <stdio.h>

#include <xparameters.h>
#include <xilnet_config.h>
#include <xtmrctr.h>

#ifdef WARP_HW_VER_v3
#include <xaxiethernet.h>	// defines Axi Ethernet APIs
#include <w3_iic_eeprom.h>
#include <w3_userio.h>
#endif

#ifdef WARP_HW_VER_v2
#include <xlltemac.h>
#include "warp_v4_userio.h"
#endif



/*************************** Constant Definitions ****************************/



/*********************** Global Variable Definitions *************************/

extern u16 node;

//TODO: Wrap timer calls in a driver in wl_common
extern XTmrCtr TimerCounter;


/*************************** Variable Definitions ****************************/

// NOTE:  This structure has different member types depending on the WARP version
wl_eth_device       wl_eth_devices[WL_NUM_ETH_DEVICES];

unsigned int        myPort;
wl_host_message     toNode, fromNode;

u8                  node_group_ID_membership;

// Global socket variables
int                 sock_unicast      = -1; // UDP socket for unicast traffic to / from the board
struct sockaddr_in  addr_unicast;

int                 sock_bcast        = -1; // UDP socket for broadcast traffic to the board
struct sockaddr_in  addr_bcast;



/*************************** Function Prototypes *****************************/

void nullCallback(void* param){};
void (*usr_receiveCallback) ();

int transport_get_hw_info( unsigned int, unsigned char*, unsigned char* );
int transport_config_sockets(unsigned int);

/******************************** Functions **********************************/


/*****************************************************************************/
/**
*
* This function is the Transport callback that allows WARPLab to process
* a received Ethernet packet
*
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
* @note		The transport must be initialized before this function will be
*       called.  The user can modify the file to add additional functionality
*       to the WARPLAB Transport.
*
******************************************************************************/

int transport_processCmd(const wl_cmdHdr* cmdHdr,const void* cmdArgs, wl_respHdr* respHdr,void* respArgs, void* pktSrc, unsigned int eth_dev_num){
	//IMPORTANT ENDIAN NOTES:
	// -cmdHdr is safe to access directly (pre-swapped if needed)
	// -cmdArgs is *not* pre-swapped, since the framework doesn't know what it is
	// -respHdr will be swapped by the framework; user code should fill it normally
	// -respArgs will *not* be swapped by the framework, since only user code knows what it is
	//    Any data added to respArgs by the code below must be endian-safe (swapped on AXI hardware)

	unsigned int  respSent   = NO_RESP_SENT;
	const u32    *cmdArgs32  = cmdArgs;
	u32          *respArgs32 = respArgs;
	unsigned int  respIndex  = 0;
	unsigned int  cmdID      = WL_CMD_TO_CMDID(cmdHdr->cmd);


#ifdef _DEBUG_
	xil_printf("BEGIN transport_processCmd() \n");
#endif

	respHdr->cmd = cmdHdr->cmd;
	respHdr->length = 0;
	respHdr->numArgs = 0;

	switch(cmdID){
			case TRANS_PING_CMDID:
				//Nothing actually needs to be done when receiving the ping command. The framework is going
				//to respond regardless, which is all the host wants.
			break;
			case TRANS_IP_CMDID:
				//TODO: Only necessary for generalized discovery process
			break;
			case TRANS_PORT_CMDID:
				//TODO: Only necessary for generalized discovery process
			break;
			case TRANS_PAYLOADSIZETEST_CMDID:
				respArgs32[respIndex++] = Xil_Ntohl(toNode.length - PAYLOAD_PAD_NBYTES);
				respHdr->length += (respIndex * sizeof(respArgs32));
				respHdr->numArgs = 1;
			break;
			case TRANS_NODEGRPID_ADD:
				node_group_ID_membership = node_group_ID_membership | Xil_Htonl(cmdArgs32[0]);
			break;
			case TRANS_NODEGRPID_CLEAR:
				node_group_ID_membership = node_group_ID_membership &  ~Xil_Htonl(cmdArgs32[0]);
			break;
			default:
				xil_printf("Unknown user command ID: %d\n", cmdID);
			break;
	}

#ifdef _DEBUG_
	xil_printf("END transport_processCmd() \n");
#endif

	return respSent;
}



/*****************************************************************************/
/**
*
* This function is the Transport callback that allows WARPLab to process
* a received Ethernet packet
*
*
* @param	buff is a pointer to the received packet buffer
* @param	len is an int that specifies the length of the recieved packet
* @param	pktSrc is a pointer to the Ethenet packet source structure
* @param    eth_dev_num is an int that specifies the Ethernet interface to use
*
* @return	-NO_RESP_SENT to indicate no response has been sent
*		-RESP_SENT to indicate a response has been sent
*
* @note		The transport must be initialized before this function will be
*       called.  The user can modify the file to add additional functionality
*       to the WARPLAB Transport.
*
******************************************************************************/


void transport_receiveCallback(unsigned char* buff, unsigned int len, void* pktSrc, unsigned int eth_dev_num){

	u32 trigID;

	unsigned char* buffPtr = buff + PAYLOAD_PAD_NBYTES;

#ifdef _DEBUG_
	xil_printf("BEGIN transport_receiveCallback() \n");
#endif

	toNode.buffer  = buffPtr;
	toNode.payload = (unsigned int *)((unsigned char *) buffPtr + sizeof(wl_transport_header));
	toNode.length  = len;

	fromNode.buffer  = (unsigned char *)eth_device[eth_dev_num].sendbuf;
    fromNode.payload = wl_eth_devices[eth_dev_num].wl_TX_payload_ptr32;
	fromNode.length  = PAYLOAD_PAD_NBYTES;

	wl_transport_header* wl_header_rx = (wl_transport_header*)buffPtr;

	//Endian swap the received transport header (this is the first place we know what/where it is)
	wl_header_rx->destID = Xil_Ntohs(wl_header_rx->destID);
	wl_header_rx->srcID  = Xil_Ntohs(wl_header_rx->srcID);
	wl_header_rx->length = Xil_Ntohs(wl_header_rx->length);
	wl_header_rx->seqNum = Xil_Ntohs(wl_header_rx->seqNum);
	wl_header_rx->flags  = Xil_Ntohs(wl_header_rx->flags);

	switch(wl_header_rx->pktType){
		case PKTTYPE_TRIGGER:
			trigID = Xil_Ntohl(*(u32*)(toNode.payload));
			trigmngr_triggerIn(trigID);
		break;

		case PKTTYPE_HTON_MSG:
			if( ( (wl_header_rx->destID) != node        ) &&
				( (wl_header_rx->destID) != BROADCAST_ID) &&
				( ((wl_header_rx->destID) & (0xFF00 | node_group_ID_membership))==0 ) ) return;

			//Form outgoing WARPLab header in case robust mode is on
			// Note- the u16/u32 fields here will be endian swapped in transport_send
			wl_eth_devices[eth_dev_num].wl_header_tx->destID  = wl_header_rx->srcID;
			wl_eth_devices[eth_dev_num].wl_header_tx->seqNum  = wl_header_rx->seqNum;
			wl_eth_devices[eth_dev_num].wl_header_tx->srcID   = node;
			wl_eth_devices[eth_dev_num].wl_header_tx->pktType = PKTTYPE_NTOH_MSG;

			usr_receiveCallback(&toNode, &fromNode, pktSrc, eth_dev_num);

			if( ((wl_header_rx->flags) & TRANSPORT_ROBUST_MASK) && fromNode.length > PAYLOAD_PAD_NBYTES) {
				transport_send(&fromNode, pktSrc, eth_dev_num);
			}

			fromNode.length = 0;
		break;
		default:
			xil_printf("Got packet with unknown type: %d\n", wl_header_rx->pktType);
			break;
	}

#ifdef _DEBUG_
	xil_printf("END transport_receiveCallback() \n");
#endif
}




/*****************************************************************************/
/**
*
* Close the WARPLab Transport stream by closing the Xilnet Socket
*
* @param    eth_dev_num is an int that specifies the Ethernet interface to use
*
* @return	None.
*
* @note		None.
*
****************************************************************************/

void transport_close(unsigned int eth_dev_num) {
	xilsock_close(sock_unicast, eth_dev_num);
	xilsock_close(sock_bcast, eth_dev_num);
}



/*****************************************************************************/
/**
*
* This function will initialize the transport
*
* @param    eth_dev_num is an int that specifies the Ethernet interface to use
*
* @return	-SUCCESS to indicate that the transport was initialized
*		-FAILURE to indicated that the transport was not initialized
*
* @note		None.
*
******************************************************************************/
int transport_init( unsigned int eth_dev_num ){

	int                   status = SUCCESS;

	unsigned char         node_ip_addr[IP_VERSION];
	unsigned char         node_hw_addr[ETH_ADDR_LEN];
	unsigned int          mac_device_id;

#ifdef WARP_HW_VER_v2
    XLlTemac            * mac_instance_ptr;
    XLlTemac_Config     * mac_cfg_ptr;
#endif

#ifdef WARP_HW_VER_v3
	XAxiEthernet        * mac_instance_ptr;
    XAxiEthernet_Config * mac_cfg_ptr;
#endif



	xil_printf("Configuring transport...\n");

    // Initialize the User callback
	usr_receiveCallback = nullCallback;

    // Reset node group ID membership
	node_group_ID_membership = 0;

	/******************* Ethernet ********************************/

	// Check to see if we are receiving on a valid interface
	if ( eth_dev_num >= WL_NUM_ETH_DEVICES ) {
		xil_printf("  **** ERROR:  Ethernet %d is not available on WARP HW \n", eth_dev_num);
		return FAILURE;
	}

#ifdef WARP_HW_VER_v3

	// Populate the wl_eth_devices structure with constants from wl_transport.h
	switch (eth_dev_num) {
	case WL_ETH_A:
		wl_eth_devices[eth_dev_num].mac_device_id        = WL_ETH_A_MAC_DEVICE_ID;
		wl_eth_devices[eth_dev_num].eth_mdio_phyaddr     = WL_ETH_A_MDIO_PHYADDR;
		wl_eth_devices[eth_dev_num].eth_speed            = WL_ETH_A_SPEED;
		break;
	case WL_ETH_B:
		wl_eth_devices[eth_dev_num].mac_device_id        = WL_ETH_B_MAC_DEVICE_ID;
		wl_eth_devices[eth_dev_num].eth_mdio_phyaddr     = WL_ETH_B_MDIO_PHYADDR;
		wl_eth_devices[eth_dev_num].eth_speed            = WL_ETH_B_SPEED;
		break;
	default:
		break;
	}

	// Pull information based on the Ethernet device to initialize
	mac_device_id    =  wl_eth_devices[eth_dev_num].mac_device_id;
	mac_instance_ptr = &wl_eth_devices[eth_dev_num].mac_inst;

	// Find Ethernet device
	mac_cfg_ptr = XAxiEthernet_LookupConfig(mac_device_id);

	// Initialize Xilnet for Ethernet device
	status = transport_get_hw_info( eth_dev_num, (unsigned char*)&node_ip_addr, (unsigned char*)&node_hw_addr);
	if (status != SUCCESS)
		xil_printf("*** Transport get HW info error:  %d \n", status);

	status = xilnet_eth_device_init( eth_dev_num, mac_cfg_ptr->AxiDevBaseAddress, (unsigned char*)&node_ip_addr, (unsigned char*)&node_hw_addr );
	if (status != SUCCESS)
		xil_printf("*** Transport Xilnet Ethernet Device %d initialization error:  %d \n", eth_dev_num, status);

	xilnet_eth_init_hw_addr_tbl(eth_dev_num);

	// Initialize Ethernet Device
	status = XAxiEthernet_CfgInitialize(mac_instance_ptr, mac_cfg_ptr, mac_cfg_ptr->BaseAddress);
	if (status != XST_SUCCESS)
		xil_printf("*** EMAC init error\n");

	status  = XAxiEthernet_ClearOptions(mac_instance_ptr, XAE_LENTYPE_ERR_OPTION | XAE_FLOW_CONTROL_OPTION | XAE_FCS_STRIP_OPTION);
	if (status != XST_SUCCESS)
		xil_printf("*** Error clearing EMAC A options\n, code %d", status);

	status |= XAxiEthernet_SetOptions(mac_instance_ptr, XAE_PROMISC_OPTION | XAE_MULTICAST_OPTION | XAE_BROADCAST_OPTION | XAE_RECEIVER_ENABLE_OPTION | XAE_TRANSMITTER_ENABLE_OPTION | XAE_JUMBO_OPTION);
	if (status != XST_SUCCESS)
		xil_printf("*** Error setting EMAC A options\n, code %d", status);

	XAxiEthernet_SetOperatingSpeed(mac_instance_ptr, wl_eth_devices[eth_dev_num].eth_speed);
	usleep(1 * 10000);

	XAxiEthernet_Start(mac_instance_ptr);

	// Start Xilnet
	xilnet_eth_device_start(eth_dev_num);
#endif    // END WARP_HW_VER_v3


#ifdef WARP_HW_VER_v2

	// Populate the wl_eth_devices structure with constants from wl_transport.h
	wl_eth_devices[eth_dev_num].mac_device_id        = WL_ETH_A_MAC_DEVICE_ID;
	wl_eth_devices[eth_dev_num].eth_mdio_phyaddr     = WL_ETH_A_MDIO_PHYADDR;
	wl_eth_devices[eth_dev_num].eth_speed            = WL_ETH_A_SPEED;

	// Pull information based on the Ethernet device to initialize
	mac_device_id    =  wl_eth_devices[eth_dev_num].mac_device_id;
	mac_instance_ptr = &wl_eth_devices[eth_dev_num].mac_inst;

	// Find Ethernet device
	mac_cfg_ptr = XLlTemac_LookupConfig(mac_device_id);

	// Initialize Xilnet for Ethernet device
	status = transport_get_hw_info( eth_dev_num, (unsigned char*)&node_ip_addr, (unsigned char*)&node_hw_addr);
	if (status != SUCCESS)
		xil_printf("*** Transport get HW info error:  %d \n", status);

//	status = xilnet_eth_device_init( eth_dev_num, XLlTemac_LlDevBaseAddress(mac_instance_ptr), (unsigned char*)&node_ip_addr, (unsigned char*)&node_hw_addr );
	status = xilnet_eth_device_init( eth_dev_num, mac_cfg_ptr->LLDevBaseAddress, (unsigned char*)&node_ip_addr, (unsigned char*)&node_hw_addr );
	if (status != SUCCESS)
		xil_printf("*** Transport Xilnet Ethernet Device %d initialization error:  %d \n", eth_dev_num, status);

	xilnet_eth_init_hw_addr_tbl(eth_dev_num);

	// Initialize Ethernet Device
	status = XLlTemac_CfgInitialize(mac_instance_ptr, mac_cfg_ptr, mac_cfg_ptr->BaseAddress);
	if (status != XST_SUCCESS)
		xil_printf("*** EMAC init error\n");

	status  = XLlTemac_ClearOptions(mac_instance_ptr, XTE_LENTYPE_ERR_OPTION | XTE_FLOW_CONTROL_OPTION | XTE_FCS_STRIP_OPTION);
	if (status != XST_SUCCESS)
		xil_printf("*** Error clearing EMAC A options\n, code %d", status);

	status |= XLlTemac_SetOptions(mac_instance_ptr, XTE_PROMISC_OPTION | XTE_MULTICAST_OPTION | XTE_BROADCAST_OPTION | XTE_RECEIVER_ENABLE_OPTION | XTE_TRANSMITTER_ENABLE_OPTION | XTE_JUMBO_OPTION);
	if (status != XST_SUCCESS)
		xil_printf("*** Error setting EMAC A options\n, code %d", status);

	XLlTemac_SetOperatingSpeed(mac_instance_ptr, wl_eth_devices[eth_dev_num].eth_speed);
	usleep(1 * 10000);

	XLlTemac_Start(mac_instance_ptr);

	// Start Xilnet
	xilnet_eth_device_start(eth_dev_num);
#endif    // END WARP_HW_VER_v2


	// Populate the wl_eth_devices structure
	wl_eth_devices[eth_dev_num].mac_cfg_ptr          = mac_cfg_ptr;
	wl_eth_devices[eth_dev_num].wl_header_tx         = (wl_transport_header*)(((unsigned char *)eth_device[eth_dev_num].sendbuf) + PAYLOAD_OFFSET);
	wl_eth_devices[eth_dev_num].wl_header_tx->srcID  = node;
	wl_eth_devices[eth_dev_num].wl_TX_payload_ptr32  = (unsigned int *)((((unsigned char *)eth_device[eth_dev_num].sendbuf) + PAYLOAD_OFFSET) + sizeof(wl_transport_header));

	// Configure the Sockets for each Ethernet Interface
	status = transport_config_sockets(eth_dev_num);

	return status;
}



/*****************************************************************************/
/**
*
* This is a helper function that will determine the MAC and IP addresses
*   for a given interface
*
*
* @param    eth_dev_num is an int that specifies the Ethernet interface to use
* @param	ip_addr is a pointer to an unsigned char array to store the IP address
* @param	hw_addr is a pointer to an unsigned char array to store the MAC address
*
* @return	-SUCCESS to indicate that the MAC and IP are valid
*		-FAILURE to indicate that the MAC and IP are not valid
*
* @note		None.
*
******************************************************************************/

int transport_get_hw_info( unsigned int eth_dev_num, unsigned char* ip_addr, unsigned char* hw_addr) {
	int status = SUCCESS;

#ifdef WARP_HW_VER_v3
	//Read the MAC addr from the EEPROM
	w3_eeprom_readEthAddr(EEPROM_BASEADDR, eth_dev_num, hw_addr);

	xil_printf("  Set MAC Address ETH %c via EEPROM: %02x:%02x:%02x:%02x:%02x:%02x\n",
               wl_conv_eth_dev_num(eth_dev_num), hw_addr[0], hw_addr[1], hw_addr[2], hw_addr[3], hw_addr[4], hw_addr[5]);
#endif

#ifdef WARP_HW_VER_v2
	//Use the Rice WARP range of MAC addresses for v2 boards
	hw_addr[0] = 0x00;
	hw_addr[1] = 0x50;
	hw_addr[2] = 0xC2;
	hw_addr[3] = 0x63;
	hw_addr[4] = 0x30;
	hw_addr[5] = node;
#endif

	ip_addr[0] = (NODE_IP_ADDR_BASE>>24)&0xFF;
	ip_addr[1] = (NODE_IP_ADDR_BASE>>16)&0xFF;
	ip_addr[2] = (NODE_IP_ADDR_BASE>>8)&0xFF;
	ip_addr[3] = (node+1);                           // IP ADDR = x.y.z.( node + 1 )
// TODO:  Modify code to allow both Ethernet devices to be used at the same time
//	ip_addr[3] = (eth_dev_num<<6) + (node+1);        // IP ADDR = x.y.z.( 64*eth_dev_num + node + 1 )

	xil_printf("  ETH %c IP Address: %d.%d.%d.%d \n", wl_conv_eth_dev_num(eth_dev_num), ip_addr[0], ip_addr[1], ip_addr[2], ip_addr[3]);

    return status;
}



/*****************************************************************************/
/**
*
* This function will configure the Xilnet sockets to be used by the transport
*
*
* @param    eth_dev_num is an int that specifies the Ethernet interface to use
*
* @return	-SUCCESS to indicate sockets successfully configured
*		-FAILURE to indicate the sockets were not configured
*
* @note		None.
*
******************************************************************************/

int transport_config_sockets(unsigned int eth_dev_num) {
	int status = SUCCESS;
	int tempStatus = 0;

    if ( node == 0xFFFF ) {
	    myPort = NODE_UDP_UNICAST_PORT_BASE;
    } else {
	    myPort = NODE_UDP_UNICAST_PORT_BASE + node;
    }

    //-----------------------------------------------------
    // Unicast PORT

    // Release socket if it is already bound
    if ( sock_unicast  != -1 ) { xilsock_close( sock_unicast,  eth_dev_num ); }

    // Create new socket and bind it
    //   UDP socket with domain Internet and UDP connection.
    sock_unicast = xilsock_socket(AF_INET, SOCK_DGRAM, 0, eth_dev_num);
	if (sock_unicast == -1) {
		xil_printf("Error in creating sock_unicast\n");
		status = FAILURE;
		return status;
	}

	addr_unicast.sin_family = AF_INET;
	addr_unicast.sin_port = myPort;                     // TODO: FIX - no need for different unicast ports
	addr_unicast.sin_addr.s_addr = INADDR_ANY;			// Create the input socket with any incoming address. (0x00000000)

	tempStatus = xilsock_bind(sock_unicast, (struct sockaddr *)&addr_unicast, sizeof(struct sockaddr),(void *)transport_receiveCallback, eth_dev_num);
	if (tempStatus != 1) {
		xil_printf("Unable to bind sock_msg\n");
		status = -1;
		return status;
	}


    //-----------------------------------------------------
    // Broadcast PORT

    // Release socket if it is already bound
    if ( sock_bcast != -1 ) { xilsock_close( sock_bcast, eth_dev_num ); }

    // Create new socket and bind it
    //   UDP socket with domain Internet and UDP connection.
    sock_bcast = xilsock_socket(AF_INET, SOCK_DGRAM, 0, eth_dev_num);
	if (sock_bcast == -1) {
		xil_printf("Error in creating sock_msg\n");
		status = FAILURE;
		return status;
	}

	addr_bcast.sin_family = AF_INET;
	addr_bcast.sin_port = 10000;                        // TODO:  FIX hard coded broadcast port
	addr_bcast.sin_addr.s_addr = INADDR_ANY;			// Create the input socket with any incoming address. (0x00000000)

	tempStatus = xilsock_bind(sock_bcast, (struct sockaddr *)&addr_bcast, sizeof(struct sockaddr),(void *)transport_receiveCallback, eth_dev_num);
	if (tempStatus != 1) {
		xil_printf("Unable to bind sock_trig\n");
		status = -1;
		return status;
	}

	xil_printf("  Listening on UDP ports %d (unicast) and %d (broadcast)\n", myPort, NODE_UDP_MCAST_BASE);

	return status;
}



/*****************************************************************************/
/**
*
* This function is the Transport callback that allows WARPLab to process
* a received Ethernet packet
*
*
* @param	handler is a pointer to the receive callback function
*
* @return	Always returns SUCCESS
*
* @note		None.
*
******************************************************************************/

int transport_setReceiveCallback(void(*handler)){
	usr_receiveCallback = handler;
	return SUCCESS;
}



/*****************************************************************************/
/**
*
* This function will poll the Ethernet interfaces
*
*
* @param    eth_dev_num is an int that specifies the Ethernet interface to use
*
* @return	None.
*
* @note		None.
*
******************************************************************************/

void transport_poll(unsigned int eth_dev_num){
    xilnet_eth_recv_frame(eth_dev_num);
}


/*****************************************************************************/
/**
*
* This function is used to send a message over Ethernet
*
*
* @param	currMsg is a pointer to the WARPLab Host Message
* @param	pktSrc is a pointer to the Ethenet packet source structure
* @param    eth_dev_num is an int that specifies the Ethernet interface to use
*
* @return	None.
*
* @note		The transport must be initialized before this function will be
*       called.  The user can modify the file to add additional functionality
*       to the WARPLAB Transport.
*
******************************************************************************/

void transport_send(wl_host_message* currMsg, pktSrcInfo* pktSrc, unsigned int eth_dev_num){
	wl_transport_header   tmp_hdr;
	wl_transport_header * wl_header_tx;
	int                   len_to_send;
	struct sockaddr_in    sendAddr;

#ifdef _DEBUG_
	xil_printf("BEGIN transport_send() \n");
#endif

	// Initialize the header and length
	wl_header_tx = (wl_transport_header *)(currMsg->buffer + PAYLOAD_OFFSET);
	len_to_send  = currMsg->length + sizeof(wl_transport_header);

	// Grab a copy of the current header
	//     The send function will perform a network swap of the header to send
	//     over the wire and then restore the original version for future
	//     processing
	memcpy(&tmp_hdr, wl_header_tx, sizeof(wl_transport_header));

	// Update length field in outgoing transport header
	wl_header_tx->length = len_to_send;

	// Make the outgoing transport header endian safe for sending on the network
	wl_header_tx->destID = Xil_Htons(wl_header_tx->destID);
	wl_header_tx->srcID  = Xil_Htons(wl_header_tx->srcID);
	wl_header_tx->length = Xil_Htons(wl_header_tx->length);
	wl_header_tx->seqNum = Xil_Htons(wl_header_tx->seqNum);
	wl_header_tx->flags  = Xil_Htons(wl_header_tx->flags);

	// Create address structure based on the pkt info structure
	sendAddr.sin_addr.s_addr = pktSrc->srcIPAddr;
	sendAddr.sin_family      = AF_INET;
	sendAddr.sin_port        = pktSrc->destPort;

#ifdef _DEBUG_
	xil_printf("sendAddr.sin_addr.s_addr = %d.%d.%d.%d\n",(sendAddr.sin_addr.s_addr>>24)&0xFF,
			                                              (sendAddr.sin_addr.s_addr>>16)&0xFF,
			                                              (sendAddr.sin_addr.s_addr>>8)&0xFF,
			                                              (sendAddr.sin_addr.s_addr)&0xFF);
	xil_printf("sendAddr.sin_family      = %d\n",sendAddr.sin_family);
	xil_printf("sendAddr.sin_port        = %d\n",sendAddr.sin_port);

	xil_printf("buffer                   = 0x%x;\n", currMsg->buffer );
	xil_printf("len                      = %d;  \n", len_to_send );

//	print_pkt((unsigned char *)eth_device[eth_dev_num].sendbuf, len_to_send);
#endif

	xilsock_sendto(sock_unicast, (unsigned char *)currMsg->buffer, len_to_send, (struct sockaddr *)&sendAddr, eth_dev_num);

	//Restore wl_header_tx
	memcpy(wl_header_tx, &tmp_hdr, sizeof(wl_transport_header));

#ifdef _DEBUG_
	xil_printf("END transport_send() \n");
#endif
}



/*****************************************************************************/
/**
*
* This function will check the link status of all Ethernet controllers
*
*
* @param    eth_dev_num is an int that specifies the Ethernet interface to use
*
* @return	-LINK_READY to indicate both Ethernet controllers (depending on defines)
*       are ready to be used.
*		-LINK_NOT_READY to indicate one of the Ethernet controllers is not
*		read to be used.
*
* @note		The transport must be initialized before this function will be
*       called.  The user can modify the file to add additional functionality
*       to the WARPLAB Transport.
*
******************************************************************************/

int transport_linkStatus(unsigned int eth_dev_num) {

	u16 status = LINK_READY;
	u16 regA   = 0;

	// Check to see if we are receiving on a valid interface
	if ( eth_dev_num >= WL_NUM_ETH_DEVICES ) {
		xil_printf("  **** ERROR:  Ethernet %d is not available on WARP HW \n", eth_dev_num);
		return LINK_NOT_READY;
	}

//Check if the Ethernet PHY reports a valid link
#ifdef WARP_HW_VER_v2

	if (eth_dev_num != WL_ETH_A) {
		xil_printf("**** Error:  Ethernet B is not available on WARP HW V2\n");
		return LINK_NOT_READY;
	}

	XLlTemac_PhyRead(&wl_eth_devices[eth_dev_num].mac_inst, wl_eth_devices[eth_dev_num].eth_mdio_phyaddr, 17, &regA);
#endif    // End WARP_HW_VER_v2

#ifdef WARP_HW_VER_v3

	XAxiEthernet_PhyRead(&wl_eth_devices[eth_dev_num].mac_inst, wl_eth_devices[eth_dev_num].eth_mdio_phyaddr, 17, &regA);

#endif    // End WARP_HW_VER_v3

	if(regA & ETHPHYREG_17_0_LINKUP) {
		status = LINK_READY;
	} else {
		status = LINK_NOT_READY;
	}

	return status;
}




#ifdef WARP_HW_VER_v3
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

#ifdef _DEBUG_

void print_XAxiEthernet_Config( XAxiEthernet_Config * ETH_CFG_ptr ) {

	xil_printf("Ethernet Config Pointer: \n");
    xil_printf("  DeviceId:          0x%x \n", ETH_CFG_ptr->DeviceId);
    xil_printf("  BaseAddress:       0x%x \n", ETH_CFG_ptr->BaseAddress);
    xil_printf("  TemacType:         0x%x \n", ETH_CFG_ptr->TemacType);
    xil_printf("  Checksum:     TX:  0x%x      RX: 0x%x\n", ETH_CFG_ptr->TxCsum, ETH_CFG_ptr->RxCsum );
    xil_printf("  PhyType:           0x%x \n", ETH_CFG_ptr->PhyType);
    xil_printf("  VlanTran:     TX:  0x%x      RX: 0x%x\n", ETH_CFG_ptr->TxVlanTran, ETH_CFG_ptr->RxVlanTran);
    xil_printf("  VlanTag:      TX:  0x%x      RX: 0x%x\n", ETH_CFG_ptr->TxVlanTag, ETH_CFG_ptr->RxVlanTag);
    xil_printf("  VlanStrp:     TX:  0x%x      RX: 0x%x\n", ETH_CFG_ptr->TxVlanStrp, ETH_CFG_ptr->RxVlanStrp);
    xil_printf("  ExtMcast:          0x%x \n", ETH_CFG_ptr->ExtMcast);
    xil_printf("  Stats:             0x%x \n", ETH_CFG_ptr->Stats);
    xil_printf("  Avb:               0x%x \n", ETH_CFG_ptr->Avb);
    xil_printf("  TemacIntr:         0x%x \n", ETH_CFG_ptr->TemacIntr);
    xil_printf("  AxiDevBaseAddress: 0x%x \n", ETH_CFG_ptr->AxiDevBaseAddress);
    xil_printf("  AxiDevType:        0x%x \n", ETH_CFG_ptr->AxiDevType);
    xil_printf("  AxiFifoIntr:       0x%x \n", ETH_CFG_ptr->AxiFifoIntr);
    xil_printf("  AxiDmaRxIntr:      0x%x \n", ETH_CFG_ptr->AxiDmaRxIntr);
    xil_printf("  AxiDmaTxIntr:      0x%x \n", ETH_CFG_ptr->AxiDmaTxIntr);
	xil_printf("\n");
}

#endif

#endif
