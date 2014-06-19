////////////////////////////////////////////////////////////////////////////////
// File   :	wl_trigger_manager.c
// Authors:	Chris Hunter (chunter [at] mangocomm.com)
//			Patrick Murphy (murphpo [at] mangocomm.com)
//          Erik Welsh (welsh [at] mangocomm.com)
// License:	Copyright 2013, Mango Communications. All rights reserved.
//			Distributed under the WARP license  (http://warpproject.org/license)
////////////////////////////////////////////////////////////////////////////////

#include "include/wl_trigger_manager.h"
#include "include/wl_common.h"
#include "include/wl_node.h"
#include "include/wl_baseband.h"
#include "include/wl_transport.h"
#include "include/wl_interface.h"

#include <xparameters.h>
#include <stdlib.h>
#include <stdio.h>
#include <Xio.h>
#include <string.h>
#include <xil_types.h>
#include <xil_io.h>
#include <string.h>

u32 active_triggerID_mask;
u32 broadcast_test_flag;

int trigmngr_processCmd(const wl_cmdHdr* cmdHdr,const void* cmdArgs, wl_respHdr* respHdr, void* respArgs, void* pktSrc, unsigned int eth_dev_num){

	//Cast argument buffers into arrays for easy accessing
	const u32 *cmdArgs32 = cmdArgs;
	//const u16 *cmdArgs16 = cmdArgs;
	//const u8  *cmdArgs8  = cmdArgs;

	u32 *respArgs32 = respArgs;
	//u16 *respArgs16 = respArgs;
	//u8  *respArgs8  = respArgs;

	unsigned int respIndex = 0;

	unsigned int respSent = NO_RESP_SENT;

	unsigned int cmdID;

	cmdID = WL_CMD_TO_CMDID(cmdHdr->cmd);

	respHdr->cmd = cmdHdr->cmd;
	respHdr->length = 0;
	respHdr->numArgs = 0;
	u32 trigID;
	u32 k,j, OR_start, AND_start, OR_inputs, AND_inputs;
	u32 delay,MODE_input;
	switch(cmdID){
		case TRIG_MNGR_ADD_ETHERNET_TRIG:
			trigID = Xil_Ntohl(cmdArgs32[0]);

			//Add the new trigger ID to the internal mask of all enabled trigger IDs
			// Trigger IDs must be one-hot encoded
			active_triggerID_mask = active_triggerID_mask | trigID;

			#ifdef FAST_TRIG
			if(active_triggerID_mask) {
				wl_pkt_proc_setTrigID(0, active_triggerID_mask);
				trigger_proc_in_eth_clearReset();
			} else {
				trigger_proc_in_eth_setReset();
				wl_pkt_proc_setTrigID(0, active_triggerID_mask);
			}
			#endif

			respArgs32[respIndex++] = Xil_Htonl(active_triggerID_mask);

			respHdr->length += (respIndex * sizeof(respArgs32));
			respHdr->numArgs = respIndex;
		break;
		case TRIG_MNGR_DEL_ETHERNET_TRIG:
			trigID = Xil_Ntohl(cmdArgs32[0]);
			active_triggerID_mask = active_triggerID_mask & ~trigID;

			#ifdef FAST_TRIG
			if(active_triggerID_mask) {
				wl_pkt_proc_setTrigID(0, active_triggerID_mask);
				trigger_proc_in_eth_clearReset();
			} else {
				trigger_proc_in_eth_setReset();
				wl_pkt_proc_setTrigID(0, active_triggerID_mask);
			}
			#endif

			respArgs32[respIndex++] = Xil_Htonl(active_triggerID_mask);
			respHdr->length += (respIndex * sizeof(respArgs32));
			respHdr->numArgs = respIndex;
		break;
		case TRIG_MNGR_CLR_ETHERNET_TRIGS:
			active_triggerID_mask = 0;

			#ifdef FAST_TRIG
			//Disable fast triggers and clear the template trigger IDs
			//warplab_buffers_ConfigReg_Clear(WL_BUFFERS_CFG_REG_HW_TRIG_EN);
			wl_pkt_proc_setTrigID(0, 0);
			#endif

			respArgs32[respIndex++] = Xil_Htonl(active_triggerID_mask);
			respHdr->length += (respIndex * sizeof(respArgs32));
			respHdr->numArgs = respIndex;
		break;
		case TRIG_MNGR_INPUT_SEL:
			//Loop through outputs
			//xil_printf("numOutputs = %d\n", Xil_Ntohl(cmdArgs32[0]));
			OR_start = Xil_Ntohl(cmdArgs32[0])+1;
			//xil_printf("numOr = %d\n", Xil_Ntohl(cmdArgs32[OR_start]));
			AND_start = OR_start+Xil_Ntohl(cmdArgs32[OR_start])+1;
			//xil_printf("numAnd = %d\n", Xil_Ntohl(cmdArgs32[AND_start]));
			for(k=0;k<Xil_Ntohl(cmdArgs32[0]);k++){ //Loop Through Outputs
				//xil_printf("	Output: %d\n",Xil_Ntohl(cmdArgs32[k+1]));
				OR_inputs = 0;
				AND_inputs = 0;

				for(j=0;j<Xil_Ntohl(cmdArgs32[OR_start]);j++){ //Loop Through OR inputs
					//xil_printf("		OR Input: %d\n",Xil_Ntohl(cmdArgs32[j+OR_start+1]));
					OR_inputs = OR_inputs + trigmngr_trigID_to_OR_mask(Xil_Ntohl(cmdArgs32[j+OR_start+1]));
				}

				for(j=0;j<Xil_Ntohl(cmdArgs32[AND_start]);j++){ //Loop Through OR inputs
					//xil_printf("		AND Input: %d\n",Xil_Ntohl(cmdArgs32[j+AND_start+1]));
					AND_inputs = OR_inputs + trigmngr_trigID_to_OR_mask(Xil_Ntohl(cmdArgs32[j+AND_start+1]));
				}

				switch(Xil_Ntohl(cmdArgs32[k+1])){
					case 1:
						trigger_proc_out0_ConfigReg_Clear(AND_ALL | OR_ALL);
						trigger_proc_out0_ConfigReg_Set(AND_inputs | OR_inputs);
					break;
					case 2:
						trigger_proc_out1_ConfigReg_Clear(AND_ALL | OR_ALL);
						trigger_proc_out1_ConfigReg_Set(AND_inputs | OR_inputs);
					break;
					case 3:
						trigger_proc_out2_ConfigReg_Clear(AND_ALL | OR_ALL);
						trigger_proc_out2_ConfigReg_Set(AND_inputs | OR_inputs);
					break;
					case 4:
						trigger_proc_out3_ConfigReg_Clear(AND_ALL | OR_ALL);
						trigger_proc_out3_ConfigReg_Set(AND_inputs | OR_inputs);
					break;
					case 5:
						trigger_proc_out4_ConfigReg_Clear(AND_ALL | OR_ALL);
						trigger_proc_out4_ConfigReg_Set(AND_inputs | OR_inputs);
					break;
					case 6:
						trigger_proc_out5_ConfigReg_Clear(AND_ALL | OR_ALL);
						trigger_proc_out5_ConfigReg_Set(AND_inputs | OR_inputs);
					break;
				}


			}
			//xil_printf("\n");
		break;

		case TRIG_MNGR_OUTPUT_DELAY:
				delay = Xil_Ntohl(cmdArgs32[(Xil_Ntohl(cmdArgs32[0])+1)]);
				for(k=0;k<Xil_Ntohl(cmdArgs32[0]);k++){ //Loop Through Outputs

					switch(Xil_Ntohl(cmdArgs32[k+1])){
						case 1:
							trigger_proc_out0_ConfigReg_Delay(delay);
						break;
						case 2:
							trigger_proc_out1_ConfigReg_Delay(delay);
						break;
						case 3:
							trigger_proc_out2_ConfigReg_Delay(delay);
						break;
						case 4:
							trigger_proc_out3_ConfigReg_Delay(delay);
						break;
						case 5:
							trigger_proc_out4_ConfigReg_Delay(delay);
						break;
						case 6:
							trigger_proc_out5_ConfigReg_Delay(delay);
						break;
					}
				}
		break;

		case TRIG_MNGR_OUTPUT_HOLD:
			MODE_input = Xil_Ntohl(cmdArgs32[(Xil_Ntohl(cmdArgs32[0])+1)]);

			for(k=0;k<Xil_Ntohl(cmdArgs32[0]);k++){ //Loop Through Outputs

				switch(Xil_Ntohl(cmdArgs32[k+1])){
					case 1:
						trigger_proc_out0_ConfigReg_HoldMode(MODE_input);
					break;
					case 2:
						trigger_proc_out1_ConfigReg_HoldMode(MODE_input);
					break;
					case 3:
						trigger_proc_out2_ConfigReg_HoldMode(MODE_input);
					break;
					case 4:
						trigger_proc_out3_ConfigReg_HoldMode(MODE_input);
					break;
					case 5:
						trigger_proc_out4_ConfigReg_HoldMode(MODE_input);
					break;
					case 6:
						trigger_proc_out5_ConfigReg_HoldMode(MODE_input);
					break;
				}
			}

		break;

		case TRIG_MNGR_OUTPUT_READ:
			for(k=0;k<Xil_Ntohl(cmdArgs32[0]);k++){ //Loop Through Outputs
				switch(Xil_Ntohl(cmdArgs32[k+1])){
					case 1:
						respArgs32[respIndex++] = Xil_Htonl((trigger_proc_Output_Read() & OUT0)>0);
					break;
					case 2:
						respArgs32[respIndex++] = Xil_Htonl((trigger_proc_Output_Read() & OUT1)>0);
					break;
					case 3:
						respArgs32[respIndex++] = Xil_Htonl((trigger_proc_Output_Read() & OUT2)>0);
					break;
					case 4:
						respArgs32[respIndex++] = Xil_Htonl((trigger_proc_Output_Read() & OUT3)>0);
					break;
					case 5:
						respArgs32[respIndex++] = Xil_Htonl((trigger_proc_Output_Read() & OUT4)>0);
					break;
					case 6:
						respArgs32[respIndex++] = Xil_Htonl((trigger_proc_Output_Read() & OUT5)>0);
					break;
				}
			}
			respHdr->length += (respIndex * sizeof(respArgs32));
			respHdr->numArgs = respIndex;
		break;

		case TRIG_MNGR_OUTPUT_CLEAR:
			/// The HoldMode register is active-low.
			for(k=0;k<Xil_Ntohl(cmdArgs32[0]);k++){ //Loop Through Outputs

				switch(Xil_Ntohl(cmdArgs32[k+1])){
					case 1:
						if(trigger_proc_out0_ConfigReg_Read() & ~OUT_RESET){ //If in hold mode
							trigger_proc_out0_ConfigReg_HoldMode(1);
							trigger_proc_out0_ConfigReg_HoldMode(0);
						}
					break;
					case 2:
						if(trigger_proc_out1_ConfigReg_Read() & ~OUT_RESET){ //If in hold mode
							trigger_proc_out1_ConfigReg_HoldMode(1);
							trigger_proc_out1_ConfigReg_HoldMode(0);
						}
					break;
					case 3:
						if(trigger_proc_out2_ConfigReg_Read() & ~OUT_RESET){ //If in hold mode
							trigger_proc_out2_ConfigReg_HoldMode(1);
							trigger_proc_out2_ConfigReg_HoldMode(0);
						}
					break;
					case 4:
						if(trigger_proc_out3_ConfigReg_Read() & ~OUT_RESET){ //If in hold mode
							trigger_proc_out3_ConfigReg_HoldMode(1);
							trigger_proc_out3_ConfigReg_HoldMode(0);
						}
					break;
					case 5:
						if(trigger_proc_out4_ConfigReg_Read() & ~OUT_RESET){ //If in hold mode
							trigger_proc_out4_ConfigReg_HoldMode(1);
							trigger_proc_out4_ConfigReg_HoldMode(0);
						}
					break;
					case 6:
						if(trigger_proc_out5_ConfigReg_Read() & ~OUT_RESET){ //If in hold mode
							trigger_proc_out5_ConfigReg_HoldMode(1);
							trigger_proc_out5_ConfigReg_HoldMode(0);
						}
					break;
				}
			}

		break;

		case TRIG_MNGR_INPUT_ENABLE:
			trigger_proc_in_eth_setReset();
			trigger_proc_in_energy_setReset();
			trigger_proc_in_agcDone_setReset();
			trigger_proc_in_soft_setReset();
			trigger_proc_in_debug0_setReset();
			trigger_proc_in_debug1_setReset();
			trigger_proc_in_debug2_setReset();
			trigger_proc_in_debug3_setReset();
			for(k=0;k<Xil_Ntohl(cmdArgs32[0]);k++){ //Loop Through Inputs
				switch(Xil_Ntohl(cmdArgs32[k+1])){
					case 1:
						trigger_proc_in_eth_clearReset();
					break;
					case 2:
						trigger_proc_in_energy_clearReset();
					break;
					case 3:
						trigger_proc_in_agcDone_clearReset();
					break;
					case 4:
						trigger_proc_in_soft_clearReset();
					break;
					case 5:
						trigger_proc_in_debug0_clearReset();
					break;
					case 6:
						trigger_proc_in_debug1_clearReset();
					break;
					case 7:
						trigger_proc_in_debug2_clearReset();
					break;
					case 8:
						trigger_proc_in_debug3_clearReset();
					break;
				}
			}

		break;

		case TRIG_MNGR_INPUT_DEBOUNCE:
			MODE_input = Xil_Ntohl(cmdArgs32[(Xil_Ntohl(cmdArgs32[0])+1)]);

			for(k=0;k<Xil_Ntohl(cmdArgs32[0]);k++){ //Loop Through Inputs

				switch(Xil_Ntohl(cmdArgs32[k+1])){
				case 1:
					//Invalid -- Ethernet input has no debounce circuit
				break;
				case 2:
					//Invalid -- Energy detection input has no debounce circuit
				break;
				case 3:
					//Invalid -- AGC done input has no debounce circuit
				break;
				case 4:
					//Invalid -- Soft trigger input has no debounce circuit
				break;
				case 5:
					trigger_proc_in_debug0_debounceMode(MODE_input);
				break;
				case 6:
					trigger_proc_in_debug1_debounceMode(MODE_input);
				break;
				case 7:
					trigger_proc_in_debug2_debounceMode(MODE_input);
				break;
				case 8:
					trigger_proc_in_debug3_debounceMode(MODE_input);
				break;
				}
			}


		break;

		case TRIG_MNGR_BUSY_THRESHOLD:
			warplab_pktdet_SetBusyThreshold(Xil_Ntohl(cmdArgs32[0]));
			//warplab_pktdet_SetIdleThreshold(Xil_Ntohl(cmdArgs32[0])); //Temporary. Just to make sure the idle condition is always met.
			warplab_pktdet_SetIdleThreshold(0xFFFF); //Temporary. Just to make sure the idle condition is always met.

		break;

		case TRIG_MNGR_RSSI_AVG_LEN:
			warplab_pktdet_SetRSSIDuration(Xil_Ntohl(cmdArgs32[0]));
		break;


		case TRIG_MNGR_BUSY_MIN_LEN:
			warplab_pktdet_SetBusyDuration(Xil_Ntohl(cmdArgs32[0]));
		break;

		case TRIG_MNGR_IFC_SEL:
			warplab_pktdet_ConfigReg_Clear(WL_PKTDET_CONDIG_REG_MASK_ALL);
			warplab_pktdet_ConfigReg_Set(IFC_TO_PKTDETMASK(Xil_Ntohl(cmdArgs32[0])));

			/////DEL
		//	xil_printf("CORE_INFO = hex2dec('%x');\n",XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_CORE_INFO));
		//	xil_printf("TRIG_OUT = hex2dec('%x');\n",XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_OUT));
		//	xil_printf("RSSI_PKT_DET_CONFIG = hex2dec('%x');\n",XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_RSSI_PKT_DET_CONFIG));
		//	xil_printf("TRIG_CONF_OUT5 = hex2dec('%x');\n",XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT5));
		//	xil_printf("TRIG_CONF_OUT4 = hex2dec('%x');\n",XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT4));
		//	xil_printf("TRIG_CONF_OUT3 = hex2dec('%x');\n",XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT3));
		//	xil_printf("TRIG_CONF_OUT2 = hex2dec('%x');\n",XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT2));
		//	xil_printf("TRIG_CONF_OUT1 = hex2dec('%x');\n",XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT1));
		//	xil_printf("TRIG_CONF_OUT0 = hex2dec('%x');\n",XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT0));
		//	xil_printf("RSSI_PKT_DET_DURATIONS = hex2dec('%x');\n",XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_RSSI_PKT_DET_DURATIONS));
		//	xil_printf("RSSI_PKT_DET_THRESHOLDS = hex2dec('%x');\n",XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_RSSI_PKT_DET_THRESHOLDS));
		//	xil_printf("TRIG_CONF_INBANK0 = hex2dec('%x');\n",XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK0));
		//	xil_printf("TRIG_CONF_INBANK1 = hex2dec('%x');\n",XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK1));
		//	xil_printf("PKTOPS0 = hex2dec('%x');\n",XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_PKTOPS0));
		//	xil_printf("PKTTEMPLATE0 = hex2dec('%x');\n",XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_PKTTEMPLATE0));
			/////DEL


		break;

		case TRIG_MNGR_TEST_TRIGGER:
			if(cmdHdr->numArgs == 1){ //Received test trigger
				broadcast_test_flag =  Xil_Ntohl(cmdArgs32[0]);
			} else { //Requesting status of test trigger reception
				respArgs32[respIndex++] = Xil_Htonl(broadcast_test_flag);
				respHdr->length += (respIndex * sizeof(respArgs32));
				respHdr->numArgs = respIndex;
				broadcast_test_flag = 0;
			}
		break;


	}
	return respSent;
}

u32 trigmngr_trigID_to_AND_mask(u32 trigID){
	u32 mask;
	switch(trigID){
		case 1:
			mask = AND_ETH;
		break;
		case 2:
			mask = AND_ENERGY;
		break;
		case 3:
			mask = AND_AGC_DONE;
		break;
		case 4:
			mask = AND_SOFT;
		break;
		case 5:
			mask = AND_DEBUG0;
		break;
		case 6:
			mask = AND_DEBUG1;
		break;
		case 7:
			mask = AND_DEBUG2;
		break;
		case 8:
			mask = AND_DEBUG3;
		break;
	}
	return mask;
}

u32 trigmngr_trigID_to_OR_mask(u32 trigID){
	u32 mask;

	switch(trigID){
		case 1:
			mask = OR_ETH;
		break;
		case 2:
			mask = OR_ENERGY;
		break;
		case 3:
			mask = OR_AGC_DONE;
		break;
		case 4:
			mask = OR_SOFT;
		break;
		case 5:
			mask = OR_DEBUG0;
		break;
		case 6:
			mask = OR_DEBUG1;
		break;
		case 7:
			mask = OR_DEBUG2;
		break;
		case 8:
			mask = OR_DEBUG3;
		break;
	}
	return mask;
}

void trigmngr_triggerIn(u32 trigID){
	u32 txRx_running, cont_tx, wait;

	if(trigID & active_triggerID_mask){
#ifndef FAST_TRIG
		trigger_proc_in_soft_raiseTrigger();
		trigger_proc_in_soft_lowerTrigger();
#endif

		//Wait for the baseband to finish its Tx/Rx cycle before returning
		// This prevents the next command packet from interrupting a Tx/Rx cycle
		do {
			// WL_BUFFERS_STATUS_REG_TX_RUNNING and WL_BUFFERS_STATUS_REG_RX_RUNNING
			txRx_running = warplab_buffers_ReadReg_Status();

			// 0x1: Continuous Tx
			cont_tx = warplab_buffers_ReadReg_Config() & WL_BUFFERS_CFG_REG_CONT_TX;

			wait = (txRx_running & WL_BUFFERS_STATUS_REG_RX_RUNNING) || ( (txRx_running & 0x1) && (cont_tx == 0));
		} while( wait );

		return;
	}
}

int trigmngr_init(){
	int status = 0;
	active_triggerID_mask = 0;
	broadcast_test_flag = 0;


	//Initialize Trigger Processor
	trigger_proc_in_eth_setDelay(0);
	trigger_proc_in_debug0_setDelay(0);
	trigger_proc_in_debug1_setDelay(0);
	trigger_proc_in_debug2_setDelay(0);
	trigger_proc_in_debug3_setDelay(0);

	trigger_proc_in_eth_clearReset();
	trigger_proc_in_energy_clearReset();
	trigger_proc_in_agcDone_clearReset();
	trigger_proc_in_soft_clearReset();
	trigger_proc_in_debug0_clearReset();
	trigger_proc_in_debug1_clearReset();
	trigger_proc_in_debug2_clearReset();
	trigger_proc_in_debug3_clearReset();

	trigger_proc_in_debug0_debounceMode(1);
	trigger_proc_in_debug1_debounceMode(1);
	trigger_proc_in_debug2_debounceMode(1);
	trigger_proc_in_debug3_debounceMode(1);

	trigger_proc_in_soft_lowerTrigger();

	//Clear all connections to output triggers
	trigger_proc_out0_ConfigReg_Clear(AND_ALL | OR_ALL);
	trigger_proc_out1_ConfigReg_Clear(AND_ALL | OR_ALL);
	trigger_proc_out2_ConfigReg_Clear(AND_ALL | OR_ALL);
	trigger_proc_out3_ConfigReg_Clear(AND_ALL | OR_ALL);
	trigger_proc_out4_ConfigReg_Clear(AND_ALL | OR_ALL);
	trigger_proc_out5_ConfigReg_Clear(AND_ALL | OR_ALL);

	//Disable "sticky" bits
	trigger_proc_out0_ConfigReg_Set(OUT_RESET);
	trigger_proc_out1_ConfigReg_Set(OUT_RESET);
	trigger_proc_out2_ConfigReg_Set(OUT_RESET);
	trigger_proc_out3_ConfigReg_Set(OUT_RESET);
	trigger_proc_out4_ConfigReg_Set(OUT_RESET);
	trigger_proc_out4_ConfigReg_Set(OUT_RESET);

	//By default, the crossbar is configured to allow triggers from Ethernet, soft triggers to both outputs 0 and 1
	trigger_proc_out0_ConfigReg_Set(OR_ETH | OR_SOFT); //Out0: warplab_buffers baseband
	trigger_proc_out1_ConfigReg_Set(OR_ETH | OR_SOFT); //Out1: AGC start

	trigger_proc_in_eth_setReset();

	//Currently, the idle-checking functionality of the trigger manager core is disabled
	warplab_pktdet_SetIdleDuration(10); //Even a 1 cycle drop below the idle threshold will count as idle

	warplab_pktdet_ConfigReg_Set(WL_PKTDET_CONFIG_REG_RESET);
	warplab_pktdet_ConfigReg_Clear(WL_PKTDET_CONFIG_REG_RESET);

	return status;
}


//Configures the fast trigger logic in the warplab_pkt_proc core
void wl_pkt_proc_setTrigID(u32 matchID, u32 trigID) {
#ifdef WARP_HW_VER_v3
	//Ethernet-IP-UDP-PAD-WL_TRANSPORT-TRIGID
	const int pkt_match_len =  sizeof(ethernet_header) + sizeof(ipv4_header) + sizeof(udp_header) + 2 + sizeof(wl_transport_header) + 4;

	//Local arrays to construct packet template and template operators
	// The template cannot be constructed directly in the pkt_proc BRAM, since Sysgen shared memories
	//  aren't byte-addressable (they lack byte enables, so any write access writes the full word)
	u8 template0[pkt_match_len];
	u8 ops0[pkt_match_len];

	//Helper pointers, for interpretting various packet parts
	ethernet_header* hdr_eth;
	ipv4_header* hdr_ip;
	udp_header* hdr_udp;
	wl_transport_header* hdr_xport;
	u32* trig_payload;

	//Ensure the local arrays are zero (makes it safe between soft boots)
	bzero(template0, pkt_match_len);
	bzero(ops0, pkt_match_len);

	//Assign helper pointers to their respsctive first bytes in the blank template buffer
	// Address offsets here match received Ethernet frames (ETH-IP-UDP-WL)
	hdr_eth = (ethernet_header*)template0;
	hdr_ip = (ipv4_header*)((void*)hdr_eth + sizeof(ethernet_header));
	hdr_udp = (udp_header*)((void*)hdr_ip + sizeof(ipv4_header));
	hdr_xport = (wl_transport_header*)((void*)hdr_udp + sizeof(udp_header) + 2);
	trig_payload = (u32*)((void*)hdr_xport + sizeof(wl_transport_header));

	//Configure the packet template
	hdr_eth->dst_addr[0] = 0xFF;
	hdr_eth->dst_addr[1] = 0xFF;
	hdr_eth->dst_addr[2] = 0xFF;
	hdr_eth->dst_addr[3] = 0xFF;
	hdr_eth->dst_addr[4] = 0xFF;
	hdr_eth->dst_addr[5] = 0xFF;
	hdr_eth->ethertype = Xil_Htons(ETHERTYPE_IP);

	hdr_ip->proto = IPPROTO_UDP;
	hdr_ip->src_addr = 0; //any src address is fine
	hdr_ip->dst_addr = Xil_Htonl((u32)(NODE_IP_ADDR_BASE | 0xFF));//X.X.X.255

	hdr_udp->src_port = 0;
	hdr_udp->dst_port = Xil_Htons((u16)NODE_UDP_MCAST_BASE);//WL broadcast port

	hdr_xport->srcID = Xil_Htons((u16)0);
	hdr_xport->destID = Xil_Htons((u16)BROADCAST_ID);
	hdr_xport->pktType = PKTTYPE_TRIGGER;

	trig_payload[0] = Xil_Htonl(trigID);

	//Zero out full template, then write the part we need
	bzero((void*)XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_PKTTEMPLATE0, 64*4);
	memcpy((void*)XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_PKTTEMPLATE0, template0, pkt_match_len);

	//Configure the packet template operators
	hdr_eth = (ethernet_header*)ops0;
	hdr_ip = (ipv4_header*)((void*)hdr_eth + sizeof(ethernet_header));
	hdr_udp = (udp_header*)((void*)hdr_ip + sizeof(ipv4_header));
	hdr_xport = (wl_transport_header*)((void*)hdr_udp + sizeof(udp_header) + 2);
	trig_payload = (u32*)((void*)hdr_xport + sizeof(wl_transport_header));

	//Dest Ethernet address must match (all 0xFF for broadcast)
	hdr_eth->dst_addr[0] = U8_OP_EQ;
	hdr_eth->dst_addr[1] = U8_OP_EQ;
	hdr_eth->dst_addr[2] = U8_OP_EQ;
	hdr_eth->dst_addr[3] = U8_OP_EQ;
	hdr_eth->dst_addr[4] = U8_OP_EQ;
	hdr_eth->dst_addr[5] = U8_OP_EQ;
	hdr_eth->ethertype = U16_OP_EQ;

	//IP protocol (UDP) and dest addr (.255) must match
	hdr_ip->proto = U8_OP_EQ;
	hdr_ip->src_addr = U32_OP_NC;
	hdr_ip->dst_addr = U32_OP_EQ;

	//UDP src/dest ports must match
	hdr_udp->src_port = U16_OP_NC;
	hdr_udp->dst_port = U16_OP_EQ;

	//WARPLab transport dest ID and packet type must match
	hdr_xport->srcID = U16_OP_NC;
	hdr_xport->destID = U16_OP_EQ;
	hdr_xport->pktType = U8_OP_EQ;

	//Trigger IDs are one-hot encoded; a trigger packet will have one or more
	// bit-OR'd IDs in its payload
	//If any ID matches any previously-enabled trigger ID, the core should assert
	trig_payload[0] = U32_OP_AA;

	//Zero out full template, then write the part we need
	bzero((void*)XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_PKTOPS0, 64*4);
	memcpy((void*)XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_PKTOPS0, ops0, pkt_match_len);

//	if(trigID == 0) {
//		//If no trigger IDs are enabled, never assert the trigger output
//		Xil_Out32(XPAR_WARPLAB_PKT_PROC_0_MEMMAP_MATCH_OUTPUT_EN, 0);
//	} else {
//		Xil_Out32(XPAR_WARPLAB_PKT_PROC_0_MEMMAP_MATCH_OUTPUT_EN, 1);
//	}
#endif
	return;
}
