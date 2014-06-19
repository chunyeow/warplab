////////////////////////////////////////////////////////////////////////////////
// File   :	wl_baseband.c
// Authors:	Chris Hunter (chunter [at] mangocomm.com)
//			Patrick Murphy (murphpo [at] mangocomm.com)
//          Erik Welsh (welsh [at] mangocomm.com)
// License:	Copyright 2013, Mango Communications. All rights reserved.
//			Distributed under the WARP license  (http://warpproject.org/license)
////////////////////////////////////////////////////////////////////////////////

#include "warp_hw_ver.h"
#include "include/wl_baseband.h"
#include "include/wl_interface.h"
#include "include/wl_common.h"
#include "include/wl_node.h"
#include <xparameters.h>
#include <stdlib.h>
#include <stdio.h>
#include <Xio.h>
#include <xil_io.h>
#include <string.h>
#include <xil_types.h>

#ifdef WARP_HW_VER_v2

#endif

#ifdef WARP_HW_VER_v3
#include <xaxicdma.h>
extern XAxiCdma cdma_inst;
#endif

//short int agcNoiseEstimate,agcTarget;
unsigned int agcThresholds,agc_dco_state,agc_trig_delay;


int baseband_processCmd(const wl_cmdHdr* cmdHdr,const void* cmdArgs, wl_respHdr* respHdr,void* respArgs, void* pktSrc, unsigned int eth_dev_num){
	//IMPORTANT ENDIAN NOTES:
	// -cmdHdr is safe to access directly (pre-swapped if needed)
	// -cmdArgs is *not* pre-swapped, since the framework doesn't know what it is
	// -respHdr will be swapped by the framework; user code should fill it normally
	// -respArgs will *not* be swapped by the framework, since only user code knows what it is
	//    Any data added to respArgs by the code below must be endian-safe (swapped on AXI hardware)

	//Cast argument buffers into arrays for easy accessing
	const u32 *cmdArgs32 = cmdArgs;
	u32 *respArgs32 = respArgs;

	unsigned int respIndex = 0;

	unsigned int respSent = NO_RESP_SENT;

	unsigned int cmdID;
	unsigned int txDelay,txLength,txMode;
	unsigned int rfsel,buffSel,currChecksum,startSamp,currSamp,startByte,numSamp,sampLen,numPkts,i,offset,end_byte;
	unsigned int totalSamp, maxSampLen_perPkt, maxSamp_perPkt, nextStartSamp;
	u32 newchkinput_32;
	u16 newchkinput_16;
	short int agcNoiseEstimate, agcTarget;
	u32 agc_timeout;
	unsigned char flags;
	void * sampAddr;
	u32 * sampAddr32;
	wl_bb_sampHdr* sampHdr;


	cmdID = WL_CMD_TO_CMDID(cmdHdr->cmd);

	respHdr->cmd = cmdHdr->cmd;
	respHdr->length = 0;
	respHdr->numArgs = 0;


	switch(cmdID){
		case BB_TX_DELAY:
			if(cmdHdr->numArgs == 0){ //Read
				respArgs32[respIndex++] = Xil_Htonl(warplab_buffers_ReadReg_TxDelay());
				respHdr->length += (respIndex * sizeof(respArgs32));
				respHdr->numArgs = respIndex;
			} else {
				txDelay = Xil_Ntohl(cmdArgs32[0]);
				warplab_buffers_WriteReg_TxDelay(txDelay);
			}
		break;
		case BB_TX_LENGTH:
			if(cmdHdr->numArgs == 0){ //Read
				respArgs32[respIndex++] = Xil_Htonl(warplab_buffers_ReadReg_TxLength()+1);
				respHdr->length += (respIndex * sizeof(respArgs32));
				respHdr->numArgs = respIndex;
			} else {
				txLength = Xil_Ntohl(cmdArgs32[0]);
				warplab_buffers_WriteReg_TxLength(txLength-1);
			}
		break;
		case BB_RX_LENGTH:
			if(cmdHdr->numArgs == 0){ //Read
				respArgs32[respIndex++] = Xil_Htonl(warplab_buffers_ReadReg_RxLength()+1);
				respHdr->length += (respIndex * sizeof(respArgs32));
				respHdr->numArgs = respIndex;
			} else {
				txLength = Xil_Ntohl(cmdArgs32[0]);
				warplab_buffers_WriteReg_RxLength(txLength-1);
			}
		break;
		case BB_TX_MODE:
			txMode = Xil_Ntohl(cmdArgs32[0]);
			if(txMode)
				warplab_buffers_ConfigReg_Set(WL_BUFFERS_CFG_REG_CONT_TX);
			else
				warplab_buffers_ConfigReg_Clear(WL_BUFFERS_CFG_REG_CONT_TX);
		break;

		case BB_TX_BUFF_EN:
			buffSel = Xil_Ntohl(cmdArgs32[0]);
			warplab_buffers_WriteReg_TxEn(buffSel);
		break;

		case BB_RX_BUFF_EN:
			buffSel = Xil_Ntohl(cmdArgs32[0]);
			warplab_buffers_WriteReg_RxEn(buffSel);
		break;

		case BB_TXRX_BUFF_DIS:
			buffSel = Xil_Ntohl(cmdArgs32[0]);
			warplab_buffers_WriteReg_TxRxDis(buffSel);
		break;

		case BB_WRITE_IQ:

#ifdef _PERFORMANCE_MONITOR_
			wl_setDebugGPIO(0x4);
#endif

			sampHdr        = (wl_bb_sampHdr *)cmdArgs;

			//Parse the command arguments
			buffSel        = (u32) Xil_Ntohs(sampHdr->buffSel);
			startSamp      = Xil_Ntohl(sampHdr->startSamp);
			offset         = startSamp * sizeof(wl_samp);
			flags          = sampHdr->flags;
			numSamp        = Xil_Ntohl(sampHdr->numSamp);

			//Samples start after the sample header
			sampAddr       = (void *)sampHdr + sizeof(wl_bb_sampHdr);
			sampAddr32     = (u32 *)sampAddr;
			sampLen        = numSamp*sizeof(wl_samp);
			newchkinput_32 = Xil_Ntohl(sampAddr32[numSamp-1]);
			newchkinput_16 = (newchkinput_32>>16)^(0xFFFF & newchkinput_32);

			if(flags&&SAMPHDR_FLAG_CHKSUMRESET){
				currChecksum = baseband_updateChecksum( (startSamp & 0xFFFF), 1);
			}else{
				currChecksum = baseband_updateChecksum( (startSamp & 0xFFFF), 0);
			}
			currChecksum = baseband_updateChecksum(newchkinput_16,0);

			// xil_printf("checkInput: 0x%4x, currChecksum = 0x%8x, startSamp = %8d, numSamp = %d, flags = 0x%x \n", newchkinput_16, currChecksum, startSamp, numSamp, flags);
			// xil_printf("lastSampleInd = %d, firstSample: 0x%x, lastSample = 0x%x\n", numSamp-1, Xil_Ntohl(sampAddr32[0]), Xil_Ntohl(sampAddr32[numSamp-1]));
			// xil_printf("[%x %x] checkInput: 0x%x, currChecksum: 0x%x, startSamp: %d, sampLen: %d\n",(newchkinput_32>>16),(0xFFFF & newchkinput_32),newchkinput_16,currChecksum,startSamp,sampLen);
			respArgs32[respIndex++] = Xil_Htonl(currChecksum);
			respHdr->length        += (respIndex * sizeof(respArgs32));
			respHdr->numArgs        = respIndex;


			//IMPORTANT: Sysgen-created shared memories do not implement byte enables, so all writes *must* be word-aligned
			// The WARPLab wire frame format inclues a 2-byte pad to ensure sample payloads are word-aligned

#ifdef WARP_HW_VER_v2
			if(buffSel&BB_A) memcpy((void*)(WARPLAB_IQ_TX_BUFFA) + offset,(void *)sampAddr,sampLen);
			if(buffSel&BB_B) memcpy((void*)(WARPLAB_IQ_TX_BUFFB) + offset,(void *)sampAddr,sampLen);
			if(buffSel&BB_C) memcpy((void*)(WARPLAB_IQ_TX_BUFFC) + offset,(void *)sampAddr,sampLen);
			if(buffSel&BB_D) memcpy((void*)(WARPLAB_IQ_TX_BUFFD) + offset,(void *)sampAddr,sampLen);
#endif

#ifdef WARP_HW_VER_v3

            // In WARPLab 7.3.0 for WARPv3, the Read IQ buffers have been moved out of the WARPLab buffers
			// core and now sit on the bus infrastructure directly.  This allows DMA access to those memories.
            //
			end_byte = offset + sampLen - 1;
			if(buffSel&BB_A) {
				if ( end_byte <= WARPLAB_IQ_TX_BUFFA_SIZE ) {
					while(XAxiCdma_IsBusy(&cdma_inst)) {}
					XAxiCdma_SimpleTransfer(&cdma_inst, (u32)sampAddr, (u32)((WARPLAB_IQ_TX_BUFFA) + offset), sampLen, NULL, NULL);
				} else {
					xil_printf("*** ERROR:  Too many bytes for RF A Buffer - Size = %d;  Write end = %d\n\n", WARPLAB_IQ_TX_BUFFA_SIZE, end_byte);
				}
			}
			if(buffSel&BB_B) {
				if ( end_byte <= WARPLAB_IQ_TX_BUFFB_SIZE ) {
					while(XAxiCdma_IsBusy(&cdma_inst)) {}
					XAxiCdma_SimpleTransfer(&cdma_inst, (u32)sampAddr, (u32)((WARPLAB_IQ_TX_BUFFB) + offset), sampLen, NULL, NULL);
				} else {
					xil_printf("*** ERROR:  Too many bytes for RF B Buffer - Size = %d;  Write end = %d\n\n", WARPLAB_IQ_TX_BUFFB_SIZE, end_byte);
				}
			}
			if(buffSel&BB_C) {
#if WARPLAB_CONFIG_4RF
				if ( end_byte <= WARPLAB_IQ_TX_BUFFC_SIZE ) {
					while(XAxiCdma_IsBusy(&cdma_inst)) {}
					XAxiCdma_SimpleTransfer(&cdma_inst, (u32)sampAddr, (u32)((WARPLAB_IQ_TX_BUFFC) + offset), sampLen, NULL, NULL);
				} else {
					xil_printf("*** ERROR:  Too many bytes for RF C Buffer - Size = %d;  Write end = %d\n\n", WARPLAB_IQ_TX_BUFFC_SIZE, end_byte);
				}
#else
				xil_printf("*** ERROR:  Trying to write to RF C buffer on a 2RF design.\n\n");
#endif
			}
			if(buffSel&BB_D) {
#if WARPLAB_CONFIG_4RF
				if ( end_byte <= WARPLAB_IQ_TX_BUFFD_SIZE ) {
					while(XAxiCdma_IsBusy(&cdma_inst)) {}
					XAxiCdma_SimpleTransfer(&cdma_inst, (u32)sampAddr, (u32)((WARPLAB_IQ_TX_BUFFD) + offset), sampLen, NULL, NULL);
				} else {
					xil_printf("*** ERROR:  Too many bytes for RF D Buffer - Size = %d;  Write end = %d\n\n", WARPLAB_IQ_TX_BUFFD_SIZE, end_byte);
				}
#else
				xil_printf("*** ERROR:  Trying to write to RF D buffer on a 2RF design.\n\n");
#endif
			}

			// Check if there was an error and reset the DMA
			if ( XAxiCdma_GetError(&cdma_inst) != 0x0 ) {
				xil_printf("*** ERROR in DMA transfer.  Resetting DMA ... \n\n");
				XAxiCdma_Reset(&cdma_inst);
				while(!XAxiCdma_ResetIsDone(&cdma_inst)) {}
			}

#endif

#ifdef _PERFORMANCE_MONITOR_
			wl_clearDebugGPIO(0x4);
#endif
		break;

		case BB_READ_IQ:
		case BB_READ_RSSI:

			//myCmd.setArgs(currBuffSel,offset,numSamp,maxPayload_uint8,numPktsRequired);

			buffSel           = Xil_Ntohl(cmdArgs32[0]);
			startSamp         = Xil_Ntohl(cmdArgs32[1]);
			totalSamp         = Xil_Ntohl(cmdArgs32[2]);
			maxSampLen_perPkt = Xil_Ntohl(cmdArgs32[3]);
			numPkts           = Xil_Ntohl(cmdArgs32[4]);

			maxSamp_perPkt    = maxSampLen_perPkt / sizeof(wl_samp); //This is an constant integer division that is optimized away by the compiler

			sampHdr           = (wl_bb_sampHdr *)respArgs;
			sampHdr->buffSel  = (u16)buffSel;
			sampHdr->buffSel  = Xil_Htons(sampHdr->buffSel);
			sampHdr->flags    = 0;

			numSamp           = 0;
			currSamp          = startSamp;

#ifdef _DEBUG_
			xil_printf("WARPLab Baseband READ_IQ/READ_RSSI \n");
			xil_printf("  buffSel=0x%x  startSamp=%d  totalSamp=%d  maxSampLen_perPkt=%d  maxSamp_perPkt=%d  numPkts=%d\n", buffSel, startSamp, totalSamp, maxSampLen_perPkt, maxSamp_perPkt, numPkts);
			xil_printf("  Initial Buffer:  \n");
			print_pkt( (unsigned char *)((void*)respArgs + sizeof(wl_bb_sampHdr)), maxSampLen_perPkt);
#endif

			for(i=0;i<numPkts;i++){

#ifdef _PERFORMANCE_MONITOR_
				wl_setDebugGPIO(0x8);
#endif

				nextStartSamp = currSamp + maxSamp_perPkt;

				if(nextStartSamp > (startSamp+totalSamp)){
					numSamp = (startSamp+totalSamp)-currSamp;
				} else {
					numSamp = maxSamp_perPkt;
				}


				sampHdr->startSamp = Xil_Htonl(currSamp);
				sampHdr->numSamp   = Xil_Htonl(numSamp);

				sampLen            = numSamp * sizeof(wl_samp);

				respHdr->length    = sampLen + sizeof(wl_bb_sampHdr);
				respHdr->numArgs   = 1;

				startByte          = currSamp * sizeof(wl_samp);

#ifdef WARP_HW_VER_v2
				if(cmdID == BB_READ_IQ){
					if(buffSel&BB_A) memcpy(((void*)respArgs + sizeof(wl_bb_sampHdr)),((void*)WARPLAB_IQ_RX_BUFFA)+startByte,sampLen);
					else if(buffSel&BB_B) memcpy(((void*)respArgs + sizeof(wl_bb_sampHdr)),((void*)WARPLAB_IQ_RX_BUFFB)+startByte,sampLen);
					else if(buffSel&BB_C) memcpy(((void*)respArgs + sizeof(wl_bb_sampHdr)),((void*)WARPLAB_IQ_RX_BUFFC)+startByte,sampLen);
					else if(buffSel&BB_D) memcpy(((void*)respArgs + sizeof(wl_bb_sampHdr)),((void*)WARPLAB_IQ_RX_BUFFD)+startByte,sampLen);
				}

				if(cmdID == BB_READ_RSSI) {
					if(buffSel&BB_A) memcpy(((void*)respArgs + sizeof(wl_bb_sampHdr)),((void*)WARPLAB_RSSI_BUFFA)+startByte,sampLen);
					else if(buffSel&BB_B) memcpy(((void*)respArgs + sizeof(wl_bb_sampHdr)),((void*)WARPLAB_RSSI_BUFFB)+startByte,sampLen);
					else if(buffSel&BB_C) memcpy(((void*)respArgs + sizeof(wl_bb_sampHdr)),((void*)WARPLAB_RSSI_BUFFC)+startByte,sampLen);
					else if(buffSel&BB_D) memcpy(((void*)respArgs + sizeof(wl_bb_sampHdr)),((void*)WARPLAB_RSSI_BUFFD)+startByte,sampLen);
				}
#endif

#ifdef WARP_HW_VER_v3

                // In WARPLab 7.3.0 for WARPv3, the Read IQ buffers have been moved out of the WARPLab buffers core
				// and now sit on the bus infrastructure directly.  This allows DMA access to those memories.
                //
				end_byte = startByte + sampLen - 1;
				if(cmdID == BB_READ_IQ) {
					if(buffSel&BB_A) {
						if ( end_byte <= WARPLAB_IQ_TX_BUFFA_SIZE ) {
							while(XAxiCdma_IsBusy(&cdma_inst)) {}
							XAxiCdma_SimpleTransfer(&cdma_inst, (u32)(((void*)WARPLAB_IQ_RX_BUFFA)+startByte), (u32)((void*)respArgs + sizeof(wl_bb_sampHdr)), sampLen, NULL, NULL);
						} else {
							bzero((void *)((void*)respArgs + sizeof(wl_bb_sampHdr)), sampLen);
							xil_printf("*** ERROR:  Too many bytes for RF A Buffer - Size = %d;  Read end = %d\n\n", WARPLAB_IQ_TX_BUFFA_SIZE, end_byte);
						}
					} else if(buffSel&BB_B) {
						if ( end_byte <= WARPLAB_IQ_TX_BUFFB_SIZE ) {
							while(XAxiCdma_IsBusy(&cdma_inst)) {}
							XAxiCdma_SimpleTransfer(&cdma_inst, (u32)(((void*)WARPLAB_IQ_RX_BUFFB)+startByte), (u32)((void*)respArgs + sizeof(wl_bb_sampHdr)), sampLen, NULL, NULL);
						} else {
							bzero((void *)((void*)respArgs + sizeof(wl_bb_sampHdr)), sampLen);
							xil_printf("*** ERROR:  Too many bytes for RF B Buffer - Size = %d;  Read end = %d\n\n", WARPLAB_IQ_TX_BUFFB_SIZE, end_byte);
						}
					} else if(buffSel&BB_C) {
#if WARPLAB_CONFIG_4RF
						if ( end_byte <= WARPLAB_IQ_TX_BUFFC_SIZE ) {
							while(XAxiCdma_IsBusy(&cdma_inst)) {}
							XAxiCdma_SimpleTransfer(&cdma_inst, (u32)(((void*)WARPLAB_IQ_RX_BUFFC)+startByte), (u32)((void*)respArgs + sizeof(wl_bb_sampHdr)), sampLen, NULL, NULL);
						} else {
							bzero((void *)((void*)respArgs + sizeof(wl_bb_sampHdr)), sampLen);
							xil_printf("*** ERROR:  Too many bytes for RF C Buffer - Size = %d;  Read end = %d\n\n", WARPLAB_IQ_TX_BUFFC_SIZE, end_byte);
						}
#else
						// Return samples of all zeros if buffer read was incorrect
						bzero((void *)((void*)respArgs + sizeof(wl_bb_sampHdr)), sampLen);
						xil_printf("*** ERROR:  Trying to read to RF C buffer on a 2RF design.\n\n");
#endif
					} else if(buffSel&BB_D) {
#if WARPLAB_CONFIG_4RF
						if ( end_byte <= WARPLAB_IQ_TX_BUFFD_SIZE ) {
							while(XAxiCdma_IsBusy(&cdma_inst)) {}
							XAxiCdma_SimpleTransfer(&cdma_inst, (u32)(((void*)WARPLAB_IQ_RX_BUFFD)+startByte), (u32)((void*)respArgs + sizeof(wl_bb_sampHdr)), sampLen, NULL, NULL);
						} else {
							bzero((void *)((void*)respArgs + sizeof(wl_bb_sampHdr)), sampLen);
							xil_printf("*** ERROR:  Too many bytes for RF D Buffer - Size = %d;  Read end = %d\n\n", WARPLAB_IQ_TX_BUFFD_SIZE, end_byte);
						}
#else
						// Return samples of all zeros if buffer read was incorrect
						bzero((void *)((void*)respArgs + sizeof(wl_bb_sampHdr)), sampLen);
                        xil_printf("*** ERROR:  Trying to read to RF D buffer on a 2RF design.\n\n");
#endif
					}
				}

                // In WARPLab 7.3.0, the Read RSSI buffers are still maintained within the WARPLab buffers core.  Due to 
                // limitations in the AXI interconnect in XPS 14.4, we cannot bridge the AXI and AXI-Lite interconnects.
                // Therefore, the DMA is not able to access the RSSI buffers and we need to use a CPU memcpy in order to
                // move the data to the Ethernet buffer.
                //
				if(cmdID == BB_READ_RSSI) {
					if(buffSel&BB_A) memcpy(((void*)respArgs + sizeof(wl_bb_sampHdr)),((void*)WARPLAB_RSSI_BUFFA)+startByte,sampLen);
					else if(buffSel&BB_B) memcpy(((void*)respArgs + sizeof(wl_bb_sampHdr)),((void*)WARPLAB_RSSI_BUFFB)+startByte,sampLen);
					else if(buffSel&BB_C) memcpy(((void*)respArgs + sizeof(wl_bb_sampHdr)),((void*)WARPLAB_RSSI_BUFFC)+startByte,sampLen);
					else if(buffSel&BB_D) memcpy(((void*)respArgs + sizeof(wl_bb_sampHdr)),((void*)WARPLAB_RSSI_BUFFD)+startByte,sampLen);
				}

				// Check if there was an error and reset the DMA
				if ( XAxiCdma_GetError(&cdma_inst) != 0x0 ) {
					xil_printf("*** ERROR in DMA transfer.  Resetting DMA ... \n\n");
					XAxiCdma_Reset(&cdma_inst);
					while(!XAxiCdma_ResetIsDone(&cdma_inst)) {}
				}

#endif

#ifdef _PERFORMANCE_MONITOR_
				wl_clearDebugGPIO(0x8);
#endif

				// xil_printf("readIQ send: buffSel=0x%x currSamp=%d numSamp=%d i=%d\n", buffSel, currSamp, numSamp, i);

				//This is getting called 54 times, with good indexes; a sleep(100ms) here didn't fix it; MATLAB still gets every-other packet
				node_sendEarlyResp(respHdr, pktSrc, eth_dev_num);

				// xil_printf("Done sending packet \n");

				currSamp = nextStartSamp;
			}
			respSent = RESP_SENT;

		break;
		case BB_AGC_STATE:
			rfsel =  Xil_Ntohl(cmdArgs32[0]);

			if(rfsel&AGC_A){
				respArgs32[respIndex++] = Xil_Htonl(warplab_AGC_ReadReg_GRF_A() + 4* warplab_AGC_ReadReg_GBB_A());
				respArgs32[respIndex++] = Xil_Htonl(warplab_buffers_ReadReg_RFA_AGCDoneRSSI());
			}
			if(rfsel&AGC_B){
				respArgs32[respIndex++] = Xil_Htonl(warplab_AGC_ReadReg_GRF_B() + 4* warplab_AGC_ReadReg_GBB_B());
				respArgs32[respIndex++] = Xil_Htonl(warplab_buffers_ReadReg_RFB_AGCDoneRSSI());
			}
			if(rfsel&AGC_C){
				respArgs32[respIndex++] = Xil_Htonl(warplab_AGC_ReadReg_GRF_C() + 4* warplab_AGC_ReadReg_GBB_C());
				respArgs32[respIndex++] = Xil_Htonl(warplab_buffers_ReadReg_RFC_AGCDoneRSSI());
			}
			if(rfsel&AGC_D){
				respArgs32[respIndex++] = Xil_Htonl(warplab_AGC_ReadReg_GRF_D() + 4* warplab_AGC_ReadReg_GBB_D());
				respArgs32[respIndex++] = Xil_Htonl(warplab_buffers_ReadReg_RFD_AGCDoneRSSI());
			}
			respHdr->length += (respIndex * sizeof(respArgs32));
			respHdr->numArgs = respIndex;

		break;
		case BB_AGC_THRESH:
			if(cmdHdr->numArgs == 0){ //Read
				respArgs32[respIndex++] = Xil_Htonl(warplab_AGC_ReadReg_Thresholds());
				respHdr->length += (respIndex * sizeof(respArgs32));
				respHdr->numArgs = respIndex;
			} else { //Write
				agcThresholds = Xil_Ntohl(cmdArgs32[0]);
				warplab_AGC_WriteReg_Thresholds(agcThresholds);
			}
			//xil_printf("Set AGC thresholds: 0x%08x\n", agcThresholds);
		break;
		case BB_AGC_TARGET:
			agcTarget = Xil_Ntohl(cmdArgs32[0]);
			warplab_AGC_WriteReg_T_dB((short int)agcTarget);
			//xil_printf("Set AGC target: %d\n", agcTarget);
		break;

		case BB_AGC_NOISE_EST:
			agcNoiseEstimate = Xil_Ntohl(cmdArgs32[0]);
			warplab_agc_setNoiseEstimate((short int)agcNoiseEstimate);
		break;

		case BB_AGC_DCO_EN_DIS:
			agc_dco_state = Xil_Ntohl(cmdArgs32[0]);
			warplab_agc_setDCO(agc_dco_state);

			if(agc_dco_state)
				warplab_buffers_ConfigReg_Set(WL_BUFFERS_CFG_REG_AGC_IQ_SEL);
			else
				warplab_buffers_ConfigReg_Clear(WL_BUFFERS_CFG_REG_AGC_IQ_SEL);

		break;

		case BB_AGC_TRIG_DELAY:
			agc_trig_delay = Xil_Ntohl(cmdArgs32[0]);
			warplab_AGC_WriteReg_AGC_TRIGGER_DELAY(agc_trig_delay);
			//xil_printf("Set AGC trig delay: %d\n", agc_trig_delay);
		break;

		case BB_AGC_RESET:
			warplab_agc_reset();
		break;

		case BB_AGC_DONE_ADDR:
			respArgs32[respIndex++] = Xil_Htonl(warplab_buffers_ReadReg_AGCDoneAddr());
			respHdr->length += (respIndex * sizeof(respArgs32));
			respHdr->numArgs = respIndex;
		break;

		case BB_AGC_RESET_TIMEOUT:
			// AGC timeout register:
			//   [31]   = AGC No Reset
			//   [30:0] = AGC Timeout value
			if(cmdHdr->numArgs == 0){
				// Read
				// xil_printf("BB_AGC_RESET_TIMEOUT:  READ  %08x   %08x\n", (warplab_AGC_ReadReg_RESET_TIMEOUT() & 0x80000000),
				// 		                                                 (warplab_AGC_ReadReg_RESET_TIMEOUT() & 0x7FFFFFFF));
				respArgs32[respIndex++] = Xil_Htonl((warplab_AGC_ReadReg_RESET_TIMEOUT() & 0x80000000));
				respArgs32[respIndex++] = Xil_Htonl((warplab_AGC_ReadReg_RESET_TIMEOUT() & 0x7FFFFFFF));
				respHdr->length += (respIndex * sizeof(respArgs32));
				respHdr->numArgs = respIndex;
			} else {
				// Write
				// xil_printf("BB_AGC_RESET_TIMEOUT:  WRITE %08x   %08x\n", Xil_Ntohl(cmdArgs32[0]), Xil_Ntohl(cmdArgs32[1]));
				agc_timeout = (Xil_Ntohl(cmdArgs32[0]) & 0x80000000) | (Xil_Ntohl(cmdArgs32[1]) & 0x7FFFFFFF);
	            warplab_AGC_WriteReg_RESET_TIMEOUT(agc_timeout);
			}
		break;

		default:
			xil_printf("Invalid Baseband command ID: %d\n", cmdID);
			break;
	}
	return respSent;
}


int baseband_init(){
	int status = 0;

	//xil_printf("configuring baseband...\n");

	//XIo_Out32(WARPLAB_IQ_TX_BUFFA,105);
	//xil_printf("%d\n",XIo_In32(WARPLAB_IQ_TX_BUFFA));

	warplab_buffers_WriteReg_TxRxDis(BB_A+BB_B+BB_C+BB_D);

	warplab_buffers_WriteReg_TxDelay(0);
	warplab_buffers_WriteReg_TxLength((unsigned int)16383);

	//Set default config register values
#ifdef WARP_HW_VER_v3
	warplab_buffers_ConfigReg_Set(WL_BUFFERS_CFG_REG_BYTE_ORDER); //Enable byte swapping
#endif

	warplab_buffers_ConfigReg_Clear(
				WL_BUFFERS_CFG_REG_SW_TRIG | //De-assert the sw trigger
				WL_BUFFERS_CFG_REG_CONT_TX | //Disable continuous Tx
				WL_BUFFERS_CFG_REG_STOP_TX |
				WL_BUFFERS_CFG_REG_AGC_IQ_SEL ); //Select non-AGC IQ


	// NOTE:  This can be used to generate a data stream in the RX buffers that
	//    can be used to debug higher level SW issues.
	// warplab_buffers_ConfigReg_Set( WL_BUFFERS_CFG_REG_COUNTER_DATA_SEL );

	//Initialize the AGC core, but disable it by default
	warplab_AGC_WriteReg_Dis(AGC_A+AGC_B+AGC_C+AGC_D);

	//Initialize the AGC core
	// _init sets sane defaults for target and noise estimate
	warplab_agc_init();
	warplab_agc_reset();
	warplab_agc_setDCO(1);

	warplab_AGC_WriteReg_AGC_TRIGGER_DELAY((unsigned int)500);
	//warplab_AGC_WriteReg_Thresholds(0x00D5CBA6);
	warplab_AGC_WriteReg_Thresholds(0x00D5CB81);
	warplab_AGC_WriteReg_T_dB(-14);
	warplab_agc_setNoiseEstimate(-95);

	warplab_AGC_WriteReg_AGC_TRIGGER_DELAY((unsigned int)500);

	warplab_AGC_WriteReg_TrigEn(0);

	warplab_buffers_ConfigReg_SetRSSIClk(1);

	return status;
}

unsigned int baseband_updateChecksum(unsigned short int newdata, unsigned char reset ){
	//xil_printf("*******\n");
	//xil_printf("input to Chk: %d, reset: %d\n",currCheck,reset);

	//Fletcher-32 Checksum
	static unsigned int sum1 = 0;
	static unsigned int sum2 = 0;

	if(reset){
		sum1 = 0;
		sum2 = 0;
	}

	sum1 = (sum1 + newdata) % 65535;
	sum2 = (sum2 + sum1) % 65535;

	//xil_printf("sum1: %d sum2: %d\n",sum1,sum2);

	return ((sum2<<16) + sum1);
}



void warplab_agc_init(){
	// Turn off both resets and the master enable
	warplab_AGC_WriteReg_AGC_EN(0);
	warplab_AGC_WriteReg_SRESET_IN(0);
	warplab_AGC_WriteReg_MRESET_IN(0);

	// An adjustment parameter
	warplab_AGC_WriteReg_ADJ(8);

	// Timing for the DC-offset correction
	warplab_AGC_WriteReg_DCO_Timing(0x12001000);

	// Initial baseband gain setting
	warplab_AGC_WriteReg_GBB_init(52);

	// RF gain AGCstate thresholds
	//warplab_AGC_WriteReg_Thresholds(0xD5CBA6);
	warplab_AGC_WriteReg_Thresholds(0xD5CB81);

	// Overall AGC timing
	warplab_AGC_WriteReg_Timing(0x9A962A28);//0x826E3C0A;

	// vIQ and RSSI average lengths
	warplab_AGC_WriteReg_AVG_LEN(0x10F); //103

	// Disable DCO, disable DCO subtraction
	warplab_AGC_WriteReg_Bits(0x0);

	//Set sane defaults for the AGC target and noise estimate
	warplab_AGC_WriteReg_T_dB(-14);
	warplab_agc_setNoiseEstimate(-95);

	// Initialize the RESET_TIMEOUT to a sane default
	warplab_AGC_WriteReg_RESET_TIMEOUT(warplab_buffers_ReadReg_RxLength() + 100);

	// Perform a master reset
	warplab_AGC_WriteReg_AGC_EN(0);
	usleep(10);
	warplab_AGC_WriteReg_MRESET_IN(0);
	usleep(10);
	warplab_AGC_WriteReg_MRESET_IN(1);
	usleep(10);
	warplab_AGC_WriteReg_MRESET_IN(0);
	usleep(10);
	warplab_AGC_WriteReg_AGC_EN(1);

	// Agc is now reset and enabled, ready to go!
	return;

}

void warplab_agc_setDCO(unsigned int AGCstate){
// Enables DCO and DCO subtraction (correction scheme and butterworth hipass are active)

	unsigned int bits;

	bits = warplab_AGC_ReadReg_Bits();

	if(AGCstate)
		bits = bits | 0x6;
	else
		bits = bits & 0x1;

	warplab_AGC_WriteReg_Bits(bits);

	return;
}

inline void warplab_agc_setNoiseEstimate(short int agcNoiseEstimate){
	int g_bbset;
	g_bbset = -19 - 32 - agcNoiseEstimate;
	warplab_AGC_WriteReg_GBB_init(g_bbset);
	return;
}

void warplab_agc_reset(){
	// Cycle the agc's software reset port

	warplab_AGC_WriteReg_SRESET_IN(1);
	usleep(10);
	warplab_AGC_WriteReg_SRESET_IN(0);
	usleep(100);

	return;
}




