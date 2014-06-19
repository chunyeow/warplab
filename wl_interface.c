////////////////////////////////////////////////////////////////////////////////
// File   :	wl_interface.c
// Authors:	Chris Hunter (chunter [at] mangocomm.com)
//			Patrick Murphy (murphpo [at] mangocomm.com)
//          Erik Welsh (welsh [at] mangocomm.com)
// License:	Copyright 2013, Mango Communications. All rights reserved.
//			Distributed under the WARP license  (http://warpproject.org/license)
////////////////////////////////////////////////////////////////////////////////

#include "include/wl_interface.h"
//TODO: Only way to handle AGC stuff at the moment until we allow multi-command packets
#include "include/wl_baseband.h"
#include "include/wl_common.h"

#include <xparameters.h>
#include <stdlib.h>
#include <stdio.h>
#include <radio_controller.h>
#include <Xio.h>

#ifdef WARP_HW_VER_v3
#include <w3_ad_controller.h>
#endif


#if WARPLAB_CONFIG_4RF
#define ALL_RF_SEL (RC_RFA | RC_RFB | RC_RFC | RC_RFD)
#define ALL_AD_SEL (RFA_AD_CS | RFB_AD_CS | RFC_AD_CS | RFD_AD_CS)
#else
#define ALL_RF_SEL (RC_RFA | RC_RFB)
#define ALL_AD_SEL (RFA_AD_CS | RFB_AD_CS)
#endif

unsigned char Radios_WirelessChan;
unsigned char Radios_Tx_LPF_Corn_Freq;
unsigned char Radios_Rx_LPF_Corn_Freq;


//TODO: Would readback of gains be useful? Tempted to leave that in the domain of m-code
//unsigned char TxGain_BB[NUMRADIOS];
//unsigned char TxGain_RF[NUMRADIOS];
//unsigned char RxGain_BB[NUMRADIOS];
//unsigned char RxGain_RF[NUMRADIOS];


int ifc_processCmd(const wl_cmdHdr* cmdHdr,const void* cmdArgs, wl_respHdr* respHdr,void* respArgs, void* pktSrc, unsigned int eth_dev_num){
	//IMPORTANT ENDIAN NOTES:
	// -cmdHdr is safe to access directly (pre-swapped if needed)
	// -cmdArgs is *not* pre-swapped, since the framework doesn't know what it is
	// -respHdr will be swapped by the framework; user code should fill it normally
	// -respArgs will *not* be swapped by the framework, since only user code knows what it is
	//    Any data added to respArgs by the code below must be endian-safe (swapped on AXI hardware)

	//Cast argument buffers into arrays for easy accessing
	const u32 *cmdArgs32 = cmdArgs;

	//Un-comment if an interface command ever needs to send a reply payload
	//u32 *respArgs32 = respArgs;
	//unsigned int respIndex = 0;

	unsigned int respSent = NO_RESP_SENT;

	unsigned int cmdID,rfsel;
	unsigned int band,chan;
	unsigned int bbGain, rfGain;
	unsigned int agc_en;
	cmdID = WL_CMD_TO_CMDID(cmdHdr->cmd);

	respHdr->cmd = cmdHdr->cmd;
	respHdr->length = 0;
	respHdr->numArgs = 0;

	switch(cmdID){
			case IFC_TX_EN:
				rfsel =  Xil_Ntohl(cmdArgs32[0]);
				radio_controller_TxEnable(RC_BASEADDR, rfsel);
			break;
			case IFC_RX_EN:
				rfsel =  Xil_Ntohl(cmdArgs32[0]);
				radio_controller_RxEnable(RC_BASEADDR, rfsel);
			break;
			case IFC_TXRX_DIS:
				rfsel =  Xil_Ntohl(cmdArgs32[0]);
				radio_controller_TxRxDisable(RC_BASEADDR, rfsel);
			break;
			case IFC_CHANNEL:
				rfsel =  Xil_Ntohl(cmdArgs32[0]);
				band =  Xil_Ntohl(cmdArgs32[1]);
				chan =  Xil_Ntohl(cmdArgs32[2]);
				radio_controller_setCenterFrequency(RC_BASEADDR, rfsel, band, chan);
			break;
			case IFC_TX_GAINS:
				rfsel =  Xil_Ntohl(cmdArgs32[0]);
				bbGain =  Xil_Ntohl(cmdArgs32[1]);
				rfGain =  Xil_Ntohl(cmdArgs32[2]);

				radio_controller_setRadioParam(RC_BASEADDR, rfsel, RC_PARAMID_TXGAIN_BB, bbGain);
				radio_controller_setRadioParam(RC_BASEADDR, rfsel, RC_PARAMID_TXGAIN_RF, rfGain);
			break;
			case IFC_RX_GAINS:
				rfsel =  Xil_Ntohl(cmdArgs32[0]);
				rfGain =  Xil_Ntohl(cmdArgs32[1]);
				bbGain =  Xil_Ntohl(cmdArgs32[2]);

				radio_controller_setRadioParam(RC_BASEADDR, rfsel, RC_PARAMID_RXGAIN_RF, rfGain);
				radio_controller_setRadioParam(RC_BASEADDR, rfsel, RC_PARAMID_RXGAIN_BB, bbGain);
			break;
			case IFC_TX_LPF_CORN_FREQ:
				rfsel =  Xil_Ntohl(cmdArgs32[0]);
				Radios_Tx_LPF_Corn_Freq =  Xil_Ntohl(cmdArgs32[1]);

				radio_controller_setRadioParam(RC_BASEADDR, rfsel, RC_PARAMID_TXLPF_BW, Radios_Tx_LPF_Corn_Freq);
			break;
			case IFC_RX_LPF_CORN_FREQ:
				rfsel =  Xil_Ntohl(cmdArgs32[0]);
				Radios_Rx_LPF_Corn_Freq =  Xil_Ntohl(cmdArgs32[1]);

				radio_controller_setRadioParam(RC_BASEADDR, rfsel, RC_PARAMID_RXLPF_BW, Radios_Rx_LPF_Corn_Freq);
			break;
			case IFC_RX_GAIN_CTRL_SRC:
				rfsel =  Xil_Ntohl(cmdArgs32[0]);
				agc_en = Xil_Ntohl(cmdArgs32[1]) & 0x1;

				if(agc_en==0){
					//Manual gain control
					radio_controller_setCtrlSource(RC_BASEADDR, (rfsel), RC_REG0_RXHP_CTRLSRC, RC_CTRLSRC_REG);
					radio_controller_setRxGainSource(RC_BASEADDR, (rfsel), RC_GAINSRC_SPI);

					//Disable all AGC state machines and trigger inputs
					warplab_AGC_WriteReg_AGC_EN(0);
					warplab_AGC_WriteReg_TrigEn(0);

					//De-select AGC I/Q signals for Rx buffers input and disable buffers->AGC trigger
					warplab_buffers_ConfigReg_Clear(WL_BUFFERS_CFG_REG_AGC_IQ_SEL);

				} else {
					//Automatic gain control
					radio_controller_setCtrlSource(RC_BASEADDR, (rfsel), RC_REG0_RXHP_CTRLSRC, RC_CTRLSRC_HW);
					radio_controller_setRxGainSource(RC_BASEADDR, (rfsel), RC_GAINSRC_HW);

					//Enable AGC state machines (bit per interface) and trigger input (shared by all interfaces)
					warplab_AGC_WriteReg_AGC_EN(rfsel>>28);
					warplab_AGC_WriteReg_TrigEn(0x1);

					//Select AGC I/Q signals for Rx buffers input and enable buffers->AGC trigger
					warplab_buffers_ConfigReg_Set(WL_BUFFERS_CFG_REG_AGC_IQ_SEL);
				}

			break;


			default:
				xil_printf("Unknown interface command ID: %d\n", cmdID);
			break;
	}
	return respSent;
}


int ifc_init(){

#ifdef _DEBUG_
	xil_printf("0x%08x\n", Xil_In32(RC_BASEADDR + RC_SLV_REG9_OFFSET));
#endif

	int status = 0;

#ifdef WARP_HW_VER_v3
	//Initialize the AD9963 ADCs/DACs
	status = ad_init(AD_BASEADDR, ALL_AD_SEL, 2);
	if(status != XST_SUCCESS) {
		xil_printf("w3_node_init: Error in ad_init (%d)\n", status);
		xil_printf("\n************************************************************\n");
		xil_printf(" Check that software and hardware config match\n  (this error may indicate 4-radio code on 2-radio hadrware)\n");
		xil_printf("************************************************************\n\n");
		return(XST_FAILURE);
	}
#endif

	//Initialize the radio_controller core and MAX2829 transceivers
	//status = radio_controller_init(RC_BASEADDR, (ALL_RF_SEL), 1, 1);
	//if(status != XST_SUCCESS) {
	//	xil_printf("node_init: Error in radio_controller_initialize (%d)\n", status);
	//	return(XST_FAILURE);
	//}
	radio_controller_init(RC_BASEADDR, (ALL_RF_SEL), 1, 1);

	//Default the Tx/Rx gain control sources to SPI
	radio_controller_setTxGainSource(RC_BASEADDR, (ALL_RF_SEL), RC_GAINSRC_SPI);
	radio_controller_setRxGainSource(RC_BASEADDR, (ALL_RF_SEL), RC_GAINSRC_SPI);

	//Apply the TxDCO correction values stored in the on-board (and FMC, if present) EEPROMs
#ifdef WARP_HW_VER_v3
	radio_controller_apply_TxDCO_calibration(AD_BASEADDR, EEPROM_BASEADDR, (ALL_RF_SEL));
#endif
#ifdef WARP_HW_VER_v2
	radio_controller_apply_TxDCO_calibration(RC_BASEADDR, EEPROM_BASEADDR, (ALL_RF_SEL));
#endif

	//Set some sane defaults for the MAX2829 Tx/Rx paths; these can be changed by WARPLab commands later

	//Set Tx bandwidth to nominal mode
	Radios_Tx_LPF_Corn_Freq = 1;
	radio_controller_setRadioParam(RC_BASEADDR, (ALL_RF_SEL), RC_PARAMID_TXLPF_BW, Radios_Tx_LPF_Corn_Freq);

	//Set Rx bandwidth to nominal mode
	Radios_Rx_LPF_Corn_Freq = 1;
	radio_controller_setRadioParam(RC_BASEADDR, (ALL_RF_SEL), RC_PARAMID_RXLPF_BW, Radios_Rx_LPF_Corn_Freq);

	//Set Radios to use 30KHz cutoff on HPF
	radio_controller_setRadioParam(RC_BASEADDR, (ALL_RF_SEL), RC_PARAMID_RXHPF_HIGH_CUTOFF_EN, 1);
	radio_controller_setRadioParam(RC_BASEADDR, (ALL_RF_SEL), RC_PARAMID_TXLINEARITY_VGA, 2);
	radio_controller_TxRxDisable(RC_BASEADDR, (ALL_RF_SEL));

	return status;
}

