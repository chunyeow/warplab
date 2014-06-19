////////////////////////////////////////////////////////////////////////////////
// File   :	wl_interface.h
// Authors:	Chris Hunter (chunter [at] mangocomm.com)
//			Patrick Murphy (murphpo [at] mangocomm.com)
//          Erik Welsh (welsh [at] mangocomm.com)
// License:	Copyright 2013, Mango Communications. All rights reserved.
//			Distributed under the WARP license  (http://warpproject.org/license)
////////////////////////////////////////////////////////////////////////////////

#include "wl_common.h"
#include <xparameters.h>

#ifndef WL_IFC_H_
#define WL_IFC_H_

#define IFC_TX_EN 				1
#define IFC_RX_EN 				2
#define IFC_TXRX_DIS 			3
#define IFC_CHANNEL 			4
#define IFC_TX_GAINS 			5
#define IFC_RX_GAINS 			6
#define IFC_TX_LPF_CORN_FREQ 	7
#define IFC_RX_LPF_CORN_FREQ 	8
#define IFC_RX_GAIN_CTRL_SRC	9


#define NUMRADIOS 4

#define AGC_A 0x10000000
#define AGC_B 0x20000000
#define AGC_C 0x40000000
#define AGC_D 0x80000000


#define warplab_AGC_WriteReg_SRESET_IN(data) XIo_Out32(XPAR_WARPLAB_AGC_MEMMAP_SRESET_IN, data)
#define warplab_AGC_WriteReg_MRESET_IN(data) XIo_Out32(XPAR_WARPLAB_AGC_MEMMAP_MRESET_IN, data)
#define warplab_AGC_WriteReg_PACKET_IN(data) XIo_Out32(XPAR_WARPLAB_AGC_MEMMAP_PACKET_IN, data)
#define warplab_AGC_WriteReg_T_dB(data) XIo_Out32(XPAR_WARPLAB_AGC_MEMMAP_T_DB, data)
#define warplab_AGC_WriteReg_AGC_EN(data) XIo_Out32(XPAR_WARPLAB_AGC_MEMMAP_AGC_EN, data)
#define warplab_AGC_WriteReg_AVG_LEN(data) XIo_Out32(XPAR_WARPLAB_AGC_MEMMAP_AVG_LEN, data)
#define warplab_AGC_WriteReg_Timing(data) XIo_Out32(XPAR_WARPLAB_AGC_MEMMAP_TIMING, data)
#define warplab_AGC_WriteReg_Thresholds(data) XIo_Out32(XPAR_WARPLAB_AGC_MEMMAP_THRESHOLDS, data)
#define warplab_AGC_WriteReg_ADJ(data) XIo_Out32(XPAR_WARPLAB_AGC_MEMMAP_ADJ, data)
#define warplab_AGC_WriteReg_GBB_init(data) 	XIo_Out32(XPAR_WARPLAB_AGC_MEMMAP_GBB_INIT, data)
#define warplab_AGC_WriteReg_TrigEn(data) XIo_Out32(XPAR_WARPLAB_AGC_MEMMAP_AGC_EN_GLOBAL,data)
#define warplab_AGC_WriteReg_En(rfSel) (XIo_Out32(XPAR_WARPLAB_AGC_MEMMAP_AGC_EN, \
											 (XIo_In32(XPAR_WARPLAB_AGC_MEMMAP_AGC_EN)&(~rfSel)) | \
											 (rfSel)))

#define warplab_AGC_WriteReg_Dis(rfSel) (XIo_Out32(XPAR_WARPLAB_AGC_MEMMAP_AGC_EN, \
											 (XIo_In32(XPAR_WARPLAB_AGC_MEMMAP_AGC_EN)&(~rfSel))))

#define warplab_AGC_WriteReg_AGC_TRIGGER_DELAY(data) XIo_Out32(XPAR_WARPLAB_AGC_MEMMAP_AGC_TRIGGER_DELAY, data);
#define warplab_AGC_WriteReg_DCO_Timing(data) XIo_Out32(XPAR_WARPLAB_AGC_MEMMAP_DCO_TIMING, data);
#define warplab_AGC_WriteReg_Bits(data) XIo_Out32(XPAR_WARPLAB_AGC_MEMMAP_BITS, data);
#define warplab_AGC_WriteReg_RESET_TIMEOUT(data) XIo_Out32(XPAR_WARPLAB_AGC_MEMMAP_RESET_TIMEOUT, data);

#define warplab_AGC_ReadReg_GBB_A() XIo_In32(XPAR_WARPLAB_AGC_MEMMAP_GBB_A)
#define warplab_AGC_ReadReg_GBB_B() XIo_In32(XPAR_WARPLAB_AGC_MEMMAP_GBB_B)
#define warplab_AGC_ReadReg_GBB_C() XIo_In32(XPAR_WARPLAB_AGC_MEMMAP_GBB_C)
#define warplab_AGC_ReadReg_GBB_D() XIo_In32(XPAR_WARPLAB_AGC_MEMMAP_GBB_D)
#define warplab_AGC_ReadReg_GRF_A() XIo_In32(XPAR_WARPLAB_AGC_MEMMAP_GRF_A)
#define warplab_AGC_ReadReg_GRF_B() XIo_In32(XPAR_WARPLAB_AGC_MEMMAP_GRF_B)
#define warplab_AGC_ReadReg_GRF_C() XIo_In32(XPAR_WARPLAB_AGC_MEMMAP_GRF_C)
#define warplab_AGC_ReadReg_GRF_D() XIo_In32(XPAR_WARPLAB_AGC_MEMMAP_GRF_D)
#define warplab_AGC_ReadReg_Thresholds() XIo_In32(XPAR_WARPLAB_AGC_MEMMAP_THRESHOLDS)
#define warplab_AGC_ReadReg_Bits() XIo_In32(XPAR_WARPLAB_AGC_MEMMAP_BITS)
#define warplab_AGC_ReadReg_RESET_TIMEOUT() XIo_In32(XPAR_WARPLAB_AGC_MEMMAP_RESET_TIMEOUT)


int ifc_processCmd(const wl_cmdHdr*, const void*, wl_respHdr*, void*, void*, unsigned int);
int ifc_init();
int ifc_process_cmd(unsigned int* rxBuffer,unsigned int* txBuffer);


#endif /* WL_IFC_H_ */
