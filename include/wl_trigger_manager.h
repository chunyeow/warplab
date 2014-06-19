////////////////////////////////////////////////////////////////////////////////
// File   :	wl_trigger_manager.h
// Authors:	Chris Hunter (chunter [at] mangocomm.com)
//			Patrick Murphy (murphpo [at] mangocomm.com)
//          Erik Welsh (welsh [at] mangocomm.com)
// License:	Copyright 2013, Mango Communications. All rights reserved.
//			Distributed under the WARP license  (http://warpproject.org/license)
////////////////////////////////////////////////////////////////////////////////

#include "wl_common.h"

#ifndef TRIGCONF_H_
#define TRIGCONF_H_

#define TRIG_MNGR_ADD_ETHERNET_TRIG 	1
#define TRIG_MNGR_DEL_ETHERNET_TRIG 	2
#define TRIG_MNGR_CLR_ETHERNET_TRIGS 	3
#define TRIG_MNGR_INPUT_SEL				4
#define TRIG_MNGR_OUTPUT_DELAY			5
#define TRIG_MNGR_OUTPUT_HOLD			6
#define TRIG_MNGR_OUTPUT_READ			7
#define TRIG_MNGR_OUTPUT_CLEAR			8
#define TRIG_MNGR_INPUT_ENABLE			9
#define TRIG_MNGR_INPUT_DEBOUNCE		10
#define TRIG_MNGR_BUSY_THRESHOLD		11
#define TRIG_MNGR_RSSI_AVG_LEN			12
#define TRIG_MNGR_BUSY_MIN_LEN			13
#define TRIG_MNGR_IFC_SEL				14
#define TRIG_MNGR_TEST_TRIGGER 	101

//Fast trig mode uses the warplab_pkt_proc core to "sniff" trigger packets
// at the EMAC and assert the warplab_buffers core trigger in hardawre
//The warplab_pkt_proc packet template is configured below
//Comment out this #define to revert to software triggers
#ifdef WARP_HW_VER_v3
#define FAST_TRIG
#endif

int trigmngr_processCmd(const wl_cmdHdr*, const void*, wl_respHdr*, void*, void*, unsigned int);
int trigmngr_init();
void trigmngr_triggerIn(u32 trigID);
void wl_pkt_proc_setTrigID(u32 matchID, u32 trigID);
u32 trigmngr_trigID_to_OR_mask(u32 trigID);
u32 trigmngr_trigID_to_AND_mask(u32 trigID);


//Defines for warplab_trigger_proc core operators
//Equals
#define U8_OP_EQ 0x01
#define U16_OP_EQ ((U8_OP_EQ<<8) | U8_OP_EQ)
#define U32_OP_EQ ((U8_OP_EQ<<24) | (U8_OP_EQ<<16) | (U8_OP_EQ<<8) | U8_OP_EQ)

//Not-equals
#define U8_OP_NEQ 0x02
#define U16_OP_NEQ ((U8_OP_NEQ<<8) | U8_OP_NEQ)
#define U32_OP_NEQ ((U8_OP_NEQ<<24) | (U8_OP_NEQ<<16) | (U8_OP_NEQ<<8) | U8_OP_NEQ)

//No care (byte is ignored)
#define U8_OP_NC 0x00
#define U16_OP_NC ((U8_OP_NC<<8) | U8_OP_NC)
#define U32_OP_NC ((U8_OP_NC<<24) | (U8_OP_NC<<16) | (U8_OP_NC<<8) | U8_OP_NC)

//Any-of-and (and(x,y) > 0)
#define U8_OP_AA 0x03
#define U16_OP_AA ((U8_OP_AA<<8) | U8_OP_AA)
#define U32_OP_AA ((U8_OP_AA<<24) | (U8_OP_AA<<16) | (U8_OP_AA<<8) | U8_OP_AA)

#define trigger_proc_in_eth_setDelay(val) XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK0, \
		(XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK0) & (0xFFFFFFE0)) | \
		((val & 0x1F)) )


////INPUT TRIGGER CONFIGURATION////

#define trigger_proc_in_eth_setReset() XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK0, \
		(XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK0) & (0xFFFFFFBF)) | \
		((1)<<6) )
#define trigger_proc_in_eth_clearReset() XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK0, \
		(XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK0) & (0xFFFFFFBF)))
#define trigger_proc_in_energy_setReset() XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK0, \
		(XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK0) & (0xFFFFBFFF)) | \
		((1)<<14) )
#define trigger_proc_in_energy_clearReset() XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK0, \
		(XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK0) & (0xFFFFBFFF)))
#define trigger_proc_in_agcDone_setReset() XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK0, \
		(XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK0) & (0xFFBFFFFF)) | \
		((1)<<22) )
#define trigger_proc_in_agcDone_clearReset() XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK0, \
		(XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK0) & (0xFFBFFFFF)))
#define trigger_proc_in_soft_setReset() XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK0, \
		(XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK0) & (0xBFFFFFFF)) | \
		((1)<<30) )
#define trigger_proc_in_soft_clearReset() XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK0, \
		(XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK0) & (0xBFFFFFFF)))



#define trigger_proc_in_soft_raiseTrigger() XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK0, \
		(XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK0) & (0xDFFFFFFF)) | \
		((1)<<29) )
#define trigger_proc_in_soft_lowerTrigger() XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK0, \
		(XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK0) & (0xDFFFFFFF)))


#define trigger_proc_in_debug0_setDelay(val) XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK1, \
		(XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK1) & (0xFFFFFFE0)) | \
		((val & 0x1F)) )
#define trigger_proc_in_debug1_setDelay(val) XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK1, \
		(XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK1) & (0xFFFFE0FF)) | \
		((val & 0x1F)<<8) )
#define trigger_proc_in_debug2_setDelay(val) XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK1, \
		(XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK1) & (0xFFE0FFFF)) | \
		((val & 0x1F)<<16) )
#define trigger_proc_in_debug3_setDelay(val) XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK1, \
		(XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK1) & (0xE0FFFFFF)) | \
		((val & 0x1F)<<24) )


#define trigger_proc_in_debug0_debounceMode(val) XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK1, \
		(XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK1) & (0xFFFFFFDF)) | \
		((val&1)<<5) )
#define trigger_proc_in_debug1_debounceMode(val) XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK1, \
		(XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK1) & (0xFFFFDFFF)) | \
		((val&1)<<13) )
#define trigger_proc_in_debug2_debounceMode(val) XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK1, \
		(XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK1) & (0xFFDFFFFF)) | \
		((val&1)<<21) )
#define trigger_proc_in_debug3_debounceMode(val) XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK1, \
		(XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK1) & (0xDFFFFFFF)) | \
		((val&1)<<29) )

#define trigger_proc_in_debug0_setReset() XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK1, \
		(XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK1) & (0xFFFFFFBF)) | \
		((1)<<6) )
#define trigger_proc_in_debug0_clearReset() XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK1, \
		(XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK1) & (0xFFFFFFBF)))
#define trigger_proc_in_debug1_setReset() XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK1, \
		(XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK1) & (0xFFFFBFFF)) | \
		((1)<<14) )
#define trigger_proc_in_debug1_clearReset() XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK1, \
		(XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK1) & (0xFFFFBFFF)))
#define trigger_proc_in_debug2_setReset() XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK1, \
		(XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK1) & (0xFFBFFFFF)) | \
		((1)<<22) )
#define trigger_proc_in_debug2_clearReset() XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK1, \
		(XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK1) & (0xFFBFFFFF)))
#define trigger_proc_in_debug3_setReset() XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK1, \
		(XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK1) & (0xBFFFFFFF)) | \
		((1)<<30) )
#define trigger_proc_in_debug3_clearReset() XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK1, \
		(XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK1) & (0xBFFFFFFF)))

#define trigger_proc_in_eth_getDelay(val) (XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK0)) & 0x1F
#define trigger_proc_in_debug0_getDelay(val) (XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK1)) & 0x1F
#define trigger_proc_in_debug1_getDelay(val) (XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK1)>>8) & 0x1F
#define trigger_proc_in_debug2_getDelay(val) (XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK1)>>16) & 0x1F
#define trigger_proc_in_debug3_getDelay(val) (XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_INBANK1)>>24) & 0x1F

////OUTPUT TRIGGER CONFIGURATION////


//Masks for output configuration registers
#define AND_ETH 		0x00000001
#define AND_ENERGY 		0x00000002
#define AND_AGC_DONE 	0x00000004
#define AND_SOFT 		0x00000008
#define AND_DEBUG0 		0x00000010
#define AND_DEBUG1 		0x00000020
#define AND_DEBUG2 		0x00000040
#define AND_DEBUG3 		0x00000080
#define OR_ETH 			0x00000100
#define OR_ENERGY 		0x00000200
#define OR_AGC_DONE 	0x00000400
#define OR_SOFT 		0x00000800
#define OR_DEBUG0 		0x00001000
#define OR_DEBUG1 		0x00002000
#define OR_DEBUG2 		0x00004000
#define OR_DEBUG3 		0x00008000
#define AND_ALL			0x000000FF
#define OR_ALL			0x0000FF00
#define OUT_DLY			0x001F0000
#define OUT_RESET		0x80000000

#define trigger_proc_out0_ConfigReg_Set(mask) XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT0, (XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT0) | (mask)))
#define trigger_proc_out0_ConfigReg_Clear(mask) XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT0, (XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT0) & (~(mask))))
#define trigger_proc_out0_ConfigReg_HoldMode(val) XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT0, (XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT0) & (~(OUT_RESET))) | ((val<<31)&OUT_RESET))
#define trigger_proc_out0_ConfigReg_Delay(val) XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT0, (XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT0) & (~(OUT_DLY))) | ((val<<16)&OUT_DLY))
#define trigger_proc_out0_ConfigReg_Read() XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT0)

#define trigger_proc_out1_ConfigReg_Set(mask) XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT1, (XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT1) | (mask)))
#define trigger_proc_out1_ConfigReg_Clear(mask) XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT1, (XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT1) & (~(mask))))
#define trigger_proc_out1_ConfigReg_HoldMode(val) XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT1, (XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT1) & (~(OUT_RESET))) | ((val<<31)&OUT_RESET))
#define trigger_proc_out1_ConfigReg_Delay(val) XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT1, (XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT1) & (~(OUT_DLY))) | ((val<<16)&OUT_DLY))
#define trigger_proc_out1_ConfigReg_Read() XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT1)

#define trigger_proc_out2_ConfigReg_Set(mask) XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT2, (XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT2) | (mask)))
#define trigger_proc_out2_ConfigReg_Clear(mask) XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT2, (XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT2) & (~(mask))))
#define trigger_proc_out2_ConfigReg_HoldMode(val) XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT2, (XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT2) & (~(OUT_RESET))) | ((val<<31)&OUT_RESET))
#define trigger_proc_out2_ConfigReg_Delay(val) XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT2, (XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT2) & (~(OUT_DLY))) | ((val<<16)&OUT_DLY))
#define trigger_proc_out2_ConfigReg_Read() XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT2)

#define trigger_proc_out3_ConfigReg_Set(mask) XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT3, (XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT3) | (mask)))
#define trigger_proc_out3_ConfigReg_Clear(mask) XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT3, (XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT3) & (~(mask))))
#define trigger_proc_out3_ConfigReg_HoldMode(val) XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT3, (XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT3) & (~(OUT_RESET))) | ((val<<31)&OUT_RESET))
#define trigger_proc_out3_ConfigReg_Delay(val) XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT3, (XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT3) & (~(OUT_DLY))) | ((val<<16)&OUT_DLY))
#define trigger_proc_out3_ConfigReg_Read() XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT3)

#define trigger_proc_out4_ConfigReg_Set(mask) XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT4, (XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT4) | (mask)))
#define trigger_proc_out4_ConfigReg_Clear(mask) XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT4, (XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT4) & (~(mask))))
#define trigger_proc_out4_ConfigReg_HoldMode(val) XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT4, (XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT4) & (~(OUT_RESET))) | ((val<<31)&OUT_RESET))
#define trigger_proc_out4_ConfigReg_Delay(val) XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT4, (XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT4) & (~(OUT_DLY))) | ((val<<16)&OUT_DLY))
#define trigger_proc_out4_ConfigReg_Read() XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT4)

#define trigger_proc_out5_ConfigReg_Set(mask) XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT5, (XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT5) | (mask)))
#define trigger_proc_out5_ConfigReg_Clear(mask) XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT5, (XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT5) & (~(mask))))
#define trigger_proc_out5_ConfigReg_HoldMode(val) XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT5, (XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT5) & (~(OUT_RESET))) | ((val<<31)&OUT_RESET))
#define trigger_proc_out5_ConfigReg_Delay(val) XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT5, (XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT5) & (~(OUT_DLY))) | ((val<<16)&OUT_DLY))
#define trigger_proc_out5_ConfigReg_Read() XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_CONF_OUT5)

#define trigger_proc_CoreInfo_Read() ((XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_CORE_INFO)))

#define OUT0	0x0001
#define OUT1	0x0002
#define OUT2	0x0004
#define OUT3	0x0008
#define OUT4	0x0010
#define OUT5	0x0020

#define trigger_proc_Output_Read() ((XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_TRIG_OUT)))


//Energy Detection Registers
#define WL_PKTDET_CONFIG_REG_RESET		0x01
#define WL_PKTDET_CONFIG_REG_MASK_A		0x04
#define WL_PKTDET_CONFIG_REG_MASK_B		0x08
#define WL_PKTDET_CONFIG_REG_MASK_C		0x10
#define WL_PKTDET_CONFIG_REG_MASK_D		0x20
#define WL_PKTDET_CONDIG_REG_MASK_ALL	0x3C

#define IFC_TO_PKTDETMASK(val) (val>>26) //convert interface masks to packet detect mask

#define warplab_pktdet_ConfigReg_Set(mask) XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_RSSI_PKT_DET_CONFIG, (XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_RSSI_PKT_DET_CONFIG) | (mask)))
#define warplab_pktdet_ConfigReg_Clear(mask) XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_RSSI_PKT_DET_CONFIG, (XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_RSSI_PKT_DET_CONFIG) & (~(mask))))

//#define warplab_pktdet_SetThresholds(idle, busy) XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_RSSI_PKT_DET_THRESHOLDS, ( (idle & 0xFFFF) | ((busy<<16)&0xFFFF0000)))
//#define warplab_pktdet_SetDurations(rssi, idle, busy) XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_RSSI_PKT_DET_DURATIONS, ( (idle & 0xFF) | ((busy<<8)&0xFF00) | ((rssi<<16)&0x1F0000) ))

#define warplab_pktdet_SetIdleThreshold(idle) XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_RSSI_PKT_DET_THRESHOLDS, (XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_RSSI_PKT_DET_THRESHOLDS)&(~0xFFFF)) | ( (idle & 0xFFFF) ))
#define warplab_pktdet_SetBusyThreshold(busy) XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_RSSI_PKT_DET_THRESHOLDS, (XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_RSSI_PKT_DET_THRESHOLDS)&(~0xFFFF0000)) | ( (busy<<16)&0xFFFF0000 ))
#define warplab_pktdet_SetRSSIDuration(rssi) XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_RSSI_PKT_DET_DURATIONS, (XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_RSSI_PKT_DET_DURATIONS)&(~0x1F0000)) | ( ((rssi<<16)&0x1F0000) ))
#define warplab_pktdet_SetIdleDuration(idle) XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_RSSI_PKT_DET_DURATIONS, (XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_RSSI_PKT_DET_DURATIONS)&(~0xFF)) | ( (idle & 0xFF) ))
#define warplab_pktdet_SetBusyDuration(busy) XIo_Out32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_RSSI_PKT_DET_DURATIONS, (XIo_In32(XPAR_WARPLAB_TRIGGER_PROC_MEMMAP_RSSI_PKT_DET_DURATIONS)&(~0xFF00)) | (((busy<<8)&0xFF00) ))

#endif /* TRIGCONF_H_ */
