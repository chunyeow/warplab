////////////////////////////////////////////////////////////////////////////////
// File   :	wl_baseband.h
// Authors:	Chris Hunter (chunter [at] mangocomm.com)
//			Patrick Murphy (murphpo [at] mangocomm.com)
//          Erik Welsh (welsh [at] mangocomm.com)
// License:	Copyright 2013, Mango Communications. All rights reserved.
//			Distributed under the WARP license  (http://warpproject.org/license)
////////////////////////////////////////////////////////////////////////////////

#include "wl_common.h"
#include <xparameters.h>

#ifndef WL_BASEBAND_H_
#define WL_BASEBAND_H_

// NOTE:  Due to hardware updates in WARPLab 7.3.0 for WARP v3, the buffers
//   cores between WARP v2 and WARP v3 no longer have the same defines.
//   The following file contains two separate sets of defines that will 
//   support:
//      WARP v3 - w3_warplab_buffers_axiw_v2_03_a
//      WARP v2 - w2_warplab_buffers_plbw_v2_02_d
//
//   Please refer to the appropriate section of this file for the correct 
//   defines.
//



//Command IDs (must match the CMD_ properties in wl_baseband_buffers.m)
#define BB_AGC_STATE		           30
#define BB_AGC_THRESH		           31
#define BB_AGC_TARGET                  32
#define BB_AGC_NOISE_EST	           33
#define BB_AGC_DCO_EN_DIS	           34
#define BB_AGC_TRIG_DELAY	           35
#define BB_AGC_RESET		           36
#define BB_AGC_DONE_ADDR	           37
#define BB_AGC_RESET_TIMEOUT           38

#define BB_TX_DELAY			            1
#define BB_TX_LENGTH		            2
#define BB_TX_MODE			            3
#define BB_TX_BUFF_EN		            4
#define BB_RX_BUFF_EN		            5
#define BB_TXRX_BUFF_DIS	            6
#define BB_WRITE_IQ			            7
#define BB_READ_IQ			            8
#define BB_READ_RSSI		            9
#define BB_RX_LENGTH		           10

#define BB_A                           0x1
#define BB_B                           0x2
#define BB_C                           0x4
#define BB_D                           0x8

#define BYTES_PER_SAMP                 4

typedef u32 wl_samp;

//5: [cmdID,len,buffSel,pktIndex,startOffset]
//#define wl_getNumSamps(x) (x>>2)-5

#define SAMPHDR_FLAG_CHKSUMRESET	0x1

typedef struct{
	u16 buffSel;
	u8  flags;
	u8  reserved;
	u32 startSamp;
	u32 numSamp;
} wl_bb_sampHdr;




#ifdef WARP_HW_VER_v3

//defines for sample buffer addresses- renamed from XPAR* here for easier maintanecnce
//
// Defines for w3_warplab_buffers_axiw_v2_02_b and earlier
// #define WARPLAB_IQ_TX_BUFFA            XPAR_WARPLAB_BUFFERS_MEMMAP_RFA_IQ_TX_BUFFER
// #define WARPLAB_IQ_TX_BUFFB            XPAR_WARPLAB_BUFFERS_MEMMAP_RFB_IQ_TX_BUFFER
// #define WARPLAB_IQ_TX_BUFFC            XPAR_WARPLAB_BUFFERS_MEMMAP_RFC_IQ_TX_BUFFER
// #define WARPLAB_IQ_TX_BUFFD            XPAR_WARPLAB_BUFFERS_MEMMAP_RFD_IQ_TX_BUFFER
//
// #define WARPLAB_IQ_RX_BUFFA            XPAR_WARPLAB_BUFFERS_MEMMAP_RFA_IQ_RX_BUFFER
// #define WARPLAB_IQ_RX_BUFFB            XPAR_WARPLAB_BUFFERS_MEMMAP_RFB_IQ_RX_BUFFER
// #define WARPLAB_IQ_RX_BUFFC            XPAR_WARPLAB_BUFFERS_MEMMAP_RFC_IQ_RX_BUFFER
// #define WARPLAB_IQ_RX_BUFFD            XPAR_WARPLAB_BUFFERS_MEMMAP_RFD_IQ_RX_BUFFER

#define WARPLAB_IQ_TX_BUFFA            XPAR_RFA_IQ_TX_BUFFER_CTRL_S_AXI_BASEADDR
#define WARPLAB_IQ_RX_BUFFA            XPAR_RFA_IQ_RX_BUFFER_CTRL_S_AXI_BASEADDR
#define WARPLAB_IQ_TX_BUFFB            XPAR_RFB_IQ_TX_BUFFER_CTRL_S_AXI_BASEADDR
#define WARPLAB_IQ_RX_BUFFB            XPAR_RFB_IQ_RX_BUFFER_CTRL_S_AXI_BASEADDR

#define WARPLAB_IQ_TX_BUFFA_SIZE      (XPAR_RFA_IQ_TX_BUFFER_CTRL_S_AXI_HIGHADDR - XPAR_RFA_IQ_TX_BUFFER_CTRL_S_AXI_BASEADDR)
#define WARPLAB_IQ_RX_BUFFA_SIZE      (XPAR_RFA_IQ_RX_BUFFER_CTRL_S_AXI_HIGHADDR - XPAR_RFA_IQ_RX_BUFFER_CTRL_S_AXI_BASEADDR)
#define WARPLAB_IQ_TX_BUFFB_SIZE      (XPAR_RFB_IQ_TX_BUFFER_CTRL_S_AXI_HIGHADDR - XPAR_RFB_IQ_TX_BUFFER_CTRL_S_AXI_BASEADDR)
#define WARPLAB_IQ_RX_BUFFB_SIZE      (XPAR_RFB_IQ_RX_BUFFER_CTRL_S_AXI_HIGHADDR - XPAR_RFB_IQ_RX_BUFFER_CTRL_S_AXI_BASEADDR)

// NOTE:  Since the 2RF design does not contain memories for the RFC and RFD buffers
//   we will map RFC and RFD to 0 and set the buffer size to 0 so there are no issues
//   since there is no physical memory allocated (unlike previous revisions
//   where the memory was still allocated even though it wasn't used).
#if WARPLAB_CONFIG_4RF
#define WARPLAB_IQ_TX_BUFFC XPAR_RFC_IQ_TX_BUFFER_CTRL_S_AXI_BASEADDR
#define WARPLAB_IQ_RX_BUFFC XPAR_RFC_IQ_RX_BUFFER_CTRL_S_AXI_BASEADDR
#define WARPLAB_IQ_TX_BUFFD XPAR_RFD_IQ_TX_BUFFER_CTRL_S_AXI_BASEADDR
#define WARPLAB_IQ_RX_BUFFD XPAR_RFD_IQ_RX_BUFFER_CTRL_S_AXI_BASEADDR

#define WARPLAB_IQ_TX_BUFFC_SIZE      (XPAR_RFC_IQ_TX_BUFFER_CTRL_S_AXI_HIGHADDR - XPAR_RFC_IQ_TX_BUFFER_CTRL_S_AXI_BASEADDR)
#define WARPLAB_IQ_RX_BUFFC_SIZE      (XPAR_RFC_IQ_RX_BUFFER_CTRL_S_AXI_HIGHADDR - XPAR_RFC_IQ_RX_BUFFER_CTRL_S_AXI_BASEADDR)
#define WARPLAB_IQ_TX_BUFFD_SIZE      (XPAR_RFD_IQ_TX_BUFFER_CTRL_S_AXI_HIGHADDR - XPAR_RFD_IQ_TX_BUFFER_CTRL_S_AXI_BASEADDR)
#define WARPLAB_IQ_RX_BUFFD_SIZE      (XPAR_RFD_IQ_RX_BUFFER_CTRL_S_AXI_HIGHADDR - XPAR_RFD_IQ_RX_BUFFER_CTRL_S_AXI_BASEADDR)
#else
#define WARPLAB_IQ_TX_BUFFC            0
#define WARPLAB_IQ_RX_BUFFC            0
#define WARPLAB_IQ_TX_BUFFD            0
#define WARPLAB_IQ_RX_BUFFD            0

#define WARPLAB_IQ_TX_BUFFC_SIZE       0
#define WARPLAB_IQ_RX_BUFFC_SIZE       0
#define WARPLAB_IQ_TX_BUFFD_SIZE       0
#define WARPLAB_IQ_RX_BUFFD_SIZE       0
#endif

#endif


#ifdef WARP_HW_VER_v2

//defines for sample buffer addresses- renamed from XPAR* here for easier maintanecnce
#define WARPLAB_IQ_TX_BUFFA            XPAR_WARPLAB_BUFFERS_MEMMAP_RFA_IQ_TX_BUFFER
#define WARPLAB_IQ_TX_BUFFB            XPAR_WARPLAB_BUFFERS_MEMMAP_RFB_IQ_TX_BUFFER
#define WARPLAB_IQ_TX_BUFFC            XPAR_WARPLAB_BUFFERS_MEMMAP_RFC_IQ_TX_BUFFER
#define WARPLAB_IQ_TX_BUFFD            XPAR_WARPLAB_BUFFERS_MEMMAP_RFD_IQ_TX_BUFFER

#define WARPLAB_IQ_RX_BUFFA            XPAR_WARPLAB_BUFFERS_MEMMAP_RFA_IQ_RX_BUFFER
#define WARPLAB_IQ_RX_BUFFB            XPAR_WARPLAB_BUFFERS_MEMMAP_RFB_IQ_RX_BUFFER
#define WARPLAB_IQ_RX_BUFFC            XPAR_WARPLAB_BUFFERS_MEMMAP_RFC_IQ_RX_BUFFER
#define WARPLAB_IQ_RX_BUFFD            XPAR_WARPLAB_BUFFERS_MEMMAP_RFD_IQ_RX_BUFFER

#endif


#define WARPLAB_RSSI_BUFFA             XPAR_WARPLAB_BUFFERS_MEMMAP_RFA_RSSI_BUFFER
#define WARPLAB_RSSI_BUFFB             XPAR_WARPLAB_BUFFERS_MEMMAP_RFB_RSSI_BUFFER
#define WARPLAB_RSSI_BUFFC             XPAR_WARPLAB_BUFFERS_MEMMAP_RFC_RSSI_BUFFER
#define WARPLAB_RSSI_BUFFD             XPAR_WARPLAB_BUFFERS_MEMMAP_RFD_RSSI_BUFFER


#define warplab_buffers_WriteReg_TxDelay(data)        XIo_Out32(XPAR_WARPLAB_BUFFERS_MEMMAP_TXDELAY, data)

#define warplab_buffers_WriteReg_TxEn(rfSel)         (XIo_Out32(XPAR_WARPLAB_BUFFERS_MEMMAP_TX_BUFF_EN, \
											           (XIo_In32(XPAR_WARPLAB_BUFFERS_MEMMAP_TX_BUFF_EN)&(~rfSel)) | \
											           (rfSel)))

#define warplab_buffers_WriteReg_RxEn(rfSel)         (XIo_Out32(XPAR_WARPLAB_BUFFERS_MEMMAP_RX_BUFF_EN, \
											           (XIo_In32(XPAR_WARPLAB_BUFFERS_MEMMAP_RX_BUFF_EN)&(~rfSel)) | \
											           (rfSel)))

#define warplab_buffers_WriteReg_TxRxDis(rfSel)      (XIo_Out32(XPAR_WARPLAB_BUFFERS_MEMMAP_RX_BUFF_EN, \
											           (XIo_In32(XPAR_WARPLAB_BUFFERS_MEMMAP_RX_BUFF_EN)&(~rfSel))))

#define warplab_buffers_WriteReg_TxLength(data)       XIo_Out32(XPAR_WARPLAB_BUFFERS_MEMMAP_TXLENGTH, data)

#define warplab_buffers_WriteReg_RxLength(data)       XIo_Out32(XPAR_WARPLAB_BUFFERS_MEMMAP_RXLENGTH, data)


//Masks for CONFIG register
#define WL_BUFFERS_CFG_REG_SW_TRIG                    0x0001
//#define WL_BUFFERS_CFG_REG_HW_TRIG_EN	              0x0002
#define WL_BUFFERS_CFG_REG_CONT_TX                    0x0004
#define WL_BUFFERS_CFG_REG_STOP_TX                    0x0008
#define WL_BUFFERS_CFG_REG_AGC_IQ_SEL                 0x0010
#define WL_BUFFERS_CFG_REG_WORD_ORDER                 0x0020
#define WL_BUFFERS_CFG_REG_BYTE_ORDER                 0x0040
#define WL_BUFFERS_CFG_REG_RSSI_CLK_SEL               0x0180
#define WL_BUFFERS_CFG_REG_COUNTER_DATA_SEL           0x1000


#define warplab_buffers_ConfigReg_Set(mask)           XIo_Out32(XPAR_WARPLAB_BUFFERS_MEMMAP_CONFIG, \
                                                        (XIo_In32(XPAR_WARPLAB_BUFFERS_MEMMAP_CONFIG) | (mask)))

#define warplab_buffers_ConfigReg_SetRSSIClk(val)     XIo_Out32(XPAR_WARPLAB_BUFFERS_MEMMAP_CONFIG, \
                                                        ((XIo_In32(XPAR_WARPLAB_BUFFERS_MEMMAP_CONFIG) & \
                                                        (~(WL_BUFFERS_CFG_REG_RSSI_CLK_SEL))) | ((val<<7)&WL_BUFFERS_CFG_REG_RSSI_CLK_SEL)))

#define warplab_buffers_ConfigReg_Clear(mask)         XIo_Out32(XPAR_WARPLAB_BUFFERS_MEMMAP_CONFIG, \
                                                        (XIo_In32(XPAR_WARPLAB_BUFFERS_MEMMAP_CONFIG) & (~(mask))))

#define warplab_buffers_ReadReg_Config()              XIo_In32(XPAR_WARPLAB_BUFFERS_MEMMAP_CONFIG)


// Masks for Status register
#define WL_BUFFERS_STATUS_REG_TX_RUNNING	          0x1
#define WL_BUFFERS_STATUS_REG_RX_RUNNING	          0x2


#define warplab_buffers_ReadReg_Status()              XIo_In32(XPAR_WARPLAB_BUFFERS_MEMMAP_STATUS)
#define warplab_buffers_ReadReg_TxDelay()             XIo_In32(XPAR_WARPLAB_BUFFERS_MEMMAP_TXDELAY)
#define warplab_buffers_ReadReg_CaptOffset()          XIo_In32(XPAR_WARPLAB_BUFFERS_MEMMAP_CAPTOFFSET)
#define warplab_buffers_ReadReg_CaptureDone()         XIo_In32(XPAR_WARPLAB_BUFFERS_MEMMAP_CAPTUREDONE)
#define warplab_buffers_ReadReg_TransMode()           XIo_In32(XPAR_WARPLAB_BUFFERS_MEMMAP_TRANSMODE)
#define warplab_buffers_ReadReg_TxLength()            XIo_In32(XPAR_WARPLAB_BUFFERS_MEMMAP_TXLENGTH)
#define warplab_buffers_ReadReg_RxLength()            XIo_In32(XPAR_WARPLAB_BUFFERS_MEMMAP_RXLENGTH)
#define warplab_buffers_ReadReg_MGC_AGC_SEL()         XIo_In32(XPAR_WARPLAB_BUFFERS_MEMMAP_MGC_AGC_SEL)
#define warplab_buffers_ReadReg_DCO_EN_SEL()          XIo_In32(XPAR_WARPLAB_BUFFERS_MEMMAP_DCO_EN_SEL)
#define warplab_buffers_ReadReg_AGCDoneAddr()         XIo_In32(XPAR_WARPLAB_BUFFERS_MEMMAP_AGCDONEADDR)
#define warplab_buffers_ReadReg_BuffSizes()           XIo_In32(XPAR_WARPLAB_BUFFERS_MEMMAP_BUFF_SIZES)
#define warplab_buffers_ReadReg_DesignVer()           XIo_In32(XPAR_WARPLAB_BUFFERS_MEMMAP_DESIGN_VER)

#define warplab_buffers_ReadReg_RFA_AGCDoneRSSI()    (XIo_In32(XPAR_WARPLAB_BUFFERS_MEMMAP_RFAB_AGCDONERSSI) & 0x3FF)
#define warplab_buffers_ReadReg_RFB_AGCDoneRSSI()   ((XIo_In32(XPAR_WARPLAB_BUFFERS_MEMMAP_RFAB_AGCDONERSSI) & 0x3FF0000)>>16)
#define warplab_buffers_ReadReg_RFC_AGCDoneRSSI()    (XIo_In32(XPAR_WARPLAB_BUFFERS_MEMMAP_RFCD_AGCDONERSSI) & 0x3FF)
#define warplab_buffers_ReadReg_RFD_AGCDoneRSSI()   ((XIo_In32(XPAR_WARPLAB_BUFFERS_MEMMAP_RFCD_AGCDONERSSI) & 0x3FF0000)>>16)



int          baseband_init();
int          baseband_processCmd(const wl_cmdHdr*, const void*, wl_respHdr*, void*, void*, unsigned int);
unsigned int baseband_updateChecksum(unsigned short int currCheck, unsigned char reset );

void         warplab_agc_init();
void         warplab_agc_setDCO(unsigned int AGCstate);
void         warplab_agc_reset();
inline void  warplab_agc_setNoiseEstimate(short int noiseEst);


#endif /* WL_BASEBAND_H_ */
