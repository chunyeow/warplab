////////////////////////////////////////////////////////////////////////////////
// File   :	wl_common.c
// Authors:	Chris Hunter (chunter [at] mangocomm.com)
//			Patrick Murphy (murphpo [at] mangocomm.com)
//          Erik Welsh (welsh [at] mangocomm.com)
// License:	Copyright 2013, Mango Communications. All rights reserved.
//			Distributed under the WARP license  (http://warpproject.org/license)
////////////////////////////////////////////////////////////////////////////////

#include "include/wl_common.h"
#include "xgpio.h"

#include <xparameters.h>
#include <xtmrctr.h>
#include "stdio.h"

XTmrCtr TimerCounter; /* The instance of the Tmrctr Device */

static XGpio GPIO_debugpin;

#ifdef WARP_HW_VER_v3
#define DEBUG_GPIO_DEVICE_ID XPAR_AXI_GPIO_0_DEVICE_ID

int wl_timer_initialize(){
	int status = 0;
	XTmrCtr *TmrCtrInstancePtr = &TimerCounter;

	XTmrCtr_Config *TmrCtrConfigPtr;
	//Initialize the timer counter
	status = XTmrCtr_Initialize(TmrCtrInstancePtr, TMRCTR_DEVICE_ID);
	if (status == XST_DEVICE_IS_STARTED) {
		print("Timer was already running; clear/init manually\n");
		TmrCtrConfigPtr = XTmrCtr_LookupConfig(TMRCTR_DEVICE_ID);
		TmrCtrInstancePtr->BaseAddress = TmrCtrConfigPtr->BaseAddress;
		TmrCtrInstancePtr->IsReady = XIL_COMPONENT_IS_READY;
		XTmrCtr_Stop(TmrCtrInstancePtr, 0);
		XTmrCtr_Reset(TmrCtrInstancePtr, 0);
		status = XTmrCtr_Initialize(TmrCtrInstancePtr, TMRCTR_DEVICE_ID);
	}
	if (status != XST_SUCCESS) {
		xil_printf("w3_node_init: Error in XtmrCtr_Initialize (%d)\n", status);
	}

	// Set timer 0 to into a "count down" mode
	XTmrCtr_SetOptions(TmrCtrInstancePtr, 0, (XTC_DOWN_COUNT_OPTION));

	///Timer Setup///
	XTmrCtr_SetResetValue(TmrCtrInstancePtr,1,0); //Sets it so issuing a "start" command begins at counter=0
	/////////////////


	return status;
}

void usleep(unsigned int duration){
	XTmrCtr *TmrCtrInstancePtr = &TimerCounter;
	XTmrCtr_SetResetValue(TmrCtrInstancePtr,0,duration*(TIMER_FREQ/1000000));
	XTmrCtr_Start(TmrCtrInstancePtr,0);
	volatile u8 isExpired = 0;
	while(isExpired!=1){
		isExpired = XTmrCtr_IsExpired(TmrCtrInstancePtr,0);
	}
	XTmrCtr_Reset(TmrCtrInstancePtr,0);
	return;
}

int wl_gpio_debug_initialize(){
	XGpio_Initialize(&GPIO_debugpin, DEBUG_GPIO_DEVICE_ID);
	XGpio_DiscreteClear(&GPIO_debugpin, 1, 0xFF);
	return 0;
}
#endif

#ifdef WARP_HW_VER_v2
#define DEBUG_GPIO_DEVICE_ID XPAR_XPS_GPIO_0_DEVICE_ID

int wl_timer_initialize(){
	int status = 0;
	XTmrCtr *TmrCtrInstancePtr = &TimerCounter;

	XTmrCtr_Config *TmrCtrConfigPtr;
	//Initialize the timer counter
	status = XTmrCtr_Initialize(TmrCtrInstancePtr, TMRCTR_DEVICE_ID);
	if (status == XST_DEVICE_IS_STARTED) {
		xil_printf("Timer was already running; clear/init manually\n");
		TmrCtrConfigPtr = XTmrCtr_LookupConfig(TMRCTR_DEVICE_ID);
		TmrCtrInstancePtr->BaseAddress = TmrCtrConfigPtr->BaseAddress;
		TmrCtrInstancePtr->IsReady = XIL_COMPONENT_IS_READY;
		XTmrCtr_Stop(TmrCtrInstancePtr, 0);
		XTmrCtr_Reset(TmrCtrInstancePtr, 0);
		status = XTmrCtr_Initialize(TmrCtrInstancePtr, TMRCTR_DEVICE_ID);
	}
	if (status != XST_SUCCESS) {
		xil_printf("w3_node_init: Error in XtmrCtr_Initialize (%d)\n", status);
	}

	// Set timer 0 to into a "count down" mode
	XTmrCtr_SetOptions(TmrCtrInstancePtr, 0, (XTC_DOWN_COUNT_OPTION));

	///Timer Setup///
	XTmrCtr_SetResetValue(TmrCtrInstancePtr,1,0); //Sets it so issuing a "start" command begins at counter=0
	/////////////////


	return status;
}

#ifdef WARP_HW_VER_v3
void usleep(unsigned int duration){
	XTmrCtr *TmrCtrInstancePtr = &TimerCounter;
	XTmrCtr_SetResetValue(TmrCtrInstancePtr,0,duration*(TIMER_FREQ/1000000));
	XTmrCtr_Start(TmrCtrInstancePtr,0);
	volatile u8 isExpired = 0;
	while(isExpired!=1){
		isExpired = XTmrCtr_IsExpired(TmrCtrInstancePtr,0);
	}
	XTmrCtr_Reset(TmrCtrInstancePtr,0);
	return;
}

void wl_set_timer() {
	/* Set timer as up counter */
	XTmrCtr_SetOptions(&TimerCounter, 0, 0);
	XTmrCtr_SetResetValue(&TimerCounter, 0, 0);
	XTmrCtr_Reset(&TimerCounter, 0);
	XTmrCtr_Start(&TimerCounter, 0);
}

void wl_setback_timer() {
	/* Set timer as down counter again */
	XTmrCtr_SetOptions(&TimerCounter, 0, (XTC_DOWN_COUNT_OPTION));
	XTmrCtr_SetResetValue(&TimerCounter, 0, 0);
	XTmrCtr_Reset(&TimerCounter, 0);
	XTmrCtr_Start(&TimerCounter, 0);
}

u32 get_timestamp() {
	u32 val = XTmrCtr_GetValue(&TimerCounter, 0);
	/* Reset the counter if counter has reached its max value */
	if (val == 0){
		XTmrCtr_Reset(&TimerCounter,0);
		val = XTmrCtr_GetValue(&TimerCounter, 0);
	}
	return val;
}
#endif

int wl_gpio_debug_initialize(){
	XGpio_Initialize(&GPIO_debugpin, DEBUG_GPIO_DEVICE_ID);
	XGpio_DiscreteClear(&GPIO_debugpin, 1, 0xFF);
	return 0;
}
#endif

inline void wl_setDebugGPIO(u8 mask){
	XGpio_DiscreteSet(&GPIO_debugpin, 1, mask);
}

inline void wl_clearDebugGPIO(u8 mask){
	XGpio_DiscreteClear(&GPIO_debugpin, 1, mask);
}

unsigned char sevenSegmentMap(unsigned char x){
	switch(x)
	{
		case(0x0) : return 0x007E;
		case(0x1) : return 0x0030;
		case(0x2) : return 0x006D;
		case(0x3) : return 0x0079;
		case(0x4) : return 0x0033;
		case(0x5) : return 0x005B;
		case(0x6) : return 0x005F;
		case(0x7) : return 0x0070;
		case(0x8) : return 0x007F;
		case(0x9) : return 0x007B;

		case(0xA) : return 0x0077;
		case(0xB) : return 0x007F;
		case(0xC) : return 0x004E;
		case(0xD) : return 0x007E;
		case(0xE) : return 0x004F;
		case(0xF) : return 0x0047;
		default : return 0x0000;
	}
}


