//*****************************************************************************
//
//! The memory map registers, and bit definitions are defined in this header 
//! file.  The access writes of the memory map are also indicated as follows: 
//! (READ)          : Read only
//! (READ/WRITE)    : Read & Write
//! (READ/WRITE/E2) : Read, Write & default loaded from non-volatile memory
//
//*****************************************************************************

#ifndef __IQS5XX_H
#define __IQS5XX_H


//*****************************************************************************
//
//! ----------------------    IQS5xx-B000 BIT DEFINITIONS     -----------------
//
//*****************************************************************************

//
//! GestureEvents0 bit definitions
//
#define IQS5xx_SWIPE_Y_NEG	    		0x20
#define IQS5xx_SWIPE_Y_POS	    		0x10
#define IQS5xx_SWIPE_X_POS      		0x08
#define IQS5xx_SWIPE_X_NEG        		0x04
#define IQS5xx_TAP_AND_HOLD     		0x02
#define IQS5xx_SINGLE_TAP        		0x01
//
//! GesturesEvents1 bit definitions
//
#define IQS5xx_ZOOM	          		0x04
#define IQS5xx_SCROLL		     		0x02
#define IQS5xx_TWO_FINGER_TAP       	0x01
//
//! SystemInfo0 bit definitions
//
#define	IQS5xx_SHOW_RESET				0x80
#define	IQS5xx_ALP_REATI_OCCURRED		0x40
#define	IQS5xx_ALP_ATI_ERROR			0x20
#define	IQS5xx_REATI_OCCURRED			0x10
#define	IQS5xx_ATI_ERROR				0x08
#define	IQS5xx_CHARGING_MODE_2			0x04
#define IQS5xx_CHARGING_MODE_1			0x02
#define	IQS5xx_CHARGING_MODE_0			0x01
//
//! SystemInfo1 bit definitions
//
#define	IQS5xx_SNAP_TOGGLE				0x10
#define	IQS5xx_RR_MISSED				0x08
#define	IQS5xx_TOO_MANY_FINGERS		0x04
#define IQS5xx_PALM_DETECT				0x02
#define	IQS5xx_TP_MOVEMENT				0x01
//
//! SystemControl0 bit definitions
// 
#define IQS5xx_ACK_RESET         		0x80
#define IQS5xx_AUTO_ATI          		0x20
#define IQS5xx_ALP_RESEED        		0x10
#define IQS5xx_RESEED            		0x08
#define IQS5xx_MODE_SELECT_2			0x04
#define IQS5xx_MODE_SELECT_1			0x02
#define IQS5xx_MODE_SELECT_0			0x01
//
//! SystemControl1 bit definitions
//
#define IQS5xx_RESET             		0x02
#define IQS5xx_SUSPEND           		0x01
//
//! SystemConfig0 bit definitions
//        
#define IQS5xx_MANUAL_CONTROL     		0x80
#define IQS5xx_SETUP_COMPLETE     		0x40
#define IQS5xx_WDT_ENABLE        		0x20
#define IQS5xx_ALP_REATI        		0x08
#define IQS5xx_REATI            		0x04
#define IQS5xx_IO_WAKEUP_SELECT   		0x02
#define IQS5xx_IO_WAKE         		0x01
//
//! SystemConfig1 bit definitions
//  
#define IQS5xx_PROX_EVENT        		0x80
#define IQS5xx_TOUCH_EVENT       		0x40
#define IQS5xx_SNAP_EVENT        		0x20
#define IQS5xx_ALP_PROX_EVENT    		0x10
#define IQS5xx_REATI_EVENT      		0x08
#define IQS5xx_TP_EVENT          		0x04
#define IQS5xx_GESTURE_EVENT     		0x02
#define IQS5xx_EVENT_MODE        		0x01
//
//! FilterSettings0 bit definitions
//
#define IQS5xx_ALP_COUNT_FILTER    	0x08
#define IQS5xx_IIR_SELECT			    0x04
#define IQS5xx_MAV_FILTER     			0x02
#define IQS5xx_IIR_FILTER     			0x01
//
//! ALPChannelSetup0 bit definitions
//		
#define IQS5xx_CHARGE_TYPE  			0x80
#define IQS5xx_RX_GROUP        		0x40
#define IQS5xx_PROX_REV       			0x20
#define IQS5xx_ALP_ENABLE        		0x10
//
//! IQS525RxToTx bit definitions
//
#define IQS5xx_RX7_TX2     			0x80
#define IQS5xx_RX6_TX3     			0x40
#define IQS5xx_RX5_TX4     			0x20
#define IQS5xx_RX4_TX5    				0x10
#define IQS5xx_RX3_TX6    				0x08
#define IQS5xx_RX2_TX7     			0x04
#define IQS5xx_RX1_TX8     			0x02
#define IQS5xx_RX0_TX9     			0x01
//
//! HardwareSettingsA bit definitions
//	
#define IQS5xx_ND_ENABLE         		0x20
#define IQS5xx_RX_FLOAT          		0x04
//
//! HardwareSettingsB bit definitions
//
#define IQS5xx_CK_FREQ_2      			0x40
#define IQS5xx_CK_FREQ_1     			0x20
#define IQS5xx_CK_FREQ_0    			0x10
#define IQS5xx_ANA_DEAD_TIME     		0x02
#define IQS5xx_INCR_PHASE        		0x01
//
//! HardwareSettingsC bit definitions
//
#define IQS5xx_STAB_TIME_1     		0x80
#define IQS5xx_STAB_TIME_0     		0x40
#define IQS5xx_OPAMP_BIAS_1   			0x20
#define IQS5xx_OPAMP_BIAS_0     		0x10
#define IQS5xx_VTRIP_3					0x08
#define IQS5xx_VTRIP_2                 0x04
#define IQS5xx_VTRIP_1                 0x02
#define IQS5xx_VTRIP_0                 0x01
//
//! HardwareSettingsD bit definitions
//
#define IQS5xx_UPLEN_2    				0x40
#define IQS5xx_UPLEN_1   				0x20
#define IQS5xx_UPLEN_0     			0x10
#define IQS5xx_PASSLEN_2               0x04
#define IQS5xx_PASSLEN_1               0x02
#define IQS5xx_PASSLEN_0               0x01
//
//! XYConfig0 bit definitions
//                  		
#define IQS5xx_PALM_REJECT       		0x08
#define IQS5xx_SWITCH_XY_AXIS    		0x04
#define IQS5xx_FLIP_Y            		0x02
#define IQS5xx_FLIP_X            		0x01
//
//! SFGestureEnable bit definitions
//  
#define IQS5xx_SWIPE_Y_MINUS_EN  		0x20
#define IQS5xx_SWIPE_Y_PLUS_EN   		0x10
#define IQS5xx_SWIPE_X_PLUS_EN   		0x08
#define IQS5xx_SWIPE_X_MINUS_EN  		0x04
#define IQS5xx_TAP_AND_HOLD_EN   		0x02
#define IQS5xx_SINGLE_TAP_EN     		0x01
//
//! MFGestureEnable bit definitions
//  
#define IQS5xx_ZOOM_EN           		0x04
#define IQS5xx_SCROLL_EN         		0x02
#define IQS5xx_TWO_FINGER_TAP_EN 		0x01

//*****************************************************************************
//
//! ------------------    IQS5xx-B00 MEMORY MAP REGISTERS    ------------------
//
//*****************************************************************************		
		
/******************** DEVICE INFO REGISTERS ***************************/
#define ProductNumber_adr		0x0000	//(READ)			//2 BYTES;  
#define ProjectNumber_adr		0x0002	//(READ)			//2 BYTES;  
#define MajorVersion_adr		0x0004	//(READ)	
#define MinorVersion_adr		0x0005	//(READ)	
#define BLStatus_adr			0x0006	//(READ)	
/******************** ************************* ***************************/
#define MaxTouch_adr			0x000B	//(READ)	
#define PrevCycleTime_adr		0x000C	//(READ)	
/******************** GESTURES AND EVENT STATUS REGISTERS ***************************/
#define GestureEvents0_adr		0x000D	//(READ)	
#define GestureEvents1_adr		0x000E	//(READ)	
#define SystemInfo0_adr			0x000F	//(READ)	
#define SystemInfo1_adr			0x0010	//(READ)	
/******************** XY DATA REGISTERS ***************************/
#define NoOfFingers_adr			0x0011	//(READ)	
#define RelativeX_adr			0x0012	//(READ)			//2 BYTES;  
#define RelativeY_adr			0x0014	//(READ)		   	//2 BYTES;  
/******************** INDIVIDUAL FINGER DATA ***************************/
#define AbsoluteX_adr			0x0016	//(READ) 2 BYTES	//ADD 0x0007 FOR FINGER 2; 0x000E FOR FINGER 3; 0x0015 FOR FINGER 4 AND 0x001C FOR FINGER 5
#define AbsoluteY_adr			0x0018	//(READ) 2 BYTES	//ADD 0x0007 FOR FINGER 2; 0x000E FOR FINGER 3; 0x0015 FOR FINGER 4 AND 0x001C FOR FINGER 5	
#define TouchStrength_adr		0x001A	//(READ) 2 BYTES	//ADD 0x0007 FOR FINGER 2; 0x000E FOR FINGER 3; 0x0015 FOR FINGER 4 AND 0x001C FOR FINGER 5
#define Area_adr				0x001C	//(READ)			//ADD 0x0007 FOR FINGER 2; 0x000E FOR FINGER 3; 0x0015 FOR FINGER 4 AND 0x001C FOR FINGER 5
/******************** CHANNEL STATUS REGISTERS ***************************/
#define ProxStatus_adr			0x0039	//(READ)	  		//32 BYTES;   
#define TouchStatus_adr			0x0059	//(READ)	 	    //30 BYTES;   
#define SnapStatus_adr			0x0077	//(READ)		    //30 BYTES;   
/******************** DATA STREAMING REGISTERS ***************************/
#define Counts_adr				0x0095	//(READ)	  		//300 BYTES;   
#define Delta_adr				0x01C1	//(READ)	 		//300 BYTES;   
#define ALPCount_adr			0x02ED	//(READ)	 		//2 BYTES;  
#define ALPIndivCounts_adr		0x02EF	//(READ)	 		//20 BYTES;   
#define References_adr			0x0303	//(READ/WRITE)		//300 BYTES;   
#define ALPLTA_adr 				0x042F	//(READ/WRITE)		//2 BYTES;  
/******************** SYSTEM CONTROL REGISTERS ***************************/
#define SystemControl0_adr 		0x0431	//(READ/WRITE)
#define SystemControl1_adr 		0x0432	//(READ/WRITE)
/******************** ATI SETTINGS REGISTERS ***************************/
#define ALPATIComp_adr 			0x0435	//(READ/WRITE)  	//10 BYTES;   
#define	ATICompensation_adr		0x043F	//(READ/WRITE)  	//150 BYTES;   
#define ATICAdjust_adr         	0x04D5	//(READ/WRITE/E2)	//150 BYTES;   
#define GlobalATIC_adr         	0x056B	//(READ/WRITE/E2)
#define ALPATIC_adr				0x056C	//(READ/WRITE/E2)
#define ATITarget_adr			0x056D	//(READ/WRITE/E2)	//2 BYTES;  
#define ALPATITarget_adr		0x056F	//(READ/WRITE/E2)	//2 BYTES;  
#define RefDriftLimit_adr		0x0571	//(READ/WRITE/E2)
#define ALPLTADriftLimit_adr	0x0572	//(READ/WRITE/E2)
#define ReATILowerLimit_adr		0x0573	//(READ/WRITE/E2)
#define ReATIUpperLimit_adr		0x0574	//(READ/WRITE/E2)
#define MaxCountLimit_adr		0x0575	//(READ/WRITE/E2)	//2 BYTES; 
#define ReATIRetryTime_adr		0x0577	//(READ/WRITE/E2)
/******************** TIMING SETTINGS REGISTERS ***************************/
#define ActiveRR_adr		0x057A	//(READ/WRITE/E2)   //2 BYTES;
#define	IdleTouchRR_adr			0x057C	//(READ/WRITE/E2)	//2 BYTES; 
#define	IdleRR_adr				0x057E	//(READ/WRITE/E2)	//2 BYTES; 
#define	LP1RR_adr				0x0580	//(READ/WRITE/E2)	//2 BYTES; 
#define	LP2RR_adr				0x0582	//(READ/WRITE/E2)	//2 BYTES; 
#define	ActiveTimeout_adr		0x0584	//(READ/WRITE/E2)
#define IdleTouchTimeout_adr	0x0585	//(READ/WRITE/E2)
#define	IdleTimeout_adr			0x0586	//(READ/WRITE/E2)
#define	LP1Timeout_adr			0x0587	//(READ/WRITE/E2)
#define	RefUpdateTime_adr		0x0588	//(READ/WRITE/E2)
#define	SnapTimeout_adr			0x0589	//(READ/WRITE/E2)
#define	I2CTimeout_adr			0x058A	//(READ/WRITE/E2)
/******************** SYSTEM CONFIG REGISTERS ***************************/
#define SystemConfig0_adr  		0x058E	//(READ/WRITE/E2)
#define SystemConfig1_adr  		0x058F	//(READ/WRITE/E2)
/******************** THRESHOLD SETTINGS REGISTERS ***************************/
#define SnapThreshold_adr      	0x0592	//(READ/WRITE/E2)   //2 BYTES; 
#define	ProxThreshold_adr		0x0594	//(READ/WRITE/E2)
#define	ALPProxThreshold_adr	0x0595	//(READ/WRITE/E2)
#define	GlobalTouchSet_adr		0x0596	//(READ/WRITE/E2)
#define	GlobalTouchClear_adr	0x0597	//(READ/WRITE/E2)
#define	IndivTouchAdjust_adr	0x0598	//(READ/WRITE/E2)	//150 BYTES; 
/******************** FILTER SETTINGS REGISTERS ***************************/
#define	FilterSettings0_adr		0x0632	//(READ/WRITE/E2)
#define	XYStaticBeta_adr		0x0633	//(READ/WRITE/E2)
#define	ALPCountBeta_adr		0x0634	//(READ/WRITE/E2)
#define	ALP1LTABeta_adr			0x0635	//(READ/WRITE/E2)
#define	ALP2LTABeta_adr			0x0636	//(READ/WRITE/E2)
#define	DynamicBottomBeta_adr	0x0637	//(READ/WRITE/E2)
#define	DynamicLowerSpeed_adr	0x0638	//(READ/WRITE/E2)
#define	DynamicUpperSpeed_adr	0x0639	//(READ/WRITE/E2)   //2 BYTES; 
/******************** CHANNEL SET UP (RX-TX MAPPING) REGISTERS ***************************/
#define	TotalRx_adr				0x063D	//(READ/WRITE/E2)
#define	TotalTx_adr				0x063E	//(READ/WRITE/E2)
#define	RxMapping_adr			0x063F	//(READ/WRITE/E2)	//10 BYTES; 
#define	TxMapping_adr			0x0649	//(READ/WRITE/E2)	//15 BYTES; 
#define	ALPChannelSetup0_adr	0x0658	//(READ/WRITE/E2)
#define	ALPRxSelect_adr			0x0659	//(READ/WRITE/E2)	//2 BYTES; 
#define	ALPTxSelect_adr			0x065B	//(READ/WRITE/E2)	//2 BYTES; 
#define IQS525RxToTx_adr		0x065D  //(READ/WRITE/E2) 
/******************** HARDWARE SETTINGS REGISTERS ***************************/
#define	HardwareSettingsA_adr	0x065F	//(READ/WRITE/E2)
#define	HardwareSettingsB1_adr	0x0660	//(READ/WRITE/E2)
#define	HardwareSettingsB2_adr	0x0661	//(READ/WRITE/E2)
#define	HardwareSettingsC1_adr	0x0662	//(READ/WRITE/E2)
#define	HardwareSettingsC2_adr	0x0663	//(READ/WRITE/E2)
#define	HardwareSettingsD1_adr	0x0664	//(READ/WRITE/E2)
#define	HardwareSettingsD2_adr	0x0665	//(READ/WRITE/E2)
/******************** XY CONFIG REGISTERS ***************************/
#define	XYConfig0_adr			0x0669	//(READ/WRITE/E2)
#define	MaxMultitouches_adr		0x066A	//(READ/WRITE/E2)
#define	FingerSplitFactor_adr	0x066B	//(READ/WRITE/E2)
#define	PalmRejectThreshold_adr	0x066C	//(READ/WRITE/E2)
#define	PalmRejectTimeout_adr	0x066D	//(READ/WRITE/E2)
#define	XResolution_adr			0x066E	//(READ/WRITE/E2)	//2 BYTES; 
#define	YResolution_adr			0x0670	//(READ/WRITE/E2)	//2 BYTES; 
#define	StationaryTouchThr_adr	0x0672	//(READ/WRITE/E2)
/*********************************************************************/
#define	DefaultReadAdr_adr		0x0675	//(READ/WRITE/E2)	
/******************** DEBOUNCE SETTING REGISTERS ***************************/
#define	ProxDb_adr				0x0679	//(READ/WRITE/E2)
#define	TouchSnapDb_adr			0x067A	//(READ/WRITE/E2)
/******************** CHANNEL CONFIG REGISTERS ***************************/
#define	ActiveChannels_adr		0x067B	//(READ/WRITE/E2)	//30 BYTES; 
#define	SnapChannels_adr		0x0699	//(READ/WRITE/E2)   //30 BYTES;
/******************** GESTURE SETTING REGISTERS ***************************/
#define	SFGestureEnable_adr		0x06B7	//(READ/WRITE/E2)
#define	MFGestureEnable_adr		0x06B8	//(READ/WRITE/E2)
#define	TapTime_adr				0x06B9	//(READ/WRITE/E2)	//2 BYTES; 
#define	TapDistance_adr			0x06BB	//(READ/WRITE/E2)	//2 BYTES; 
#define HoldTime_adr           	0x06BD	//(READ/WRITE/E2)   //2 BYTES; 
#define	SwipeInitTime_adr		0x06BF	//(READ/WRITE/E2)	//2 BYTES; 
#define	SwipeInitDistance_adr	0x06C1	//(READ/WRITE/E2)	//2 BYTES; 
#define	SwipeConsTime_adr		0x06C2	//(READ/WRITE/E2)	//2 BYTES; 
#define	SwipeConsDistance_adr	0x06C5	//(READ/WRITE/E2)	//2 BYTES; 
#define	SwipeAngle_adr			0x06C7	//(READ/WRITE/E2)
#define	ScrollInitDistance_adr	0x06C8	//(READ/WRITE/E2)	//2 BYTES; 
#define	ScrollAngle_adr			0x06CA	//(READ/WRITE/E2)
#define	ZoomInitDistance_adr	0x06CB	//(READ/WRITE/E2)	//2 BYTES; 
#define	ZoomConsDistance_adr	0x06CD	//(READ/WRITE/E2)	//2 BYTES; 


#include <stdint.h>
typedef enum {buttonReleased, buttonPressed} buttonState;
typedef void (*IQS5xx_ButtonCallback)(buttonState state);
typedef void (*IQS5xx_TrackpadCallback)(uint8_t x, uint8_t y);


typedef struct {
  uint16_t Xpos;
  uint16_t Ypos;
  buttonState buttonState;
  IQS5xx_ButtonCallback callback;
}IQS5xx_ButtonTypeDef;

typedef struct {
  uint16_t Xmax;
  uint16_t Xmin;
  uint16_t Ymin;
  uint16_t Ymax;
  IQS5xx_TrackpadCallback callback;
}IQS5xx_TrackpadTypeDef;

void IQS5xx_EndCommWindows(void);
void IQS5xx_CheckVersion(void);
void IQS5xx_Refresh_Data(void);
void IQS5xx_Process_New_Data (void);
void IQS5xx_EnableALP(void);
void IQS5xx_DisableALP(void);
void IQS5xx_RegisterButton(IQS5xx_ButtonTypeDef* button);
void IQS5xx_RegisterTrackpad(IQS5xx_TrackpadTypeDef* trackpad);
void IQS5xx_UnregisterAll();
#endif



