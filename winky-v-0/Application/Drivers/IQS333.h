/******************************************************************************
*                                                                             *
*                                                                             *
*                                Copyright by                                 *
*                                                                             *
*                              Azoteq (Pty) Ltd                               *
*                          Republic of South Africa                           *
*                                                                             *
*                           Tel: +27(0)21 863 0033                            *
*                          E-mail: info@azoteq.com                            *
*                                                                             *
*=============================================================================*
* @file 	IQS333.h													      *
* @brief 	Header file for the IQS333 "object" specific commands	          *
* @author 	AJ van der Merwe - Azoteq PTY Ltd                             	  *
* @version 	V1.0.0                                                        	  *
* @date 	27/08/2019                                                     	  *
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __IQS333_H__
#define __IQS333_H__

#include "IQS333_init.h"
#include "stdbool.h"
#include "stdint.h"

/* I2C Address of IQS333 */
#define IQS333_ADDR					0x64

/* Device Info */
#define PRODUCT_NUMBER				54
#define VERSION_NUMBER				2

/* IQS333 Command Structure */
// Communication Command / Address Structure on IQS333 - ie. Memory Map
#define	VERSION_INFO_ADDR				0x00	// Product number can be read      : 2 bytes
#define FLAGS_ADDR 						0x01	// System flags and events         : 1 byte
#define WHEEL_COORDS_ADDR				0x02	// Wheel coordinates - 2 wheels    : 4 bytes
#define TOUCH_BYTES_ADDR     			0x03	// Touch channels                  : 2 bytes
#define COUNTS_ADDR                		0x04	// Count Values                    :20 bytes
#define LTA_REG_ADDR            		0x05	// LTA Values                      :20 bytes
#define MULTIPLIERS_ADDR            	0x06	// Multipliers Values              :10 bytes
#define COMPENSATION_ADDR            	0x07	// Compensation Values (PCC)       :10 bytes
#define PROXSETTINGS_ADDR           	0x08	// Prox Settings - Various         : 6 bytes
#define THRESHOLDS_ADDR              	0x09	// Threshold Values                :10 bytes
#define TIMINGS_ADDR                 	0x0A    // Timings                         : 5 bytes
#define	ATI_TARGETS_ADDR				0x0B	// Targets for ATI                 : 2 bytes
#define PWM_ADDR 						0x0C	// PWM Settings                    : 4 bytes
#define PWM_LIMIT_ADDR					0x0D	// PWM Limits and speed            : 2 bytes
#define ACTIVE_CHANNELS_ADDR     		0x0E	// Active channels                 : 2 bytes
#define BUZZER_ADDR                  	0x0F    // Buzzer                          : 1 byte

/* Base Values */
#define BASE_75						    0x40	// Base 75
#define BASE_100					    0x80	// Base 100
#define BASE_150					    0xC0	// Base 150
#define BASE_200					    0x00	// Base 200 (default)

/* Number of Active channels on IQS333 */
#define NR_OF_ACTIVE_CHANNELS			8		// 7 active channels in this setup + Prox

/* Channel Filter Bits */
#define CH0_B                           0x01
#define CH1_B                           0x02
#define CH2_B	                        0x04
#define CH3_B                           0x08
#define CH4_B                           0x10

// /* Bit Definitions */
// /*	System Flags	*/
#define ZOOM							0x01
#define NOISE							0x02
#define ATI_BUSY						0x04
#define LP_ACTIVE						0x08
#define IS_CH0							0x10
#define PROJ_MODE						0x20
#define FILTER_HALTED					0x40
#define SHOW_RESET						0x80

/*	Multipliers	*/
#define SENS_MUTLIPLIERS				0x0F
#define COMP_MULITPLIERS				0x30
#define BASE_VALUE						0xC0

/*	ProxSettings	*/
/*	ProxSettings 0	*/
#define PROJ_BIAS						0x03
#define CS_SIZE							0x04
#define RESEED							0x08
#define REDO_ATI						0x10
#define ALT_ATI							0x20
#define PARTIAL_ATI						0x40
#define ATI_OFF							0x80

/*	ProxSettings 1	*/
#define BAND							0x01
#define IQS_ERROR						0x02
#define CH0_1TX							0x04
#define HALT_CHARGE						0x10
#define TURBO_MODE						0x20
#define CHARGE_TRANSFER_SPEED			0x0C

/*	ProxSettings 2	*/
#define HALT							0x03
#define EVENT_MODE						0x04
#define TIMEOUT_DISABLE					0x08
#define ACF_DISABLE						0x10
#define FORCE_HALT						0x20
#define WDT_OFF							0x40
#define SOFT_RESET						0x80

/*	ProxSettings 3	*/
#define ACK_RESET						0x01
#define WHEEL1_DISABLE_DEFAULT			0x02
#define WHEEL2_DISABLE_DEFAULT			0x04
#define WHEEL_FILTER_DISABLE_DEFAULT	0x08
#define WHEEL_RESOLUTION_DEFAULT		0x70

/* PWM */
#define PWM_MODE						0xE0

#define PWM_LED0						0
#define PWM_LED1						1
#define PWM_LED2						2
#define PWM_LED3						3
#define PWM_LED4						4
#define PWM_LED5						5
#define PWM_LED6						6
#define PWM_LED7						7

#define DEFAULT_PWM						31
#define PWM_OFF							0

/* Channels */
#define	CH0      						0
#define	CH1      						1
#define	CH2      						2
#define	CH3      						3
#define	CH4      						4
#define	CH5      						5
#define	CH6      						6
#define	CH7      						7
#define	CH8      						8
#define	CH9      						9

#define MAX_CHANNELS					9
#define MAX_RESOLUTION					(uint16_t)2047


typedef enum ATI_Mode {
	ATI_Disabled = 0,
	Partial_ATI = 1,
	Semi_Partial_ATI = 2,
	Full_ATI = 3,
}ATI_Mode_e;

typedef enum Base_Value {
	Base_75 = 0,
	Base_100 = 1,
	Base_150 = 2,
	Base_200 = 3,
}Base_Value_e;

// power
typedef enum {IQS_OFF = 0, IQS_ON = !IQS_OFF}IQS_Power_t;

/*	System Flags	*/
typedef union
{
	struct
	{
		uint8_t Zoom				:1;
		uint8_t	Noise				:1;
		uint8_t ATI_Busy			:1;
		uint8_t	LP_Active			:1;
		uint8_t Is_CH0				:1;
		uint8_t	Proj_Mode			:1;
		uint8_t Filter_Halted		:1;
		uint8_t	Show_Reset			:1;
	};
	uint8_t SystemFlags;
}SystemFlags_t;

/*	Wheel Coordinates	*/
typedef union
{
	struct
	{
		uint8_t Wheel_Low;
		uint8_t	Wheel_High;
	};
	uint16_t 	Wheel;
}Wheel_t;

/*	Touch Bytes	*/
typedef union
{
	struct
	{
		uint8_t	Touch_Byte_1;
		uint8_t Touch_Byte_0;
	};
	uint16_t 	Touch_Byte;
}Touch_Byte_t;

/*	Multipliers	*/
typedef union
{
	struct
	{
		uint8_t Sens_Multi_CH		:4;
		uint8_t Comp_Multi_CH		:2;
		uint8_t	Base_Value_CH		:2;
	};
	uint8_t 	Multi_CH;
}Multi_CH_t;

/*	PWM	*/
typedef union
{
	struct
	{
		uint8_t Duty_Cycle	:5;
		uint8_t	Mode		:3;
	};
	uint8_t PWM;
}PWM_t;


typedef struct
{
	SystemFlags_t systemFlags;

	Wheel_t wheel1;
	Wheel_t wheel2;

	Touch_Byte_t touch_byte;

	Multi_CH_t multipliers_ch0;
	Multi_CH_t multipliers_ch1;
	Multi_CH_t multipliers_ch2;
	Multi_CH_t multipliers_ch3;
	Multi_CH_t multipliers_ch4;
	Multi_CH_t multipliers_ch5;
	Multi_CH_t multipliers_ch6;
	Multi_CH_t multipliers_ch7;
	Multi_CH_t multipliers_ch8;
	Multi_CH_t multipliers_ch9;


	/*	ProxSettings	*/
	union
	{
		struct
		{
			uint8_t Proj_Bias			:2;
			uint8_t	Cs_Size				:1;
			uint8_t Reseed				:1;
			uint8_t	Redo_ATI			:1;
			uint8_t Alt_ATI				:1;
			uint8_t	Partial_ATI			:1;
			uint8_t ATI_Off				:1;
		};
		uint8_t ProxSettings0;
	};

	union
	{
		struct
		{
			uint8_t Band					:1;
			uint8_t	Error					:1;
			uint8_t CH0_1Tx					:1;
			uint8_t	None					:1;
			uint8_t Halt_Charge				:1;
			uint8_t	Turbo_Mode				:1;
			uint8_t Charge_Transfer_Speed	:2;
		};
		uint8_t ProxSettings1;
	};

	union
	{
		struct
		{
			uint8_t Halt				:2;
			uint8_t	Event_Mode			:1;
			uint8_t Timout_Disable		:1;
			uint8_t	ACF_Disable			:1;
			uint8_t Force_Halt			:1;
			uint8_t	WDT_Off				:1;
			uint8_t Soft_Reset			:1;
		};
		uint8_t ProxSettings2;
	};

	union
	{
		struct
		{
			uint8_t ACK_Reset				:1;
			uint8_t	Wheel1_Disable			:1;
			uint8_t Wheel2_Disable			:1;
			uint8_t	Wheel_Filter_Disable	:1;
			uint8_t Wheel_Resolution		:3;
			uint8_t	PS3_None				:1;
		};
		uint8_t ProxSettings3;
	};

	union
	{
		struct
		{
			uint8_t Pass			:3;
			uint8_t	Up_Enable		:1;
			uint8_t UP				:3;
			uint8_t	PS4_None		:1;
		};
		uint8_t ProxSettings4;
	};

	uint8_t ProxSettings5;

	/*	Thresholds	*/
	uint8_t 	Prox_Threshold;
	uint8_t 	Touch_Threshold_1;
	uint8_t 	Touch_Threshold_2;
	uint8_t 	Touch_Threshold_3;
	uint8_t 	Touch_Threshold_4;
	uint8_t 	Touch_Threshold_5;
	uint8_t 	Touch_Threshold_6;
	uint8_t 	Touch_Threshold_7;
	uint8_t 	Touch_Threshold_8;
	uint8_t 	Touch_Threshold_9;

	/*	Timings	*/
	uint8_t 	Filter_Halt;
	uint8_t 	Power_Mode;
	uint8_t 	Timeout_Period;
	uint8_t 	CH0_ACF_Beta;
	uint8_t 	CH1_9_ACF_Beta;

	/*	Ati Targets	*/
	uint8_t 	ATI_Target_CH0;
	uint8_t 	ATI_Target_CH1_9;

	PWM_t pwm[8];

	uint8_t PWM_Limit;
	uint8_t PWM_Speed;

	uint8_t Active_Chan_0;
	uint8_t	Active_Chan_1;

	union
	{
		struct
		{
			uint8_t Burst			:1;
			uint8_t	Perm			:1;
			uint8_t DC				:1;
			uint8_t	Buz_None		:4;
			uint8_t Buz_Enable		:1;
		};
		uint8_t Buzzer;
	};

	/* IQS333 Power State */
	IQS_Power_t Power;
}IQS333_t;

/* Setup Registers */
// This setup is copied from the exported Header file

// System Flags
const static uint8_t System_flags[] = {
	SYSTEM_FLAGS_VAL
};

// Multipliers
const static uint8_t Multipliers[] = {
	MULTIPLIERS_CH0,
    MULTIPLIERS_CH1,
    MULTIPLIERS_CH2,
    MULTIPLIERS_CH3,
    MULTIPLIERS_CH4,
    MULTIPLIERS_CH5,
    MULTIPLIERS_CH6,
    MULTIPLIERS_CH7,
    MULTIPLIERS_CH8,
    MULTIPLIERS_CH9
};

// Compensation - note this is usually not written
const static uint8_t Compensation[] = {
	COMPENSATION_CH0,
    COMPENSATION_CH1,
    COMPENSATION_CH2,
    COMPENSATION_CH3,
    COMPENSATION_CH4,
    COMPENSATION_CH5,
    COMPENSATION_CH6,
    COMPENSATION_CH7,
    COMPENSATION_CH8,
    COMPENSATION_CH9
};

// ProxSettings
const static uint8_t Proxsettings[] = {
	PROXSETTINGS0_VAL,
    PROXSETTINGS1_VAL,
    PROXSETTINGS2_VAL&(~EVENT_MODE),  // disable event-mode for initial setup
    PROXSETTINGS3_VAL|ACK_RESET,
    PROXSETTINGS4_VAL,
    PROXSETTINGS5_VAL
};

// Thresholds
const static uint8_t Thresholds[] = {
    PROX_THRESHOLD,
    TOUCH_THRESHOLD_CH1,
    TOUCH_THRESHOLD_CH2,
    TOUCH_THRESHOLD_CH3,
    TOUCH_THRESHOLD_CH4,
    TOUCH_THRESHOLD_CH5,
    TOUCH_THRESHOLD_CH6,
    TOUCH_THRESHOLD_CH7,
    TOUCH_THRESHOLD_CH8,
    TOUCH_THRESHOLD_CH9
};

// Timings
const static uint8_t Timings[] = {
    FILTER_HALT,
    POWER_MODE,
    TIMEOUT_PERIOD,
    CH0_ACF_BETA,
    CH0_9_ACF_BET1
};

// ATI Targets
const static uint8_t Ati_Targets[] = {
	ATI_TARGET_CH0,
	ATI_TARGET_CH0_9
};

// PWM settings
const static uint8_t Pwm_Settings[] = {
	PWM_0,
	PWM_1,
	PWM_2,
	PWM_3,
	PWM_4,
	PWM_5,
	PWM_6,
	PWM_7
};

// PWM Limits and Speed
const static uint8_t Pwm_Limits[] = {
	PWM_LIMITS,
	PWM_SPEED
};

// Active Channels
const static uint8_t Active_Channels[] = {
	ACTIVE_CH0,
	ACTIVE_CH1
};

// Buzzer Output
const static uint8_t Buzzer[] = {
	BUZZER_VAL
};


void IQS333_Process(void);

#endif /* __IQS333_H */
