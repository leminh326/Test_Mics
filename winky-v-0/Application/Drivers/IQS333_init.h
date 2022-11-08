/*
* This file contains all the necessary settings for the IQS333 and this file can
* be changed from the GUI or edited here
* File:   IQS333_init.h
* Author: Azoteq
*/

#ifndef IQS333_INIT_H
#define IQS333_INIT_H

/* Used to switch to Projected mode */
#define SYSTEM_FLAGS_VAL					0x20

/*
* Change the Multipliers (BASE value) of each channel
* Please note that these values are affected by the Auto ATI routine and should
* only be written in the case of a specific setup.  Alternatively Auto ATI
* should be manually called after writing these settings.
* This is applicable for both Multipliers and Compensation.
*/
#define MULTIPLIERS_CH0                     0xB7
#define MULTIPLIERS_CH1                     0xA3
#define MULTIPLIERS_CH2                     0xA3
#define MULTIPLIERS_CH3                     0xA3
#define MULTIPLIERS_CH4                     0xA2
#define MULTIPLIERS_CH5                     0xA2
#define MULTIPLIERS_CH6                     0xA3
#define MULTIPLIERS_CH7                     0xA1
#define MULTIPLIERS_CH8                     0x00
#define MULTIPLIERS_CH9                     0x00


/*
* Change the Compensation for each channel
* Please note that these values are affected by the Auto ATI routine and should
* only be written in the case of a specific setup.  Alternatively Auto ATI
* should be manually called after writing these settings.
* This is applicable for both Multipliers and Compensation.
*/
#define COMPENSATION_CH0                    0xA4
#define COMPENSATION_CH1                    0x91
#define COMPENSATION_CH2                    0x89
#define COMPENSATION_CH3                    0x91
#define COMPENSATION_CH4                    0xA5
#define COMPENSATION_CH5                    0xA2
#define COMPENSATION_CH6                    0x8C
#define COMPENSATION_CH7                    0xA8
#define COMPENSATION_CH8                    0x00
#define COMPENSATION_CH9                    0x00


/* Change the Prox Settings or setup of the IQS333 */
#define PROXSETTINGS0_VAL                   0x05
#define PROXSETTINGS1_VAL                   0x20
#define PROXSETTINGS2_VAL                   0x04
#define PROXSETTINGS3_VAL                   0x10
#define PROXSETTINGS4_VAL                   0x07
#define PROXSETTINGS5_VAL                   0x7F


/* Change the Thresholds for each channel */
#define PROX_THRESHOLD						0x06
#define TOUCH_THRESHOLD_CH1					0x08
#define TOUCH_THRESHOLD_CH2					0x08
#define TOUCH_THRESHOLD_CH3					0x08
#define TOUCH_THRESHOLD_CH4					0x08
#define TOUCH_THRESHOLD_CH5					0x08
#define TOUCH_THRESHOLD_CH6					0x08
#define TOUCH_THRESHOLD_CH7					0x0A
#define TOUCH_THRESHOLD_CH8					0x0A
#define TOUCH_THRESHOLD_CH9					0x0A

/* Change the Timing settings */
#define FILTER_HALT							0x4F
#define POWER_MODE							0x08
#define TIMEOUT_PERIOD						0x10
#define CH0_ACF_BETA						0x02
#define CH0_9_ACF_BET1						0x02

/* Change ATI Target values  */
#define ATI_TARGET_CH0						0x64
#define ATI_TARGET_CH0_9					0x64

/* Change PWM settings  */
#define PWM_0								0x00
#define PWM_1								0x00
#define PWM_2								0x00
#define PWM_3								0x00
#define PWM_4								0x00
#define PWM_5								0x00
#define PWM_6								0x00
#define PWM_7								0x00

/* Change PWM limits and Speed */
#define PWM_LIMITS							0x0A
#define PWM_SPEED							0x00

/* Set Active Channels */
#define ACTIVE_CH0							0xFF
#define ACTIVE_CH1							0x00

/* Set Buzzer Output */
#define BUZZER_VAL							0x00

#endif	/* IQS333_INIT_H */

