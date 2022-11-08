/*
 * motorhead.h
 *
 *  Created on: 17 juil. 2019
 *      Author: Arnaud
 */

#ifndef MOTORHEAD_H_
#define MOTORHEAD_H_

#include "drv883x.h"
#include "rencoder.h"
#include <stdbool.h>

#define ENCODER_360_COUNT 59 		//TODO to beajusted with final product

typedef struct {
	drv883x_MotorController motorController;
	REncoder				encoder;
	int32_t					encoderValue;
	bool					useEncoder;
} MotorHead;

void MotorHead_init(MotorHead * motorhead, bool useEncoder);
void MotorHead_forward(MotorHead * motorhead, float speed);
void MotorHead_backward(MotorHead * motorhead, float speed);
void MotorHead_stop(MotorHead * motorhead);
void MotorHead_sleep(MotorHead * motorhead);
int32_t MotorHead_getEncoderValue(MotorHead * motorhead);
void MotorHead_resetEncoderValue(MotorHead * motorhead);

#endif /* MOTORHEAD_H_ */
