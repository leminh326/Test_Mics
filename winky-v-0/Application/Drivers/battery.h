/*
 * battery.h
 *
 *  Created on: 16 août 2019
 *      Author: Arnaud
 */

#ifndef SRC_BATTERY_H_
#define SRC_BATTERY_H_

#include <stdbool.h>

void Battery_iSetChange(bool active);
bool Battery_getPowerGood();
bool Battery_getChargeStatus();
float Battery_getVBat();
float Battery_getVBus();

#endif /* SRC_BATTERY_H_ */
