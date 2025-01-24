/*
 * waypointDemo.h
 *
 *  Created on: 23 nov 2024
 *      Author: javim_1oc1nxl
 */

#ifndef INC_WAYPOINTDEMO_H_
#define INC_WAYPOINTDEMO_H_

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "stm32h7xx_hal.h"
#include "positionMessage.h"
#include "main.h"


void initServoTim(TIM_HandleTypeDef HTimServo);
void initESCTim(TIM_HandleTypeDef HTimESC);
void setEscSpeed(uint16_t);
void setServoAngle(uint16_t);
void stopCar();
void testingServoLimits();
void testingESCLimits(void);
void waypointPathHardcode();
void waypointPathEncoder(float position,uint16_t x, uint16_t y, uint16_t a);
extern void Error_Handler(void);


#endif /* INC_WAYPOINTDEMO_H_ */
