/*
 * init.h
 *
 *  Created on: 22 jun. 2021
 *      Author: arnau
 */

#ifndef INIT_H_
#define INIT_H_

#include "msp.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>

#define High_Threshold 0x206C;
#define Low_Threshold  0x2060;

#define High_Threshold_offset 0x2100;
#define Low_Threshold_offset  0x2089;

void Clock_vInit(void);
void ini_GPIO_control_duty(void);
void init_timerA0(void);
void init_timerA1(void);
void ini_GPIO_map(void);
void init_interrupciones();
void init_SPI();
void init_ADC();

#endif /* INIT_H_ */
