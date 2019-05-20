/*
 * led.h
 *
 *  Created on: Apr 6, 2019
 *      Author: ialex
 */

#ifndef LED_H_
#define LED_H_

#include "stm32f10x.h"
#include "stm32f10x_conf.h"

void init_led();

void gpio_toggle(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);


#endif /* LED_H_ */
