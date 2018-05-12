/*
 * io_pin_stm32_hal.h
 *
 *  Created on: 10 de mai de 2018
 *      Author: João
 */

#ifndef IO_PIN_STM32_HAL_H_
#define IO_PIN_STM32_HAL_H_

#include "io_pin.h"
#include "stm32f103xb.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"

class IO_Pin_STM32:public IO_Pin
{
public:
	IO_Pin_STM32(IO_Pin_Mode mode , GPIO_TypeDef * GPIOx , uint16_t GPIO_Pin);// uint32_t pupd = GPIO_NOPULL, uint32_t GPIO_OType = GPIO_MODE_OUTPUT_PP , uint8_t af=0 , uint32_t EXTITrigger = GPIO_MODE_IT_RISING);
	//void Init (IO_Pin_Mode mode, GPIO_TypeDef *GPIOx, GPIO_InitTypeDef *GPIO_InitStructure, uint8_t af, uint32_t EXTITrigger_Mode) ;
	void Set();
	void Reset();
	void Write(uint8_t value);
	uint8_t Read();
	uint8_t GetIRQChannel();
	void Interrupt(uint8_t newstate);
protected:
	GPIO_TypeDef *_gpio;
	uint16_t _pins;
	uint8_t _NVIC_IRQChannel;
};


#endif /* IO_PIN_STM32_HAL_H_ */
