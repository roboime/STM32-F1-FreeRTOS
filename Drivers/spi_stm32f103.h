/*
 * spi_stm32f103.h
 *
 *  Created on: 3 de mai de 2018
 *      Author: João
 */

#ifndef SPI_STM32F103_H_
#define SPI_STM32F103_H_

#include<stm32f1xx_hal_spi.h>
#include "stm32f1xx_hal.h"

class SPI_STM32
{
public:
	SPI_STM32(SPI_TypeDef hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
		SPI_STM32(const SPI_STM32 &spi_stm32, GPIO_TypeDef &GPIOx, uint16_t GPIO_Pin);
		uint8_t WriteByte(uint8_t byte);
		uint8_t WriteBuffer(uint8_t *buffer, uint16_t size);
		void Assert();
		void Release();
		uint8_t SelfTest();

	protected:
		uint8_t Timeout();
		static const uint32_t SPI_TIMEOUT;
		SPI_HandleTypeDef _hspi;
		GPIO_InitTypeDef *_GPIOx;
		uint16_t _GPIO_Pin;
};

#endif /* SPI_STM32F103_H_ */
