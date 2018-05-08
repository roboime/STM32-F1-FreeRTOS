#include"stm32f1xx_hal_spi.h"
#include "stm32f1xx_hal.h"
#include "spi_stm32f103.h"
#include "stm32f103xb.h"

const uint32_t SPI_STM32::SPI_TIMEOUT=0x4000;

SPI_STM32::SPI_STM32(SPI_HandleTypeDef hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin): _hspi(hspi), _GPIOx(GPIOx), _GPIO_Pin(GPIO_Pin) //construtor
{
	if(_hspi==SPI1){
		_hspi.Instance = SPI1;
		} else if(_hspi==SPI2){
		_hspi.Instance = SPI2;
		} else if(_hspi==SPI3){
		_hspi.Instance = SPI3;
		} else if(_hspi==SPI4){
		_hspi.Instance = SPI4;
		} else if(_hspi==SPI5){
		_hspi.Instance = SPI5;
		} else if(_hspi==SPI6){
		_hspi.Instance = SPI6;
		} else {
			return;
		}
	_hspi.Init.Mode = SPI_MODE_MASTER;
	_hspi.Init.Direction = SPI_DIRECTION_2LINES;
	_hspi.Init.DataSize = SPI_DATASIZE_8BIT;
	_hspi.Init.CLKPolarity = SPI_POLARITY_LOW;
	_hspi.Init.CLKPhase = SPI_PHASE_1EDGE;
	_hspi.Init.NSS = SPI_NSS_SOFT;
	_hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
	_hspi.Init.FirstBit = SPI_FIRSTBIT_MSB;
	_hspi.Init.TIMode = SPI_TIMODE_DISABLE;
	_hspi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	_hspi.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&_hspi) != HAL_OK)
	{
	  _Error_Handler(__FILE__, __LINE__);
	}
	 __HAL_SPI_ENABLE(&_hspi);
	 Release();
}

SPI_STM32::SPI_STM32(const SPI_STM32& spi_stm32, GPIO_TypeDef& GPIOx, uint16_t GPIO_Pin): // construtor
	SPI_STM32(spi_stm32)
{
	_GPIOx=&GPIOx;
	_GPIO_Pin=GPIO_Pin;
}

uint8_t SPI_STM32::WriteByte(uint8_t byte) {
	return WriteBuffer(&byte, 1);
}

void SPI_STM32::Assert() {
	HAL_GPIO_WritePin(_GPIOx,_GPIO_Pin,DISABLE);
	//_SS_PIN->Reset();
}

void SPI_STM32::Release() {
	HAL_GPIO_WritePin(_GPIOx,_GPIO_Pin,ENABLE);
	//_SS_PIN->Set();
}

uint8_t SPI_STM32::WriteBuffer(uint8_t* buffer, uint16_t size) {
	uint32_t resp=0;
	uint32_t timeout_t=0;
	while(size--){
		timeout_t=SPI_TIMEOUT;
		while(__HAL_SPI_GET_FLAG(_hspi,SPI_FLAG_TXE) == RESET){
			if(timeout_t--==0) return Timeout();
		}

		SPI_I2S_SendData(_spi, *buffer++);
		_hspi.Instance->DR=*buffer++;
		timeout_t=SPI_TIMEOUT;
		while (__HAL_SPI_GET_FLAG(_hspi,SPI_FLAG_BSY) == SET){
			if(timeout_t--==0) return Timeout();
		}

		timeout_t=SPI_TIMEOUT;
		while (__HAL_SPI_GET_FLAG(_hspi,SPI_FLAG_RXNE) == RESET){
			if(timeout_t--==0) return Timeout();
		}
		resp += _hspi.Instance->DR ;
	}
//	for(timeout_t=0;timeout_t<200;timeout_t++);
	return resp;
}

uint8_t SPI_STM32::SelfTest() {
	return 0;
}

uint8_t SPI_STM32::Timeout() {
	return -1;
}

