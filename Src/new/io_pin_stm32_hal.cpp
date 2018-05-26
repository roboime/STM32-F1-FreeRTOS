#include "io_pin_stm32_hal.h"




IO_Pin_STM32::IO_Pin_STM32(
		IO_Pin_Mode mode, GPIO_TypeDef * GPIOx, uint16_t GPIO_Pin
		):// uint32_t pupd = GPIO_NOPULL, uint32_t GPIO_OType=GPIO_MODE_OUTPUT_PP , uint8_t af=0 , uint32_t EXTITrigger_Mode= GPIO_MODE_IT_RISING):
	IO_Pin(mode), _gpio(GPIOx), _pins(GPIO_Pin),
//		IO_Pin(IO_Pin_Mode_IN),
	_NVIC_IRQChannel(0){}
//	GPIO_InitTypeDef GPIO_InitStruct;
//
//	GPIO_InitStruct.Pin=GPIO_Pin;
//	GPIO_InitStruct.Mode=GPIO_MODE_INPUT;
//	GPIO_InitStruct.Speed=GPIO_SPEED_FREQ_HIGH;
//	GPIO_InitStruct.Pull=pupd;
//	switch(_mode){
//	case IO_Pin_Mode_IN:
//		GPIO_InitStruct.Mode=GPIO_MODE_INPUT;
//		//GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;
//		break;
//	case IO_Pin_Mode_OUT:
//		GPIO_InitStruct.Mode=GPIO_MODE_OUTPUT_PP;
////		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
////		GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
//		break;
//	case IO_Pin_Mode_IN_OUT:
//		GPIO_InitStruct.GPIO_MODE=GPIO_MODE_OUTPUT_OD;
////			GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
////			GPIO_InitStructure.GPIO_OType=GPIO_OType_OD;
//		break;
//	case IO_Pin_Mode_ANALOG:
//		GPIO_InitStruct.GPIO_MODE=GPIO_MODE_ANALOG;
////			GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AN;
//		break;
//	case IO_Pin_Mode_SPECIAL:
//		GPIO_InitStruct.GPIO_MODE=GPIO_MODE_AF_PP;  // não consigo modo AF sem setar PP,OD ou input
////			GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
//		break;

//	GPIO_InitStructure.GPIO_Pin=pins;
//	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;
//	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
//	GPIO_InitStructure.GPIO_OType=GPIO_OType;
//	GPIO_InitStructure.GPIO_PuPd=pupd;
//
//	switch(_mode){
//	case IO_Pin_Mode_IN:
//		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;
//		break;
//	case IO_Pin_Mode_OUT:
//		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
//		GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
//		break;
//	case IO_Pin_Mode_IN_OUT:
//		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
//		GPIO_InitStructure.GPIO_OType=GPIO_OType_OD;
//		break;
//	case IO_Pin_Mode_ANALOG:
//		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AN;
//		break;
//	case IO_Pin_Mode_SPECIAL:
//		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
//		break;


//HAL_GPIO_Init(GPIOx,&GPIO_InitStruct);
////	Init(mode, GPIOx, &GPIO_InitStructure, af, EXTITrigger, EXTIMode);
//}



//void IO_Pin_STM32::Init(IO_Pin_Mode mode, GPIO_TypeDef *GPIOx, GPIO_InitTypeDef *GPIO_InitStructure, uint8_t af, uint32_t EXTITrigger_Mode)
//{
////	_mode=mode;
////	_gpio=GPIOx;
////	_pins=GPIO_InitStructure->GPIO_Pin;
////	if(_gpio==GPIOA){
////	__HAL_RCC_GPIOA_CLK_ENABLE();
//////	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
////	} else if(_gpio==GPIOB){
////	__HAL_RCC_GPIOB_CLK_ENABLE();
////		//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
////	} else if(_gpio==GPIOC){
////	__HAL_RCC_GPIOC_CLK_ENABLE();
////		//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
////	} else if(_gpio==GPIOD){
////	__HAL_RCC_GPIOD_CLK_ENABLE();
////		//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
////	} else {
////		return;
////	}
////	HAL_GPIO_Init(GPIOx,GPIO_InitStructure);
////	//GPIO_Init(GPIOx, GPIO_InitStructure);
//////	if(_mode==IO_Pin_Mode_SPECIAL){
//////		uint8_t i;
//////		for(i=0;i<16;i++){
//////			if((1<<i) & _pins){
//////
//////				GPIO_PinAFConfig(GPIOx, i, af); // NÃO MUDEI
//////			}
//////	}
//////	}
////	if(GPIO_InitStructure->GPIO_Mode==GPIO_MODE_OUTPUT_OD ){
////		HAL_GPIO_WritePin(_gpio, _pins);
////	}
////	if(EXTITrigger){
////		EXTI_InitTypeDef EXTI_InitStructure;
////
////		__HAL_RCC_SYSCFG_CLK_ENABLE();
////		//RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
////
//////		EXTI_InitStructure.EXTI_LineCmd=ENABLE;
//////		EXTI_InitStructure.EXTI_Mode=EXTI_Mode;
//////		EXTI_InitStructure.EXTI_Trigger=EXTITrigger;
//////		EXTI_InitStructure.EXTI_Line=_pins;
//////		EXTI_Init(&EXTI_InitStructure);
////
////
////		uint8_t i;
////		for(i=0;i<16;i++){
////			if((1<<i) & _pins){
////				uint8_t gpiosource=(uint8_t)(((uint32_t)_gpio-(uint32_t)GPIOA)/0x0400);
////				//__HAL_GPIO_EXTI_GENERATE_SWIT(); configuro interrupção de software na linha, mas não consigo escolher a parta
////				__HAL_GPIO_EXTI_CLEAR_IT(i<<1);
////				//SYSCFG_EXTILineConfig(gpiosource, i); -- não encontrei equivalente
////				//EXTI_ClearITPendingBit(1<<i);
////				switch(i){
////				case 0:
////					_NVIC_IRQChannel=EXTI0_IRQn;
////					break;
////				case 1:
////					_NVIC_IRQChannel=EXTI1_IRQn;
////					break;
////				case 2:
////					_NVIC_IRQChannel=EXTI2_IRQn;
////					break;
////				case 3:
////					_NVIC_IRQChannel=EXTI3_IRQn;
////					break;
////				case 4:
////					_NVIC_IRQChannel=EXTI4_IRQn;
////					break;
////				case 5: case 6: case 7: case 8: case 9:
////					_NVIC_IRQChannel=EXTI9_5_IRQn;
////					break;
////				case 10: case 11: case 12: case 13: case 14: case 15:
////					_NVIC_IRQChannel=EXTI15_10_IRQn;
////					break;
////				}
////			}
////		}
////	}
//}

void IO_Pin_STM32::Set()
{
	if(_mode==IO_Pin_Mode_OUT || _mode==IO_Pin_Mode_IN_OUT){
		HAL_GPIO_WritePin(_gpio,_pins, GPIO_PIN_SET);
		//GPIO_SetBits(_gpio, _pins);
	} else if(_mode==IO_Pin_Mode_IN){
		//habilita o pino em pul-up

		uint32_t pins=_pins;
		uint8_t pos=0;
		while(pins){
			if(pins & 1){
				_gpio->ODR|=(1<<(pos));
			}
			pos++;
			pins>>=1;
		}
	}
}

void IO_Pin_STM32::Reset()
{
	if(_mode==IO_Pin_Mode_OUT || _mode==IO_Pin_Mode_IN_OUT){
		HAL_GPIO_WritePin(_gpio,_pins, GPIO_PIN_RESET);
		//GPIO_ResetBits(_gpio, _pins);
	} else if(_mode==IO_Pin_Mode_IN){
		// habilita o pino em pull down
		uint32_t pins=_pins;
		uint8_t pos=0;
		while(pins){
			if(pins & 1){
				_gpio->ODR&=~(1<<(pos));
			}
			pos++;
			pins>>=1;
		}
	}
}

void IO_Pin_STM32::Write(uint8_t value)
{
	if(value){
		Set();
	} else {
		Reset();
	}
}

uint8_t IO_Pin_STM32::Read()
{
	if(_mode==IO_Pin_Mode_OUT){
		return HAL_GPIO_ReadPin(_gpio,_pins); //GPIO_ReadOutputDataBit(_gpio, _pins);
	} else {
		return HAL_GPIO_ReadPin(_gpio,_pins); //GPIO_ReadInputDataBit(_gpio, _pins);
	}
}

uint8_t IO_Pin_STM32::GetIRQChannel() {
	return _NVIC_IRQChannel;
}

void IO_Pin_STM32::Interrupt(uint8_t newstate) {
	if(_NVIC_IRQChannel){
		if(newstate){
			NVIC_EnableIRQ((IRQn_Type)_NVIC_IRQChannel);
		} else {
			NVIC_DisableIRQ((IRQn_Type)_NVIC_IRQChannel);
		}
	}
}
