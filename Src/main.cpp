/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 * This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * Copyright (c) 2018 STMicroelectronics International N.V.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
//#include <Robo.h>
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "usb_device.h"
#include "nrf24l01p.h"
#include "io_pin_stm32_hal.h"
#include "spi_stm32.h"
#include "proto/grSim_Commands.pb.h"
#include "proto/pb_common.h"
#include "proto/pb_decode.h"
#include <time_functions.h>
#include "usbd_cdc_if.h"
#include "commandline.h"
#include "usbd_cdc_if.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

osSemaphoreId conversionWait;

I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C2_Init(void);
static void MX_RTC_Init(void);
void StartDefaultTask(void const * argument);




extern CommandLine cmdline;




extern "C" void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);


/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */


/* USER CODE END 0 */



int main(void)
{

	/* USER CODE BEGIN 1 */



	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_TIM3_Init();
	MX_TIM2_Init();
	MX_TIM4_Init();
	MX_TIM1_Init();
	MX_SPI2_Init();
	MX_I2C2_Init();
	//  MX_RTC_Init();

	/* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	/* USER CODE END 2 */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the thread(s) */
	/* definition and creation of defaultTask */
	osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 1000);
	defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */


	/* Start scheduler */
	osKernelStart();// faz com que o infinite loop não funcione

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */


	while (1)
	{


	}
//	    HAL_Delay(1000);
//
//	    CDC_Transmit_FS(data_to_send, 20);


		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */


	/* USER CODE END 3 */


}
/** System Clock Configuration
 */




void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
	//  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);


}


/* I2C2 init function */
static void MX_I2C2_Init(void)
{

	hi2c2.Instance = I2C2;
	hi2c2.Init.ClockSpeed = 100000;
	hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* RTC init function */
static void MX_RTC_Init(void)
{

	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef DateToUpdate;

	/**Initialize RTC Only
	 */
	hrtc.Instance = RTC;
	hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
	hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
	if (HAL_RTC_Init(&hrtc) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initialize RTC and set the Time and Date
	 */
	if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) != 0x32F2){
		sTime.Hours = 0x1;
		sTime.Minutes = 0x0;
		sTime.Seconds = 0x0;

		if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
		DateToUpdate.Month = RTC_MONTH_JANUARY;
		DateToUpdate.Date = 0x1;
		DateToUpdate.Year = 0x0;

		if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR1,0x32F2);
	}

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4; // para Freq< 10 Mhz
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi2) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* TIM1 init function */
/* Timer utilizado para PWM */
static void MX_TIM1_Init(void)
{

	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 71;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 999;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.Pulse = 0;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_TIM_MspPostInit(&htim1);

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

	TIM_Encoder_InitTypeDef sConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 0xffff;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 71;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 999;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.Pulse = 0;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_TIM_MspPostInit(&htim3);

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

	TIM_Encoder_InitTypeDef sConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 0xffff;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, M1_MBH_Pin|M1_MAH_Pin|SPI2_CE_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, SPI2_NSS_Pin|M0_MBH_Pin|M0_MAH_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin : LED_Pin */
	GPIO_InitStruct.Pin = LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : M1_MBH_Pin M1_MAH_Pin SPI2_CE_Pin */
	GPIO_InitStruct.Pin = M1_MBH_Pin|M1_MAH_Pin|SPI2_CE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : SPI2_NSS_Pin M0_MBH_Pin M0_MAH_Pin */
	GPIO_InitStruct.Pin = SPI2_NSS_Pin|M0_MBH_Pin|M0_MAH_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : SPI2_IRQN_Pin */
	GPIO_InitStruct.Pin = SPI2_IRQN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(SPI2_IRQN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

// FUNÇÕES PARA PROTOBUFF

bool pb_circularbuffer_read(pb_istream_t *stream, pb_byte_t *buf, size_t count){
	bool result=false;
	CircularBuffer<uint8_t> *circularbuffer=(CircularBuffer<uint8_t> *)(stream->state);
	if(circularbuffer->Out(buf, count)==count){
		result=true;
	}
	stream->bytes_left=circularbuffer->Ocupied();
	if(stream->bytes_left==0){
		stream->bytes_left=1; //keep it alive for pb_decode(...) function
	}
	return result;
}

pb_istream_t pb_istream_from_circularbuffer(CircularBuffer<uint8_t> *circularbuffer)
{
	pb_istream_t stream;
	/* Cast away the const from buf without a compiler error.  We are
	 * careful to use it only in a const manner in the callbacks.
	 */
	union {
		void *state;
		const void *c_state;
	} state;
	stream.callback = &pb_circularbuffer_read;
	state.c_state = circularbuffer;
	stream.state = state.state;
	stream.bytes_left = 1;
	if(stream.bytes_left==0){
		stream.bytes_left=1; //keep it alive for pb_decode(...) function
	}
	stream.errmsg = NULL;
	return stream;
}

// Parâmetros e constantes:

uint8_t id=10;  //ID
uint8_t channel=92;
uint64_t address=0xE7E7E7E700;
uint32_t last_packet_ms = 0;
bool controlbit=false;
grSim_Robot_Command robotcmd;
grSim_Robot_Command robotcmd_test;
//static float velocidade_des0 =0;
//static float velocidade_des1 =0;//.188495;


void processPacket(){
	//	robo.set_speed(robotcmd.veltangent, robotcmd.velnormal, robotcmd.velangular);
//	velocidade_des0=robotcmd.veltangent;
//	velocidade_des1=robotcmd.velnormal;
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	//	if(robotcmd.kickspeedx!=0)
	//		robo.ChuteBaixo(robotcmd.kickspeedx);
	//	if(robotcmd.kickspeedz!=0)
	//		robo.HighKick(robotcmd.kickspeedz);
	//	if(robotcmd.spinner)
	//		robo.drible->Set_Vel(660);
	//	else
	//		robo.drible->Set_Vel(0);
}

void interruptReceive(NRF24L01P *_nrf24){
	bool status=0;
	if(_nrf24->RxSize()){
		_nrf24->StartRX_ESB(channel, address + id, 32, 1);
		uint8_t rxsize=_nrf24->RxSize();
		if(rxsize>32) rxsize=32;
		uint8_t buffer[32];
		_nrf24->RxData(buffer, rxsize);
		//usb_device_class_cdc_vcp.SendData((uint8_t*)buffer, rxsize);
		pb_istream_t istream=pb_istream_from_buffer(buffer, rxsize);
		status=pb_decode(&istream, grSim_Robot_Command_fields, &robotcmd);
		last_packet_ms = GetLocalTime();
		controlbit = true;
	}
	if(status){
		if(robotcmd.id==id){
			processPacket();
		}
	}
	if((GetLocalTime()-last_packet_ms)>100){
		controlbit = false;
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,GPIO_PIN_RESET);
	}
}


extern "C" int8_t usb_receive(uint8_t* buffer, uint32_t size){
	if(size) {
		CircularBuffer<uint8_t> buf(buffer,size);
		buf.In(buffer,size);
		cmdline.In(buf);
	}

}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
	/* init code for USB_DEVICE */
	MX_USB_DEVICE_Init();

	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	osDelay(2000);

	HAL_GPIO_WritePin(M0_MAH_GPIO_Port, M0_MAH_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(M0_MBH_GPIO_Port, M0_MBH_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(M1_MAH_GPIO_Port, M1_MAH_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(M1_MBH_GPIO_Port, M1_MBH_Pin, GPIO_PIN_SET);

	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);

	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

	HAL_TIM_Base_Start_IT(&htim1);


	int a=0;

//	Declarações--
	controlbit = true;

	IO_Pin_STM32 nrf_nss(IO_Pin::IO_Pin_Mode_OUT, SPI2_NSS_GPIO_Port, SPI2_NSS_Pin);
	IO_Pin_STM32 nrf_ce(IO_Pin::IO_Pin_Mode_OUT,SPI2_CE_GPIO_Port,SPI2_CE_Pin);
	IO_Pin_STM32 nrf_irqn(IO_Pin::IO_Pin_Mode_IN, SPI2_IRQN_GPIO_Port,SPI2_IRQN_Pin);
	SPI_STM32 spinrf(hspi2, nrf_nss);
	NRF24L01P nrf(spinrf,nrf_nss,nrf_ce,nrf_irqn);
	//  Robo robo(nrf,id);
	a=nrf.SelfTest();

	nrf.Init();
	nrf.Config(); // configurações : DPL OK, ACK OK, ACK_NOT OK, POWER UP OK
	nrf.StartRX_ESB(channel, address + id, 32, 1);
	nrf.TxPackage_ESB(channel, address + id, 0,(uint8_t*) "TESTE", 5);
	while(nrf.Busy()){
		nrf.InterruptCallback();
	}
	nrf.StartRX_ESB(channel, address + id, 32, 1);
	//////////////////////////////////////////




//	TESTE

	  a=nrf.SelfTest();
	  nrf.TxReady();
	  a=nrf.SelfTest();


	for(;;)/*Não entendi direito*/
	{

		uint32_t size=0;
		uint8_t buffer[32];
			do{
				size=cmdline.Out(buffer, 32);
				if(size){
//					usb_device_class_cdc_vcp.SendData(buffer, size);
					CDC_Transmit_FS(buffer, size);

				}
			} while(size==32);

		//    osDelay(500);
		//  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		nrf.InterruptCallback();
		if(nrf.RxSize()){
			interruptReceive(&nrf);
			//robo.interruptAckPayload();
			last_packet_ms = GetLocalTime();
			controlbit = true;
		}
		if((GetLocalTime()-last_packet_ms)>100){
			controlbit = false;
		}
		/////
	}
	/* USER CODE END 5 */
}

int speed_usb_0;
int speed_usb_1;
int desired_speed0;
int desired_speed1;
int desired_speed;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	static float p;
	float velocidade_enviada0;
	float velocidade_enviada1;
	static int16_t m0p=0, m1p=0,old_c0=0,old_c1=0;
	static float speed_m0= 0.0f;
	static float speed_m1= 0.0f;
	bool controlbit = true;
//uint8_t data_to_send[20] = "Damogran Labs";
uint8_t buffer_to_send[64];
p=speed_m0*1000;
speed_usb_0=(int)p;
speed_usb_1=speed_m1*1000;
velocidade_enviada0=(float)desired_speed0;
velocidade_enviada0=(float)velocidade_enviada0/1000;

velocidade_enviada1=(float)desired_speed1;
velocidade_enviada1=(float)velocidade_enviada1/1000;
//	sprintf((char*)buffer_to_send,"%i | %i \n",a,b);
//
//
//    CDC_Transmit_FS(buffer_to_send, strlen((const char*)buffer_to_send));

//CDC_Transmit_FS(data_to_send, 20);



	//valor aproximado do raio da roda, em metros
	static const float R_roda = 0.030;
	static const float PI = 3.1415926;
	//valor do tempo em segundos
	static const float Tempo = 0.013;
	//número de divisões do encoder
	static const float ENC_DIV = 20;
	//para cada 75 giros do motor, a roda gira 1 vez
	static const float RED = 75;
	static float CALCULO = 0.0f;
	if(CALCULO==0.0f) CALCULO=(2*PI*R_roda/(Tempo*ENC_DIV*RED));
//	static float speed_m0= 0.0f;
//	static float speed_m1= 0.0f;
	static float e_m0 =0.0f,e_m1=0.0f;
	static float pwm_m0 =0.0f,pwm_m1=0.0f;
	//controle
	static int16_t hist_m0p[13], hist_m1p[13];
	static float pwmh[13];
	static float pwmh1[13];
	static float kp = 100.0f;
	static float ki=190.00f;
	static float kd = 00.0f;
	float ierror_m0=0.0f;
	float ierror_m1=0.0f;
	static float last_error_m0[20];
	static float last_error_m1[20];
	float derror_m0=0.0f;
	float derror_m1=0.0f;
	//SPI
	static float dado;
	static float velocidade_des0 =0;
		velocidade_des0=velocidade_enviada0;//.188495;

	static float velocidade_des1 =0;
	velocidade_des1=velocidade_enviada1;//.188495;


	static int i;
	if(htim==&htim1){
		static uint8_t i=0;
		//motor 0
		//	estava assim antes	m0p=(int16_t)(htim2.Instance->CNT), acredito que esteja trocado;
		m0p=(int16_t)(htim4.Instance->CNT);

		//motor 1
		//	estava assim antes	m2p=(int16_t)(htim4.Instance->CNT), acredito que esteja trocado;
		m1p=(int16_t)(htim2.Instance->CNT);

		old_c0 = hist_m0p[i%13];
		old_c1 = hist_m1p[i%13];
		hist_m0p[i%13] = m0p;
		hist_m1p[i%13] = m1p;

	if( controlbit){
//		if(i% 13==1){
				if(pwm_m0>=0.0f){
					HAL_GPIO_WritePin(M0_MBH_GPIO_Port, M0_MBH_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(M0_MAH_GPIO_Port, M0_MAH_Pin, GPIO_PIN_RESET);
				} else {
					HAL_GPIO_WritePin(M0_MAH_GPIO_Port, M0_MAH_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(M0_MBH_GPIO_Port, M0_MBH_Pin, GPIO_PIN_RESET);

				}

				if(pwm_m1>=0.0f){
					HAL_GPIO_WritePin(M1_MBH_GPIO_Port, M1_MBH_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(M1_MAH_GPIO_Port, M1_MAH_Pin, GPIO_PIN_RESET);
				} else {
					HAL_GPIO_WritePin(M1_MAH_GPIO_Port, M1_MAH_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(M1_MBH_GPIO_Port, M1_MBH_Pin, GPIO_PIN_RESET);
				}
		}
		} else {
			HAL_GPIO_WritePin(M0_MAH_GPIO_Port, M0_MAH_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(M0_MBH_GPIO_Port, M0_MBH_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(M1_MAH_GPIO_Port, M1_MAH_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(M1_MBH_GPIO_Port, M1_MBH_Pin, GPIO_PIN_SET);
		}

		//primeiro testa e depois incrementa
//		if( i++% 13==0){
//			i=1;

			speed_m0= CALCULO*((int16_t)(m0p - old_c0));
			speed_m1= CALCULO*((int16_t)(m1p - old_c1));

			//motor 0
			e_m0= velocidade_des0-speed_m0;
			e_m1= velocidade_des1-speed_m1;


			derror_m0=e_m0-last_error_m0[i%13];
			derror_m1=e_m1-last_error_m1[i%13];
			last_error_m0[i%13]=e_m0;
			last_error_m1[i%13]=e_m1;
			for(int j = 0; j < 13; j++){
				ierror_m0 += last_error_m0[j];
				ierror_m1 += last_error_m1[j];
			}
			if(ierror_m0 > 1000) ierror_m0 = 1000;
			if(ierror_m0 < -1000) ierror_m0 = -1000;
			if(ierror_m1 > 1000) ierror_m1 = 1000;
			if(ierror_m1 < -1000) ierror_m1 = -1000;

			pwm_m0 = 300.0f*velocidade_des0 + kp*e_m0 + ki*ierror_m0 + kd*derror_m0;
			pwm_m1 = 300.0f*velocidade_des1 + kp*e_m1 + ki*ierror_m1 + kd*derror_m1;
			pwm_m1=-pwm_m1;

			pwmh[i%13]=speed_m0;
			pwmh1[i%13]=speed_m1;

			if(pwm_m0>1000.0f)
				pwm_m0=1000.0f;
			if(pwm_m0<-1000.0f)
				pwm_m0=-1000.0f;
			if(pwm_m0>=0.0f){
				htim1.Instance->CCR1=0;
				htim1.Instance->CCR2=(uint16_t)pwm_m0;
			} else {
				htim1.Instance->CCR1=(uint16_t)(-pwm_m0);
				htim1.Instance->CCR2=0;
			}

			if(pwm_m1>1000.0f)
				pwm_m1=1000.0f;
			if(pwm_m1<-1000.0f)
				pwm_m1=-1000.0f;
			if(pwm_m1>=0.0f){
				htim3.Instance->CCR3=0;
				htim3.Instance->CCR4=(uint16_t)pwm_m1;
			} else {
				htim3.Instance->CCR3=(uint16_t)(-pwm_m1);
				htim3.Instance->CCR4=0;
			}

			//old_c0=m0p;
			//old_c1=m1p;
			i=(i+1)%13;
		}

//	}


/**
 * @brief  This function is executed in case of error occurrence.
 * @param  Noneb
 * @retval None
 */
void _Error_Handler(char * file, int line)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while(1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
