/*
 * This file is part of the µOS++ distribution.
 *   (https://github.com/micro-os-plus)
 * Copyright (c) 2014 Liviu Ionescu.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"

// ----------------------------------------------------------------------------
//
// Standalone STM32F1 led blink sample (trace via DEBUG).
//
// In debug configurations, demonstrate how to print a greeting message
// on the trace device. In release configurations the message is
// simply discarded.
//
// Then demonstrates how to blink a led with 1 Hz, using a
// continuous loop and SysTick delays.
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the DEBUG output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//
// The external clock frequency is specified as a preprocessor definition
// passed to the compiler via a command line option (see the 'C/C++ General' ->
// 'Paths and Symbols' -> the 'Symbols' tab, if you want to change it).
// The value selected during project creation was HSE_VALUE=8000000.
//
// Note: The default clock settings take the user defined HSE_VALUE and try
// to reach the maximum possible system clock. For the default 8 MHz input
// the result is guaranteed, but for other values it might not be possible,
// so please adjust the PLL settings in system/src/cmsis/system_stm32f10x.c
//

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

#include "stm32f10x.h"
#include "stm32f10x_conf.h"

#include "usart.h"
#include "math.h"

// Definitii
EXTI_InitTypeDef EXTI_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;
uint8_t interupt_flag = 0;
uint8_t send_data_flag = 0;

uint16_t CCR3_Val = 0;
uint16_t PrescalerValue = 0;

GPIO_InitTypeDef GPIO_InitStructure;
volatile uint32_t TimingDelay;

TIM_OCInitTypeDef  TIM_OCInitStructure;
void delay(uint32_t delay_count)
{
	while (delay_count) delay_count--;
}

void interupt_Config()
{
   /* Enable GPIOC clock */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

   /* Configureaza pinul  Pc13 ca pin floating */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
   GPIO_Init(GPIOC, &GPIO_InitStructure);

   /* Connect EXTI13 Line to PC.13 pin */

   //Face legatura dintre configurarea GPIO si linia de intrerupere.
   GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource13);

   /* Configurarea linii 13 -  EXTI13 */
   EXTI_InitStructure.EXTI_Line = EXTI_Line13; // Linia cu care lucram de la 0 la 15
   EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
					// Se genereaza o intererupere
					//Se poate pune EXTI_Mode_Event pentru a trezi clock-ul fara a genera o intrerupere

   EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
   EXTI_InitStructure.EXTI_LineCmd = ENABLE;
   EXTI_Init(&EXTI_InitStructure);

   /* Enable and set EXTI15_10 Interrupt to the lowest priority */
   NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

   NVIC_Init(&NVIC_InitStructure);
}

void EXTI15_10_IRQHandler(void)
{
   if(EXTI_GetITStatus(EXTI_Line13) != RESET)
   {
	   interupt_flag = 1;
	//trace_printf("bota\n");
	   EXTI_ClearITPendingBit(EXTI_Line13);
   }
}

void init_timer()
{
	SysTick_Config(SystemCoreClock / 800);
}

void SysTick_Handler(void)
{
	if (TimingDelay != 0x00)
	{
		TimingDelay--;
	}
}

void Delay(__IO uint32_t nTime)
{
	 TimingDelay = nTime;
	 while(TimingDelay != 0);
}

void SetSysClockTo72(void)
{
	ErrorStatus HSEStartUpStatus;
    /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration -----------------------------*/
    /* RCC system reset(for debug purpose) */
    RCC_DeInit();

    /* Enable HSE */
    RCC_HSEConfig( RCC_HSE_ON);

    /* Wait till HSE is ready */
    HSEStartUpStatus = RCC_WaitForHSEStartUp();

    if (HSEStartUpStatus == SUCCESS)
    {
        /* Enable Prefetch Buffer */
    	//FLASH_PrefetchBufferCmd( FLASH_PrefetchBuffer_Enable);

        /* Flash 2 wait state */
        //FLASH_SetLatency( FLASH_Latency_2);

        /* HCLK = SYSCLK */
        RCC_HCLKConfig( RCC_SYSCLK_Div1);

        /* PCLK2 = HCLK */
        RCC_PCLK2Config( RCC_HCLK_Div1);

        /* PCLK1 = HCLK/2 */
        RCC_PCLK1Config( RCC_HCLK_Div2);

        /* PLLCLK = 8MHz * 9 = 72 MHz */
        RCC_PLLConfig(0x00010000, RCC_PLLMul_9);

        /* Enable PLL */
        RCC_PLLCmd( ENABLE);

        /* Wait till PLL is ready */
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
        {
        }

        /* Select PLL as system clock source */
        RCC_SYSCLKConfig( RCC_SYSCLKSource_PLLCLK);

        /* Wait till PLL is used as system clock source */
        while (RCC_GetSYSCLKSource() != 0x08)
        {
        }
    }
    else
    { /* If HSE fails to start-up, the application will have wrong clock configuration.
     User can add here some code to deal with this error */

        /* Go to infinite loop */
        while (1)
        {
        }
    }
}

//=================================================================================
#define BUFF_SIZE 1000
volatile uint16_t ADCValues[BUFF_SIZE] = {'\0'};
volatile uint16_t BufferIndex = 0;

//=================================================================================

void ADC_init() {
	//ADC
	ADC_InitTypeDef ADC_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;
	// input of ADC (it doesn't seem to be needed, as default GPIO state is floating input)
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1 ;		// that's ADC1 (PA1 on STM32)
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//clock for ADC (max 14MHz --> 72/6=12MHz)
	RCC_ADCCLKConfig (RCC_PCLK2_Div6);
	// enable ADC system clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	// define ADC config
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	// we work in continuous sampling mode
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;

	ADC_RegularChannelConfig(ADC1,ADC_Channel_1, 1,ADC_SampleTime_28Cycles5); // define regular conversion config
	ADC_Init ( ADC1, &ADC_InitStructure);	//set config of ADC1

	// enable ADC
	ADC_Cmd (ADC1,ENABLE);	//enable ADC1

	//	ADC calibration (optional, but recommended at power on)
	ADC_ResetCalibration(ADC1);	// Reset previous calibration
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);	// Start new calibration (ADC must be off at that time)
	while(ADC_GetCalibrationStatus(ADC1));

	// start conversion
	ADC_Cmd (ADC1,ENABLE);	//enable ADC1
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);	// start conversion (will be endless as we are in continuous mode)
}

void init_tim2()
{
	RCC_PCLK1Config(RCC_HCLK_Div2);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the TIM2 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 7200-1;
	TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t)(1000-1); //aprox 20Hz (send 20 measurments per second

	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	// /* Prescaler configuration */
	// TIM_PrescalerConfig(TIM2, PrescalerValue, TIM_PSCReloadMode_Immediate);

	TIM_ITConfig( TIM2, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
}

void TIM2_IRQHandler (void) {
	if ((TIM2->SR & 0x0001) != 0) {	// check interrupt source                           // move down
		TIM2->SR &= ~(1<<0);                          // clear UIF flag

		send_data_flag = 1;
	}
} // end TIM1_UP_IRQHandler

void init_tim4_pwm()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;


	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	/* GPIOA and GPIOB clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

	/*GPIOB Configuration: TIM3 channel1, 2, 3 and 4 */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 7200 - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = 20 -1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	/* PWM1 Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = CCR3_Val;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC3Init(TIM4, &TIM_OCInitStructure);

	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

	/* PWM1 Mode configuration: Channel2 */
//	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//	TIM_OCInitStructure.TIM_Pulse = CCR4_Val;
//
//	TIM_OC4Init(TIM4, &TIM_OCInitStructure);
//
//	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM4, ENABLE);

	/* TIM3 enable counter */
	TIM_Cmd(TIM4, ENABLE);
}

float p1 = 77.84f;
float p2 = -5.442e+04;
float q1 = -673.4;

float get_db_value(uint16_t adc) {
	return (p1*adc + p2) / (adc + q1);
}

float a = 9.367e-08;
float b = 5.017;
float c = 88.55;

float get_gauge_pwm(float db) {
	return a * pow(db, b) + c;
}

void init_backlight() {
	GPIO_InitTypeDef GPIO_InitStructure;
	/* GPIOA Periph clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	/* Configure PA7 in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

int main(void) {
	int i = 0;

	uint32_t val_sum;
	uint16_t adc_value;

	SetSysClockTo72();
	init_timer();

	// VU motor controller
	init_tim4_pwm();

	// ADC
	ADC_init();

	// init HC-05
	init_tim2();
	init_usart1();

	//
	init_backlight();

	interupt_Config();

	while (1) {
		adc_value = ADC_GetConversionValue(ADC1);
		ADCValues[BufferIndex++] = adc_value;

		if (BufferIndex == BUFF_SIZE) {
			val_sum = 0;

			for (i = 0; i < BUFF_SIZE; i++) {
				val_sum += ADCValues[i];
			}

			float avg = val_sum / BufferIndex; // mean value of last readings inside the buffer

			float db = get_db_value(avg);

			CCR3_Val = (int) get_gauge_pwm(db);

			BufferIndex = 0;

//			send_byte((uint8_t)(avg / 16));
//			if (send_data_flag == 1) {
//				send_data_flag = 0;
//
//				broadcast_reading((uint16_t) avg);
//			}
		}

		if (data_available()) {
			uint8_t data = get_data();
			if (data == CMD_BACKLIGHT_ON) {
				GPIOA->BSRR = GPIO_Pin_7;
			} else if (data == CMD_BACKLIGHT_OFF) {
				GPIOA->BRR = GPIO_Pin_7;
			}
		}

		TIM4->CCR3 = CCR3_Val; // update PWM value
	}
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
