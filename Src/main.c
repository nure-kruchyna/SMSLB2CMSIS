#include <stdint.h>
#include "stm32f4xx.h"

uint8_t Button_Flag = 0;
uint8_t Button_Long = 0;

uint64_t millis = 0;

void delay(uint32_t time) {
	while (time--)
		;
}

/*========== Interrupt handlers =============*/

void EXTI0_IRQHandler(void)
{
	uint16_t count = 0;
	uint8_t PrevState = 0;

	while(count<30)
	{
		if (PrevState != (GPIOA->IDR & GPIO_IDR_ID0_Msk))
		{
			PrevState = (GPIOA->IDR & GPIO_IDR_ID0_Msk);
			count=0;
		}
		else
		{
			count++;
		}
		delay(500);
	}

	if (PrevState >= 1)
	{
		Button_Flag ^= 1;
		TIM6->CR1 |= TIM_CR1_CEN;
	}
	else
	{
		if(Button_Long)
		{
			Button_Flag = 0;
			Button_Long = 0;
		}
		if (TIM6->CR1 & TIM_CR1_CEN)
		{
			TIM6->CR1 &= ~TIM_CR1_CEN;
			TIM6->CNT=0x0000;
		}
	}

	EXTI->PR|=EXTI_PR_PR0;
	__NVIC_ClearPendingIRQ(EXTI0_IRQn);
}



void TIM6_DAC_IRQHandler(void)
{
	if (GPIOA->IDR & GPIO_IDR_ID0_Msk)
		Button_Long = 1;
	else
		Button_Long = 0;

	TIM6->SR &= ~TIM_SR_UIF;
	__NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);
}



void SysTick_Handler (void)
{
	millis++;
	__NVIC_ClearPendingIRQ(SysTick_IRQn);
}

/*=========== Startup settings ===========*/

void RCCConfig(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOAEN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
}



void TIMConfig(void)
{
	TIM6->DIER |= TIM_DIER_UIE;
	TIM6->CR1 |= TIM_CR1_OPM;
	TIM6->PSC = 7999;
	TIM6->ARR = 1000;
	TIM6->EGR|=TIM_EGR_UG;


	TIM4->PSC = 0;
	TIM4->ARR = 65535;
	TIM4->CCR4 = 0; //DUTY
	TIM4->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE;
	TIM4->CCER |= TIM_CCER_CC4E;
	TIM4->CR1|=TIM_CR1_ARPE;
	TIM4->EGR |= TIM_EGR_UG;
	TIM4->CR1 |= TIM_CR1_CEN;

	SysTick->LOAD = 2000;
	SysTick->CTRL|=SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;

}



void GPIOConfig(void) {

	GPIOD->MODER |= GPIO_MODER_MODE12_0 | GPIO_MODER_MODE13_0 | GPIO_MODER_MODE14_0 ;//| GPIO_MODER_MODE15_0;

	GPIOD->OTYPER &= ~(GPIO_OTYPER_OT12 | GPIO_OTYPER_OT13 | GPIO_OTYPER_OT14 );//| GPIO_OTYPER_OT15);

	GPIOD->MODER |= GPIO_MODER_MODE15_1;

	GPIOD->AFR[1] |= GPIO_AFRH_AFSEL15_1;

	GPIOD->OSPEEDR |= GPIO_OSPEEDR_OSPEED12_0 | GPIO_OSPEEDR_OSPEED13_0	| GPIO_OSPEEDR_OSPEED14_0 | GPIO_OSPEEDR_OSPEED15_0 | GPIO_OSPEEDR_OSPEED15_1;

	GPIOA->MODER&=~GPIO_MODER_MODE0;

	GPIOA->PUPDR|=GPIO_PUPDR_PUPD0_1;
}



void INTConfig(void) {
	RCC->APB2ENR|= RCC_APB2ENR_SYSCFGEN;

	SYSCFG->EXTICR[0]|=SYSCFG_EXTICR1_EXTI0_PA;

	EXTI->IMR|=EXTI_IMR_IM0;

	EXTI->RTSR|=EXTI_RTSR_TR0;

	EXTI->FTSR|=EXTI_FTSR_TR0;

	__enable_irq();

	__NVIC_SetPriority(EXTI0_IRQn, 3);

	__NVIC_SetPriority(TIM6_DAC_IRQn, 5);

	__NVIC_SetPriority(SysTick_IRQn, 4);

	__NVIC_EnableIRQ(EXTI0_IRQn);

	__NVIC_EnableIRQ(TIM6_DAC_IRQn);

	__NVIC_EnableIRQ(SysTick_IRQn);
}

/*============ Main ==============*/

int main(void) {

	RCCConfig();
	TIMConfig();
	GPIOConfig();
	INTConfig();

	uint8_t i = 12;
	uint64_t Millis_Blink = 0;
	uint64_t Millis_PWM = 0;
	uint8_t DutyDir = 1;

	while (1)
	{
		if(Button_Long)
		{
			GPIOD->ODR |= GPIO_ODR_ODR_12 | GPIO_ODR_ODR_13 | GPIO_ODR_ODR_14;
			i = 28;
		}
		else	if (Button_Flag)
		{
			if(millis>=Millis_Blink)
			{
				GPIOD->BSRR |= (1UL << i++);
				if (i == (16 - 1)) {
					i += 12 + 1;
				} else if (i == (32 - 1)) {
					i = 12;
				}
				Millis_Blink = millis+500;
			}
		}
		if(millis>=Millis_PWM)
		{
			if (DutyDir)
			{
				TIM4->CCR4+=100;
				if (TIM4->CCR4-100 >= TIM4->ARR)
					DutyDir = 0;
			}
			else
			{
				TIM4->CCR4-=100;
				if (TIM4->CCR4 == 0)
					DutyDir = 1;
			}
			Millis_PWM=millis+1;
		}
	}

	return 0;
}
