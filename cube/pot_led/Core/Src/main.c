// OLD CODE

#include <stdint.h>
#include "stm32f091xc.h"
#include <stm32f0xx_hal.h>

#define PRESCALE 46
#define MAX_DUTY_CYCLE 1023

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

// PC13 for button
// PA5 for LED

void delay(volatile int del) {
	while(--del);
}

volatile int duty_cycle = 0;

void EXTI4_15_IRQHandler(void) {
	if (EXTI->PR & 1u << 13) {
		EXTI->PR |= 1u << 13;
        duty_cycle += 100;
        if (duty_cycle > 1000) {
            duty_cycle = 0;
        }
		TIM2->CCR1 = duty_cycle;

		// GPIOA->ODR ^= 1u << 5;
	}
}



int main(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    GPIOA->MODER &= ~(0b11 << (2 * 5));
    GPIOA->MODER |= (0b10 << (2 * 5)); // Set PA5 (LED) to alternate function
    GPIOA->AFR[0] &= ~(0b1111 << (4 * 5));
    GPIOA->AFR[0] |= (2u << (4 * 5)); // Alternate function 2 (timer 2 channel 1)

    GPIOC->MODER &= ~(0b11 << (2 * 13)); // Set PC13 to input mode
    GPIOC->PUPDR &= ~(0b11 << (2 * 13));
    GPIOC->PUPDR |= (0b01 << (2 * 13)); // Set PC13 to pullup mode

    SYSCFG->EXTICR[3] &= ~(0b1111 << 4 * 1);
    SYSCFG->EXTICR[3] |= (0b0010 << 4 * 1); // Set EXTI line 13 to port C

    EXTI->IMR |= 1u << 13; //Unmask
    EXTI->FTSR |= 1u << 13; // Enable falling edge
    EXTI->RTSR &= ~(1u << 13); // Disable rising edge
    EXTI->PR |= 1u << 13;

    NVIC_EnableIRQ(EXTI4_15_IRQn);

    TIM2->PSC = PRESCALE;
    TIM2->ARR = MAX_DUTY_CYCLE;
    TIM2->CCR1 = 0;


    TIM2->CCMR1 &= ~(0b11 << 0 * 2); // Set T2C1 to output mode
    TIM2->CCMR1 |= 1u << 3; // Enable preload
    TIM2->CCMR1 &= ~(0b111 << 4);
    TIM2->CCMR1 |= (0b110 << 4); // Set T2C1 to PWM Mode 1

    TIM2->CCER |= 1u; // Activate output channel 1
    TIM2->CCER &= ~(1u << 1); // Set to active high


    TIM2->CR1 |= TIM_CR1_CEN; // enable timer


    // Disable ADC
    if (ADC1->CR & ADC_CR_ADSTART) { // Check for ongoing conversion
    	ADC1->CR |= ADC_CR_ADSTP; // Stop conversion
    	while (ADC1->CR & ADC_CR_ADSTP); // Wait until conversion is stopped
    }

    ADC1->CR |= ADC_CR_ADDIS; // Disable ADC
    while (ADC1->CR & ADC_CR_ADEN); // Wait until ADC is disabled
    ADC1->ADC_ISR |= ADC_ISR_ADRDY; // Clear ISR flag



    while(1) {
    	//GPIOA->ODR ^= 1u << 5;
    	delay(1000000);
    }
}




