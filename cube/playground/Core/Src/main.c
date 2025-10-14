#include <stdint.h>
#include "stm32f091xc.h"
#include <stm32f0xx_hal.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

// PC13 for button
// PA5 for LED

void delay(volatile int del) {
	while(--del);
}

void EXTI4_15_IRQHandler(void) {
	if (EXTI->PR &= 1u << 13) {
		EXTI->PR |= 1u << 13;
		GPIOA->ODR ^= 1u << 5;
	}
}


int main(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    GPIOA->MODER &= ~(0b11 << 2 * 5);
    GPIOA->MODER |= (0b01 << 2 * 5); // Set PA5 (LED) to output mode

    GPIOC->MODER &= ~(0b11 << 2 * 13); // Set PC13 to input mode
    GPIOC->PUPDR &= ~(0b11 << 2 * 13);
    GPIOC->PUPDR |= (0b01 << 2 * 13); // Set PC13 to pullup mode

    SYSCFG->EXTICR[3] &= ~(0b1111 << 4 * 1);
    SYSCFG->EXTICR[3] |= (0b0010 << 4 * 1); // Set EXTI line 13 to port C

    EXTI->IMR |= 1u << 13; //Unmask
    EXTI->FTSR |= 1u << 13; // Enable falling edge
    EXTI->RTSR &= ~(1u << 13); // Disable rising edge
    EXTI->PR |= 1u << 13;

    NVIC_EnableIRQ(EXTI4_15_IRQn);

    while(1) {
    	GPIOA->ODR ^= 1u << 5;
    	delay(1000000);
    }
}




