#include <stdint.h>
#include "stm32f091xc.h"
#include "system_stm32f0xx.h"
extern uint32_t SystemCoreClock;

#define SYSCLK_FREQ SystemCoreClock

#define ON 0
#define OFF 1
#define BLINK 2

#define MAX_STATE BLINK
#define MIN_STATE ON

#define DEBOUNCE_TIME 50 // milliseconds

#define GET_PSC(desired_freq) ((SYSCLK_FREQ / desired_freq) - 1)

#define TIM2_FREQ 1000
#define OFF_DUTY 0
#define ON_DUTY TIM2_FREQ-1
#define BLINK_DUTY (ON_DUTY+1)/2 - 1


// PC13 for button
// PA5 for LED


/*
	Green LED is controlled by PWM
	Blinking has 50% duty cycle
 */
#define PWM_MODE_1 0b110
static inline void setup_blink_timer() {
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	TIM2->CCMR1 &= ~(TIM_CCMR1_CC1S); // Set T2C1 to output mode
	TIM2->CCMR1 &= ~(TIM_CCMR1_OC1PE); // Disable preload
	TIM2->CCMR1 &= ~(TIM_CCMR1_OC1M);
	TIM2->CCMR1 |= (PWM_MODE_1 << TIM_CCMR1_OC1M_Pos); // Set T2C1 to PWM Mode 1
	// Disabling preload ensures changes to duty cycle take place immediately
	TIM2->CCMR1 &= ~TIM_CCMR1_OC1PE; // Disable CCR1 (duty cycle) preload

	TIM2->CCER |= TIM_CCER_CC1E; // Activate output channel 1
	TIM2->CCER &= ~TIM_CCER_CC1P; // Set to active high


	TIM2->PSC = GET_PSC(TIM2_FREQ); // Set clock to 1000 Hz
	TIM2->ARR = TIM2_FREQ-1; // Set PWM freq to 1Hz
	TIM2->CCR1 = ON_DUTY; // Duty cycle 100%

	TIM2->CR1 |= TIM_CR1_CEN; // enable timer
}


/*
	Debounce Algorithm:
	- When button isr triggered, restart timer
	- If timer is allowed to reach ARR without resetting, enough time has passed
	- Timer ISR switches state
 */
int state = ON;
void TIM3_IRQHandler(void) {
	if (!(TIM3->SR & TIM_SR_UIF)) return; // Check flag
	TIM3->SR &= ~TIM_SR_UIF; // Clear interrupt flag
	if (state == MAX_STATE) state = MIN_STATE;
	else state++;
}

static inline void setup_debounce_timer() {
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	TIM3->PSC = GET_PSC(1000); // Set prescaler so that frequency = 1KHZ
	TIM3->ARR = DEBOUNCE_TIME; // Set timer delay to debounce time

	TIM3->CR1 |= TIM_CR1_OPM; // Set to one pulse mode

	TIM3->DIER |= TIM_DIER_UIE; // Enable update interrupt
	TIM3->SR &= ~TIM_SR_UIF; // Clear interrupt flag
	NVIC_EnableIRQ(TIM3_IRQn);

}

void EXTI4_15_IRQHandler(void) {
	if (!(EXTI->PR & EXTI_PR_PR13)) return;
	EXTI->PR |= EXTI_PR_PR13; // Clear interrupt flag
	TIM3->CNT = 0; // Reset debounce timer
	TIM3->CR1 |= TIM_CR1_CEN; // Start debounce timer

}

static inline void setup_gpio()
{
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;


    GPIOA->MODER &= ~GPIO_MODER_MODER5;
    GPIOA->MODER |= (0b10 << GPIO_MODER_MODER5_Pos); // Set PA5 (LED) to alternate function
	GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL5;
    GPIOA->AFR[0] |= (2u << GPIO_AFRL_AFSEL5_Pos); // Alternate function 2 (timer 2 channel 1)

    GPIOC->MODER &= ~GPIO_MODER_MODER13; // Set PC13 to input mode
    GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR13;
    GPIOC->PUPDR |= (0b01 << GPIO_PUPDR_PUPDR13_Pos); // Set PC13 to pullup mode


    SYSCFG->EXTICR[3] &= SYSCFG_EXTICR4_EXTI13;
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC; // Set EXTI line 13 to port C

    EXTI->IMR |= EXTI_IMR_MR13; //Unmask
    EXTI->FTSR |= EXTI_FTSR_TR13; // Enable falling edge
    EXTI->RTSR &= ~EXTI_RTSR_TR13; // Disable rising edge
    EXTI->PR |= EXTI_PR_PR13; // Clear interrupt flag


    NVIC_EnableIRQ(EXTI4_15_IRQn);

}

void setup() {
	SystemCoreClockUpdate();
	setup_debounce_timer();
	setup_blink_timer();
	setup_gpio();

}


int main() {
	setup();
    while(1) {
    	switch (state) {
    	case ON:
    		TIM2->CCR1 = ON_DUTY; // 100% duty cycle
    		break;
    	case OFF:
    		TIM2->CCR1 = OFF_DUTY; // 0% duty cycle
    		break;
    	case BLINK:
    		TIM2->CCR1 = BLINK_DUTY; // 50% duty cycle
    		break;
    	}
    }
}




