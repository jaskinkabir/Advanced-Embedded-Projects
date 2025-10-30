#include <stdint.h>  
#include "stm32f091xc.h"  
#include "macros.h"


inline void set_gpio_mode(GPIO_TypeDef *GPIOx, uint8_t pin, uint8_t mode) {
  GPIOx->MODER &= ~(0b11 << (pin * 2));
  GPIOx->MODER |= (mode << (pin * 2));
}

inline void set_af(GPIO_TypeDef *GPIOx, uint8_t pin, uint8_t af) {
  if (pin < 8) {
    GPIOx->AFR[0] &= ~(0b1111 << (pin * 4));
    GPIOx->AFR[0] |= (af << (pin * 4));
  } else {
    GPIOx->AFR[1] &= ~(0b1111 << ((pin - 8) * 4));
    GPIOx->AFR[1] |= (af << ((pin - 8) * 4));
  }
}

inline void set_gpio_data(GPIO_TypeDef *GPIOx, uint8_t pin, uint8_t value) {
  if (value) {
    GPIOx->ODR |= (1U << pin);
  } else {
    GPIOx->ODR &= ~(1U << pin);
  }
}

inline uint8_t read_gpio_data(GPIO_TypeDef *GPIOx, uint8_t pin) {
  return (GPIOx->IDR & (1U << pin)) ? 1 : 0;
}

static inline void setup_gpio() {
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;

  set_gpio_mode(TRIG_PORT, TRIG_PIN, GPIO_AF_MODE);
  set_af(TRIG_PORT, TRIG_PIN, TRIG_AF);

  set_gpio_mode(ECHO_PORT, ECHO_PIN, GPIO_AF_MODE);
  set_af(ECHO_PORT, ECHO_PIN, ECHO_AF);

  set_gpio_mode(SPEAKER_PORT, SPEAKER_PIN, GPIO_AF_MODE);
  set_af(SPEAKER_PORT, SPEAKER_PIN, SPEAKER_AF);
}


/*
  The TRIG timer generates a 10us pulse to trigger the ultrasonic sensor.
  It is configured in one-pulse PWM mode.
*/
static inline void setup_trig_timer() {
  RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;
  TRIG_TIMER->CR1 = 0; // Disable timer
  
  TRIG_TIMER->CR1 |= TIM_CR1_OPM; // One pulse mode


  TRIG_TIMER->PSC = GET_PSC(1000000); // Prescaler to get 1 MHz timer clock
  TRIG_TIMER->ARR = TRIGGER_PULSE_LEN_US; // 10us
  TRIG_TIMER->CCR1 = TRIGGER_PULSE_LEN_US; // PWM mode, pulse length 10us
  TRIG_TIMER->CCMR1 |= (OC1M_PWM_MODE_1 << TIM_CCMR1_OC1M_Pos); // PWM mode 1

  TRIG_TIMER->CCER |= TIM_CCER_CC1E; // Enable output for channel 1
  TRIG_TIMER->BDTR |= TIM_BDTR_MOE; // Enable main output
}

/* (1) Select the active input TI1 for TIMx_CCR1 (CC1S = 01),
select the active input TI1 for TIMx_CCR2 (CC2S = 10) */
/* (2) Select TI1FP1 as valid trigger input (TS = 101)
configure the slave mode in reset mode (SMS = 100) */
/* (3) Enable capture by setting CC1E and CC2E
select the rising edge on CC1 and CC1N (CC1P = 0 and CC1NP = 0, reset
value),
select the falling edge on CC2 (CC2P = 1). */
/* (4) Enable interrupt on Capture/Compare 1 */
/* (5) Enable counter */
static inline void setup_echo_timer() {
  RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
  ECHO_TIMER->CR1 = 0; // Disable timer

  ECHO_TIMER->CCR1 =0;
  ECHO_TIMER->CCR1 |= (TIM15_CC1S << TIM_CCMR1_CC1S_Pos); // (1)

  ECHO_TIMER->CCR2 =0;
  ECHO_TIMER->CCR2 |= (TIM15_CCR2 << TIM_CCMR1_CC2S_Pos); // (1)

  ECHO_TIMER->SMCR = 0;
  ECHO_TIMER->SMCR |= (TIM15_SMCR_TS << TIM_SMCR_TS_Pos) | (TIM15_SMCR_SMS << TIM_SMCR_SMS_Pos); // (2)

  ECHO_TIMER->CCER = 0;
  ECHO_TIMER->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC2P; // (3)

  ECHO_TIMER->DIER = 0;
  ECHO_TIMER->DIER |= TIM_DIER_CC2IE; // (4)

  ECHO_TIMER->PSC = GET_PSC(1000000); // Prescaler to get 1 MHz timer clock
  ECHO_TIMER->ARR = ECHO_TIMEOUT_US; // Set max timeout
  NVIC_EnableIRQ(TIM15_IRQn);
}

static inline void setup() {

}
