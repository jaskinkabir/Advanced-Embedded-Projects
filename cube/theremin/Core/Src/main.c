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

static inline void setup_echo_timer() {
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  ECHO_TIMER->CR1 = 0; // Disable timer

  ECHO_TIMER->PSC = GET_PSC(1000000); // Prescaler to get 1 MHz timer clock
  ECHO_TIMER->ARR = ECHO_TIMEOUT_US; // Set max timeout

  ECHO_TIMER->CCMR2 &= ~(TIM_CCMR2_CC3S);
  ECHO_TIMER->CCMR2 |= (0b01 << TIM_CCMR2_CC3S_Pos); // Set CH3 to input, IC3 mapped on TI3

  ECHO_TIMER->CCER |= TIM_CCER_CC3E; // Enable capture on CH3
  ECHO_TIMER->CCER &= ~TIM_CCER_CC3P; // Capture rising edge

  ECHO_TIMER->DIER |= TIM_DIER_CC3IE; // Enable capture interrupt
  NVIC_EnableIRQ(TIM2_IRQn);
}

static inline void setup() {

}
