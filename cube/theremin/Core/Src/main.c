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


  TRIG_TIMER->PSC = 48 - 1; // Prescaler to get 1 MHz timer clock
  TRIG_TIMER->ARR = 0xFFFF; // Max auto-reload value

}

static inline void setup() {

}
