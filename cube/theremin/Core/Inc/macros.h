#ifndef MACROS_H
#define MACROS_H
#include "stm32f091xc.h"
#include "system_stm32f0xx.h"
#define GET_PSC(desired_freq) ((SYSCLK_FREQ / desired_freq) - 1)

/*
  PA7 -> TRIG (TIM17_CH1) (AF5) (D11)
  PB10 -> ECHO  (TIM2_CH3) (AF2) (D6)
  PB4 -> Speaker (TIM3_CH1) (AF1) (D5)
  PA5 -> LED (GPIO)
  
*/  

extern uint32_t SystemCoreClock;
#define SYSCLK_FREQ SystemCoreClock

#define TRIG_PORT         GPIOA
#define TRIG_PIN          7
#define ECHO_PORT         GPIOB
#define ECHO_PIN          10
#define SPEAKER_PORT      GPIOB
#define SPEAKER_PIN       4



#define TRIG_AF          0b0101    // AF5 for TIM17_CH1 on PA7
#define ECHO_AF          0b0010    // AF2 for TIM2_CH3 on PB10
#define SPEAKER_AF      0b0001    // AF1 for TIM3_CH1 on PB4

#define GPIO_IN_MODE     0b00
#define GPIO_OUT_MODE    0b01
#define GPIO_AF_MODE     0b10
#define GPIO_ANALOG_MODE 0b11

#define TRIG_TIMER       TIM17
#define ECHO_TIMER       TIM2
#define SPEAKER_TIMER    TIM3



#endif // MACROS_H