#ifndef MACROS_H
#define MACROS_H
#include "stm32f091xc.h"
#include "system_stm32f0xx.h"
#define GET_PSC(desired_freq) ((SYSCLK_FREQ / desired_freq) - 1)

/*
  PA7 -> TRIG (TIM17_CH1) (AF5) (D11)
  PA2 -> ECHO  (TIM15_CH1) (AF0) (D1)
  PB4 -> Speaker (TIM3_CH1) (AF1) (D5)
  PA5 -> LED (GPIO)
  
*/  

extern uint32_t SystemCoreClock;
#define SYSCLK_FREQ SystemCoreClock

#define TRIG_PORT         GPIOA
#define TRIG_PIN          7
#define ECHO_PORT         GPIOA
#define ECHO_PIN          2
#define SPEAKER_PORT      GPIOB
#define SPEAKER_PIN       4

#define TRIGGER_PULSE_LEN_US 10  // 10 microseconds
#define OC1M_PWM_MODE_1 0b110


#define TIM15_CC1S 0b01 // CC1 channel is configured as input, IC1 is mapped on TI1
#define TIM15_CCR2 0b10 // CC2 channel is configured as input, IC2 is mapped on TI1
#define TIM15_SMCR_TS 0b101 // TI1FP1 selected as trigger input
#define TIM15_SMCR_SMS 0b100 // Slave mode selection: Reset Mode
#define ECHO_TIMEOUT_US      30000 // 30 milliseconds
#define CM_PER_US 0.0343f // Speed of sound in cm/us 
#define MAX_CM 300 // Maximum distance in cm


#define TRIG_AF          0b0101    // AF5 for TIM17_CH1 on PA7
#define ECHO_AF          0b0000    // AF0 for TIM15_CH1 on PA2
#define SPEAKER_AF      0b0001    // AF1 for TIM3_CH1 on PB4

#define GPIO_IN_MODE     0b00
#define GPIO_OUT_MODE    0b01
#define GPIO_AF_MODE     0b10
#define GPIO_ANALOG_MODE 0b11

#define TRIG_TIMER       TIM17
#define ECHO_TIMER       TIM15
#define SPEAKER_TIMER    TIM3



#endif // MACROS_H