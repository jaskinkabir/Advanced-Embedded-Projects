#include "stm32f0xx.h"
#include <string.h>

#define APP_BASE        0x08004000UL
#define SRAM_VECTOR     0x20000000UL

static volatile uint8_t jump_request = 0;

static void delay(volatile uint32_t c){ while (c--) { __NOP(); } }

static void gpio_init(void) {
    RCC->AHBENR  |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOCEN;
    GPIOA->MODER &= ~(3u << (5*2));
    GPIOA->MODER |=  (1u << (5*2));      // PA5 output
    GPIOC->MODER &= ~(3u << (13*2));
    GPIOC->PUPDR &= ~(3u << (13*2));
    GPIOC->PUPDR |=  (1u << (13*2));     // PC13 pull-up
}

static void exti13_init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG->EXTICR[3] &= ~(0xFu << 4);
    SYSCFG->EXTICR[3] |=  (0x2u << 4);   // Route PC13
    EXTI->IMR  |= (1u << 13);
    EXTI->FTSR |= (1u << 13);
    NVIC_EnableIRQ(EXTI4_15_IRQn);
}

void EXTI4_15_IRQHandler(void) {
    if (EXTI->PR & (1u << 13)) {
        EXTI->PR = (1u << 13);
        jump_request = 1;
    }
}

static void jump_to_application(void) {
    __disable_irq();                      // Temporarily disable interrupts

    // === TODO: Copy the application's vector table ===
    // Copy the first 64 words (256 bytes) from Flash (APP_BASE)
    // to SRAM (SRAM_VECTOR). Use a simple loop with uint32_t pointers.
    // This ensures a valid vector table exists at 0x20000000 before remapping.
    memcpy((void *)SRAM_VECTOR, (const void *)APP_BASE, 256);

    // 1. Set the MSP to the application's initial stack pointer
    __set_MSP(*(uint32_t *)APP_BASE);

    // 2. Remap 0x00000000 to SRAM using SYSCFG->CFGR1
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG->CFGR1 |= 0x3u;                // SRAM aliased to 0x00000000

    // 3. Jump to application's Reset_Handler
    void (*app_reset)(void) = (void (*)(void))(*(uint32_t *)(APP_BASE + 4u));
    app_reset();
}

int main(void) {
    gpio_init();
    exti13_init();

    while (1) {
        GPIOA->ODR ^= (1u << 5);          // Blink LED
        delay(400000);

        if (jump_request) {
            jump_to_application();
        }
    }
}
