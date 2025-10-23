#include <stdint.h>
#include "stm32f091xc.h"

#define SCK_PIN 5
#define MOSI_PIN 7
#define GPIO_AF_MODE 0b10


#define SPI_CLOCK_PRESCALE 0b001
#define SPI_DATA_SIZE_8BIT 0b0111



static inline void init_spi() {
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

  GPIOA->MODER &= ~(GPIO_MODER_MODER4);
  GPIOA->MODER |= (GPIO_AF_MODE << GPIO_MODER_MODER4_Pos); 
  GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL4); // NSSI/Chip Select

  GPIOA->MODER &= ~(GPIO_MODER_MODER5);
  GPIOA->MODER |= (GPIO_AF_MODE << GPIO_MODER_MODER5_Pos); // SCL
  GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL5); // SCL

  GPIOA->MODER &= ~(GPIO_MODER_MODER7);
  GPIOA->MODER |= (GPIO_AF_MODE << GPIO_MODER_MODER7_Pos);
  GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL7); // SDA

  SPI1->CR1 = 0;
  SPI1->CR1 |= SPI_CR1_MSTR; // Master mode
  SPI1->CR1 |= (SPI_CLOCK_PRESCALE << SPI_CR1_BR_Pos); // Set clock prescaler
  SPI1->CR1 |= SPI_CR1_SSM; // Software slave management
  SPI1->CR1 |= SPI_CR1_SSI; // Set internal slave select high (active low)
  SPI1->CR1 |= (SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE); // 1-line TX only mode

  SPI1->CR2 = 0;
  SPI1->CR2 |= (SPI_CR2_SSOE); // SS output enable
  // Remove this if it doesn't work
  SPI1->CR2 |= (SPI_CR2_NSSP); // NSS pulse management enable (datasheet says chip select *can* be pulsed between writes)
  
  SPI1->CR2 |= (SPI_DATA_SIZE_8BIT << SPI_CR2_DS_Pos); // 8-bit data size

  SPI1->CR1 |= SPI_CR1_SPE; // Enable SPI
}

static inline void setup() {

}


int main() {
  
  setup();
  while (1) {

  }
  return 0;
}