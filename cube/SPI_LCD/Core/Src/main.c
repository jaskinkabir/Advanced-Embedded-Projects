#include <stdint.h>
#include "stm32f091xc.h"

/*
GFX01M2 LCD Pins
Pins:
  PA1: ~Reset (RST) (active low)
  PA5: Serial Clock (SCK)
  PA7: Master Out Slave In (MOSI)
  PA9: Chip Select (CS) (STM ds says active high)
  
  PB10: Data Command (DC) (STM datasheet calls this write enable)
*/
#define RESET_PORT GPIOA
#define RESET_PIN 1
#define SCK_PORT GPIOA
#define SCK_PIN 5
#define MOSI_PORT GPIOA
#define MOSI_PIN 7
#define CHIP_SELECT_PORT GPIOA
#define CHIP_SELECT_PIN 9
#define GPIO_AF_MODE 0b10
#define GPIO_OUT_MODE 0b01

#define DATA_COMMAND_PORT GPIOB
#define DATA_COMMAND_PIN 10
#define BACKLIGHT_PORT GPIOB
#define BACKLIGHT_PIN 8


#define SPI_CLOCK_PRESCALE 0b011
#define SPI_DATA_SIZE_8BIT 0b0111
#define SPI_AF_MODE 0b0000

#define SLEEP_OUT 0x11
#define PIXEL_FORMAT_SET 0x3A
#define PIXEL_FORMAT_16BIT 0x55
#define MEM_ACCESS_CTRL 0x36
#define DISPLAY_ORIENTATION 0x48
#define DISPLAY_ON 0x29
#define MEM_WRITE 0x2C
#define COL_ADDR_SET 0x2A
#define ROW_ADDR_SET 0x2B
#define DISPLAY_OFF 0x28

#define RESET_ACTIVE_VAL 0
#define RESET_INACTIVE_VAL 1

#define CHIP_SELECT_ACTIVE_VAL 1
#define CHIP_SELECT_INACTIVE_VAL 0

#define DC_COMMAND_MODE_VAL 0
#define DC_DATA_MODE_VAL 1

#define LCD_WIDTH 240
#define LCD_HEIGHT 320



static inline void set_gpio_mode(GPIO_TypeDef *GPIOx, uint8_t pin, uint8_t mode) {
  GPIOx->MODER &= ~(0b11 << (pin * 2));
  GPIOx->MODER |= (mode << (pin * 2));
}
static inline void set_af(GPIO_TypeDef *GPIOx, uint8_t pin, uint8_t af) {
  if (pin < 8) {
    GPIOx->AFR[0] &= ~(0b1111 << (pin * 4));
    GPIOx->AFR[0] |= (af << (pin * 4));
  } else {
    GPIOx->AFR[1] &= ~(0b1111 << ((pin - 8) * 4));
    GPIOx->AFR[1] |= (af << ((pin - 8) * 4));
  }
}

static inline void set_gpio_data(GPIO_TypeDef *GPIOx, uint8_t pin, uint8_t value) {
  if (value) {
    GPIOx->ODR |= (1 << pin);
  } else {
    GPIOx->ODR &= ~(1 << pin);
  }
}



static inline void init_lcd_gpio() {
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;

  set_gpio_mode(RESET_PORT, RESET_PIN, GPIO_OUT_MODE);
  set_gpio_mode(DATA_COMMAND_PORT, DATA_COMMAND_PIN, GPIO_OUT_MODE);
  set_gpio_mode(BACKLIGHT_PORT, BACKLIGHT_PIN, GPIO_OUT_MODE);
  set_gpio_mode(CHIP_SELECT_PORT, CHIP_SELECT_PIN, GPIO_OUT_MODE);

  set_gpio_mode(SCK_PORT, SCK_PIN, GPIO_AF_MODE);
  set_af(SCK_PORT, SCK_PIN, SPI_AF_MODE); // AF0 for SPI SCK

  set_gpio_mode(MOSI_PORT, MOSI_PIN, GPIO_AF_MODE);
  set_af(MOSI_PORT, MOSI_PIN, SPI_AF_MODE); // AF0 for SPI MOSI

}
static inline void init_spi() {
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

  SPI1->CR1 = 0;
  SPI1->CR1 |= SPI_CR1_MSTR; // Master mode
  SPI1->CR1 |= (SPI_CLOCK_PRESCALE << SPI_CR1_BR_Pos); // Set clock prescaler
  SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI; // Software slave management
  SPI1->CR1 |= (SPI_CR1_BIDIOE); // 2-line TX only mode


  SPI1->CR2 = 0;
  // Remove this if it doesn't work
  // SPI1->CR2 |= (SPI_CR2_SSOE); // SS output enable
  //SPI1->CR2 |= (SPI_CR2_NSSP); // NSS pulse management enable (datasheet says chip select *can* be pulsed between writes)
  
  SPI1->CR2 |= (SPI_DATA_SIZE_8BIT << SPI_CR2_DS_Pos); // 8-bit data size

  SPI1->CR1 |= SPI_CR1_SPE; // Enable SPI
}


static void spi_send(uint8_t data) {
  while (
    (!(SPI1->SR & SPI_SR_TXE)) ||
    (SPI1->SR & SPI_SR_BSY)
  ); // Wait until transmit buffer is empty
  *((__IO uint8_t *)&SPI1->DR) = data; // Send data
}

static void lcd_send_command(uint8_t cmd) {
  set_gpio_data(DATA_COMMAND_PORT, DATA_COMMAND_PIN, DC_COMMAND_MODE_VAL); // Set DC for command
  set_gpio_data(CHIP_SELECT_PORT, CHIP_SELECT_PIN, CHIP_SELECT_ACTIVE_VAL); // Activate chip select
  spi_send(cmd);
  set_gpio_data(CHIP_SELECT_PORT, CHIP_SELECT_PIN, CHIP_SELECT_INACTIVE_VAL); // Deactivate chip select

}

static void lcd_send_command_with_arg(uint8_t cmd, uint8_t arg) {
  set_gpio_data(DATA_COMMAND_PORT, DATA_COMMAND_PIN, DC_COMMAND_MODE_VAL); // Set DC for command
  set_gpio_data(CHIP_SELECT_PORT, CHIP_SELECT_PIN, CHIP_SELECT_ACTIVE_VAL); // Activate chip select
  spi_send(cmd);
  set_gpio_data(DATA_COMMAND_PORT, DATA_COMMAND_PIN, DC_DATA_MODE_VAL); // Set DC for data
  spi_send(arg);
  set_gpio_data(CHIP_SELECT_PORT, CHIP_SELECT_PIN, CHIP_SELECT_INACTIVE_VAL); // Deactivate chip select
}

static void lcd_send_data(uint8_t data) {
  set_gpio_data(DATA_COMMAND_PORT, DATA_COMMAND_PIN, DC_DATA_MODE_VAL); // Set DC high for data
  set_gpio_data(CHIP_SELECT_PORT, CHIP_SELECT_PIN, CHIP_SELECT_ACTIVE_VAL); // Activate chip select
  spi_send(data);
  set_gpio_data(CHIP_SELECT_PORT, CHIP_SELECT_PIN, CHIP_SELECT_INACTIVE_VAL); // Deactivate chip select
}
static void init_lcd() {
  init_lcd_gpio();
  init_spi();

  // Reset LCD
  // set_gpio_data(RESET_PORT, RESET_PIN, RESET_ACTIVE_VAL); // Start reset
  // for (volatile int i = 0; i < 100000; i++); // Delay
  set_gpio_data(RESET_PORT, RESET_PIN, RESET_INACTIVE_VAL); // End reset
  for (volatile int i = 0; i < 100000; i++); // Delay

  // Send initialization commands
  lcd_send_command(SLEEP_OUT);
  for (volatile int i = 0; i < 100000; i++); // Delay
  lcd_send_command(DISPLAY_OFF);
  //for (volatile int i = 0; i < 100000000; i++); // Delay

  lcd_send_command_with_arg(PIXEL_FORMAT_SET, PIXEL_FORMAT_16BIT);

  /*
  Bit 7: MY (row address order) = 0 ()
  Bit 6: MX (column address order) = 1 (mirrored)
  Bit 5: MV (row/column exchange) = 0 (normal)
  Bit 4: ML (vertical refresh order) = 0 (normal)
  Bit 3: BGR (RGB/BGR order) = 1 (BGR)
  Bit 2: MH (horizontal refresh order) = 0 (normal)
  
  The 3 most significant bits mirror the x axis
  */
  lcd_send_command_with_arg(MEM_ACCESS_CTRL, DISPLAY_ORIENTATION);
  lcd_send_data(DISPLAY_ORIENTATION);
  lcd_send_command(DISPLAY_ON);

  // Turn on backlight
  //BACKLIGHT_PORT->ODR |= (1 << BACKLIGHT_PIN);

}



int main() {
  
  init_lcd();
  lcd_send_command(MEM_WRITE);
  set_gpio_data(CHIP_SELECT_PORT, CHIP_SELECT_PIN, CHIP_SELECT_ACTIVE_VAL); // Activate chip select
  set_gpio_data(DATA_COMMAND_PORT, DATA_COMMAND_PIN, DC_DATA_MODE_VAL); // Set DC for data
  for (uint32_t i = 0; i < LCD_WIDTH * LCD_HEIGHT; i++) {
    spi_send(0xF8); // Red high byte
    spi_send(0x00); // Red low byte
  }
  set_gpio_data(CHIP_SELECT_PORT, CHIP_SELECT_PIN, CHIP_SELECT_INACTIVE_VAL); // Deactivate chip select
  return 0;
}