// lcd_display.c  
#include <stdint.h>  
#include "stm32f091xc.h"  
#include "main.h" 
#include "walter.h"

/*  
  GFX01M2 / MB1642-DT022CTFT module  
  Pins (wired from MCU):  
    PA1  -> RST (active low)  
    PA5  -> SCK  
    PA7  -> MOSI  
    PA9  -> CS  (Chip Select) (Active low)  
    PB10 -> DC  (Data/Command)  (0=Command, 1=Data)
*/  

#define RESET_PORT        GPIOA  
#define RESET_PIN         1  
#define SCK_PORT          GPIOA  
#define SCK_PIN           5  
#define MOSI_PORT         GPIOA  
#define MOSI_PIN          7  
#define CS_PORT           GPIOA  
#define CS_PIN            9  
#define DC_PORT           GPIOB  
#define DC_PIN            10  

#define GPIO_AF_MODE      0b10  
#define GPIO_OUT_MODE     0b01  
#define GPIO_SPI_AF      0    // AF0 for SPI1 on PA5, PA7

#define SPI_CLOCK_PRESCALE  0b000   // f_PCLK / 2  
#define SPI_DATA_SIZE_8BIT  0b0111  // for CR2 DS bits  
#define SPI_DATA_SIZE_16BIT  0b1111  // for CR2 DS bits  

#define SLEEP_OUT         0x11  
#define PIXEL_FORMAT_SET  0x3A  
#define PIXEL_FORMAT_16BIT 0x55  
#define MEM_ACCESS_CTRL   0x36  
#define DISPLAY_ORIENTATION 0x48  
#define DISPLAY_ON        0x29  
#define MEM_WRITE         0x2C  
#define COL_ADDR_SET      0x2A  
#define ROW_ADDR_SET      0x2B  

#define RESET_ACTIVE_VAL   0  
#define RESET_INACTIVE_VAL 1  

#define CS_ACTIVE_VAL      0   // assuming active‐low  
#define CS_INACTIVE_VAL    1  

#define DC_COMMAND_MODE_VAL  0  
#define DC_DATA_MODE_VAL     1  

#define LCD_WIDTH     240  
#define LCD_HEIGHT    320  


#define RESET_DELAY 20000

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
    GPIOx->ODR |= (1U << pin);
  } else {
    GPIOx->ODR &= ~(1U << pin);
  }
}

static inline void lcd_cs_active(void) {
  set_gpio_data(CS_PORT, CS_PIN, CS_ACTIVE_VAL);
}
static inline void lcd_cs_inactive(void) {
  set_gpio_data(CS_PORT, CS_PIN, CS_INACTIVE_VAL);
}

static void delay_approx(uint32_t loops) {
  for (volatile uint32_t i = 0; i < loops; ++i) {
    __asm__("nop");
  }
}

static inline void init_lcd_gpio(void) {
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;

  // CS, RESET, DC, as outputs
  set_gpio_mode(CS_PORT,      CS_PIN,      GPIO_OUT_MODE);
  set_gpio_mode(RESET_PORT,   RESET_PIN,   GPIO_OUT_MODE);
  set_gpio_mode(DC_PORT,      DC_PIN,      GPIO_OUT_MODE);

  // SCK, MOSI as AF
  set_gpio_mode(SCK_PORT,  SCK_PIN,  GPIO_AF_MODE);
  set_af(SCK_PORT,  SCK_PIN,  GPIO_SPI_AF);  // AF0
  set_gpio_mode(MOSI_PORT, MOSI_PIN, GPIO_AF_MODE);
  set_af(MOSI_PORT, MOSI_PIN, GPIO_SPI_AF);  // AF0

  // Default states
  set_gpio_data(CS_PORT, CS_PIN, CS_INACTIVE_VAL);
  set_gpio_data(DC_PORT, DC_PIN, DC_COMMAND_MODE_VAL);
  set_gpio_data(RESET_PORT, RESET_PIN, RESET_INACTIVE_VAL);

}

static inline void init_spi(void) {
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

  SPI1->CR1 = 0;
  SPI1->CR2 = 0;

  SPI1->CR1 |= SPI_CR1_MSTR;  // Master mode
  SPI1->CR1 |= (SPI_CLOCK_PRESCALE << SPI_CR1_BR_Pos);
  SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;  // Software slave management, SSI=1

  // SPI mode 0
  SPI1->CR1 &= ~(SPI_CR1_CPOL | SPI_CR1_CPHA);

  // Data size = 8-bit, set RX threshold for 8-bit
  SPI1->CR2 |= (SPI_DATA_SIZE_8BIT << SPI_CR2_DS_Pos);

  SPI1->CR1 |= SPI_CR1_SPE;  // Enable SPI
}

static void spi_send(uint8_t data) {
  while (!(SPI1->SR & SPI_SR_TXE)) { }
  *((__IO uint8_t *)&SPI1->DR) = data;
}

static void spi_wait_done(void) {
  while (SPI1->SR & SPI_SR_BSY) { }
}

static void lcd_send_command(uint8_t cmd) {
  set_gpio_data(DC_PORT, DC_PIN, DC_COMMAND_MODE_VAL);
  lcd_cs_active();
  spi_send(cmd);
  lcd_cs_inactive();
}

static void lcd_send_command_with_args(uint8_t cmd, const uint8_t *args, uint32_t n) {
  set_gpio_data(DC_PORT, DC_PIN, DC_COMMAND_MODE_VAL);
  lcd_cs_active();
  spi_send(cmd);
  if (n) {
    set_gpio_data(DC_PORT, DC_PIN, DC_DATA_MODE_VAL);
    for (uint32_t i = 0; i < n; ++i) {
      spi_send(args[i]);
    }
  }
  lcd_cs_inactive();
}

static void init_lcd(void) {
  init_lcd_gpio();

  // Hardware reset
  set_gpio_data(RESET_PORT, RESET_PIN, RESET_ACTIVE_VAL);
  delay_approx(RESET_DELAY);  
  set_gpio_data(RESET_PORT, RESET_PIN, RESET_INACTIVE_VAL);
  delay_approx(RESET_DELAY); 

  init_spi();

  // Sleep Out
  lcd_send_command(SLEEP_OUT);

  // Pixel Format: 16 bit
  {
    const uint8_t pf = PIXEL_FORMAT_16BIT;
    lcd_send_command_with_args(PIXEL_FORMAT_SET, &pf, 1);
  }

  // Memory Access Control (orientation)
  {
    const uint8_t mac = DISPLAY_ORIENTATION;
    lcd_send_command_with_args(MEM_ACCESS_CTRL, &mac, 1);
  }

  // Display ON
  lcd_send_command(DISPLAY_ON);
}

#define WALTER_MODE 0  

int main(void) {
  init_lcd();


  // Memory Write & flood red
  lcd_send_command(MEM_WRITE);
  set_gpio_data(DC_PORT, DC_PIN, DC_DATA_MODE_VAL);
  lcd_cs_active();


  SPI1->CR2 |= (SPI_DATA_SIZE_16BIT << SPI_CR2_DS_Pos);

  if (WALTER_MODE == 0) {

    for (uint32_t i = 0; i < (uint32_t)LCD_WIDTH * (uint32_t)LCD_HEIGHT; ++i) {
      spi_send(0xF800);  // Red high byte (0xF8 >> 3 gives R=31)
    }
    spi_wait_done();
    lcd_cs_inactive();
  }

  else {

    for (int i = 0; i < LCD_HEIGHT; i++) {
      for (int j = 0; j < LCD_WIDTH; j++) {
        uint16_t color = image_data[i][j];
        spi_send(color);
      }
    }
    spi_wait_done();
    lcd_cs_inactive();
  }

  // Stay here
  while (1) {
    // optionally toggle an LED to show alive
  }

  return 0;
}
