#ifndef DEFINES_H
#define DEFINES_H
#include "stm32f091xc.h"
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
#define DISPLAY_OFF        0x28  
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

//#define DMA_PRIORITY_VERY_HIGH (DMA_CCR_PL_0 | DMA_CCR_PL_1)
#define DMA_MSIZE_16BIT     (0b01) << DMA_CCR_MSIZE_Pos
#define DMA_PSIZE_16BIT     (0b01) << DMA_CCR_PSIZE_Pos



#endif // DEFINES_H