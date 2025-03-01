#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>

#define AUDIO_INPUT_SAMPLE_RATE  16000
#define AUDIO_OUTPUT_SAMPLE_RATE 24000

#define AUDIO_I2S_MIC_GPIO_WS   GPIO_NUM_25
#define AUDIO_I2S_MIC_GPIO_SCK  GPIO_NUM_26
#define AUDIO_I2S_MIC_GPIO_DIN  GPIO_NUM_32
#define AUDIO_I2S_SPK_GPIO_DOUT GPIO_NUM_33
#define AUDIO_I2S_SPK_GPIO_BCLK GPIO_NUM_14
#define AUDIO_I2S_SPK_GPIO_LRCK GPIO_NUM_27


#define BOOT_BUTTON_GPIO        GPIO_NUM_0
#define ASR_BUTTON_GPIO         GPIO_NUM_13

#define DISPLAY_SDA_PIN         GPIO_NUM_NC
#define DISPLAY_SCL_PIN         GPIO_NUM_NC

#define DISPLAY_BACKLIGHT_PIN   GPIO_NUM_4
#define DISPLAY_SCLK_PIN    GPIO_NUM_18
#define DISPLAY_MOSI_PIN    GPIO_NUM_23
#define DISPLAY_CS_PIN      GPIO_NUM_5
#define DISPLAY_DC_PIN      GPIO_NUM_2
#define DISPLAY_RESET_PIN   GPIO_NUM_NC

#define DISPLAY_SPI_SCLK_HZ     (20 * 1000 * 1000)

#define LCD_TYPE_ST7789_SERIAL
#define DISPLAY_WIDTH   128
#define DISPLAY_HEIGHT  128
#define DISPLAY_MIRROR_X true
#define DISPLAY_MIRROR_Y true
#define DISPLAY_SWAP_XY false
#define DISPLAY_INVERT_COLOR  false
#define DISPLAY_RGB_ORDER  LCD_RGB_ELEMENT_ORDER_BGR
#define DISPLAY_OFFSET_X  2
#define DISPLAY_OFFSET_Y  3
#define DISPLAY_BACKLIGHT_OUTPUT_INVERT false
#define DISPLAY_SPI_MODE 0

#endif // _BOARD_CONFIG_H_
