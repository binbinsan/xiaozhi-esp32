#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>

#define AUDIO_INPUT_SAMPLE_RATE  16000
#define AUDIO_OUTPUT_SAMPLE_RATE 24000
#define AUDIO_I2S_MIC_GPIO_WS   GPIO_NUM_4
#define AUDIO_I2S_MIC_GPIO_SCK  GPIO_NUM_5
#define AUDIO_I2S_MIC_GPIO_DIN  GPIO_NUM_6
#define AUDIO_I2S_SPK_GPIO_DOUT GPIO_NUM_7
#define AUDIO_I2S_SPK_GPIO_BCLK GPIO_NUM_15
#define AUDIO_I2S_SPK_GPIO_LRCK GPIO_NUM_16


#define BUILTIN_LED_GPIO        GPIO_NUM_48
#define BOOT_BUTTON_GPIO        GPIO_NUM_0
#define VOLUME_UP_BUTTON_GPIO   GPIO_NUM_40
#define VOLUME_DOWN_BUTTON_GPIO GPIO_NUM_39

#define DISPLAY_OFFSET_X 0
#define DISPLAY_OFFSET_Y 0
#define DISPLAY_WIDTH    240
#define DISPLAY_HEIGHT   240
#define DISPLAY_SWAP_XY  false
#define DISPLAY_MIRROR_X false
#define DISPLAY_MIRROR_Y false

#define DISPLAY_BACKLIGHT_PIN GPIO_NUM_13
#define DISPLAY_BACKLIGHT_OUTPUT_INVERT true


// Pin Definitions 
#define LCD_NUM_CS GPIO_NUM_NC   //GND
#define LCD_NUM_DC GPIO_NUM_18
#define LCD_NUM_RD GPIO_NUM_NC  //3V3
#define LCD_NUM_WR GPIO_NUM_8
#define LCD_NUM_RST GPIO_NUM_NC //EN

#define GPIO_LCD_D0 GPIO_NUM_9
#define GPIO_LCD_D1 GPIO_NUM_10
#define GPIO_LCD_D2 GPIO_NUM_14
#define GPIO_LCD_D3 GPIO_NUM_21
#define GPIO_LCD_D4 GPIO_NUM_47
#define GPIO_LCD_D5 GPIO_NUM_45
#define GPIO_LCD_D6 GPIO_NUM_44
#define GPIO_LCD_D7 GPIO_NUM_43

#endif // _BOARD_CONFIG_H_
