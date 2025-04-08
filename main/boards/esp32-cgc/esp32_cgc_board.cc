#include "wifi_board.h"
#include "audio_codecs/no_audio_codec.h"
#include "display/lcd_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "iot/thing_manager.h"
#include "led/single_led.h"
#include "i2c_device.h"
#include "ft6236.h"

#include <wifi_station.h>
#include <esp_log.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <driver/spi_common.h>
#include <driver/i2c.h>
#include <driver/ledc.h>
#include <esp_timer.h>
#include <cmath>

#if defined(LCD_TYPE_ILI9341_SERIAL)
#include "esp_lcd_ili9341.h"
#endif

#if defined(LCD_TYPE_GC9A01_SERIAL)
#include "esp_lcd_gc9a01.h"
static const gc9a01_lcd_init_cmd_t gc9107_lcd_init_cmds[] = {
    //  {cmd, { data }, data_size, delay_ms}
    {0xfe, (uint8_t[]){0x00}, 0, 0},
    {0xef, (uint8_t[]){0x00}, 0, 0},
    {0xb0, (uint8_t[]){0xc0}, 1, 0},
    {0xb1, (uint8_t[]){0x80}, 1, 0},
    {0xb2, (uint8_t[]){0x27}, 1, 0},
    {0xb3, (uint8_t[]){0x13}, 1, 0},
    {0xb6, (uint8_t[]){0x19}, 1, 0},
    {0xb7, (uint8_t[]){0x05}, 1, 0},
    {0xac, (uint8_t[]){0xc8}, 1, 0},
    {0xab, (uint8_t[]){0x0f}, 1, 0},
    {0x3a, (uint8_t[]){0x05}, 1, 0},
    {0xb4, (uint8_t[]){0x04}, 1, 0},
    {0xa8, (uint8_t[]){0x08}, 1, 0},
    {0xb8, (uint8_t[]){0x08}, 1, 0},
    {0xea, (uint8_t[]){0x02}, 1, 0},
    {0xe8, (uint8_t[]){0x2A}, 1, 0},
    {0xe9, (uint8_t[]){0x47}, 1, 0},
    {0xe7, (uint8_t[]){0x5f}, 1, 0},
    {0xc6, (uint8_t[]){0x21}, 1, 0},
    {0xc7, (uint8_t[]){0x15}, 1, 0},
    {0xf0,
    (uint8_t[]){0x1D, 0x38, 0x09, 0x4D, 0x92, 0x2F, 0x35, 0x52, 0x1E, 0x0C,
                0x04, 0x12, 0x14, 0x1f},
    14, 0},
    {0xf1,
    (uint8_t[]){0x16, 0x40, 0x1C, 0x54, 0xA9, 0x2D, 0x2E, 0x56, 0x10, 0x0D,
                0x0C, 0x1A, 0x14, 0x1E},
    14, 0},
    {0xf4, (uint8_t[]){0x00, 0x00, 0xFF}, 3, 0},
    {0xba, (uint8_t[]){0xFF, 0xFF}, 2, 0},
};
#endif
 
#define TAG "ESP32_CGC"

LV_FONT_DECLARE(font_puhui_14_1);
LV_FONT_DECLARE(font_awesome_14_1);

class ESP32_CGC : public WifiBoard {
private:
 
    Button boot_button_;
    LcdDisplay* display_;
    Button asr_button_;
    i2c_master_bus_handle_t i2c_bus_ = nullptr;
    Ft6236* touch_controller_ = nullptr;
    static int current_brightness_; // 添加静态变量跟踪当前亮度

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_MOSI_PIN;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = DISPLAY_SCLK_PIN;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeLcdDisplay() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;
        // 液晶屏控制IO初始化
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_DC_PIN;
        io_config.spi_mode = DISPLAY_SPI_MODE;
        io_config.pclk_hz = DISPLAY_SPI_SCLK_HZ;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        // 初始化液晶屏驱动芯片
        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RESET_PIN;
        panel_config.rgb_ele_order = DISPLAY_RGB_ORDER;
        panel_config.bits_per_pixel = 16;
#if defined(LCD_TYPE_ILI9341_SERIAL)
        ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(panel_io, &panel_config, &panel));
#elif defined(LCD_TYPE_GC9A01_SERIAL)
        ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(panel_io, &panel_config, &panel));
        gc9a01_vendor_config_t gc9107_vendor_config = {
            .init_cmds = gc9107_lcd_init_cmds,
            .init_cmds_size = sizeof(gc9107_lcd_init_cmds) / sizeof(gc9a01_lcd_init_cmd_t),
        };        
#else
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));
#endif
        
        esp_lcd_panel_reset(panel);
 

        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
#ifdef  LCD_TYPE_GC9A01_SERIAL
        panel_config.vendor_config = &gc9107_vendor_config;
#endif
        display_ = new SpiLcdDisplay(panel_io, panel,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY,
                                    {
                                        .text_font = &font_puhui_14_1,
                                        .icon_font = &font_awesome_14_1,
                                        .emoji_font = font_emoji_32_init(),
                                    });
    }


 
    void InitializeButtons() {
        
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                ResetWifiConfiguration();
            }
            app.ToggleChatState();
        });

        asr_button_.OnClick([this]() {
            std::string wake_word="你好小智";
            Application::GetInstance().WakeWordInvoke(wake_word);
        });

    }

    // 物联网初始化，添加对 AI 可见设备
    void InitializeIot() {
        auto& thing_manager = iot::ThingManager::GetInstance();
        thing_manager.AddThing(iot::CreateThing("Speaker"));
        thing_manager.AddThing(iot::CreateThing("Screen"));
        thing_manager.AddThing(iot::CreateThing("Lamp"));
        thing_manager.AddThing(iot::CreateThing("SmartHome"));
    }

    // 初始化I2C总线
    void InitializeI2c() {
        ESP_LOGI(TAG, "Initializing I2C bus for touch controller");
        
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = TOUCH_I2C_PORT,
            .sda_io_num = TOUCH_I2C_SDA_PIN,
            .scl_io_num = TOUCH_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));
        
        // 调试：检测I2C总线上的设备
        I2cDetect();
    }

    // I2C设备检测函数（调试用）
    void I2cDetect() {
        uint8_t address;
        printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
        for (int i = 0; i < 128; i += 16) {
            printf("%02x: ", i);
            for (int j = 0; j < 16; j++) {
                fflush(stdout);
                address = i + j;
                esp_err_t ret = i2c_master_probe(i2c_bus_, address, pdMS_TO_TICKS(200));
                if (ret == ESP_OK) {
                    printf("%02x ", address);
                } else if (ret == ESP_ERR_TIMEOUT) {
                    printf("UU ");
                } else {
                    printf("-- ");
                }
            }
            printf("\r\n");
        }
    }

    // 初始化触摸控制器
    void InitializeTouchController() {
        ESP_LOGI(TAG, "Initializing FT6236 touch controller");
        
        // 创建FT6236触摸控制器实例
        touch_controller_ = new Ft6236(i2c_bus_, TOUCH_FT6236_I2C_ADDR);
        
        // 创建触摸事件处理任务
        xTaskCreate(TouchControllerTask, "touch_task", 4096, this, 5, nullptr);
    }
    
    // 触摸控制器任务函数
    static void TouchControllerTask(void* param) {
        ESP32_CGC* board = static_cast<ESP32_CGC*>(param);
        Ft6236* touch_controller = board->GetTouchController();
        bool was_touched = false;
        
        // 上一次触摸点坐标
        int last_x = -1, last_y = -1;
        // 触摸开始时间
        uint32_t touch_start_time = 0;
        
        // 等待2秒让系统稳定
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        while (true) {
            // 更新触摸状态
            touch_controller->UpdateTouchPoint();
            auto tp = touch_controller->GetTouchPoint();
            auto& app = Application::GetInstance();
            
            if (tp.num > 0) {
                // 触摸按下或移动
                if (!was_touched) {
                    // 触摸开始
                    was_touched = true;
                    touch_start_time = esp_timer_get_time() / 1000; // 毫秒
                    last_x = tp.x;
                    last_y = tp.y;
                    
                    ESP_LOGI(TAG, "Touch start: x=%d, y=%d", tp.x, tp.y);
                } else if (TOUCH_ENABLE_SLIDE) {
                    // 触摸移动中，检测滑动手势
                    int dx = tp.x - last_x;
                    int dy = tp.y - last_y;
                    
                    // 计算滑动距离
                    int distance = sqrt(dx*dx + dy*dy);
                    
                    // 如果滑动距离超过阈值，处理滑动事件
                    if (distance > TOUCH_SLIDE_THRESHOLD) {
                        // 判断滑动方向
                        if (abs(dx) > abs(dy)) {
                            // 水平滑动
                            if (dx > 0) {
                                ESP_LOGI(TAG, "Slide right");
                                // 右滑事件处理
                                // 调整音量增加
                                auto codec = board->GetAudioCodec();
                                int volume = codec->output_volume() + 10;
                                if (volume > 100) volume = 100;
                                codec->SetOutputVolume(volume);
                                board->GetDisplay()->ShowNotification("音量: " + std::to_string(volume));
                            } else {
                                ESP_LOGI(TAG, "Slide left");
                                // 左滑事件处理
                                // 调整音量减小
                                auto codec = board->GetAudioCodec();
                                int volume = codec->output_volume() - 10;
                                if (volume < 0) volume = 0;
                                codec->SetOutputVolume(volume);
                                board->GetDisplay()->ShowNotification("音量: " + std::to_string(volume));
                            }
                        } else {
                            // 垂直滑动
                            if (dy > 0) {
                                ESP_LOGI(TAG, "Slide down");
                                // 下滑事件处理 - 调整屏幕亮度减小
                                auto backlight = board->GetBacklight();
                                current_brightness_ -= 10;
                                if (current_brightness_ < 10) current_brightness_ = 10;
                                backlight->SetBrightness(current_brightness_);
                                board->GetDisplay()->ShowNotification("亮度: " + std::to_string(current_brightness_));
                            } else {
                                ESP_LOGI(TAG, "Slide up");
                                // 上滑事件处理 - 调整屏幕亮度增加
                                auto backlight = board->GetBacklight();
                                current_brightness_ += 10;
                                if (current_brightness_ > 100) current_brightness_ = 100;
                                backlight->SetBrightness(current_brightness_);
                                board->GetDisplay()->ShowNotification("亮度: " + std::to_string(current_brightness_));
                            }
                        }
                        
                        // 更新最后触摸位置
                        last_x = tp.x;
                        last_y = tp.y;
                    }
                }
            } 
            // 触摸释放
            else if (was_touched) {
                was_touched = false;
                
                // 计算触摸持续时间
                uint32_t touch_duration = esp_timer_get_time() / 1000 - touch_start_time;
                
                // 如果是短触摸，视为点击
                if (TOUCH_ENABLE_CLICK && touch_duration < 500) {
                    ESP_LOGI(TAG, "Touch click: x=%d, y=%d", last_x, last_y);
                    // 处理点击事件
                    app.ToggleChatState();
                }
            }
            
            // 延时
            vTaskDelay(pdMS_TO_TICKS(TOUCH_UPDATE_INTERVAL));
        }
        
        vTaskDelete(NULL);
    }

public:
    ESP32_CGC() :
	boot_button_(BOOT_BUTTON_GPIO), asr_button_(ASR_BUTTON_GPIO) {
        InitializeSpi();
        InitializeLcdDisplay();
        InitializeI2c();
        InitializeTouchController();
        InitializeButtons();
        InitializeIot();
        GetBacklight()->RestoreBrightness();
        current_brightness_ = 50; // 设置初始亮度值
    }

    virtual AudioCodec* GetAudioCodec() override 
    {
#ifdef AUDIO_I2S_METHOD_SIMPLEX
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT, AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
#else
        static NoAudioCodecDuplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN);
#endif
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    virtual Backlight* GetBacklight() override {
        static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        return &backlight;
    }

    // 获取触摸控制器
    Ft6236* GetTouchController() {
        return touch_controller_;
    }
};

// 初始化静态变量
int ESP32_CGC::current_brightness_ = 50;

DECLARE_BOARD(ESP32_CGC);
