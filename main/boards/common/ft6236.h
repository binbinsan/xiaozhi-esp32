#ifndef FT6236_H
#define FT6236_H

#include "i2c_device.h"

// FT6236寄存器定义
#define FT6236_REG_DEV_MODE          0x00
#define FT6236_REG_GEST_ID           0x01
#define FT6236_REG_TD_STATUS         0x02
#define FT6236_REG_P1_XH             0x03
#define FT6236_REG_P1_XL             0x04
#define FT6236_REG_P1_YH             0x05
#define FT6236_REG_P1_YL             0x06
#define FT6236_REG_P1_WEIGHT         0x07
#define FT6236_REG_P1_MISC           0x08
#define FT6236_REG_P2_XH             0x09
#define FT6236_REG_P2_XL             0x0A
#define FT6236_REG_P2_YH             0x0B
#define FT6236_REG_P2_YL             0x0C
#define FT6236_REG_P2_WEIGHT         0x0D
#define FT6236_REG_P2_MISC           0x0E
#define FT6236_REG_TH_GROUP          0x80
#define FT6236_REG_PERIODACTIVE      0x88
#define FT6236_REG_LIB_VER_H         0xA1
#define FT6236_REG_LIB_VER_L         0xA2
#define FT6236_REG_CIPHER            0xA3
#define FT6236_REG_FIRMID            0xA6
#define FT6236_REG_FOCALTECH_ID      0xA8
#define FT6236_REG_RELEASE_CODE_ID   0xAF

// 手势ID定义
#define FT6236_GESTURE_NONE          0x00
#define FT6236_GESTURE_MOVE_UP       0x10
#define FT6236_GESTURE_MOVE_RIGHT    0x14
#define FT6236_GESTURE_MOVE_DOWN     0x18
#define FT6236_GESTURE_MOVE_LEFT     0x1C
#define FT6236_GESTURE_ZOOM_IN       0x48
#define FT6236_GESTURE_ZOOM_OUT      0x49

// FT6236默认I2C地址
#define FT6236_I2C_ADDR             0x38

class Ft6236 : public I2cDevice {
public:
    struct TouchPoint_t {
        int num = 0;         // 触摸点数量
        int x = -1;          // X坐标
        int y = -1;          // Y坐标
        uint8_t gesture = 0; // 手势类型
    };

    Ft6236(i2c_master_bus_handle_t i2c_bus, uint8_t addr = FT6236_I2C_ADDR);
    ~Ft6236();

    /**
     * @brief 更新触摸点数据
     */
    void UpdateTouchPoint();

    /**
     * @brief 获取当前触摸点数据
     * @return 当前触摸点信息
     */
    const Ft6236::TouchPoint_t &GetTouchPoint();

    /**
     * @brief 获取芯片固件版本
     * @return 固件版本号
     */
    uint8_t GetFirmwareVersion();

    /**
     * @brief 获取触摸面板阈值
     * @return 触摸阈值(0-255)
     */
    uint8_t GetThreshold();

    /**
     * @brief 设置触摸面板阈值
     * @param threshold 触摸阈值(0-255)
     */
    void SetThreshold(uint8_t threshold);

private:
    uint8_t *read_buffer_ = nullptr;
    TouchPoint_t tp_;
};

#endif // FT6236_H 