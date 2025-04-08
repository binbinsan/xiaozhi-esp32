#include "ft6236.h"
#include <esp_log.h>

#define TAG "FT6236"

Ft6236::Ft6236(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr) {
    uint8_t chip_id = ReadReg(FT6236_REG_FOCALTECH_ID);
    ESP_LOGI(TAG, "FT6236 chip ID: 0x%02X", chip_id);
    
    uint8_t firmware_version = GetFirmwareVersion();
    ESP_LOGI(TAG, "FT6236 firmware version: 0x%02X", firmware_version);
    
    // 分配读取缓冲区
    read_buffer_ = new uint8_t[16];
}

Ft6236::~Ft6236() {
    if (read_buffer_) {
        delete[] read_buffer_;
        read_buffer_ = nullptr;
    }
}

void Ft6236::UpdateTouchPoint() {
    // 读取手势ID
    tp_.gesture = ReadReg(FT6236_REG_GEST_ID);
    
    // 读取触摸点状态和坐标
    ReadRegs(FT6236_REG_TD_STATUS, read_buffer_, 7);
    
    // 解析触摸点数据
    tp_.num = read_buffer_[0] & 0x0F;  // 触摸点数量在低4位
    
    if (tp_.num > 0) {
        // 解析第一个触摸点的X坐标 (高8位在P1_XH的低4位，低8位在P1_XL)
        uint8_t x_high = read_buffer_[1] & 0x0F;
        uint8_t x_low = read_buffer_[2];
        tp_.x = (x_high << 8) | x_low;
        
        // 解析第一个触摸点的Y坐标 (高8位在P1_YH的低4位，低8位在P1_YL)
        uint8_t y_high = read_buffer_[3] & 0x0F;
        uint8_t y_low = read_buffer_[4];
        tp_.y = (y_high << 8) | y_low;
        
        ESP_LOGD(TAG, "Touch point: num=%d, x=%d, y=%d, gesture=0x%02X", 
                tp_.num, tp_.x, tp_.y, tp_.gesture);
    } else {
        tp_.x = -1;
        tp_.y = -1;
    }
}

const Ft6236::TouchPoint_t& Ft6236::GetTouchPoint() {
    return tp_;
}

uint8_t Ft6236::GetFirmwareVersion() {
    return ReadReg(FT6236_REG_FIRMID);
}

uint8_t Ft6236::GetThreshold() {
    return ReadReg(FT6236_REG_TH_GROUP);
}

void Ft6236::SetThreshold(uint8_t threshold) {
    WriteReg(FT6236_REG_TH_GROUP, threshold);
} 