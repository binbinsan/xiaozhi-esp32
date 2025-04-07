#include "iot/thing.h"
#include "board.h"
#include "esp_mqtt.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "wifi_board.h"
#include <esp_log.h>
#include <string>


#define TAG "SmartHome"
#define MQTT_TOPIC "smarthome/livingroom/led"
const char* DEFAULT_MQTT_BROKER = "home.ajk.life";
const int DEFAULT_MQTT_PORT = 1883;
EspMqtt* mqtt_client = nullptr;  // MQTT客户端实例


static EventGroupHandle_t wifi_event_group = xEventGroupCreate();
#define WIFI_CONNECTED_BIT BIT0
#define MQTT_RETRY_DELAY_MS 5000
static void wifi_event_handler(void* arg, esp_event_base_t base, int32_t id, void* data) {
    if (base == WIFI_EVENT) {
        if (id == WIFI_EVENT_STA_CONNECTED) {
            ESP_LOGI(TAG, "Wi-Fi Connected");
        } else if (id == WIFI_EVENT_STA_DISCONNECTED) {
            xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "Got IP Address");
    }
}
void InitMqttConnection() {
    // 从系统获取设备标识
    const std::string device_id = "xiaozhi";
    
    // 创建并配置MQTT客户端
    mqtt_client = new EspMqtt();
    const bool connect_result = mqtt_client->Connect(
        DEFAULT_MQTT_BROKER,
        DEFAULT_MQTT_PORT,
        "device_" + device_id,  // 客户端ID
        "binbin",             // 从设置读取
        "wb021102-"              // 从设置读取
    );

    // 连接结果处理
    if (connect_result) {
        ESP_LOGI(TAG, "MQTT Connected to %s:%d", 
               DEFAULT_MQTT_BROKER, DEFAULT_MQTT_PORT);
        

        // 订阅系统主题
        const std::string status_topic = "iot/" + device_id + "/status";
        if (mqtt_client->Subscribe(status_topic, 1)) {
            ESP_LOGI(TAG, "Subscribed to: %s", status_topic.c_str());
        }

        // 发布上线消息
        mqtt_client->Publish(status_topic, "online", 1);
    } else {
        ESP_LOGE(TAG, "MQTT Connection failed");
    }
}

void mqtt_task(void *pvParameters) {
    ESP_LOGI(TAG, "MQTT task started");

    // 等待 Wi-Fi 连接事件
    ESP_LOGI(TAG, "Waiting for Wi-Fi connection...");
    xEventGroupWaitBits(
        wifi_event_group,          // 事件组句柄
        WIFI_CONNECTED_BIT,        // 等待的位掩码
        pdFALSE,                   // 不清除事件位
        pdTRUE,                    // 等待所有指定位被置位
        portMAX_DELAY              // 无限期等待
    );
    ESP_LOGI(TAG, "Wi-Fi connected!");

    // 初始化 MQTT
    InitMqttConnection();

    vTaskDelete(NULL); // 任务完成，删除自身
}

namespace iot {

class SmartHome : public Thing {

private:
    bool livingroom_led_state = false;
    bool bedroom_led_state = false;
    bool kitchen_led_state = false;
public:
    // 构造函数接收已连接的MQTT客户端指针
    SmartHome() : Thing("SmartHome", "智慧家")
    {
        // auto& board = Board::GetInstance();
        // WifiBoard& wifi_board = static_cast<WifiBoard&>(board);

        // // 1. 创建MQTT客户端（建议在Board初始化阶段完成）
        // mqtt_client_ = wifi_board.mqtt_client;
        // if (!wifi_board.mqtt_connected) {
        //     ESP_LOGE(TAG, "MQTT连接未就绪");
        // }
        esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
        esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL);

        //MQTT客户端实例
        xTaskCreate(
            mqtt_task,   // 任务函数
            "mqtt_task",      // 任务名称
            4096,             // 堆栈大小
            NULL,             // 传递this指针
             5, // 优先级
            NULL
        );
        // 注册设备属性
        properties_.AddBooleanProperty("livingroom_led_state", "客厅灯状态",[this]{ return livingroom_led_state; });
        properties_.AddBooleanProperty("bedroom_led_state", "卧室灯状态",[this]{ return bedroom_led_state; });
        properties_.AddBooleanProperty("kitchen_led_state", "厨房灯状态",[this]{ return kitchen_led_state; });

        // 注册控制方法
        methods_.AddMethod("turn_on_livingroom_led_state","打开客厅灯",ParameterList(), [this](const ParameterList& parameters) {
            SetLightState("livingroom", 1);
        });
        methods_.AddMethod("turn_off_livingroom_led_state", "关闭客厅灯",ParameterList(), [this](const ParameterList& parameters) {
            SetLightState("livingroom", 0);
        });
        methods_.AddMethod("turn_on_bedroom_led_state","打开卧室灯",ParameterList(), [this](const ParameterList& parameters) {
            SetLightState("bedroom", 1);
        });
        methods_.AddMethod("turn_off_bedroom_led_state", "关闭卧室灯",ParameterList(), [this](const ParameterList& parameters) {
            SetLightState("bedroom", 0);
        });
        methods_.AddMethod("turn_on_kitchen_led_state","打开厨房灯",ParameterList(), [this](const ParameterList& parameters) {
            SetLightState("kitchen", 1);
        });
        methods_.AddMethod("turn_off_kitchen_led_state", "关闭厨房灯",ParameterList(), [this](const ParameterList& parameters) {
            SetLightState("kitchen", 0);
        });


    }

    // 设置灯光状态（带状态同步）
    void SetLightState(const std::string& room, bool state) {
        bool* led_state = nullptr;
        std::string topic = "smarthome/" + room + "/led";
        
        if (room == "livingroom") {
            led_state = &livingroom_led_state;
        } else if (room == "bedroom") {
            led_state = &bedroom_led_state;
        } else if (room == "kitchen") {
            led_state = &kitchen_led_state;
        }

        if (led_state && *led_state != state) {
            *led_state = state;
            
            // 状态变更时发送MQTT通知
            mqtt_client->Publish(topic, state ? "1" : "0");
            ESP_LOGI(TAG, "%s灯状态已更新: %s", room.c_str(), state ? "ON" : "OFF");
        }
    }
};

} // namespace iot

DECLARE_THING(SmartHome);
