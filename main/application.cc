#include "application.h"
#include "board.h"
#include "display.h"
#include "system_info.h"
#include "ml307_ssl_transport.h"
#include "audio_codec.h"
#include "mqtt_protocol.h"
#include "websocket_protocol.h"
#include "font_awesome_symbols.h"
#include "iot/thing_manager.h"
#include "assets/lang_config.h"

#include <cstring>
#include <esp_log.h>
#include <cJSON.h>
#include <driver/gpio.h>
#include <arpa/inet.h>
#include <esp_app_desc.h>

#define TAG "Application"


static const char* const STATE_STRINGS[] = {
    "unknown",
    "starting",
    "configuring",
    "idle",
    "connecting",
    "listening",
    "speaking",
    "upgrading",
    "activating",
    "fatal_error",
    "invalid_state"
};

Application::Application() {
    // 创建事件组，用于任务间通信
    event_group_ = xEventGroupCreate();
    // 创建后台任务，用于处理耗时操作
    background_task_ = new BackgroundTask(4096 * 8);

    // 创建时钟定时器，用于定期更新时间显示和状态检查
    esp_timer_create_args_t clock_timer_args = {
        .callback = [](void* arg) {
            Application* app = (Application*)arg;
            app->OnClockTimer();
        },
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "clock_timer",
        .skip_unhandled_events = true
    };
    esp_timer_create(&clock_timer_args, &clock_timer_handle_);
}

Application::~Application() {
    // 清理资源：停止并删除时钟定时器
    if (clock_timer_handle_ != nullptr) {
        esp_timer_stop(clock_timer_handle_);
        esp_timer_delete(clock_timer_handle_);
    }
    // 清理后台任务
    if (background_task_ != nullptr) {
        delete background_task_;
    }
    // 删除事件组
    vEventGroupDelete(event_group_);
}

void Application::CheckNewVersion() {
    auto& board = Board::GetInstance();
    auto display = board.GetDisplay();
    // 检查是否有新的固件版本可用
    ota_.SetPostData(board.GetJson());

    const int MAX_RETRY = 10;
    int retry_count = 0;

    while (true) {
        // 尝试检查版本，失败时进行重试
        if (!ota_.CheckVersion()) {
            retry_count++;
            if (retry_count >= MAX_RETRY) {
                ESP_LOGE(TAG, "Too many retries, exit version check");
                return;
            }
            ESP_LOGW(TAG, "Check new version failed, retry in %d seconds (%d/%d)", 60, retry_count, MAX_RETRY);
            vTaskDelay(pdMS_TO_TICKS(60000));
            continue;
        }
        retry_count = 0;

        // 如果有新版本，进行升级
        if (ota_.HasNewVersion()) {
            // 显示升级提示
            Alert(Lang::Strings::OTA_UPGRADE, Lang::Strings::UPGRADING, "happy", Lang::Sounds::P3_UPGRADE);
            // 等待设备状态变为空闲
            do {
                vTaskDelay(pdMS_TO_TICKS(3000));
            } while (GetDeviceState() != kDeviceStateIdle);

            // 使用主任务进行升级，不可取消
            Schedule([this, display]() {
                // 设置设备状态为升级中
                SetDeviceState(kDeviceStateUpgrading);
                
                display->SetIcon(FONT_AWESOME_DOWNLOAD);
                std::string message = std::string(Lang::Strings::NEW_VERSION) + ota_.GetFirmwareVersion();
                display->SetChatMessage("system", message.c_str());

                auto& board = Board::GetInstance();
                // 禁用省电模式
                board.SetPowerSaveMode(false);
#if CONFIG_USE_WAKE_WORD_DETECT
                wake_word_detect_.StopDetection();
#endif
                // 预先关闭音频输出，避免升级过程有音频操作
                auto codec = board.GetAudioCodec();
                codec->EnableInput(false);
                codec->EnableOutput(false);
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    audio_decode_queue_.clear();
                }
                background_task_->WaitForCompletion();
                delete background_task_;
                background_task_ = nullptr;
                vTaskDelay(pdMS_TO_TICKS(1000));

                // 开始升级，显示进度
                ota_.StartUpgrade([display](int progress, size_t speed) {
                    char buffer[64];
                    snprintf(buffer, sizeof(buffer), "%d%% %zuKB/s", progress, speed / 1024);
                    display->SetChatMessage("system", buffer);
                });

                // 如果升级成功，设备会重启，不会执行到这里
                // 如果执行到这里，说明升级失败
                display->SetStatus(Lang::Strings::UPGRADE_FAILED);
                ESP_LOGI(TAG, "Firmware upgrade failed...");
                vTaskDelay(pdMS_TO_TICKS(3000));
                Reboot();
            });

            return;
        }

        // 没有新版本，标记当前版本为有效
        ota_.MarkCurrentVersionValid();
        std::string message = std::string(Lang::Strings::VERSION) + ota_.GetCurrentVersion();
        display->ShowNotification(message.c_str());
    
        // 检查是否有激活码
        if (ota_.HasActivationCode()) {
            // 激活码有效
            SetDeviceState(kDeviceStateActivating);
            ShowActivationCode();

            // 60秒后或设备空闲时再次检查
            for (int i = 0; i < 60; ++i) {
                if (device_state_ == kDeviceStateIdle) {
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
            continue;
        }

        // 设置设备为空闲状态
        SetDeviceState(kDeviceStateIdle);
        display->SetChatMessage("system", "");
        PlaySound(Lang::Sounds::P3_SUCCESS);
        // 退出循环
        break;
    }
}

void Application::ShowActivationCode() {
    // 显示激活码和激活消息
    auto& message = ota_.GetActivationMessage();
    auto& code = ota_.GetActivationCode();

    // 数字语音映射
    struct digit_sound {
        char digit;
        const std::string_view& sound;
    };
    static const std::array<digit_sound, 10> digit_sounds{{
        digit_sound{'0', Lang::Sounds::P3_0},
        digit_sound{'1', Lang::Sounds::P3_1}, 
        digit_sound{'2', Lang::Sounds::P3_2},
        digit_sound{'3', Lang::Sounds::P3_3},
        digit_sound{'4', Lang::Sounds::P3_4},
        digit_sound{'5', Lang::Sounds::P3_5},
        digit_sound{'6', Lang::Sounds::P3_6},
        digit_sound{'7', Lang::Sounds::P3_7},
        digit_sound{'8', Lang::Sounds::P3_8},
        digit_sound{'9', Lang::Sounds::P3_9}
    }};

    // 播放激活提示语
    Alert(Lang::Strings::ACTIVATION, message.c_str(), "happy", Lang::Sounds::P3_ACTIVATION);
    vTaskDelay(pdMS_TO_TICKS(1000));
    background_task_->WaitForCompletion();

    // 播放激活码的每一位数字
    for (const auto& digit : code) {
        auto it = std::find_if(digit_sounds.begin(), digit_sounds.end(),
            [digit](const digit_sound& ds) { return ds.digit == digit; });
        if (it != digit_sounds.end()) {
            PlaySound(it->sound);
        }
    }
}

void Application::Alert(const char* status, const char* message, const char* emotion, const std::string_view& sound) {
    // 显示提示信息，并播放提示音
    ESP_LOGW(TAG, "Alert %s: %s [%s]", status, message, emotion);
    auto display = Board::GetInstance().GetDisplay();
    display->SetStatus(status);
    display->SetEmotion(emotion);
    display->SetChatMessage("system", message);
    if (!sound.empty()) {
        ResetDecoder();
        PlaySound(sound);
    }
}

void Application::DismissAlert() {
    // 如果设备处于空闲状态，清除提示信息
    if (device_state_ == kDeviceStateIdle) {
        auto display = Board::GetInstance().GetDisplay();
        display->SetStatus(Lang::Strings::STANDBY);
        display->SetEmotion("neutral");
        display->SetChatMessage("system", "");
    }
}

void Application::PlaySound(const std::string_view& sound) {
    // 设置解码的采样率和帧持续时间
    // 资源文件以16000Hz，60ms帧长度编码
    SetDecodeSampleRate(16000, 60);
    const char* data = sound.data();
    size_t size = sound.size();
    
    // 解析音频数据并添加到解码队列
    for (const char* p = data; p < data + size; ) {
        auto p3 = (BinaryProtocol3*)p;
        p += sizeof(BinaryProtocol3);

        auto payload_size = ntohs(p3->payload_size);
        std::vector<uint8_t> opus;
        opus.resize(payload_size);
        memcpy(opus.data(), p3->payload, payload_size);
        p += payload_size;

        std::lock_guard<std::mutex> lock(mutex_);
        audio_decode_queue_.emplace_back(std::move(opus));
    }
}

void Application::ToggleChatState() {
    if (device_state_ == kDeviceStateActivating) {
        SetDeviceState(kDeviceStateIdle);
        return;
    }

    if (!protocol_) {
        ESP_LOGE(TAG, "Protocol not initialized");
        return;
    }

    if (device_state_ == kDeviceStateIdle) {
        Schedule([this]() {
            SetDeviceState(kDeviceStateConnecting);
            if (!protocol_->OpenAudioChannel()) {
                return;
            }

            SetListeningMode(realtime_chat_enabled_ ? kListeningModeRealtime : kListeningModeAutoStop);
        });
    } else if (device_state_ == kDeviceStateSpeaking) {
        Schedule([this]() {
            AbortSpeaking(kAbortReasonNone);
        });
    } else if (device_state_ == kDeviceStateListening) {
        Schedule([this]() {
            protocol_->CloseAudioChannel();
        });
    }
}

void Application::StartListening() {
    if (device_state_ == kDeviceStateActivating) {
        SetDeviceState(kDeviceStateIdle);
        return;
    }

    if (!protocol_) {
        ESP_LOGE(TAG, "Protocol not initialized");
        return;
    }
    
    if (device_state_ == kDeviceStateIdle) {
        Schedule([this]() {
            if (!protocol_->IsAudioChannelOpened()) {
                SetDeviceState(kDeviceStateConnecting);
                if (!protocol_->OpenAudioChannel()) {
                    return;
                }
            }

            SetListeningMode(kListeningModeManualStop);
        });
    } else if (device_state_ == kDeviceStateSpeaking) {
        Schedule([this]() {
            AbortSpeaking(kAbortReasonNone);
            SetListeningMode(kListeningModeManualStop);
        });
    }
}

void Application::StopListening() {
    Schedule([this]() {
        if (device_state_ == kDeviceStateListening) {
            protocol_->SendStopListening();
            SetDeviceState(kDeviceStateIdle);
        }
    });
}

void Application::Start() {
    // 获取板级对象实例
    auto& board = Board::GetInstance();
    // 设置设备状态为启动中
    SetDeviceState(kDeviceStateStarting);

    /* 初始化显示器 */
    auto display = board.GetDisplay();

    /* 初始化音频编解码器 */
    auto codec = board.GetAudioCodec();
    // 创建Opus编解码器
    opus_decoder_ = std::make_unique<OpusDecoderWrapper>(codec->output_sample_rate(), 1, OPUS_FRAME_DURATION_MS);
    opus_encoder_ = std::make_unique<OpusEncoderWrapper>(16000, 1, OPUS_FRAME_DURATION_MS);
    // 根据设备类型和实时聊天模式设置编码器复杂度
    if (realtime_chat_enabled_) {
        ESP_LOGI(TAG, "Realtime chat enabled, setting opus encoder complexity to 0");
        opus_encoder_->SetComplexity(0);
    } else if (board.GetBoardType() == "ml307") {
        ESP_LOGI(TAG, "ML307 board detected, setting opus encoder complexity to 5");
        opus_encoder_->SetComplexity(5);
    } else {
        ESP_LOGI(TAG, "WiFi board detected, setting opus encoder complexity to 3");
        opus_encoder_->SetComplexity(3);
    }

    // 如果输入采样率不是16kHz，配置重采样器
    if (codec->input_sample_rate() != 16000) {
        input_resampler_.Configure(codec->input_sample_rate(), 16000);
        reference_resampler_.Configure(codec->input_sample_rate(), 16000);
    }
    // 启动编解码器
    codec->Start();

    // 创建音频循环任务，在核心0或1上运行（取决于是否启用实时聊天）
    xTaskCreatePinnedToCore([](void* arg) {
        Application* app = (Application*)arg;
        app->AudioLoop();
        vTaskDelete(NULL);
    }, "audio_loop", 4096 * 2, this, 8, &audio_loop_task_handle_, realtime_chat_enabled_ ? 1 : 0);

    /* 启动主循环任务 */
    xTaskCreatePinnedToCore([](void* arg) {
        Application* app = (Application*)arg;
        app->MainLoop();
        vTaskDelete(NULL);
    }, "main_loop", 4096 * 2, this, 4, &main_loop_task_handle_, 0);

    /* 等待网络就绪 */
    board.StartNetwork();

    // 初始化通信协议（WebSocket或MQTT）
    display->SetStatus(Lang::Strings::LOADING_PROTOCOL);
#ifdef CONFIG_CONNECTION_TYPE_WEBSOCKET
    protocol_ = std::make_unique<WebsocketProtocol>();
#else
    protocol_ = std::make_unique<MqttProtocol>();
#endif
    // 设置协议回调函数
    protocol_->OnNetworkError([this](const std::string& message) {
        // 网络错误回调
        SetDeviceState(kDeviceStateIdle);
        Alert(Lang::Strings::ERROR, message.c_str(), "sad", Lang::Sounds::P3_EXCLAMATION);
    });
    protocol_->OnIncomingAudio([this](std::vector<uint8_t>&& data) {
        // 接收音频数据回调
        std::lock_guard<std::mutex> lock(mutex_);
        audio_decode_queue_.emplace_back(std::move(data));
    });
    protocol_->OnAudioChannelOpened([this, codec, &board]() {
        // 音频通道打开回调
        board.SetPowerSaveMode(false);
        if (protocol_->server_sample_rate() != codec->output_sample_rate()) {
            ESP_LOGW(TAG, "Server sample rate %d does not match device output sample rate %d, resampling may cause distortion",
                protocol_->server_sample_rate(), codec->output_sample_rate());
        }
        SetDecodeSampleRate(protocol_->server_sample_rate(), protocol_->server_frame_duration());
        auto& thing_manager = iot::ThingManager::GetInstance();
        protocol_->SendIotDescriptors(thing_manager.GetDescriptorsJson());
        std::string states;
        if (thing_manager.GetStatesJson(states, false)) {
            protocol_->SendIotStates(states);
        }
    });
    protocol_->OnAudioChannelClosed([this, &board]() {
        // 音频通道关闭回调
        board.SetPowerSaveMode(true);
        Schedule([this]() {
            auto display = Board::GetInstance().GetDisplay();
            display->SetChatMessage("system", "");
            SetDeviceState(kDeviceStateIdle);
        });
    });
    protocol_->OnIncomingJson([this, display](const cJSON* root) {
        // 接收JSON数据回调
        // 解析JSON数据
        auto type = cJSON_GetObjectItem(root, "type");
        if (strcmp(type->valuestring, "tts") == 0) {
            // 文本到语音相关消息
            auto state = cJSON_GetObjectItem(root, "state");
            if (strcmp(state->valuestring, "start") == 0) {
                // TTS开始
                Schedule([this]() {
                    aborted_ = false;
                    if (device_state_ == kDeviceStateIdle || device_state_ == kDeviceStateListening) {
                        SetDeviceState(kDeviceStateSpeaking);
                    }
                });
            } else if (strcmp(state->valuestring, "stop") == 0) {
                // TTS结束
                Schedule([this]() {
                    background_task_->WaitForCompletion();
                    if (device_state_ == kDeviceStateSpeaking) {
                        if (listening_mode_ == kListeningModeManualStop) {
                            SetDeviceState(kDeviceStateIdle);
                        } else {
                            SetDeviceState(kDeviceStateListening);
                        }
                    }
                });
            } else if (strcmp(state->valuestring, "sentence_start") == 0) {
                // TTS句子开始
                auto text = cJSON_GetObjectItem(root, "text");
                if (text != NULL) {
                    ESP_LOGI(TAG, "<< %s", text->valuestring);
                    Schedule([this, display, message = std::string(text->valuestring)]() {
                        display->SetChatMessage("assistant", message.c_str());
                    });
                }
            }
        } else if (strcmp(type->valuestring, "stt") == 0) {
            // 语音到文本相关消息
            auto text = cJSON_GetObjectItem(root, "text");
            if (text != NULL) {
                ESP_LOGI(TAG, ">> %s", text->valuestring);
                Schedule([this, display, message = std::string(text->valuestring)]() {
                    display->SetChatMessage("user", message.c_str());
                });
            }
        } else if (strcmp(type->valuestring, "llm") == 0) {
            // 大语言模型相关消息
            auto emotion = cJSON_GetObjectItem(root, "emotion");
            if (emotion != NULL) {
                Schedule([this, display, emotion_str = std::string(emotion->valuestring)]() {
                    display->SetEmotion(emotion_str.c_str());
                });
            }
        } else if (strcmp(type->valuestring, "iot") == 0) {
            // 物联网命令
            auto commands = cJSON_GetObjectItem(root, "commands");
            if (commands != NULL) {
                auto& thing_manager = iot::ThingManager::GetInstance();
                for (int i = 0; i < cJSON_GetArraySize(commands); ++i) {
                    auto command = cJSON_GetArrayItem(commands, i);
                    thing_manager.Invoke(command);
                }
            }
        }
    });
    // 启动协议
    protocol_->Start();

    // 检查新固件版本或获取MQTT代理地址
    ota_.SetCheckVersionUrl(CONFIG_OTA_VERSION_URL);
    ota_.SetHeader("Device-Id", SystemInfo::GetMacAddress().c_str());
    ota_.SetHeader("Client-Id", board.GetUuid());
    ota_.SetHeader("Accept-Language", Lang::CODE);
    auto app_desc = esp_app_get_description();
    ota_.SetHeader("User-Agent", std::string(BOARD_NAME "/") + app_desc->version);

    // 创建检查新版本任务
    xTaskCreate([](void* arg) {
        Application* app = (Application*)arg;
        app->CheckNewVersion();
        vTaskDelete(NULL);
    }, "check_new_version", 4096 * 2, this, 2, nullptr);

#if CONFIG_USE_AUDIO_PROCESSOR
    // 初始化音频处理器
    audio_processor_.Initialize(codec, realtime_chat_enabled_);
    audio_processor_.OnOutput([this](std::vector<int16_t>&& data) {
        // 音频处理器输出回调
        background_task_->Schedule([this, data = std::move(data)]() mutable {
            opus_encoder_->Encode(std::move(data), [this](std::vector<uint8_t>&& opus) {
                Schedule([this, opus = std::move(opus)]() {
                    protocol_->SendAudio(opus);
                });
            });
        });
    });
    audio_processor_.OnVadStateChange([this](bool speaking) {
        // 语音活动检测状态变化回调
        if (device_state_ == kDeviceStateListening) {
            Schedule([this, speaking]() {
                if (speaking) {
                    voice_detected_ = true;
                } else {
                    voice_detected_ = false;
                }
                auto led = Board::GetInstance().GetLed();
                led->OnStateChanged();
            });
        }
    });
#endif

#if CONFIG_USE_WAKE_WORD_DETECT
    // 初始化唤醒词检测
    wake_word_detect_.Initialize(codec);
    wake_word_detect_.OnWakeWordDetected([this](const std::string& wake_word) {
        // 唤醒词检测到回调
        Schedule([this, &wake_word]() {
            if (device_state_ == kDeviceStateIdle) {
                // 设备空闲时，启动连接
                SetDeviceState(kDeviceStateConnecting);
                wake_word_detect_.EncodeWakeWordData();

                if (!protocol_->OpenAudioChannel()) {
                    wake_word_detect_.StartDetection();
                    return;
                }
                
                std::vector<uint8_t> opus;
                // 编码并发送唤醒词数据到服务器
                while (wake_word_detect_.GetWakeWordOpus(opus)) {
                    protocol_->SendAudio(opus);
                }
                // 发送唤醒词检测事件
                protocol_->SendWakeWordDetected(wake_word);
                ESP_LOGI(TAG, "Wake word detected: %s", wake_word.c_str());
                SetListeningMode(realtime_chat_enabled_ ? kListeningModeRealtime : kListeningModeAutoStop);
            } else if (device_state_ == kDeviceStateSpeaking) {
                // 设备说话时，中止说话
                AbortSpeaking(kAbortReasonWakeWordDetected);
            } else if (device_state_ == kDeviceStateActivating) {
                // 设备激活时，切换到空闲状态
                SetDeviceState(kDeviceStateIdle);
            }
        });
    });
    // 启动唤醒词检测
    wake_word_detect_.StartDetection();
#endif

    // 设置设备为空闲状态
    SetDeviceState(kDeviceStateIdle);
    // 启动定时器，周期为1秒
    esp_timer_start_periodic(clock_timer_handle_, 1000000);

#if 0
    // 调试代码：打印系统状态
    while (true) {
        SystemInfo::PrintRealTimeStats(pdMS_TO_TICKS(1000));
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
#endif
}

// 显示放大的时钟和日期
void Application::ShowClock() {
    if (!ota_.HasServerTime() || device_state_ != kDeviceStateIdle) {
        return;
    }

    auto display = Board::GetInstance().GetDisplay();
    
    // 获取当前时间
    time_t now = time(NULL);
    struct tm* time_info = localtime(&now);
    
    // 获取当前小时、分钟和秒
    int hour = time_info->tm_hour;
    int minute = time_info->tm_min;
    int second = time_info->tm_sec;
    
    // 在状态栏显示日期 (格式: DOW YYYY-MM-DD)
    char date_str[64];
    strftime(date_str, sizeof(date_str), "%a %Y-%m-%d", time_info);
    display->SetStatus(date_str);
    
    // 在聊天消息区域显示大字体时间，使用不同的显示风格
    char time_str[128];
    
    // 根据秒数循环显示不同的时钟样式
    int style = (second / 20) % 3;  // 每20秒切换一次样式，共3种样式
    
    switch (style) {
        case 0:
            // 样式1: 简单框架
            snprintf(time_str, sizeof(time_str), 
                     "+----------+\n"
                     "|  %02d:%02d:%02d  |\n"
                     "+----------+", 
                     hour, minute, second);
            break;
            
        case 1:
            // 样式2: 双行显示
            snprintf(time_str, sizeof(time_str), 
                     "TIME NOW:\n"
                     "==> %02d:%02d:%02d <==", 
                     hour, minute, second);
            break;
            
        case 2:
            // 样式3: 数字时钟风格
            snprintf(time_str, sizeof(time_str), 
                     "[%02d]:[%02d]:[%02d]\n"
                     "CURRENT TIME", 
                     hour, minute, second);
            break;
    }
    
    display->SetChatMessage("system", time_str);
    
    // 根据一天中的时间段选择不同表情
    if (hour >= 6 && hour < 12) {
        // 早晨
        display->SetEmotion("happy");
    } else if (hour >= 12 && hour < 18) {
        // 下午
        display->SetEmotion("neutral");
    } else {
        // 晚上
        display->SetEmotion("sleepy");
    }
}

void Application::OnClockTimer() {
    // 时钟计数递增
    clock_ticks_++;

    // 每秒检查一次是否需要更新时钟显示
    // 使用取模来每分钟更新一次时钟，或者在状态变化时更新
    time_t now = time(NULL);
    struct tm* time_info = localtime(&now);
    
    // 每分钟更新一次或状态刚变为空闲时更新
    static int last_minute = -1;
    if ((time_info->tm_min != last_minute && device_state_ == kDeviceStateIdle) || 
        (device_state_ == kDeviceStateIdle && last_device_state_ != kDeviceStateIdle)) {
        last_minute = time_info->tm_min;
        Schedule([this]() {
            ShowClock();
        });
    }
    last_device_state_ = device_state_;

    // 每10秒打印一次调试信息
    if (clock_ticks_ % 10 == 0) {
        // 打印内存使用情况
        int free_sram = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
        int min_free_sram = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
        ESP_LOGI(TAG, "Free internal: %u minimal internal: %u", free_sram, min_free_sram);
    }
}

// 添加异步任务到MainLoop
void Application::Schedule(std::function<void()> callback) {
    {
        // 将回调函数添加到任务队列
        std::lock_guard<std::mutex> lock(mutex_);
        main_tasks_.push_back(std::move(callback));
    }
    // 通知主循环有新任务
    xEventGroupSetBits(event_group_, SCHEDULE_EVENT);
}

// 主循环：控制聊天状态和网络连接
// 如果其他任务需要访问网络连接或聊天状态，
// 应该使用Schedule函数来调度
void Application::MainLoop() {
    while (true) {
        // 等待事件
        auto bits = xEventGroupWaitBits(event_group_, SCHEDULE_EVENT, pdTRUE, pdFALSE, portMAX_DELAY);

        if (bits & SCHEDULE_EVENT) {
            // 处理任务队列
            std::unique_lock<std::mutex> lock(mutex_);
            std::list<std::function<void()>> tasks = std::move(main_tasks_);
            lock.unlock();
            for (auto& task : tasks) {
                task();
            }
        }
    }
}

// The Audio Loop is used to input and output audio data
void Application::AudioLoop() {
    // 音频循环：处理音频输入和输出
    auto codec = Board::GetInstance().GetAudioCodec();
    while (true) {
        OnAudioInput();
        if (codec->output_enabled()) {
            OnAudioOutput();
        }
    }
}

void Application::OnAudioOutput() {
    // 当前时间
    auto now = std::chrono::steady_clock::now();
    auto codec = Board::GetInstance().GetAudioCodec();
    const int max_silence_seconds = 10;

    // 获取解码队列中的数据
    std::unique_lock<std::mutex> lock(mutex_);
    if (audio_decode_queue_.empty()) {
        // 如果队列为空且设备空闲，长时间无音频输出则关闭输出
        if (device_state_ == kDeviceStateIdle) {
            auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - last_output_time_).count();
            if (duration > max_silence_seconds) {
                codec->EnableOutput(false);
            }
        }
        return;
    }

    // 如果设备正在监听，清空解码队列
    if (device_state_ == kDeviceStateListening) {
        audio_decode_queue_.clear();
        return;
    }

    // 获取一帧音频数据
    auto opus = std::move(audio_decode_queue_.front());
    audio_decode_queue_.pop_front();
    lock.unlock();

    // 在后台任务中解码音频并输出
    background_task_->Schedule([this, codec, opus = std::move(opus)]() mutable {
        if (aborted_) {
            return;
        }

        // 解码opus数据为PCM
        std::vector<int16_t> pcm;
        if (!opus_decoder_->Decode(std::move(opus), pcm)) {
            return;
        }
        // 如果采样率不同，进行重采样
        if (opus_decoder_->sample_rate() != codec->output_sample_rate()) {
            int target_size = output_resampler_.GetOutputSamples(pcm.size());
            std::vector<int16_t> resampled(target_size);
            output_resampler_.Process(pcm.data(), pcm.size(), resampled.data());
            pcm = std::move(resampled);
        }
        // 输出音频数据
        codec->OutputData(pcm);
        last_output_time_ = std::chrono::steady_clock::now();
    });
}

void Application::OnAudioInput() {
    // 处理音频输入
    std::vector<int16_t> data;

#if CONFIG_USE_WAKE_WORD_DETECT
    // 如果唤醒词检测正在运行，进行唤醒词处理
    if (wake_word_detect_.IsDetectionRunning()) {
        ReadAudio(data, 16000, wake_word_detect_.GetFeedSize());
        wake_word_detect_.Feed(data);
        return;
    }
#endif
#if CONFIG_USE_AUDIO_PROCESSOR
    // 如果音频处理器正在运行，进行音频处理
    if (audio_processor_.IsRunning()) {
        ReadAudio(data, 16000, audio_processor_.GetFeedSize());
        audio_processor_.Feed(data);
        return;
    }
#else
    // 如果设备正在监听且没有音频处理器，直接编码并发送音频
    if (device_state_ == kDeviceStateListening) {
        ReadAudio(data, 16000, 30 * 16000 / 1000);
        background_task_->Schedule([this, data = std::move(data)]() mutable {
            opus_encoder_->Encode(std::move(data), [this](std::vector<uint8_t>&& opus) {
                Schedule([this, opus = std::move(opus)]() {
                    protocol_->SendAudio(opus);
                });
            });
        });
        return;
    }
#endif
    // 没有需要处理的音频输入，延时30ms
    vTaskDelay(pdMS_TO_TICKS(30));
}

void Application::ReadAudio(std::vector<int16_t>& data, int sample_rate, int samples) {
    // 读取音频数据并进行必要的重采样
    auto codec = Board::GetInstance().GetAudioCodec();
    // 如果编解码器采样率与目标采样率不匹配，需要重采样
    if (codec->input_sample_rate() != sample_rate) {
        data.resize(samples * codec->input_sample_rate() / sample_rate);
        if (!codec->InputData(data)) {
            return;
        }
        // 如果是双通道，需要分离通道并分别重采样
        if (codec->input_channels() == 2) {
            auto mic_channel = std::vector<int16_t>(data.size() / 2);
            auto reference_channel = std::vector<int16_t>(data.size() / 2);
            // 分离通道
            for (size_t i = 0, j = 0; i < mic_channel.size(); ++i, j += 2) {
                mic_channel[i] = data[j];
                reference_channel[i] = data[j + 1];
            }
            // 重采样两个通道
            auto resampled_mic = std::vector<int16_t>(input_resampler_.GetOutputSamples(mic_channel.size()));
            auto resampled_reference = std::vector<int16_t>(reference_resampler_.GetOutputSamples(reference_channel.size()));
            input_resampler_.Process(mic_channel.data(), mic_channel.size(), resampled_mic.data());
            reference_resampler_.Process(reference_channel.data(), reference_channel.size(), resampled_reference.data());
            // 重新组合通道
            data.resize(resampled_mic.size() + resampled_reference.size());
            for (size_t i = 0, j = 0; i < resampled_mic.size(); ++i, j += 2) {
                data[j] = resampled_mic[i];
                data[j + 1] = resampled_reference[i];
            }
        } else {
            // 单通道直接重采样
            auto resampled = std::vector<int16_t>(input_resampler_.GetOutputSamples(data.size()));
            input_resampler_.Process(data.data(), data.size(), resampled.data());
            data = std::move(resampled);
        }
    } else {
        // 采样率匹配，直接读取
        data.resize(samples);
        if (!codec->InputData(data)) {
            return;
        }
    }
}

void Application::AbortSpeaking(AbortReason reason) {
    ESP_LOGI(TAG, "Abort speaking");
    aborted_ = true;
    protocol_->SendAbortSpeaking(reason);
}

void Application::SetListeningMode(ListeningMode mode) {
    listening_mode_ = mode;
    SetDeviceState(kDeviceStateListening);
}

void Application::SetDeviceState(DeviceState state) {
    // 如果状态没有变化，直接返回
    if (device_state_ == state) {
        return;
    }
    
    // 重置时钟计数并记录上一个状态
    clock_ticks_ = 0;
    auto previous_state = device_state_;
    device_state_ = state;
    ESP_LOGI(TAG, "STATE: %s", STATE_STRINGS[device_state_]);
    // 状态发生变化，等待所有后台任务完成
    background_task_->WaitForCompletion();

    // 获取板级对象和显示器
    auto& board = Board::GetInstance();
    auto display = board.GetDisplay();
    auto led = board.GetLed();
    // 通知LED状态变化
    led->OnStateChanged();
    
    // 根据不同的状态执行相应操作
    switch (state) {
        case kDeviceStateUnknown:
        case kDeviceStateIdle:
            // 空闲状态：显示待机信息，停止音频处理，启动唤醒词检测
            if (ota_.HasServerTime()) {
                // 如果有服务器时间，显示时钟界面
                ShowClock();
            } else {
                display->SetStatus(Lang::Strings::STANDBY);
                display->SetEmotion("neutral");
                display->SetChatMessage("system", "");
            }
#if CONFIG_USE_AUDIO_PROCESSOR
            audio_processor_.Stop();
#endif
#if CONFIG_USE_WAKE_WORD_DETECT
            wake_word_detect_.StartDetection();
#endif
            break;
        case kDeviceStateConnecting:
            // 连接状态：显示连接信息
            display->SetStatus(Lang::Strings::CONNECTING);
            display->SetEmotion("neutral");
            display->SetChatMessage("system", "");
            break;
        case kDeviceStateListening:
            // 监听状态：显示监听信息，启动音频处理
            display->SetStatus(Lang::Strings::LISTENING);
            display->SetEmotion("neutral");

            // 在发送开始监听命令前更新IoT状态
            UpdateIotStates();

            // 确保音频处理器在运行
#if CONFIG_USE_AUDIO_PROCESSOR
            if (!audio_processor_.IsRunning()) {
#else
            if (true) {
#endif
                // 发送开始监听命令
                protocol_->SendStartListening(listening_mode_);
                if (listening_mode_ == kListeningModeAutoStop && previous_state == kDeviceStateSpeaking) {
                    // FIXME: 等待扬声器清空缓冲区
                    vTaskDelay(pdMS_TO_TICKS(120));
                }
                // 重置编码器状态
                opus_encoder_->ResetState();
#if CONFIG_USE_WAKE_WORD_DETECT
                // 停止唤醒词检测
                wake_word_detect_.StopDetection();
#endif
#if CONFIG_USE_AUDIO_PROCESSOR
                // 启动音频处理
                audio_processor_.Start();
#endif
            }
            break;
        case kDeviceStateSpeaking:
            // 说话状态：显示说话信息，停止音频处理，启动唤醒词检测（仅非实时模式）
            display->SetStatus(Lang::Strings::SPEAKING);

            if (listening_mode_ != kListeningModeRealtime) {
#if CONFIG_USE_AUDIO_PROCESSOR
                audio_processor_.Stop();
#endif
#if CONFIG_USE_WAKE_WORD_DETECT
                wake_word_detect_.StartDetection();
#endif
            }
            // 重置解码器
            ResetDecoder();
            break;
        default:
            // 其他状态不做特殊处理
            break;
    }
}

void Application::ResetDecoder() {
    // 重置解码器状态和清空解码队列
    std::lock_guard<std::mutex> lock(mutex_);
    opus_decoder_->ResetState();
    audio_decode_queue_.clear();
    last_output_time_ = std::chrono::steady_clock::now();
    
    // 启用音频输出
    auto codec = Board::GetInstance().GetAudioCodec();
    codec->EnableOutput(true);
}

void Application::SetDecodeSampleRate(int sample_rate, int frame_duration) {
    // 如果采样率和帧持续时间没有变化，直接返回
    if (opus_decoder_->sample_rate() == sample_rate && opus_decoder_->duration_ms() == frame_duration) {
        return;
    }

    // 重新创建解码器
    opus_decoder_.reset();
    opus_decoder_ = std::make_unique<OpusDecoderWrapper>(sample_rate, 1, frame_duration);

    // 如果解码器采样率与编解码器输出采样率不同，配置重采样器
    auto codec = Board::GetInstance().GetAudioCodec();
    if (opus_decoder_->sample_rate() != codec->output_sample_rate()) {
        ESP_LOGI(TAG, "Resampling audio from %d to %d", opus_decoder_->sample_rate(), codec->output_sample_rate());
        output_resampler_.Configure(opus_decoder_->sample_rate(), codec->output_sample_rate());
    }
}

void Application::UpdateIotStates() {
    auto& thing_manager = iot::ThingManager::GetInstance();
    std::string states;
    if (thing_manager.GetStatesJson(states, true)) {
        protocol_->SendIotStates(states);
    }
}

void Application::Reboot() {
    ESP_LOGI(TAG, "Rebooting...");
    esp_restart();
}

void Application::WakeWordInvoke(const std::string& wake_word) {
    if (device_state_ == kDeviceStateIdle) {
        ToggleChatState();
        Schedule([this, wake_word]() {
            if (protocol_) {
                protocol_->SendWakeWordDetected(wake_word); 
            }
        }); 
    } else if (device_state_ == kDeviceStateSpeaking) {
        Schedule([this]() {
            AbortSpeaking(kAbortReasonNone);
        });
    } else if (device_state_ == kDeviceStateListening) {   
        Schedule([this]() {
            if (protocol_) {
                protocol_->CloseAudioChannel();
            }
        });
    }
}

bool Application::CanEnterSleepMode() {
    if (device_state_ != kDeviceStateIdle) {
        return false;
    }

    if (protocol_ && protocol_->IsAudioChannelOpened()) {
        return false;
    }

    // Now it is safe to enter sleep mode
    return true;
}
