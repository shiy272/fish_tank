#include <WiFiClientSecure.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <WiFi.h>
#include <esp_camera.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <DHT.h>
#include <ElegantOTA.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <base64.h>
#include <WiFiClient.h>
#include <freertos/semphr.h>
#include <esp_task_wdt.h>
#include <Preferences.h>
#include <sys/time.h>

// 引入自定义库
#include "OLED_Fonts.h"
#include "CameraConfig.h"
#include "OLED_Driver.h"


// ========== 硬件引脚与参数定义 ==========
#define ONE_WIRE_BUS 21       // DS18B20数据引脚
#define TDS_PIN 14            // TDS水质传感器引脚
#define OLED_SDA 42           // OLED I2C SDA
#define OLED_SCL 41           // OLED I2C SCL
#define SERVO_PIN 38          // 舵机引脚
#define DHT11_PIN 2           // DHT11传感器引脚
#define DHT_TYPE DHT11        // DHT传感器类型
#define SERVO_DURATION 10000  // 舵机运行时间（毫秒）

// ========== WiFi配置 ==========
const char* ssid = "Xiaomi_F94A";     // WiFi名称
const char* password = "sy20122012";  // WiFi密码
// ========== 时间服务器配置 ==========
const char* NTP_SERVER_PRIMARY = "cn.pool.ntp.org";
const char* NTP_SERVER_BACKUP = "ntp.aliyun.com";


// ============== 视频流服务器 ==============
bool isStreaming = false;
unsigned long lastFrameSent = 0;
const int STREAM_INTERVAL = 200;  // 5 FPS (200ms)
const char* streamServer = "47.120.69.173";
const int streamPort = 5002;
WiFiClient streamClient;

WiFiClient persistentStreamClient;      // 持久化连接对象
bool persistentConnection = false;      // 连接状态标志
unsigned long lastConnectionCheck = 0;  // 上次连接检查时间

// ============== MQTT配置 ==============
const char* mqtt_server = "47.120.69.173";
const int mqtt_port = 8883;  // TLS端口
const char* mqtt_user = "esp32";
const char* mqtt_password = "39891120Liyu";
// 设备唯一标识
const char* device_id = "fish_tank_monitor_01";
// TLS证书 (替换为你的CA证书)
const char* ca_cert = R"EOF(
-----BEGIN CERTIFICATE-----
-----END CERTIFICATE-----
)EOF";

WiFiClientSecure espClient;
PubSubClient mqttClient(espClient);
// ================== 全局对象与状态结构体 ==================
WiFiUDP ntpUDP;                                              // NTP时间同步UDP对象
NTPClient timeClient(ntpUDP, NTP_SERVER_PRIMARY, 8 * 3600);  // NTP客户端，东八区
Servo feederServo;                                           // 舵机对象
AsyncWebServer server(80);                                   // Web服务器对象
int lastDay = -1;                                            // 上一次记录的日期
unsigned long lastCleanup = 0;                               // 记录上次清理时间
static unsigned long lastHeartbeat = 0;                      // MQTT心跳时间
DHT dht(DHT11_PIN, DHT_TYPE);
// OTA相关变量
unsigned long ota_progress_millis = 0;  // 记录OTA进度时间
bool camera_released = false;           // 摄像头资源是否已释放标志
const uint8_t EEPROM_VERSION = 1;       // 版本号
// ========== 函数前置声明 ==========
bool sendFeedStatus();
bool sendSensorDiscoveryConfig();
bool sendCameraDiscoveryConfig();
bool sendSwitchDiscoveryConfig();
void triggerFeeding();
void updateDisplay();
void sendSensorData();
// 传感器状态机枚举
enum SensorState {
  SENSOR_IDLE,
  SENSOR_DHT11_START,
  SENSOR_DHT11_WAIT,
  SENSOR_TDS_READ,
  SENSOR_WATER_TEMP_START,
  SENSOR_WATER_TEMP_WAIT
};
SensorState sensorState = SENSOR_IDLE;
unsigned long sensorTimer = 0;
// DS18B20温度传感器对象
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensor(&oneWire);
// 系统状态结构体，保存所有传感器与喂食状态
struct SystemStatus {
  uint32_t feedCount = 0;              // 喂食次数
  time_t lastFeedTime = 0;             // 上次喂食时间
  bool feeding = false;                // 是否正在喂食
  unsigned long feedingStartTime = 0;  // 喂食开始时间
  bool feedingCompleted = true;        // 喂食是否完成
  int envTemp = 0;                     // 环境温度
  int envHumidity = 0;                 // 环境湿度
  int tdsValue = 0;                    // TDS值
  float waterTemp = 0.0;               // 水温
} status;
struct SystemSettings {
  time_t lastFeedTime;     // 4字节
  uint32_t feedCount;      // 4字节
  uint32_t servoDuration;  // 4字节
} systemSettings;
// 增强时间同步函数
bool syncTimeWithRetry() {
  const int MAX_RETRIES = 3;

  for (int i = 0; i < MAX_RETRIES; i++) {
    if (timeClient.update()) {
      return true;
    }

    // 切换备用服务器
    if (i == 1) {
      Serial.println("切换到备用NTP服务器");
      timeClient.setPoolServerName(NTP_SERVER_BACKUP);
    }

    delay(1000);
    resetWatchdog();
  }

  return false;
}
// ========== Preferences配置 ==========
Preferences preferences;
// ========== Preferences操作函数 ==========
void loadSettings() {
  preferences.begin("fish_tank", true);  // false表示可读写

  // 检查版本号
  uint8_t storedVersion = preferences.getUChar("version", 0xFF);

  if (storedVersion != EEPROM_VERSION) {
    // 结构体初始化
    systemSettings.lastFeedTime = 0;
    systemSettings.feedCount = 0;
    systemSettings.servoDuration = SERVO_DURATION;
    saveSettings();
    preferences.end();
    Serial.println("Preferences版本不匹配，初始化为默认设置");
    return;
  }
  // 加载设置
  systemSettings.lastFeedTime = preferences.getULong("last_feed", 0);
  systemSettings.feedCount = preferences.getULong("feedCount", 0);
  systemSettings.servoDuration = preferences.getULong("servo_duration", SERVO_DURATION);  // 加载舵机运行时间

  preferences.end();

  Serial.printf("加载设置: 上次喂食时间=%lu, 喂食次数=%lu\n",
                systemSettings.lastFeedTime,
                systemSettings.feedCount);
  preferences.begin("fish_tank", true);
  unsigned long rawValue = preferences.getULong("servo_duration", 0);
  preferences.end();
  Serial.printf("喂食时长: %lu\n", rawValue);
}
void saveSettings() {
  preferences.begin("fish_tank", false);
  resetWatchdog();

  // 存储所有设置
  preferences.putUChar("version", EEPROM_VERSION);

  // 添加调试输出
  char formattedTime[30];
  if (systemSettings.lastFeedTime == 0) {
    snprintf(formattedTime, sizeof(formattedTime), "从未喂食");
  } else {
    time_t rawtime = systemSettings.lastFeedTime;
    struct tm* timeinfo = localtime(&rawtime);
    strftime(formattedTime, sizeof(formattedTime), "%Y-%m-%d %H:%M:%S", timeinfo);
  }


  preferences.putULong("last_feed", systemSettings.lastFeedTime);
  preferences.putULong("feedCount", systemSettings.feedCount);
  preferences.putULong("servo_duration", systemSettings.servoDuration);
  preferences.end();
  Serial.printf("储存设定: 上次喂食时间=%s, 喂食次数=%lu, 舵机运行时间=%lu\n",
                formattedTime,
                systemSettings.feedCount,
                systemSettings.servoDuration);
}

/******************** 辅助函数 ********************/
void displayMessage(const char* line1, const char* line2) {
  OLED_clear();
  int center1 = (OLED_COLUMN_NUMBER - strlen(line1) * 6) / 2;
  int center2 = (OLED_COLUMN_NUMBER - strlen(line2) * 6) / 2;
  draw_text(2, center1 > 0 ? center1 : 0, line1);
  draw_text(4, center2 > 0 ? center2 : 0, line2);
}


// ========== 看门狗配置 ==========
const int WDT_TIMEOUT = 8;  // 看门狗超时时间（秒）

// 新增：看门狗喂食函数
void resetWatchdog() {
  esp_task_wdt_reset();  // 重置看门狗计数器
}

// 新增：看门狗初始化函数
void initWatchdog() {
  // 首先禁用可能已经启用的看门狗
  esp_task_wdt_deinit();

  // 设置看门狗配置
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WDT_TIMEOUT * 1000,
    .trigger_panic = true
  };

  if (esp_task_wdt_init(&wdt_config) == ESP_OK) {
    // 添加当前任务到看门狗监控
    if (esp_task_wdt_add(xTaskGetCurrentTaskHandle()) == ESP_OK) {
      Serial.println("[INFO] 看门狗初始化成功");
    }
  }
}

// 新增：看门狗清理函数（可选）
void disableWatchdog() {
  esp_task_wdt_deinit();
}
/******************** MQTT函数 ********************/
// MQTT回调函数
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // 使用字符数组存储主题和消息
  char message[64];
  memset(message, 0, sizeof(message));
  for (int i = 0; i < length && i < sizeof(message) - 1; i++) {
    message[i] = (char)payload[i];
  }

  Serial.printf("MQTT消息 [%s]: %s\n", topic, message);

  // 使用字符串比较代替String对象
  if (strncmp(topic, "home/fish_tank/feed/set", 21) == 0) {
    if (strcmp(message, "ON") == 0) {
      triggerFeeding();
    }
  } else if (strcmp(topic, "home/fish_tank/servoDuration/set") == 0) {  // 新增：处理舵机运行时间设置
    long newDuration = atol(message);
    // 验证舵机运行时间有效性 (最小500ms，最大20000ms)
    if (newDuration >= 500 && newDuration <= 20000) {
      systemSettings.servoDuration = newDuration;
      saveSettings();
      Serial.printf("舵机运行时间设置为: %ld ms\n", newDuration);
      // 增加反馈信息
      char buffer[20];
      snprintf(buffer, sizeof(buffer), "%lu", newDuration);
      mqttClient.publish("home/fish_tank/servoDuration/state", buffer, true);
    } else {
      Serial.printf("无效的舵机运行时间: %ld (必须在500-20000ms之间)\n", newDuration);
      char buffer[20];
      snprintf(buffer, sizeof(buffer), "%lu", systemSettings.servoDuration);
      mqttClient.publish("home/fish_tank/servoDuration/state", buffer, true);
    }
  } else if (strcmp(topic, "home/fish_tank/camera/stream/set") == 0) {
    bool newState = (strcmp(message, "ON") == 0);
    if (newState != isStreaming) {
      isStreaming = newState;
      sensor_t* s = esp_camera_sensor_get();
    }
  } else if (strcmp(topic, "home/fish_tank/reset_settings") == 0) {
    resetSettingsToDefault();
  }
  mqttClient.publish("home/fish_tank/camera/stream/state",
                     isStreaming ? "ON" : "OFF", true);
}


// ============== 视频流发送函数 ==============
void sendVideoStream() {
  if (!isStreaming) return;

  unsigned long currentTime = millis();
  if (currentTime - lastFrameSent < STREAM_INTERVAL) return;

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    resetWatchdog();
    return;
  }

  // 构建完整的MJPEG帧
  String frameHeader = "--frame\r\n";
  frameHeader += "Content-Type: image/jpeg\r\n";
  frameHeader += "Content-Length: " + String(fb->len) + "\r\n\r\n";

  // 计算帧总长度
  size_t totalFrameSize = frameHeader.length() + fb->len + 2;  // +2 for trailing \r\n

  // 创建新连接
  WiFiClient client;
  if (!client.connect(streamServer, streamPort)) {
    Serial.println("Connection failed");
    esp_camera_fb_return(fb);
    resetWatchdog();
    return;
  }

  // 发送POST请求头
  String httpHeader = "POST /video_feed HTTP/1.1\r\n";
  httpHeader += "Host: " + String(streamServer) + "\r\n";
  httpHeader += "Connection: close\r\n";
  httpHeader += "Content-Type: multipart/x-mixed-replace\r\n";
  httpHeader += "Content-Length: " + String(totalFrameSize) + "\r\n\r\n";

  if (client.print(httpHeader) != httpHeader.length()) {
    Serial.println("HTTP header send failed");
    client.stop();
    esp_camera_fb_return(fb);
    return;
  }
  resetWatchdog();

  // 发送帧头
  if (client.print(frameHeader) != frameHeader.length()) {
    Serial.println("Frame header send failed");
    client.stop();
    esp_camera_fb_return(fb);
    resetWatchdog();
    return;
  }
  resetWatchdog();

  // 发送图像数据
  size_t dataSent = client.write(fb->buf, fb->len);

  // 发送帧结束标记
  client.print("\r\n");
  resetWatchdog();


  // 清理
  client.stop();
  esp_camera_fb_return(fb);
  resetWatchdog();
  lastFrameSent = currentTime;
}

/******************** 增强版MQTT连接管理 ********************/
void setupMqtt() {
  espClient.setInsecure();
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
}

bool reconnectMqtt() {
  static unsigned long lastAttempt = 0;
  static int retryCount = 0;

  // 如果已连接则返回true
  if (mqttClient.connected()) {
    return true;
  }
  // WiFi未连接则返回false
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[MQTT] WiFi未连接，无法连接MQTT");
    retryCount = 0;  // 重置重试计数器
    return false;
  }

  // 指数退避重试
  unsigned long currentTime = millis();
  unsigned long delayTime = min(5000 * (retryCount + 1), 60000);  // 最大重试间隔60秒

  if (currentTime - lastAttempt < delayTime) {
    return false;
  }

  Serial.printf("[MQTT] 尝试连接(重试 %d)...", ++retryCount);
  resetWatchdog();

  // 使用遗嘱信息建立连接
  bool connected = mqttClient.connect(
    device_id,
    mqtt_user,
    mqtt_password,
    "home/fish_tank/status",  // 遗嘱主题
    1,                        // QoS
    true,                     // retained
    "offline"                 // 遗嘱消息
  );

  if (connected) {
    Serial.println("成功");
    retryCount = 0;
    // 订阅主题
    mqttClient.subscribe("home/fish_tank/camera/stream/set", 1);
    mqttClient.subscribe("home/fish_tank/feed/set", 1);
    mqttClient.subscribe("home/fish_tank/reset_settings", 1);
    mqttClient.subscribe("home/fish_tank/servoDuration/set", 1);
    sendSensorData();
    return true;  // 连接成功返回true
  } else {
    Serial.printf("失败, 错误码: %d\n", mqttClient.state());
    resetWatchdog();
    lastAttempt = millis();  // 更新最后尝试时间
    // 增加网络复位逻辑
    if (retryCount > 5) {
      WiFi.disconnect(true);
      delay(1000);
      WiFi.begin(ssid, password);
      retryCount = 0;
    }

    return false;
  }
}
/******************** 时间处理安全版 ********************/
String getLastFeedTimeString() {
  if (status.lastFeedTime == 0) return "从未喂食";

  struct tm* tm_info;
  time_t rawtime = status.lastFeedTime;
  tm_info = localtime(&rawtime);

  char buffer[20];
  strftime(buffer, sizeof(buffer), "%H:%M:%S", tm_info);
  return String(buffer);
}
/******************** WiFi管理 ********************/
void connectWiFi() {
  WiFi.begin(ssid, password);
  OLED_clear();
  draw_text(0, 0, "Connecting:");
  draw_text(1, 0, ssid);
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries++ < 20) {
    draw_char(2, retries * 6, CHAR_BLOCK);  // 显示实心方块
    delay(500);
  }
  if (WiFi.status() == WL_CONNECTED) {
    char ipStr[20];
    IPAddress ip = WiFi.localIP();
    snprintf(ipStr, sizeof(ipStr), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
    displayMessage("Connected", ipStr);
    delay(2000);
  } else {
    displayMessage("Connect Failed", "Check Config");
    while (1) delay(1000);  // 停止执行
  }
  resetWatchdog();
}
/******************** 喂食控制 ********************/
void triggerFeeding() {
  // 确保时间有效
  if (timeClient.getEpochTime() < 1609459200) {  // 2021-01-01 00:00:00
    Serial.println("系统时间无效，无法记录喂食");
    return;
  }
  status.feeding = true;
  status.feedingCompleted = false;
  status.feedCount++;

  // 获取当前时间并确保有效
  time_t currentTime = timeClient.getEpochTime();
  if (currentTime > 1609459200) {  // 有效时间检查
    status.lastFeedTime = currentTime;
    systemSettings.lastFeedTime = currentTime;
  } else {
    Serial.println("获取到无效时间，使用0代替");
    status.lastFeedTime = 0;
    systemSettings.lastFeedTime = 0;
  }

  status.feedingStartTime = millis();

  // 保存到EEPROM
  systemSettings.feedCount = status.feedCount;
  saveSettings();
  resetWatchdog();

  feederServo.write(60);
  Serial.println("开始喂食...");
}
/******************** 传感器读取 ********************/
// 预计算系数
const float TDS_K = 0.115;  // TDS传感器的校准系数
void readSensorsNonBlocking() {


  static unsigned long sensorStartTime = 0;
  static int sensorStep = 0;
  static int rawTdsValue = 0;
  static float tempCelsius = 0.0;

  switch (sensorStep) {
    case 0:  // 开始新的读取周期
      sensorStep = 1;
      sensorStartTime = millis();
      resetWatchdog();
      break;

    case 1:  // 启动DHT11读取
      {
        static unsigned long lastDHTRead = 0;
        static uint8_t timeoutCount = 0;

        // 确保两次读取间隔≥1秒
        if (millis() - lastDHTRead < 1000) {
          sensorStep = 2;
          break;
        }

        status.envHumidity = dht.readHumidity();
        status.envTemp = dht.readTemperature();
        lastDHTRead = millis();

        // 检查读取是否有效
        if (isnan(status.envHumidity) || isnan(status.envTemp)) {
          Serial.println("DHT11读取失败");
          timeoutCount++;

          if (timeoutCount > 3) {
            Serial.println("复位DHT11引脚...");
            pinMode(DHT11_PIN, OUTPUT);
            digitalWrite(DHT11_PIN, LOW);
            delay(20);
            pinMode(DHT11_PIN, INPUT_PULLUP);
            timeoutCount = 0;
          }
        } else {
          timeoutCount = 0;  // 重置超时计数
        }

        sensorStep = 2;
        break;
      }
    case 2:  // 读取TDS值
      rawTdsValue = analogRead(TDS_PIN);
      {
        float rawVoltage = rawTdsValue / 4095.0 * 3.3;
        
        status.tdsValue = rawVoltage * TDS_K * 1000; // 转换为ppm
      }
      sensorStep = 3;
      break;

    case 3:  // 启动温度传感器
      tempSensor.requestTemperatures();
      sensorStep = 4;
      sensorStartTime = millis();
      break;

    case 4:  // 温度修正和最终计算
      if (millis() - sensorStartTime > 750) {
        tempCelsius = tempSensor.getTempCByIndex(0);
        status.waterTemp = tempCelsius;

        Serial.print("传感器读取: ");
        Serial.print("水温=");
        Serial.print(tempCelsius);
        Serial.print("℃, TDS=");
        Serial.print(status.tdsValue);
        Serial.print(", 环境温度=");
        Serial.print(status.envTemp);
        Serial.print("℃, 湿度=");
        Serial.print(status.envHumidity);
        Serial.println("%");

        sensorStep = 0;
      }
      break;
  }
}
void updateDisplay() {
  static String lastTimeStr = "";
  static String lastWaterStr = "";
  static String lastEnvStr = "";
  static String lastStatusStr = "";
  static String lastMqttStatus = "";
  static String lastServoDurationStr = "";
  // 获取当前数据

  char mqttStatus[20];
  snprintf(mqttStatus, sizeof(mqttStatus), "HA: %s",
           mqttClient.connected() ? "Connected" : "Disconnected");

  char timeStr[20];
  snprintf(timeStr, sizeof(timeStr), "%s", timeClient.getFormattedTime().c_str());

  char servoDurationStr[20];
  float durationSec = systemSettings.servoDuration / 1000.0;
  snprintf(servoDurationStr, sizeof(servoDurationStr), "FeedDur: %.1fs", durationSec);

  char feedStr[50];
  if (status.feeding) {
    int remaining = (SERVO_DURATION - (millis() - status.feedingStartTime)) / 1000;
    snprintf(feedStr, sizeof(feedStr), "Feeding %ds", max(0, remaining));
  }

  char waterTempStr[30];
  snprintf(waterTempStr, sizeof(waterTempStr), "WT:%.1f°C TDS:%d", status.waterTemp, status.tdsValue);
  char envStr[30];
  snprintf(envStr, sizeof(envStr), "ET:%d°C EH:%d%%", status.envTemp, status.envHumidity);

  char statusStr[20];
  if (WiFi.status() == WL_CONNECTED) {
    IPAddress ip = WiFi.localIP();
    snprintf(statusStr, sizeof(statusStr), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
  } else {
    snprintf(statusStr, sizeof(statusStr), "No IP");
  }
  // 只更新变化的内容
  if (String(timeStr) != lastTimeStr) {
    clearLine(0);  // 只清除这一行
    draw_text(0, 0, timeStr);
    lastTimeStr = String(timeStr);
  }
  if (String(waterTempStr) != lastWaterStr) {
    clearLine(2);
    draw_text(2, 0, waterTempStr);
    lastWaterStr = String(waterTempStr);
  }
  if (String(envStr) != lastEnvStr) {
    clearLine(3);
    draw_text(3, 0, envStr);
    lastEnvStr = String(envStr);
  }
  if (String(mqttStatus) != lastMqttStatus) {
    clearLine(4);
    draw_text(4, 0, mqttStatus);
    lastMqttStatus = String(mqttStatus);
  }
  if (String(servoDurationStr) != lastServoDurationStr) {
    clearLine(5);
    draw_text(5, 0, servoDurationStr);
    lastServoDurationStr = String(servoDurationStr);
  }
  if (String(statusStr) != lastStatusStr) {
    clearLine(6);
    draw_text(6, 0, statusStr);
    lastStatusStr = String(statusStr);
  }
}


/**************** OTA回调函数 ****************/
void onOTAStart() {
  Serial.println("OTA更新开始");
  // 增加摄像头资源释放逻辑
  if (camera_released == false) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) {
      esp_camera_fb_return(fb);
      camera_released = true;
      Serial.println("已释放摄像头资源");
    }
  }
  resetWatchdog();
}

void onOTAProgress(size_t current, size_t final) {
  // 每秒打印进度
  if (millis() - ota_progress_millis > 1000) {
    ota_progress_millis = millis();
    Serial.printf("OTA进度: %u/%u bytes (%.1f%%)\n",
                  current, final, current * 100.0 / final);

    // 低内存时二次释放资源
    if (ESP.getFreeHeap() < 50000 && !camera_released) {
      camera_fb_t* fb = esp_camera_fb_get();
      if (fb) esp_camera_fb_return(fb);
    }
  }
  resetWatchdog();
}

void onOTAEnd(bool success) {
  if (success) {
    Serial.println("OTA更新成功，即将重启");
  } else {
    Serial.println("OTA更新失败");
    // 恢复摄像头功能
    if (camera_released) {
      // 修改：正确使用getCameraConfig函数
      camera_config_t config = getCameraConfig();  // 先获取配置
      esp_err_t err = esp_camera_init(&config);    // 然后取其地址传递
      if (err == ESP_OK) Serial.println("摄像头重新初始化成功");
    }
  }
  camera_released = false;
  resetWatchdog();
}


//发送mqtt数据
void sendSensorData() {
  if (!mqttClient.connected()) return;

  char buffer[32];
  char state_topic[100];
  //舵机运行时间状态主题
  snprintf(buffer, sizeof(buffer), "%u", (unsigned int)systemSettings.servoDuration);
  mqttClient.publish("home/fish_tank/servoDuration/state", buffer, true);
  // 水温传感器状态主题
  snprintf(state_topic, sizeof(state_topic), "home/fish_tank/sensor/water_temp/state");
  dtostrf(status.waterTemp, 4, 1, buffer);
  mqttClient.publish(state_topic, buffer, true);

  // 环境温度
  snprintf(buffer, sizeof(buffer), "%d", status.envTemp);
  mqttClient.publish("home/fish_tank/sensor/temperature/state", buffer, true);

  // 环境湿度
  snprintf(buffer, sizeof(buffer), "%d", status.envHumidity);
  mqttClient.publish("home/fish_tank/sensor/humidity/state", buffer, true);

  // TDS值
  snprintf(buffer, sizeof(buffer), "%d", status.tdsValue);
  mqttClient.publish("home/fish_tank/sensor/tds/state", buffer, true);

  // 喂食次数
  snprintf(buffer, sizeof(buffer), "%d", systemSettings.feedCount);
  mqttClient.publish("home/fish_tank/sensor/feed_count/state", buffer, true);

  // 最后喂食时间
  snprintf(buffer, sizeof(buffer), "%u", (unsigned int)systemSettings.lastFeedTime);
  mqttClient.publish("home/fish_tank/sensor/feed_lastFeedTime/state", buffer, true);
  // 喂食开关状态
  mqttClient.publish("home/fish_tank/feed/state",
                     status.feeding ? "ON" : "OFF", true);
  // 视频流状态
  mqttClient.publish("home/fish_tank/camera/stream/state",
                     isStreaming ? "ON" : "OFF", true);
  resetWatchdog();
}
// 巴法云MQTT配置
const char* bemfa_server = "bemfa.com";
const int bemfa_port = 9501;
const char* bemfa_uid = "d69d90a3ad5349449ddab3a6c734599a";  // 开发者密钥
const char* bemfa_topic = "home004";

unsigned long bemfaReconnectTime = 0;
const unsigned long BEMFA_RETRY_INTERVAL = 5000;

WiFiClient bemfa_wifi_client;
PubSubClient bemfa_client(bemfa_wifi_client);

void initBemfaMQTT() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("等待WiFi连接...");
    return;
  }

  bemfa_client.setServer(bemfa_server, bemfa_port);

  Serial.println("连接巴法云...");
  resetWatchdog();
  if (bemfa_client.connect(bemfa_uid)) {  // 仅使用开发者密钥
    Serial.println("连接成功");
    bemfa_client.subscribe(bemfa_topic);  // 订阅主题
  } else {
    Serial.print("连接失败，错误: ");
    Serial.println(bemfa_client.state());
    bemfaReconnectTime = millis() + BEMFA_RETRY_INTERVAL;
  }
}

void loopBemfaMQTT() {
  if (!bemfa_client.connected()) {
    if (millis() > bemfaReconnectTime) {
      initBemfaMQTT();
    }
  } else {
    bemfa_client.loop();
  }
}

void sendBemfaData(float waterTemp, float tdsValue) {
  if (!bemfa_client.connected()) {
    Serial.println("MQTT未连接，跳过发送");
    resetWatchdog();
    return;
  }

  // 使用官方推荐格式：以#分隔的值
  char payload[50];
  snprintf(payload, sizeof(payload), "#%.1f#%.1f#",
           waterTemp, tdsValue);

  if (bemfa_client.publish(bemfa_topic, payload)) {
    Serial.print("发送成功: ");
    resetWatchdog();
    Serial.println(payload);
  } else {
    Serial.println("发送失败");
  }
}
void resetSettingsToDefault() {
  preferences.begin("fish_tank", false);
  preferences.clear();  // 清除所有设置
  preferences.end();

  // 重新初始化设置
  systemSettings.lastFeedTime = 0;
  systemSettings.feedCount = 0;
  systemSettings.servoDuration = SERVO_DURATION;
  saveSettings();
  Serial.println("已重置所有设置为默认值");
}
void handleSerialCommand(String command) {
  if (command == "load_settings") {
    loadSettings();  // 从Preferences加载设置
    Serial.println("设置已加载");
  } else {
    Serial.println("未知命令");
  }
}
void setup() {
  delay(3000);  // 给硬件上电稳定时间
  Serial.begin(115200);
  delay(1000);
  initWatchdog();  // 初始化看门狗
  // 添加初始喂狗
  resetWatchdog();

  pinMode(OLED_SCL, OUTPUT);
  pinMode(OLED_SDA, OUTPUT);
  digitalWrite(OLED_SCL, LOW);
  digitalWrite(OLED_SDA, LOW);
  pinMode(19, INPUT_PULLUP);       // 按键引脚初始化
  status.feeding = false;          // 确保喂食状态初始为 false
  status.feedingCompleted = true;  //确保喂食完成状态初始为 true
  OLED_init();
  OLED_clear();
  connectWiFi();
  resetWatchdog();

  timeClient.begin();
  timeClient.setTimeOffset(8 * 3600);  // 设置时区偏移（北京时间）

  if (!syncTimeWithRetry()) {
    Serial.println("NTP同步失败，使用默认时间");
    resetWatchdog();
    // 替代方案：直接设置系统时间
    struct tm timeinfo = { 0 };
    timeinfo.tm_year = 123;  // 2023 - 1900
    timeinfo.tm_mon = 0;     // 一月 (0-11)
    timeinfo.tm_mday = 1;    // 1号
    timeinfo.tm_hour = 8;    // 北京时间8点
    time_t defaultTime = mktime(&timeinfo);

    // 使用标准C库函数设置系统时间
    struct timeval tv;
    tv.tv_sec = defaultTime;
    tv.tv_usec = 0;
    settimeofday(&tv, NULL);

    Serial.println("已设置默认时间: 2023-01-01 08:00:00");

    // 强制更新NTP客户端
    timeClient.forceUpdate();
  } else {
    Serial.printf("时间同步成功: %s\n", timeClient.getFormattedTime().c_str());
  }
  // 温度传感器
  tempSensor.begin();

  // TDS传感器
  analogReadResolution(12);
  pinMode(TDS_PIN, INPUT);

  // DHT11传感器
  dht.begin();
  pinMode(DHT11_PIN, INPUT_PULLUP);

  // 舵机
  feederServo.attach(SERVO_PIN);
  feederServo.write(90);
  delay(500);

  // 优化WiFi性能
  WiFi.setSleep(false);
  // 初始化MQTT
  setupMqtt();
  resetWatchdog();
  // 加载持久化设置
  loadSettings();

  // 初始化巴法云MQTT连接
  initBemfaMQTT();
  resetWatchdog();

  // 添加视频流状态初始化
  lastFrameSent = 0;
  isStreaming = false;

  ElegantOTA.begin(&server);
  // 设置回调
  ElegantOTA.onStart(onOTAStart);
  ElegantOTA.onProgress(onOTAProgress);
  ElegantOTA.onEnd(onOTAEnd);

  server.begin();
  Serial.printf("OTA更新地址: http://%s/update\n", WiFi.localIP().toString().c_str());
  resetWatchdog();
  feederServo.attach(SERVO_PIN);
  feederServo.write(90);  // 初始位置为90度（复位位置）


  if (psramFound()) {
    camera_config_t config = getCameraConfig();
    config.fb_location = CAMERA_FB_IN_PSRAM;
    Serial.println("Using PSRAM");

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
      displayMessage("Camera Error", "Check Wiring");
      Serial.printf("Camera init failed: 0x%x\n", err);
      delay(5000);
      ESP.restart();
    }
  } else {
    camera_config_t config = getCameraConfig();
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
      displayMessage("Camera Error", "Check Wiring");
      Serial.printf("Camera init failed: 0x%x\n", err);
      delay(5000);
      ESP.restart();
    }
  }

  // 摄像头翻转180
  sensor_t* s = esp_camera_sensor_get();
  if (s != NULL) {
    s->set_vflip(s, 1);    // 垂直翻转
    s->set_hmirror(s, 1);  // 水平镜像
    Serial.println("摄像头已设置为180度翻转");
  }

  resetWatchdog();
  // 显示系统准备就绪
  displayMessage("System Ready", WiFi.localIP().toString().c_str());

  // WiFi事件处理
  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
    if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
      Serial.println("WiFi断开，尝试重连...");
      WiFi.reconnect();
      resetWatchdog();
    }
  });
}

void loop() {
  static unsigned long lastLoopEntry = 0;
  if (millis() - lastLoopEntry > 5000) {
    Serial.printf("Loop阻塞检测: %lums\n", millis() - lastLoopEntry);
  }
  lastLoopEntry = millis();
  resetWatchdog();

  // 按键喂食逻辑
  static bool buttonPressed = false;  // 新增：按钮按下状态标志
  static unsigned long lastButtonPress = 0;
  const unsigned long debounceTime = 50;  // 50ms防抖时间
  // 按钮检测（带防抖）
  if (digitalRead(19) == LOW && (millis() - lastButtonPress) > debounceTime) {
    buttonPressed = true;  // 标记按钮已按下
    lastButtonPress = millis();
  }
  if (buttonPressed && digitalRead(19) == HIGH) {
    triggerFeeding();
    resetWatchdog();
    buttonPressed = false;  // 重置状态
  }

  ElegantOTA.loop();

  // 处理摄像头流
  if (isStreaming) {
    sendVideoStream();
    resetWatchdog();
  }
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    handleSerialCommand(command);
  }
  // 检查内存使用情况
  static unsigned long lastMemoryCheck = 0;
  if (millis() - lastMemoryCheck > 10000) {
    if (ESP.getFreeHeap() < 30000) {
      Serial.printf("内存不足: %d bytes, 清理中...\n", ESP.getFreeHeap());
      if (streamClient.connected()) streamClient.stop();
      // 新增：强制垃圾回收
      heap_caps_print_heap_info(MALLOC_CAP_8BIT);
      delay(100);
    }
    lastMemoryCheck = millis();
    resetWatchdog();
  }
  // 每小时强制更新时间
  static unsigned long lastTimeCheck = 0;
  if (millis() - lastTimeCheck > 3600000) {  // 30分钟
    if (!timeClient.update()) {
      Serial.println("定期时间同步失败，尝试重新同步...");
      syncTimeWithRetry();
    }
    lastTimeCheck = millis();
  }

  // 每小时持久化保存
  static unsigned long lastSaveTime = 0;
  if (millis() - lastSaveTime >= 3600000) {  // 每小时保存
    Serial.println("执行定时持久化保存");
    saveSettings();
    lastSaveTime = millis();
    resetWatchdog();  // 保存后立即喂狗
  }
  // 每10秒读取一次传感器
  static unsigned long lastSensorRead = 0;
  if (millis() - lastSensorRead > 5000) {
    readSensorsNonBlocking();
    lastSensorRead = millis();
  }

  // 每1秒更新一次显示（但只更新变化的内容）
  static unsigned long lastDisplayUpdate = 0;
  if (millis() - lastDisplayUpdate > 1000) {
    updateDisplay();
    lastDisplayUpdate = millis();
  }

  // 检查喂食状态，如果正在喂食且持续时间已到，则停止喂食
  if (status.feeding && !status.feedingCompleted) {
    if (millis() - status.feedingStartTime >= systemSettings.servoDuration) {
      feederServo.write(90);  // 复位舵机
      status.feedingCompleted = true;
      resetWatchdog();
      Serial.println("喂食完成");



      // 更新显示状态
      updateDisplay();

      // 重置状态
      delay(100);
      status.feeding = false;
    }
  }

  // 检测日期变化并重置喂食计数
  int currentDay = timeClient.getDay();  // 获取当前日期
  static int lastDay = -1;
  if (currentDay != lastDay) {  // 如果日期发生变化
    lastDay = currentDay;
    status.feedCount = 0;  // 重置喂食计数
    Serial.println("喂食计数已重置为 0");
    resetWatchdog();
  }

  // MQTT连接管理
  static unsigned long lastMqttPublish = 0;
  if (!reconnectMqtt()) {
    Serial.println("MQTT连接失败，将继续尝试连接...");
    resetWatchdog();
  } else {
    mqttClient.loop();

    // 每60秒发送传感器数据
    if (millis() - lastMqttPublish > 30000) {
      sendSensorData();
      lastMqttPublish = millis();
      resetWatchdog();
    }
  }

  // 维护巴法云MQTT连接
  loopBemfaMQTT();

  // 发送巴法云数据
  static unsigned long lastSend = 0;
  if (millis() - lastSend > 60000) {  // 每10秒发送一次
    sendBemfaData(status.waterTemp, status.tdsValue);
    lastSend = millis();
  }

  // 发送在线状态
  static unsigned long lastHeartbeat = 0;
  if (millis() - lastHeartbeat > 30000) {
    if (mqttClient.connected()) {
      mqttClient.publish("home/fish_tank/status", "online", true);  // 保留消息
    }
    lastHeartbeat = millis();
  }

  // 短暂延迟
  delay(100);
}
