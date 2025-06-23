#include <WiFiClientSecure.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <WiFi.h>               // ESP32 WiFi库
#include <esp_camera.h>         // ESP32摄像头库
#include <OneWire.h>            // 单总线库（DS18B20）
#include <DallasTemperature.h>  // DS18B20温度传感器库
#include <Wire.h>               // I2C库
#include <ESP32Servo.h>         // 舵机库
#include <NTPClient.h>          // 网络时间库
#include <WiFiUdp.h>            // UDP库（NTP用）
#include <DHT11.h>              // DHT11温湿度传感器库
#include <ElegantOTA.h>
//进入<ElegantOTA.h>修改  #define ELEGANTOTA_USE_ASYNC_WEBSERVER 1（把0改为1）
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <base64.h>
#include <WiFiClient.h>
#include <freertos/semphr.h>
// ========== 硬件引脚与参数定义 ==========
#define ONE_WIRE_BUS 1        // DS18B20数据引脚
#define TDS_PIN 14            // TDS水质传感器引脚
#define OLED_SDA 42           // OLED I2C SDA
#define OLED_SCL 41           // OLED I2C SCL
#define SERVO_PIN 38          // 舵机引脚
#define DHT11_PIN 2           // DHT11传感器引脚
#define FEEDING_TIME1 7       // 第一次喂食时间（小时）
#define FEEDING_TIME2 18      // 第二次喂食时间（小时）
#define SERVO_DURATION 10000  // 舵机运行时间（毫秒）
// ========== WiFi配置 ==========
const char* ssid = "====";     // WiFi名称
const char* password = "====";  // WiFi密码
// ============== 视频流服务器 ==============
bool isStreaming = false;
unsigned long lastFrameSent = 0;
const int STREAM_INTERVAL = 200;  // 5 FPS (200ms)
const char* streamServer = "====";//服务器ip
const int streamPort = 5002;//端口号
WiFiClient streamClient;

// ============== MQTT配置 ==============
const char* mqtt_server = "====";//服务器ip
const int mqtt_port = 8883;  // TLS端口
const char* mqtt_user = "====";//MQTT账号
const char* mqtt_password = "====";//MQTT密码
// 设备唯一标识
const char* device_id = "fish_tank_monitor_01";
// TLS证书 (替换为你的CA证书)
const char* ca_cert = R"EOF(
-----BEGIN CERTIFICATE-----
====
-----END CERTIFICATE-----
)EOF";

WiFiClientSecure espClient;
PubSubClient mqttClient(espClient);
// ========== OLED显示参数 ==========
#define OLED_COLUMN_NUMBER 128
#define OLED_LINE_NUMBER 64
#define OLED_COLUMN_OFFSET 2
#define OLED_PAGE_NUMBER (OLED_LINE_NUMBER / 8)
// ========== 摄像头引脚配置 ==========
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 15
#define SIOD_GPIO_NUM 4
#define SIOC_GPIO_NUM 5
#define Y2_GPIO_NUM 11
#define Y3_GPIO_NUM 9
#define Y4_GPIO_NUM 8
#define Y5_GPIO_NUM 10
#define Y6_GPIO_NUM 12
#define Y7_GPIO_NUM 18
#define Y8_GPIO_NUM 17
#define Y9_GPIO_NUM 16
#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM 7
#define PCLK_GPIO_NUM 13
// ================== 全局对象与状态结构体 ==================
WiFiUDP ntpUDP;                                          // NTP时间同步UDP对象
NTPClient timeClient(ntpUDP, "pool.ntp.org", 8 * 3600);  // NTP客户端，东八区
Servo feederServo;                                       // 舵机对象
AsyncWebServer server(80);                               // Web服务器对象
DHT11 dht11(DHT11_PIN);                                  // DHT11对象
bool feedingScheduleEnabled = true;                      // 定时喂食开关
int feedingTime1 = FEEDING_TIME1;                        // 第一次喂食时间
int feedingTime2 = FEEDING_TIME2;                        // 第二次喂食时间
int lastDay = -1;                                        // 上一次记录的日期
String nextFeedTime = "";                                // 用于存储下次喂食时间
unsigned long lastCleanup = 0;                           // 记录上次清理时间
static unsigned long lastHeartbeat = 0;
// OTA相关变量
unsigned long ota_progress_millis = 0;
bool camera_released = false;
// ========== 函数前置声明 ==========
bool sendFeedStatus();
bool sendAutoDiscoveryConfig();
bool sendSensorDiscoveryConfig();
bool sendCameraDiscoveryConfig();
bool sendSwitchDiscoveryConfig();
String getLastFeedTimeString();
String getNextFeedingTime();
String getNextFeedingTimeShort();
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
  uint16_t feedCount = 0;              // 喂食次数
  time_t lastFeedTime = 0;             // 上次喂食时间
  bool feeding = false;                // 是否正在喂食
  unsigned long feedingStartTime = 0;  // 喂食开始时间
  bool feedingCompleted = false;       // 喂食是否完成
  int envTemp = 0;                     // 环境温度
  int envHumidity = 0;                 // 环境湿度
  int tdsValue = 0;                    // TDS值
  float waterTemp = 0.0;               // 水温
} status;
// ========== 摄像头配置结构体 ==========
camera_config_t cameraConfig = {
  .pin_pwdn = PWDN_GPIO_NUM,
  .pin_reset = RESET_GPIO_NUM,
  .pin_xclk = XCLK_GPIO_NUM,
  .pin_sccb_sda = SIOD_GPIO_NUM,
  .pin_sccb_scl = SIOC_GPIO_NUM,
  .pin_d7 = Y9_GPIO_NUM,
  .pin_d6 = Y8_GPIO_NUM,
  .pin_d5 = Y7_GPIO_NUM,
  .pin_d4 = Y6_GPIO_NUM,
  .pin_d3 = Y5_GPIO_NUM,
  .pin_d2 = Y4_GPIO_NUM,
  .pin_d1 = Y3_GPIO_NUM,
  .pin_d0 = Y2_GPIO_NUM,
  .pin_vsync = VSYNC_GPIO_NUM,
  .pin_href = HREF_GPIO_NUM,
  .pin_pclk = PCLK_GPIO_NUM,
  .xclk_freq_hz = 20000000,  // 外部时钟20MHz
  .ledc_timer = LEDC_TIMER_0,
  .ledc_channel = LEDC_CHANNEL_0,
  .pixel_format = PIXFORMAT_JPEG,  // JPEG格式
  .frame_size = FRAMESIZE_VGA,     // 分辨率
  .jpeg_quality = 12,              // JPEG画质（0-63，数值越小越清晰）
  .fb_count = 2                    // 帧缓冲数量
};
// OLED 显示函数和定义
const unsigned char OLED_init_cmd[25] = {
  0xAE, 0xD5, 0x80, 0xA8, 0X3F, 0xD3, 0X00, 0x40,
  0x8D, 0x14, 0x20, 0x02, 0xA1, 0xC8,
  0xDA, 0x12, 0x81, 0x66, 0xD9, 0xf1, 0xDB, 0x30, 0xA4, 0xA6,
  0xAF
};
const unsigned char font_5x7[][5] = {
  // 数字 0~9
  { 0x3E, 0x51, 0x49, 0x45, 0x3E },  // 0
  { 0x00, 0x42, 0x7F, 0x40, 0x00 },  // 1
  { 0x42, 0x61, 0x51, 0x49, 0x46 },  // 2
  { 0x21, 0x41, 0x45, 0x4B, 0x31 },  // 3
  { 0x18, 0x14, 0x12, 0x7F, 0x10 },  // 4
  { 0x27, 0x45, 0x45, 0x45, 0x39 },  // 5
  { 0x3C, 0x4A, 0x49, 0x49, 0x30 },  // 6
  { 0x01, 0x71, 0x09, 0x05, 0x03 },  // 7
  { 0x36, 0x49, 0x49, 0x49, 0x36 },  // 8
  { 0x06, 0x49, 0x49, 0x29, 0x1E },  // 9
  // 大写字母 A~Z
  { 0x7C, 0x12, 0x11, 0x12, 0x7C },  // A
  { 0x7F, 0x49, 0x49, 0x49, 0x36 },  // B
  { 0x3E, 0x41, 0x41, 0x41, 0x22 },  // C
  { 0x7F, 0x41, 0x41, 0x22, 0x1C },  // D
  { 0x7F, 0x49, 0x49, 0x49, 0x41 },  // E
  { 0x7F, 0x09, 0x09, 0x09, 0x01 },  // F
  { 0x3E, 0x41, 0x49, 0x49, 0x7A },  // G
  { 0x7F, 0x08, 0x08, 0x08, 0x7F },  // H
  { 0x00, 0x41, 0x7F, 0x41, 0x00 },  // I
  { 0x20, 0x40, 0x41, 0x3F, 0x01 },  // J
  { 0x7F, 0x08, 0x14, 0x22, 0x41 },  // K
  { 0x7F, 0x40, 0x40, 0x40, 0x40 },  // L
  { 0x7F, 0x02, 0x0C, 0x02, 0x7F },  // M
  { 0x7F, 0x04, 0x08, 0x10, 0x7F },  // N
  { 0x3E, 0x41, 0x41, 0x41, 0x3E },  // O
  { 0x7F, 0x09, 0x09, 0x09, 0x06 },  // P
  { 0x3E, 0x41, 0x51, 0x21, 0x5E },  // Q
  { 0x7F, 0x09, 0x19, 0x29, 0x46 },  // R
  { 0x26, 0x49, 0x49, 0x49, 0x32 },  // S
  { 0x01, 0x01, 0x7F, 0x01, 0x01 },  // T
  { 0x3F, 0x40, 0x40, 0x40, 0x3F },  // U
  { 0x1F, 0x20, 0x40, 0x20, 0x1F },  // V
  { 0x3F, 0x40, 0x38, 0x40, 0x3F },  // W
  { 0x63, 0x14, 0x08, 0x14, 0x63 },  // X
  { 0x07, 0x08, 0x70, 0x08, 0x07 },  // Y
  { 0x61, 0x51, 0x49, 0x45, 0x43 },  // Z
  // 小写字母 a~z
  { 0x00, 0x70, 0x88, 0x88, 0x60 },  // a
  { 0x7F, 0x48, 0x44, 0x44, 0x38 },  // b
  { 0x00, 0x38, 0x44, 0x44, 0x28 },  // c
  { 0x38, 0x44, 0x44, 0x48, 0x7F },  // d
  { 0x00, 0x38, 0x54, 0x54, 0x18 },  // e
  { 0x08, 0x7E, 0x09, 0x01, 0x02 },  // f
  { 0x18, 0xA4, 0xA4, 0xA4, 0x7C },  // g
  { 0x7F, 0x08, 0x04, 0x04, 0x78 },  // h
  { 0x00, 0x44, 0x7D, 0x40, 0x00 },  // i
  { 0x20, 0x40, 0x44, 0x3D, 0x00 },  // j
  { 0x7F, 0x10, 0x28, 0x44, 0x00 },  // k
  { 0x00, 0x41, 0x7F, 0x40, 0x00 },  // l
  { 0x7C, 0x04, 0x18, 0x04, 0x78 },  // m
  { 0x7C, 0x08, 0x04, 0x04, 0x78 },  // n
  { 0x38, 0x44, 0x44, 0x44, 0x38 },  // o
  { 0x7C, 0x14, 0x14, 0x14, 0x08 },  // p
  { 0x08, 0x14, 0x14, 0x18, 0x7C },  // q
  { 0x7C, 0x08, 0x04, 0x04, 0x08 },  // r
  { 0x48, 0x54, 0x54, 0x54, 0x20 },  // s
  { 0x04, 0x3F, 0x44, 0x40, 0x20 },  // t
  { 0x3C, 0x40, 0x40, 0x20, 0x7C },  // u
  { 0x1C, 0x20, 0x40, 0x20, 0x1C },  // v
  { 0x3C, 0x40, 0x30, 0x40, 0x3C },  // w
  { 0x44, 0x28, 0x10, 0x28, 0x44 },  // x
  { 0x0C, 0x50, 0x50, 0x50, 0x3C },  // y
  { 0x44, 0x64, 0x54, 0x4C, 0x44 },  // z
  // 符号
  { 0x00, 0x36, 0x36, 0x00, 0x00 },  // :
  { 0x23, 0x13, 0x08, 0x64, 0x62 },  // %
  { 0x00, 0x00, 0x00, 0x00, 0x60 },  // .
  { 0x20, 0x30, 0x0C, 0x06, 0x03 },  // /
  { 0x00, 0x10, 0x6C, 0x82, 0x00 },  // (
  { 0x00, 0x82, 0x6C, 0x10, 0x00 },  // )
  { 0x00, 0x00, 0x00, 0x00, 0x00 },  // 空格
  { 0x06, 0x09, 0x09, 0x06, 0x00 },  // °
  { 0x7F, 0x7F, 0x7F, 0x7F, 0x7F }   // █
};
enum {
  CHAR_0,
  CHAR_1,
  CHAR_2,
  CHAR_3,
  CHAR_4,
  CHAR_5,
  CHAR_6,
  CHAR_7,
  CHAR_8,
  CHAR_9,
  CHAR_A,
  CHAR_B,
  CHAR_C,
  CHAR_D,
  CHAR_E,
  CHAR_F,
  CHAR_G,
  CHAR_H,
  CHAR_I,
  CHAR_J,
  CHAR_K,
  CHAR_L,
  CHAR_M,
  CHAR_N,
  CHAR_O,
  CHAR_P,
  CHAR_Q,
  CHAR_R,
  CHAR_S,
  CHAR_T,
  CHAR_U,
  CHAR_V,
  CHAR_W,
  CHAR_X,
  CHAR_Y,
  CHAR_Z,
  CHAR_a,
  CHAR_b,
  CHAR_c,
  CHAR_d,
  CHAR_e,
  CHAR_f,
  CHAR_g,
  CHAR_h,
  CHAR_i,
  CHAR_j,
  CHAR_k,
  CHAR_l,
  CHAR_m,
  CHAR_n,
  CHAR_o,
  CHAR_p,
  CHAR_q,
  CHAR_r,
  CHAR_s,
  CHAR_t,
  CHAR_u,
  CHAR_v,
  CHAR_w,
  CHAR_x,
  CHAR_y,
  CHAR_z,
  CHAR_COLON,    // :
  CHAR_PERCENT,  // %
  CHAR_DOT,      // .
  CHAR_SLASH,    // /
  CHAR_LPAREN,   // (
  CHAR_RPAREN,   // )
  CHAR_SPACE,    // 空格
  CHAR_DEGREE,   // °
  CHAR_BLOCK,    // █
  CHAR_COUNT     // 总数，用于检查数组长度
};
/************************** IIC 通信函数 **************************/
void IIC_write(unsigned char date) {
  unsigned char i, temp;
  temp = date;
  for (i = 0; i < 8; i++) {
    digitalWrite(OLED_SCL, LOW);
    if ((temp & 0x80) == 0)
      digitalWrite(OLED_SDA, LOW);
    else
      digitalWrite(OLED_SDA, HIGH);
    temp = temp << 1;
    digitalWrite(OLED_SCL, HIGH);
  }
  digitalWrite(OLED_SCL, LOW);
  digitalWrite(OLED_SDA, HIGH);
  digitalWrite(OLED_SCL, HIGH);
  digitalWrite(OLED_SCL, LOW);
}
void IIC_start() {
  digitalWrite(OLED_SDA, HIGH);
  digitalWrite(OLED_SCL, HIGH);
  digitalWrite(OLED_SDA, LOW);
  digitalWrite(OLED_SCL, LOW);
  IIC_write(0x78);
}
void IIC_stop() {
  digitalWrite(OLED_SDA, LOW);
  digitalWrite(OLED_SCL, HIGH);
  digitalWrite(OLED_SDA, HIGH);
  digitalWrite(OLED_SCL, LOW);
  digitalWrite(OLED_SDA, LOW);
}
void OLED_send_cmd(unsigned char o_command) {
  IIC_start();
  IIC_write(0x00);
  IIC_write(o_command);
  IIC_stop();
}
void OLED_send_data(unsigned char o_data) {
  IIC_start();
  IIC_write(0x40);
  IIC_write(o_data);
  IIC_stop();
}
void Column_set(unsigned char column) {
  column = column + OLED_COLUMN_OFFSET;
  OLED_send_cmd(0x10 | (column >> 4));
  OLED_send_cmd(0x00 | (column & 0x0F));
}
void Page_set(unsigned char page) {
  OLED_send_cmd(0xB0 | (page & 0x0F));
}
void OLED_clear() {
  for (unsigned char page = 0; page < OLED_PAGE_NUMBER; page++) {
    Page_set(page);
    Column_set(0);
    for (unsigned char col = 0; col < OLED_COLUMN_NUMBER; col++) {
      OLED_send_data(0x00);
    }
  }
}
void OLED_init() {
  for (unsigned char i = 0; i < 25; i++) {
    OLED_send_cmd(OLED_init_cmd[i]);
    delay(10);
  }
}
/************************** 文本显示函数 **************************/
void draw_char(unsigned char page, unsigned char col, char c) {
  if (page >= OLED_PAGE_NUMBER) return;
  if (col >= OLED_COLUMN_NUMBER - 5) return;
  Page_set(page);
  Column_set(col);
  int index = -1;
  if (c >= '0' && c <= '9') {
    index = CHAR_0 + (c - '0');
  } else if (c >= 'A' && c <= 'Z') {
    index = CHAR_A + (c - 'A');
  } else if (c >= 'a' && c <= 'z') {
    index = CHAR_a + (c - 'a');
  } else {
    switch (c) {
      case ':': index = CHAR_COLON; break;
      case '%': index = CHAR_PERCENT; break;
      case '.': index = CHAR_DOT; break;
      case '/': index = CHAR_SLASH; break;
      case '(': index = CHAR_LPAREN; break;
      case ')': index = CHAR_RPAREN; break;
      case ' ': index = CHAR_SPACE; break;
      case 176: index = CHAR_DEGREE; break;
      default: index = CHAR_SPACE; break;
    }
  }
  if (index >= 0 && index < CHAR_COUNT) {
    for (unsigned char i = 0; i < 5; i++) {
      OLED_send_data(font_5x7[index][i]);
    }
    OLED_send_data(0x00);  // 间距
  }
}
void draw_text(unsigned char page, unsigned char col, const char* text) {
  if (page >= OLED_PAGE_NUMBER) return;
  unsigned char current_col = col;
  for (int i = 0; text[i] != '\0'; i++) {
    if (current_col >= OLED_COLUMN_NUMBER - 5) break;
    draw_char(page, current_col, text[i]);
    current_col += 6;  // 5像素字符+1像素间距
  }
}
/******************** 硬件初始化 ********************/
void initHardware() {
  // 初始化 OLED 引脚
  pinMode(OLED_SCL, OUTPUT);
  pinMode(OLED_SDA, OUTPUT);
  digitalWrite(OLED_SCL, LOW);
  digitalWrite(OLED_SDA, LOW);
  // 初始化 OLED
  OLED_init();
  delay(100);
  OLED_clear();
  // 温度传感器
  tempSensor.begin();
  // TDS传感器
  analogReadResolution(12);
  pinMode(TDS_PIN, INPUT);
  // DHT11传感器
  pinMode(DHT11_PIN, INPUT);
  // 舵机
  feederServo.write(90);
  // 优化WiFi性能
  WiFi.setSleep(false);
}
/******************** 辅助函数 ********************/
void displayMessage(const char* line1, const char* line2) {
  OLED_clear();
  int center1 = (OLED_COLUMN_NUMBER - strlen(line1) * 6) / 2;
  int center2 = (OLED_COLUMN_NUMBER - strlen(line2) * 6) / 2;
  draw_text(2, center1 > 0 ? center1 : 0, line1);
  draw_text(4, center2 > 0 ? center2 : 0, line2);
}
/******************** MQTT函数 ********************/
// MQTT回调函数
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.printf("MQTT消息 [%s]: %s\n", topic, message.c_str());

  if (String(topic) == "home/fish_tank/feed/set") {
    if (message == "ON") {
      triggerFeeding();
    }
  }
  // 添加以下处理逻辑
  if (String(topic) == "home/fish_tank/feedingScheduleEnabled/set") {
    feedingScheduleEnabled = (message == "ON");

    // 发布状态更新
    mqttClient.publish("home/fish_tank/feedingScheduleEnabled/state",
                       feedingScheduleEnabled ? "ON" : "OFF", true);

    Serial.printf("自动喂食开关: %s\n", feedingScheduleEnabled ? "开启" : "关闭");
  } else if (String(topic) == "home/fish_tank/feedingTime1/set") {
    feedingTime1 = message.toInt();

    // 发布状态更新
    mqttClient.publish("home/fish_tank/feedingTime1/state",
                       String(feedingTime1).c_str(), true);

    Serial.printf("第一次喂食时间设置为: %d:00\n", feedingTime1);
  } else if (String(topic) == "home/fish_tank/feedingTime2/set") {
    feedingTime2 = message.toInt();

    // 发布状态更新
    mqttClient.publish("home/fish_tank/feedingTime2/state",
                       String(feedingTime2).c_str(), true);

    Serial.printf("第二次喂食时间设置为: %d:00\n", feedingTime2);
  } else if (String(topic) == "home/fish_tank/camera/stream/set") {
    bool newState = (message == "ON");

    if (newState != isStreaming) {
      isStreaming = newState;
      sensor_t* s = esp_camera_sensor_get();
    }
    // 发布状态更新
    mqttClient.publish("home/fish_tank/camera/stream/state",
                       isStreaming ? "ON" : "OFF", true);
  }
}

// ============== 视频流发送函数 ==============
void sendVideoStream() {
  if (!isStreaming) return;

  unsigned long currentTime = millis();
  if (currentTime - lastFrameSent < STREAM_INTERVAL) return;

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
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

  // 发送帧头
  if (client.print(frameHeader) != frameHeader.length()) {
    Serial.println("Frame header send failed");
    client.stop();
    esp_camera_fb_return(fb);
    return;
  }

  // 发送图像数据
  size_t dataSent = client.write(fb->buf, fb->len);

  // 发送帧结束标记
  client.print("\r\n");


  // 清理
  client.stop();
  esp_camera_fb_return(fb);
  lastFrameSent = currentTime;
}




/******************** 增强版MQTT连接管理 ********************/
void setupMqtt() {
  espClient.setInsecure();//跳过ca证书验证
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
    return false;
  }
  // 指数退避重试
  unsigned long currentTime = millis();
  unsigned long delayTime = min(5000 * (retryCount + 1), 60000);  // 最大重试间隔60秒

  if (currentTime - lastAttempt < delayTime) {
    return false;
  }

  Serial.printf("[MQTT] 尝试连接(重试 %d)...", ++retryCount);

  if (mqttClient.connect(device_id, mqtt_user, mqtt_password)) {
    Serial.println("成功");
    retryCount = 0;
    mqttClient.subscribe("home/fish_tank/camera/stream/set", 1);
    mqttClient.subscribe("home/fish_tank/feed/set", 1);
    mqttClient.subscribe("home/fish_tank/feeding_time/set", 1);
    mqttClient.subscribe("home/fish_tank/feedingScheduleEnabled/set", 1);
    mqttClient.subscribe("home/fish_tank/feedingTime1/set", 1);
    mqttClient.subscribe("home/fish_tank/feedingTime2/set", 1);
    sendSensorData();
    return true;  // 连接成功返回true
  } else {
    Serial.printf("失败, 错误码: %d\n", mqttClient.state());
    return false;  // 连接失败返回false
  }
  bool connected = mqttClient.connect(
    device_id,
    mqtt_user,
    mqtt_password,
    "home/fish_tank/status",  // 遗嘱主题
    1,                        // QoS
    true,                     // retained
    "offline"                 // 遗嘱消息
  );
  lastAttempt = millis();
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
}
/******************** 喂食控制 ********************/
// 检查喂食计划
void checkFeedingSchedule() {
  if (!feedingScheduleEnabled) return;  // 如果开关关闭，则不执行喂食逻辑
  static int lastFedDay = -1;
  int currentHour = timeClient.getHours();
  int currentDay = timeClient.getDay();
  if ((currentHour == feedingTime1 || currentHour == feedingTime2) && currentDay != lastFedDay) {
    triggerFeeding();
    lastFedDay = currentDay;
  }
}
// 触发喂食
void triggerFeeding() {
  if (status.feeding) return;

  status.feeding = true;
  status.feedingCompleted = false;
  status.feedCount++;
  status.lastFeedTime = timeClient.getEpochTime();
  status.feedingStartTime = millis();  // 记录喂食开始时间

  // 开始喂食（转动舵机）
  feederServo.write(60);
  Serial.println("开始喂食...");
}
/******************** 传感器读取 ********************/
// 预计算系数
const float K_VALUE = 0.85;
const float T_REF = 25.0;
const float TEMP_COEF = 0.02;
const float A = 500.0;
const float B = 10.0;
const float C = 0.1;
const float A_COMP = A * K_VALUE;
const float B_COMP = B * K_VALUE;
const float C_COMP = C * K_VALUE;
void readSensorsNonBlocking() {
  static unsigned long sensorStartTime = 0;
  static int sensorStep = 0;
  static int rawTdsValue = 0;
  static float tempCelsius = 0.0;
  int result = 0;

  switch (sensorStep) {
    case 0:  // 开始新的读取周期
      sensorStep = 1;
      sensorStartTime = millis();
      break;

    case 1:  // 启动DHT11读取
      result = dht11.readTemperatureHumidity(status.envTemp, status.envHumidity);
      if (result != 0) {
        Serial.print("DHT11错误: ");
        Serial.println(DHT11::getErrorString(result));
        status.envTemp = 0;
        status.envHumidity = 0;
      }
      sensorStep = 2;
      break;

    case 2:  // 读取TDS值（移除后续温度补偿）
      rawTdsValue = analogRead(TDS_PIN);
      {
        float rawVoltage = rawTdsValue / 4095.0 * 3.3;
        float V_corrected = rawVoltage * (A_COMP + rawVoltage * (B_COMP + rawVoltage * C_COMP));

        // 直接作为最终TDS值（不再等待水温校准）
        status.tdsValue = V_corrected;
      }
      sensorStep = 3;  // 继续读取水温（如需独立水温数据）
      break;

    case 3:  // 启动水温传感器
      tempSensor.requestTemperatures();
      sensorStep = 4;
      sensorStartTime = millis();
      break;

    case 4:  // 读取水温（独立于TDS）
      if (millis() - sensorStartTime > 750) {
        tempCelsius = tempSensor.getTempCByIndex(0);
        status.waterTemp = tempCelsius;

        // === 重点修改 ===
        // 1. 完全移除TDS温度补偿步骤
        // 2. 添加水温传感器故障检测
        if (tempCelsius < -50) {  // 检测异常低温
          Serial.println("水温传感器错误: 读取失败");
          status.waterTemp = 0;  // 设为安全值
        }

        Serial.print("传感器读取: ");
        Serial.print("水温=");
        Serial.print(status.waterTemp);  // 显示处理后的水温
        Serial.print("℃, TDS=");
        Serial.print(status.tdsValue);  // 显示未补偿的TDS
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

/******************** 显示功能 ********************/
// 获取下一个喂食时间（完整版本）
String getNextFeedingTime() {
  // 检查自动喂食是否关闭
  if (!feedingScheduleEnabled) {
    return "自动喂食关闭";
  }

  int currentHour = timeClient.getHours();
  time_t now = timeClient.getEpochTime();

  // 计算今天的喂食时间点
  time_t feed1_t = now - (now % 86400) + feedingTime1 * 3600;
  time_t feed2_t = now - (now % 86400) + feedingTime2 * 3600;

  // 如果喂食时间已过，设为明天
  if (feed1_t < now) feed1_t += 86400;
  if (feed2_t < now) feed2_t += 86400;

  // 选择最近的时间
  time_t nextFeed = (feed1_t < feed2_t) ? feed1_t : feed2_t;

  // 格式化为字符串
  struct tm* tm_info = localtime(&nextFeed);
  char buf[20];
  strftime(buf, sizeof(buf), "%H:%M", tm_info);

  return String(buf) + ((nextFeed - now > 86400) ? "(明天)" : "(今天)");
}
// 获取下一个喂食时间（短版本）
String getNextFeedingTimeShort() {
  String fullTime = getNextFeedingTime();
  String result = fullTime;  // 默认返回完整字符串

  int todayPos = fullTime.indexOf("(今天)");
  int tomorrowPos = fullTime.indexOf("(明天)");

  if (todayPos != -1) {
    result = fullTime.substring(0, todayPos);  // 截取到 "(今天)" 之前
  } else if (tomorrowPos != -1) {
    result = fullTime.substring(0, tomorrowPos) + "(TMW)";  // 截取到 "(明天)" 之前
  }

  return result;
}

// 清除指定行
void clearLine(uint8_t line) {
  if (line >= OLED_PAGE_NUMBER) return;
  Page_set(line);
  Column_set(0);
  for (uint8_t i = 0; i < OLED_COLUMN_NUMBER; i++) {
    OLED_send_data(0x00);
  }
}
// 更新OLED显示内容
void updateDisplay() {
  static String lastTimeStr = "";
  static String lastFeedStr = "";
  static String lastWaterStr = "";
  static String lastEnvStr = "";
  static String lastStatusStr = "";
  static String lastScheduleStr = "";
  // 获取当前数据
  char timeStr[20];
  snprintf(timeStr, sizeof(timeStr), "%s", timeClient.getFormattedTime().c_str());
  char feedStr[50];
  if (status.feeding) {
    int remaining = (SERVO_DURATION - (millis() - status.feedingStartTime)) / 1000;
    snprintf(feedStr, sizeof(feedStr), "Feeding %ds", max(0, remaining));
  } else {
    snprintf(feedStr, sizeof(feedStr), "Feed:%d Next:%s", status.feedCount, getNextFeedingTimeShort().c_str());
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
  if (String(statusStr) != lastStatusStr) {
    clearLine(6);
    draw_text(6, 0, statusStr);
    lastStatusStr = String(statusStr);
  }
  // OLED第4行：只根据自动喂食开关和喂食状态显示内容
  String scheduleStr;
  if (!feedingScheduleEnabled) {
    if (status.feeding && !status.feedingCompleted) {
      scheduleStr = "Feeding...";
    } else {
      scheduleStr = "Scheduled feeding OFF";
    }
  } else {
    if (status.feeding && !status.feedingCompleted) {
      scheduleStr = "Feeding...";
    } else {
      char feedStr[40];
      snprintf(feedStr, sizeof(feedStr), "Feed:%d Next:%s", status.feedCount, getNextFeedingTimeShort().c_str());
      scheduleStr = feedStr;
    }
  }
  if (scheduleStr != lastScheduleStr) {
    clearLine(4);
    draw_text(4, 0, scheduleStr.c_str());
    lastScheduleStr = scheduleStr;
  }
}


/**************** OTA回调函数 ****************/
void onOTAStart() {
  Serial.println("OTA更新开始");
  // 释放摄像头资源
  camera_fb_t* fb = esp_camera_fb_get();
  if (fb) {
    esp_camera_fb_return(fb);
    camera_released = true;
    Serial.println("已释放摄像头资源");
  }
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
}

void onOTAEnd(bool success) {
  if (success) {
    Serial.println("OTA更新成功，即将重启");
  } else {
    Serial.println("OTA更新失败");
    // 恢复摄像头功能
    if (camera_released) {
      esp_err_t err = esp_camera_init(&cameraConfig);
      if (err == ESP_OK) Serial.println("摄像头重新初始化成功");
    }
  }
  camera_released = false;
}
//发送mqtt数据
void sendSensorData() {
  if (!mqttClient.connected()) return;

  // 环境温湿度
  mqttClient.publish("home/fish_tank/sensor/temperature/state",
                     String(status.envTemp).c_str(), true);
  mqttClient.publish("home/fish_tank/sensor/humidity/state",
                     String(status.envHumidity).c_str(), true);

  // 水温
  mqttClient.publish("home/fish_tank/sensor/water_temp/state",
                     String(status.waterTemp).c_str(), true);

  // TDS值
  mqttClient.publish("home/fish_tank/sensor/tds/state",
                     String(status.tdsValue).c_str(), true);

  // 喂食次数
  mqttClient.publish("home/fish_tank/sensor/feed_count/state",
                     String(status.feedCount).c_str(), true);
  // 最后喂食时间
  mqttClient.publish("home/fish_tank/sensor/feed_lastFeedTime/state",
                     String(status.lastFeedTime).c_str(), true);
  // 自动喂食开关状态
  mqttClient.publish("home/fish_tank/feedingScheduleEnabled/state",
                     feedingScheduleEnabled ? "ON" : "OFF", true);
  // 下次时间
  mqttClient.publish("home/fish_tank/sensor/feed_NextFeedingTime/state",
                     String(getNextFeedingTime()).c_str(), true);
  // 喂食开关
  mqttClient.publish("home/fish_tank/feed/state",
                     status.feeding ? "ON" : "OFF", true);
  // 喂食时间状态
  mqttClient.publish("home/fish_tank/feedingTime1/state",
                     String(feedingTime1).c_str(), true);

  mqttClient.publish("home/fish_tank/feedingTime2/state",
                     String(feedingTime2).c_str(), true);
  // 添加视频流状态
  mqttClient.publish("home/fish_tank/camera/stream/state",
                     isStreaming ? "ON" : "OFF", true);
}

void setup() {
  delay(3000);  // 给硬件上电稳定时间
  Serial.begin(115200);
  delay(1000);
  // 第一阶段：基础硬件初始化
  pinMode(OLED_SCL, OUTPUT);
  pinMode(OLED_SDA, OUTPUT);
  pinMode(19, INPUT_PULLUP);  // 按键引脚初始化
  pinMode(ONE_WIRE_BUS, INPUT_PULLUP);
  status.feeding = false;  // 确保喂食状态初始为 false
  status.feedingCompleted = true;
  OLED_init();
  OLED_clear();
  displayMessage("Booting...", "Step 1/4");
  // 第二阶段：WiFi连接
  connectWiFi();
  displayMessage("Connected", "Step 2/4");
  // 第三阶段：其他硬件初始化
  timeClient.begin();
  timeClient.update();
  // ======== ElegantOTA 配置 ========
  ElegantOTA.begin(&server);
  // 设置回调
  ElegantOTA.onStart(onOTAStart);
  ElegantOTA.onProgress(onOTAProgress);
  ElegantOTA.onEnd(onOTAEnd);

  server.begin();
  Serial.printf("OTA更新地址: http://%s/update\n", WiFi.localIP().toString().c_str());
  feederServo.attach(SERVO_PIN);
  displayMessage("Sensors OK", "Step 3/4");
  // 第四阶段：摄像头初始化（最后进行）
  if (psramFound()) {
    cameraConfig.fb_location = CAMERA_FB_IN_PSRAM;
    Serial.println("Using PSRAM");
  }
  esp_err_t err = esp_camera_init(&cameraConfig);
  if (err != ESP_OK) {
    displayMessage("Camera Error", "Check Wiring");
    Serial.printf("Camera init failed: 0x%x\n", err);
    delay(5000);
    ESP.restart();
  }
  // 摄像头翻转180
  sensor_t* s = esp_camera_sensor_get();
  if (s != NULL) {
    s->set_vflip(s, 1);    // 垂直翻转
    s->set_hmirror(s, 1);  // 水平镜像
    Serial.println("摄像头已设置为180度翻转");
  }
  displayMessage("System Ready", WiFi.localIP().toString().c_str());
  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
    if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
      Serial.println("WiFi断开，尝试重连...");
      WiFi.reconnect();
    }
  });
  setupMqtt();
  // 发送在线状态
  if (millis() - lastHeartbeat > 30000) {
    mqttClient.publish("home/fish_tank/status", "online", true);  // 保留消息
    lastHeartbeat = millis();
  }
  // 初始读取传感器数据
  readSensorsNonBlocking();
  // 初始更新显示
  updateDisplay();
  esp_camera_fb_return(esp_camera_fb_get());  // 预热并释放初始帧
}


void loop() {
  // 按键喂食逻辑
  static unsigned long lastButtonPress = 0;
  const unsigned long debounceTime = 50;  // 50ms防抖时间

  // 按钮检测（带防抖）
  if (digitalRead(19) == LOW && (millis() - lastButtonPress) > debounceTime) {
    lastButtonPress = millis();
    triggerFeeding();
  }

  ElegantOTA.loop();
  sendVideoStream();
  if (millis() - lastCleanup > 10000) {
    if (!isStreaming && streamClient.connected()) {
      streamClient.stop();
    }
    lastCleanup = millis();
  }
  static unsigned long lastNTPUpdate = 0;
  static unsigned long lastSensorRead = 0;
  static unsigned long lastDisplayUpdate = 0;
  static unsigned long lastMemClean = 0;
  static bool isFeeding = false;  // 用于按键喂食的防抖状态
  static unsigned long lastFrameTime = 0;
  static unsigned long lastMemCheck = 0;
  // 检查内存使用情况
  static unsigned long lastMemoryCheck = 0;
  if (millis() - lastMemClean > 10000) {
    if (ESP.getFreeHeap() < 30000) {
      Serial.printf("内存不足: %d bytes, 清理中...\n", ESP.getFreeHeap());
      if (streamClient.connected()) streamClient.stop();
    }
    lastMemClean = millis();
  }

  // 每小时强制更新时间
  if (millis() - lastNTPUpdate > 3600000) {
    timeClient.forceUpdate();
    lastNTPUpdate = millis();
  }
  // 每10秒读取一次传感器
  if (millis() - lastSensorRead > 10000) {
    readSensorsNonBlocking();
    lastSensorRead = millis();
  }
  // 每1秒更新一次显示（但只更新变化的内容）
  if (millis() - lastDisplayUpdate > 1000) {
    updateDisplay();
    lastDisplayUpdate = millis();
  }

  // 检查喂食计划
  checkFeedingSchedule();
  // 检查喂食状态，如果正在喂食且持续时间已到，则停止喂食
  if (status.feeding && !status.feedingCompleted) {
    if (millis() - status.feedingStartTime >= SERVO_DURATION) {
      feederServo.write(90);  // 复位舵机
      status.feedingCompleted = true;
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
  if (currentDay != lastDay) {           // 如果日期发生变化
    lastDay = currentDay;
    status.feedCount = 0;  // 重置喂食计数
    Serial.println("喂食计数已重置为 0");
  }

  static unsigned long lastMqttPublish = 0;
  // MQTT连接管理
  if (!mqttClient.connected()) {
    reconnectMqtt();
  }
  mqttClient.loop();
  // 发送在线状态
  if (millis() - lastHeartbeat > 30000) {
    mqttClient.publish("home/fish_tank/status", "online", true);  // 保留消息
    lastHeartbeat = millis();
  }
  // 每60秒发送传感器数据
  if (millis() - lastMqttPublish > 60000) {
    sendSensorData();
    lastMqttPublish = millis();
  }
}
