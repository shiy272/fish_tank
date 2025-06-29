#include "CameraConfig.h"

// 创建并返回摄像头配置
camera_config_t getCameraConfig() {
  camera_config_t config;
  
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.xclk_freq_hz = 20000000;  // 外部时钟20MHz
  config.ledc_timer = LEDC_TIMER_0;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.pixel_format = PIXFORMAT_JPEG;  // JPEG格式
  config.frame_size = FRAMESIZE_VGA;     // 分辨率
  config.jpeg_quality = 12;              // JPEG画质（0-63，数值越小越清晰）
  config.fb_count = 2;                  // 帧缓冲数量
  
  return config;
}