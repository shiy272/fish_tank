#ifndef OLED_DRIVER_H
#define OLED_DRIVER_H

#include <Arduino.h>

// 定义引脚
#define OLED_SCL 41
#define OLED_SDA 42

// 初始化OLED
void OLED_init();

// 清屏
void OLED_clear();
// 清除指定行
void clearLine(uint8_t line);
// 发送命令
void OLED_send_cmd(unsigned char cmd);

// 发送数据
void OLED_send_data(unsigned char data);

// 设置列地址
void Column_set(unsigned char column);

// 设置页地址
void Page_set(unsigned char page);

// 显示字符
void draw_char(unsigned char page, unsigned char col, char c);

// 显示字符串
void draw_text(unsigned char page, unsigned char col, const char* text);

#endif // OLED_DRIVER_H