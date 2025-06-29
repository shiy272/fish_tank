#include "OLED_Driver.h"
#include "OLED_Fonts.h"  

// IIC写入函数
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

// IIC启动信号
void IIC_start() {
    digitalWrite(OLED_SDA, HIGH);  
    digitalWrite(OLED_SCL, HIGH);  
    digitalWrite(OLED_SDA, LOW);
    digitalWrite(OLED_SCL, LOW);
    IIC_write(0x78);
}

// IIC停止信号
void IIC_stop() {
    digitalWrite(OLED_SDA, LOW);  
    digitalWrite(OLED_SCL, HIGH);  
    digitalWrite(OLED_SDA, HIGH);
    digitalWrite(OLED_SCL, HIGH);
    digitalWrite(OLED_SCL, LOW);
    digitalWrite(OLED_SDA, LOW);
}

// 发送命令
void OLED_send_cmd(unsigned char o_command) {
    IIC_start();
    IIC_write(0x00);
    IIC_write(o_command);
    IIC_stop();
}

// 发送数据
void OLED_send_data(unsigned char o_data) {
    IIC_start();
    IIC_write(0x40);
    IIC_write(o_data);
    IIC_stop();
}

// 设置列地址
void Column_set(unsigned char column) {
    column = column + 2; // 偏移量
    OLED_send_cmd(0x10 | (column >> 4));
    OLED_send_cmd(0x00 | (column & 0x0F));
}

// 设置页地址
void Page_set(unsigned char page) {
    OLED_send_cmd(0xB0 | (page & 0x0F));
}

// 清屏
void OLED_clear() {
    for (unsigned char page = 0; page < 8; page++) {
        Page_set(page);
        Column_set(0);
        for (unsigned char col = 0; col < 128; col++) {
            OLED_send_data(0x00);
        }
    }
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
// 初始化OLED
void OLED_init() {
    pinMode(OLED_SCL, OUTPUT);
    pinMode(OLED_SDA, OUTPUT);
    digitalWrite(OLED_SCL, LOW);
    digitalWrite(OLED_SDA, LOW);

    const unsigned char init_cmds[25] = {
        0xAE, 0xD5, 0x80, 0xA8, 0X3F, 0xD3, 0X00, 0x40,
        0x8D, 0x14, 0x20, 0x02, 0xA1, 0xC8,
        0xDA, 0x12, 0x81, 0x66, 0xD9, 0xf1, 0xDB, 0x30, 0xA4, 0xA6,
        0xAF
    };

    for (unsigned char i = 0; i < 25; i++) {
        OLED_send_cmd(init_cmds[i]);
        delay(10);
    }
}

// 显示单个字符
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

// 显示字符串
void draw_text(unsigned char page, unsigned char col, const char* text) {
    if (page >= 8) return;
    unsigned char current_col = col;
    for (int i = 0; text[i] != '\0'; i++) {
        if (current_col >= 123) break;
        draw_char(page, current_col, text[i]);
        current_col += 6; // 5像素字符+1像素间距
    }
}