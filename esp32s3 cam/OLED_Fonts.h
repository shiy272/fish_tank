#ifndef OLED_FONTS_H
#define OLED_FONTS_H

#include <Arduino.h>

// 声明字体数据
extern const unsigned char font_5x7[][5];

// 枚举字符索引
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

// OLED显示参数
#define OLED_COLUMN_NUMBER 128
#define OLED_LINE_NUMBER 64
#define OLED_COLUMN_OFFSET 2
#define OLED_PAGE_NUMBER (OLED_LINE_NUMBER / 8)

// 文本显示函数声明
void draw_char(unsigned char page, unsigned char col, char c);
void draw_text(unsigned char page, unsigned char col, const char* text);

#endif // OLED_FONTS_H