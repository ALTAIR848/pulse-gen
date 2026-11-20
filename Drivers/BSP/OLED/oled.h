/**
 * @file    oled.h
 * @brief   0.96寸 OLED (SSD1306) 驱动 - 4线软件SPI
 *          移植自核心板例程10，引脚对接 CubeMX 生成的 GPIO 标签
 */
#ifndef __OLED_H
#define __OLED_H

#include "main.h"

/* 引脚控制宏（引脚已在 MX_GPIO_Init 中配置为推挽输出） */
#define OLED_RST(x)   HAL_GPIO_WritePin(OLED_RST_GPIO_Port,  OLED_RST_Pin,  (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define OLED_CS(x)    HAL_GPIO_WritePin(OLED_CS_GPIO_Port,   OLED_CS_Pin,   (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define OLED_DC(x)    HAL_GPIO_WritePin(OLED_DC_GPIO_Port,   OLED_DC_Pin,   (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define OLED_SCLK(x)  HAL_GPIO_WritePin(OLED_SCLK_GPIO_Port, OLED_SCLK_Pin, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define OLED_SDIN(x)  HAL_GPIO_WritePin(OLED_SDIN_GPIO_Port, OLED_SDIN_Pin, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)

#define OLED_CMD      0   /* 写命令 */
#define OLED_DATA     1   /* 写数据 */

void oled_init(void);
void oled_display_on(void);
void oled_display_off(void);
void oled_clear(void);
void oled_refresh_gram(void);
void oled_draw_point(uint8_t x, uint8_t y, uint8_t dot);
void oled_fill(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t dot);
void oled_show_char(uint8_t x, uint8_t y, uint8_t chr, uint8_t size, uint8_t mode);
void oled_show_num(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size);
void oled_show_string(uint8_t x, uint8_t y, const char *p, uint8_t size);

#endif /* __OLED_H */
