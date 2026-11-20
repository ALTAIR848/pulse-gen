/**
 * @file    display.h
 * @brief   显示封装层: 主页(滚动) / 通道选择子页 / 编辑子页
 * @note    屏幕布局(128x64):
 *          主页: 第1行固定为模式行(带开机秒数), 其余行为模式参数(滚动窗口3行)
 *          通道子页: 4个通道勾选行 + 返回行, 滚动窗口4行
 *          编辑子页: 参数名 / 大号数值 / 操作提示
 */
#ifndef __DISPLAY_H
#define __DISPLAY_H

#include "pulse.h"

void display_init(void);

/**
 * @brief  主页(整页刷新)
 * @param  page           行列表(pulse_build_main_page 生成)
 * @param  cursor         光标所在行(页面行索引)
 * @param  scroll_top     列表滚动起点(页面行索引1起, 由上层维护)
 * @param  mode_selecting 1=模式滚动选择中(模式行显示'*')
 * @param  uptime_s       开机秒数
 */
void display_show_main_page(const pulse_page_t *page, uint8_t cursor,
                            uint8_t scroll_top, uint8_t mode_selecting,
                            uint32_t uptime_s);

/**
 * @brief  通道选择子页
 * @param  cursor    0~4 (0~3=通道, 4=返回)
 * @param  scroll_top 滚动起点(0~1)
 */
void display_show_ch_page(const pulse_param_t *p, uint8_t cursor, uint8_t scroll_top);

/**
 * @brief  编辑子页(整页): 名称 / 24号大数值 / 操作提示
 */
void display_show_edit_page(const pulse_edit_info_t *info);

#endif /* __DISPLAY_H */
