/**
 * @file    key.h
 * @brief   三键扫描: 非阻塞, 去抖20ms, 长按1s, 连发自适应加速(400ms后
 *          150ms→80ms→40ms→20ms 随按住时长提速)
 * @note    K_UP(PA0)按下为高电平, K1(PA15)/K0(PC5)按下为低电平, 极性已内置
 *          GPIO 已由 MX_GPIO_Init 配置, key_init 无需做事
 */
#ifndef __KEY_H
#define __KEY_H

#include "main.h"

typedef enum
{
    KEY_ID_UP = 0,      /* K_UP  = PA0  : 向上 */
    KEY_ID_CONFIRM,     /* K1    = PA15 : 确认 */
    KEY_ID_DOWN,        /* K0    = PC5  : 向下 */
    KEY_ID_COUNT
} key_id_t;

typedef enum
{
    KEY_EVENT_NONE = 0,
    KEY_EVENT_DOWN,          /* 按下(去抖后) */
    KEY_EVENT_RELEASE_SHORT, /* 短按释放(未达到长按时间) */
    KEY_EVENT_LONG,          /* 长按达到1s(每次按下只报一次) */
    KEY_EVENT_REPEAT         /* 按住连发(400ms后每150ms一次) */
} key_event_t;

void key_init(void);
key_event_t key_scan(key_id_t *out_id);   /* 主循环轮询, 每次返回一个事件 */
uint32_t key_hold_ms(key_id_t id);        /* 当前按住时长(未按住返回0), 用于加速步进 */

#endif /* __KEY_H */
