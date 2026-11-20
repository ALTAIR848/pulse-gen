/**
 * @file    key.c
 * @brief   三键扫描实现
 */
#include "key.h"

#define KEY_DEBOUNCE_MS     20u     /* 去抖时间 */
#define KEY_LONG_MS         1000u   /* 长按判定 */
#define KEY_REPEAT_DELAY_MS 400u    /* 连发起始延时 */

/* 连发周期按住时长自适应: 按得越久发得越快(档位间隔较大, 提速平缓) */
static uint32_t repeat_interval(uint32_t hold_ms)
{
    if (hold_ms < 2000u)
    {
        return 150u;
    }

    if (hold_ms < 4000u)
    {
        return 90u;
    }

    if (hold_ms < 7000u)
    {
        return 50u;
    }

    return 30u;
}

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
    uint8_t active_high;        /* 按下时的电平 */
    uint8_t state;              /* 0空闲 1去抖中 2已确认按下 */
    uint32_t t_debounce;
    uint32_t t_press;
    uint32_t t_last_repeat;
    uint8_t long_sent;
} key_ctx_t;

static key_ctx_t g_keys[KEY_ID_COUNT] =
{
    [KEY_ID_UP]      = {KEY_UP_GPIO_Port,   KEY_UP_Pin,   1u, 0u, 0u, 0u, 0u, 0u},
    [KEY_ID_CONFIRM] = {KEY1_GPIO_Port,     KEY1_Pin,     0u, 0u, 0u, 0u, 0u, 0u},
    [KEY_ID_DOWN]    = {KEY0_GPIO_Port,     KEY0_Pin,     0u, 0u, 0u, 0u, 0u, 0u},
};

void key_init(void)
{
    /* GPIO 配置已在 MX_GPIO_Init 完成, 此处仅复位状态机 */
    for (uint32_t i = 0u; i < KEY_ID_COUNT; i++)
    {
        g_keys[i].state = 0u;
        g_keys[i].long_sent = 0u;
    }
}

/**
 * @brief  轮询一次按键, 返回一个事件(无事件返回KEY_EVENT_NONE)
 * @param  out_id: 输出产生事件的按键
 */
key_event_t key_scan(key_id_t *out_id)
{
    for (uint32_t i = 0u; i < KEY_ID_COUNT; i++)
    {
        key_ctx_t *k = &g_keys[i];
        uint32_t now = HAL_GetTick();
        uint8_t pressed = (HAL_GPIO_ReadPin(k->port, k->pin) == GPIO_PIN_SET) ? k->active_high : !k->active_high;

        switch (k->state)
        {
            case 0u:    /* 空闲: 检测按下 */
                if (pressed)
                {
                    k->state = 1u;
                    k->t_debounce = now;
                }
                break;

            case 1u:    /* 去抖中 */
                if (!pressed)
                {
                    k->state = 0u;      /* 抖动, 忽略 */
                }
                else if (now - k->t_debounce >= KEY_DEBOUNCE_MS)
                {
                    k->state = 2u;
                    k->t_press = now;
                    k->t_last_repeat = now;
                    k->long_sent = 0u;
                    *out_id = (key_id_t)i;
                    return KEY_EVENT_DOWN;
                }
                break;

            case 2u:    /* 已按下 */
                if (!pressed)
                {
                    k->state = 0u;

                    if (!k->long_sent)
                    {
                        *out_id = (key_id_t)i;
                        return KEY_EVENT_RELEASE_SHORT;
                    }

                    break;
                }

                if (!k->long_sent && (now - k->t_press >= KEY_LONG_MS))
                {
                    k->long_sent = 1u;
                    *out_id = (key_id_t)i;
                    return KEY_EVENT_LONG;
                }

                if ((now - k->t_press >= KEY_REPEAT_DELAY_MS) &&
                        (now - k->t_last_repeat >= repeat_interval(now - k->t_press)))
                {
                    k->t_last_repeat = now;
                    *out_id = (key_id_t)i;
                    return KEY_EVENT_REPEAT;
                }

                break;

            default:
                k->state = 0u;
                break;
        }
    }

    return KEY_EVENT_NONE;
}

uint32_t key_hold_ms(key_id_t id)
{
    if ((uint32_t)id < (uint32_t)KEY_ID_COUNT && g_keys[id].state == 2u)
    {
        return HAL_GetTick() - g_keys[id].t_press;
    }

    return 0u;
}
