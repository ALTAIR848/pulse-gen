/**
 * @file    display.c
 * @brief   显示封装层实现
 * @note    行格式统一整行覆盖, 避免残影; 标记: '>'光标(浏览), '*'编辑/选择中
 */
#include "display.h"
#include "oled.h"
#include <stdio.h>

void display_init(void)
{
    oled_init();
}

/**
 * @brief  行文本补齐/截断到指定字符宽度, 保证整行覆盖清除残影
 * @note   width按字号换算: 16号=16字符, 24号=10字符, 12号=21字符
 */
static void pad_line(char *buf, uint8_t size)
{
    static const uint8_t cols_of_size[3] = {21u, 16u, 10u};  /* 索引: 12/16/24号 */
    uint8_t cols;
    uint8_t len = 0u;

    switch (size)
    {
        case 24u:
            cols = cols_of_size[2];
            break;

        case 12u:
            cols = cols_of_size[0];
            break;

        default:
            cols = cols_of_size[1];
            break;
    }

    while (buf[len] != '\0')
    {
        len++;
    }

    while (len < cols)
    {
        buf[len++] = ' ';
    }

    buf[cols] = '\0';   /* 超宽则截断 */
}

/**
 * @brief  构造主页第 row_idx 行文本(page 索引), 返回写入长度
 */
static uint8_t main_row_text(char *buf, uint8_t size,
                             const pulse_page_t *page, uint8_t row_idx,
                             const pulse_param_t *p, uint8_t cursor,
                             uint8_t mode_selecting, uint32_t uptime_s)
{
    uint8_t mark;
    uint8_t type = page->row[row_idx].type;
    uint8_t idx = page->row[row_idx].idx;

    mark = (row_idx == cursor) ? '>' : ' ';

    switch ((pulse_row_type_t)type)
    {
        case PULSE_ROW_MODE:
            mark = (row_idx == cursor) ? (mode_selecting ? '*' : '>') : ' ';
            return (uint8_t)snprintf(buf, size, "%cMode:%-6s%3lus", mark,
                                     pulse_mode_name(p->mode),
                                     (unsigned long)uptime_s);

        case PULSE_ROW_PWM_EN:
            return (uint8_t)snprintf(buf, size, "%cPWMEn:%-3s", mark,
                                     p->pwm_enabled ? "ON" : "OFF");

        case PULSE_ROW_DUAL_P1:
            return (uint8_t)snprintf(buf, size, "%cP1   :%5lu us", mark,
                                     (unsigned long)p->dual_p1_us);

        case PULSE_ROW_DUAL_GAP:
            return (uint8_t)snprintf(buf, size, "%cGap  :%5lu us", mark,
                                     (unsigned long)p->dual_gap_us);

        case PULSE_ROW_DUAL_P2:
            return (uint8_t)snprintf(buf, size, "%cP2   :%5lu us", mark,
                                     (unsigned long)p->dual_p2_us);

        case PULSE_ROW_CS_EN:
            return (uint8_t)snprintf(buf, size, "%cClrEn:%-3s", mark,
                                     p->cs_enabled ? "ON" : "OFF");

        case PULSE_ROW_CS_FREQ:
            return (uint8_t)snprintf(buf, size, "%cFreq :%5lu Hz", mark,
                                     (unsigned long)p->cs_freq_hz);

        case PULSE_ROW_CS_CLRW:
            return (uint8_t)snprintf(buf, size, "%cClrW :%5lu us", mark,
                                     (unsigned long)p->cs_clrw_us);

        case PULSE_ROW_CS_CLRI:
            return (uint8_t)snprintf(buf, size, "%cClrI :%5lu us", mark,
                                     (unsigned long)p->cs_clri_us);

        case PULSE_ROW_CS_PULSE:
            return (uint8_t)snprintf(buf, size, "%cPulse:%5lu us", mark,
                                     (unsigned long)p->cs_pulse_us);

        case PULSE_ROW_TRIGGER:
            /* 触发按键样式: [GO] */
            return (uint8_t)snprintf(buf, size, "%cTrig [GO]", mark);

        case PULSE_ROW_CHSEL:
            /* 子页跳转行: 行尾'->'指示 */
            return (uint8_t)snprintf(buf, size, "%cCHSel ->", mark);

        case PULSE_ROW_TIM_FREQ:
            if (idx == 2u)      /* 同步共享频率行(T3=T4) */
            {
                return (uint8_t)snprintf(buf, size, "%cT3/4F:%6lu Hz", mark,
                                         (unsigned long)p->freq_hz[0]);
            }

            return (uint8_t)snprintf(buf, size, "%cT%uF  :%6lu Hz", mark,
                                     (unsigned)(idx + 3u), (unsigned long)p->freq_hz[idx]);

        case PULSE_ROW_CH_DUTY:
            /* 占空比万分比: 1=0.01%, 显示 xx.xx% */
            return (uint8_t)snprintf(buf, size, "%c%s D:%2u.%02u%%", mark,
                                     pulse_ch_name(idx),
                                     (unsigned)(p->ch[idx].duty_bp / 100u),
                                     (unsigned)(p->ch[idx].duty_bp % 100u));

        case PULSE_ROW_CH_PHASE:
            return (uint8_t)snprintf(buf, size, "%c%s P:%2u%%", mark,
                                     pulse_ch_name(idx), p->ch[idx].phase_pct);

        default:
            buf[0] = '\0';
            return 0u;
    }
}

void display_show_main_page(const pulse_page_t *page, uint8_t cursor,
                            uint8_t scroll_top, uint8_t mode_selecting,
                            uint32_t uptime_s)
{
    const pulse_param_t *p = pulse_param_get();
    char line[20];

    /* 4行滚动窗口: 所有行(含模式行)同权滚动 */
    for (uint8_t k = 0u; k < 4u; k++)
    {
        uint8_t page_row = (uint8_t)(scroll_top + k);

        if (page_row < page->count)
        {
            main_row_text(line, sizeof(line), page, page_row, p, cursor,
                          mode_selecting, uptime_s);
        }
        else
        {
            snprintf(line, sizeof(line), "%-16s", "");
        }

        pad_line(line, 16u);
        oled_show_string(0, (uint8_t)(16u * k), line, 16);
    }

    oled_refresh_gram();
}

void display_show_ch_page(const pulse_param_t *p, uint8_t cursor, uint8_t scroll_top)
{
    char line[20];
    uint8_t total = (uint8_t)(PULSE_CH_COUNT + 2u);    /* Back置顶 + 4通道行 + Sync行 */

    for (uint8_t k = 0u; k < 4u; k++)
    {
        uint8_t row = (uint8_t)(scroll_top + k);
        uint8_t mark = (row == cursor) ? '>' : ' ';

        if (row == 0u)      /* 返回行置顶 */
        {
            snprintf(line, sizeof(line), "%cBack", mark);
        }
        else if (row < PULSE_CH_COUNT + 1u)
        {
            snprintf(line, sizeof(line), "%c%s %s", mark,
                     pulse_ch_name_full((uint8_t)(row - 1u)),
                     p->ch[row - 1u].enabled ? " ON " : " OFF");
        }
        else if (row == (uint8_t)(PULSE_CH_COUNT + 1u))     /* 同步选项(T3主T4从) */
        {
            snprintf(line, sizeof(line), "%cSync %s", mark,
                     p->pwm_sync ? " ON " : " OFF");
        }
        else
        {
            snprintf(line, sizeof(line), "%-16s", "");
        }

        pad_line(line, 16u);
        oled_show_string(0, (uint8_t)(16u * k), line, 16);
    }

    oled_refresh_gram();
}

void display_show_edit_page(const pulse_edit_info_t *info)
{
    char line[24];

    snprintf(line, sizeof(line), "Edit:%s", info->name);
    pad_line(line, 16u);
    oled_show_string(0, 0, line, 16);

    if (info->bp)       /* 万分比: xx.xx 显示 */
    {
        snprintf(line, sizeof(line), "%lu.%02lu %s",
                 (unsigned long)(info->value / 100u),
                 (unsigned long)(info->value % 100u), info->unit);
    }
    else
    {
        snprintf(line, sizeof(line), "%lu %s", (unsigned long)info->value, info->unit);
    }

    pad_line(line, 10u);
    oled_show_string(0, 16, line, 24);

    if (info->bp)
    {
        snprintf(line, sizeof(line), "%lu.%02lu~%lu.%02lu %s",
                 (unsigned long)(info->min / 100u), (unsigned long)(info->min % 100u),
                 (unsigned long)(info->max / 100u), (unsigned long)(info->max % 100u),
                 info->unit);
    }
    else
    {
        snprintf(line, sizeof(line), "%lu~%lu %s", (unsigned long)info->min,
                 (unsigned long)info->max, info->unit);
    }

    pad_line(line, 21u);
    oled_show_string(0, 40, line, 12);

    snprintf(line, sizeof(line), "OK=save LP=cancel");
    pad_line(line, 21u);
    oled_show_string(0, 52, line, 12);

    oled_refresh_gram();
}
