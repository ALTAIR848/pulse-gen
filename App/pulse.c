/**
 * @file    pulse.c
 * @brief   脉冲参数模块实现
 * @note    各模式参数独立保存, 切换模式互不影响;
 *          频率步进用 1/2/5 序列(遥控器手感), 占空比/相位 ±1 线性步进
 */
#include "pulse.h"
#include "stdio.h"
#include "stmflash.h"

static pulse_param_t g_pulse;

/* ---- 名称表 ---- */
static const char *const s_mode_name[PULSE_MODE_COUNT] = {"Dual", "PWM", "Clr+1P"};
static const char *const s_ch_name[PULSE_CH_COUNT] = {
    "T3C1", "T3C3", "T3C4", "T4C1", "T4C2", "T4C3"
};
static const char *const s_ch_name_full[PULSE_CH_COUNT] = {
    "TIM3_CH1", "TIM3_CH3", "TIM3_CH4", "TIM4_CH1", "TIM4_CH2", "TIM4_CH3"
};

/* ---- 编辑上下文 ---- */
static uint8_t s_edit_active = 0;
static uint8_t s_edit_row_kind = 0;     /* pulse_row_type_t, 区分±1线性参数 */
static uint8_t s_edit_idx = 0;          /* 占空比/相位用通道索引 */
static uint32_t *s_edit_ptr = 0;        /* 直接指向被编辑的参数(通用方案) */
static uint32_t s_edit_backup = 0;
static char s_edit_name[16];

/* ---- 触发计数 ---- */
static uint32_t s_trig_count = 0;

void pulse_param_init(void)
{
    g_pulse.mode = PULSE_MODE_DUAL;

    for (uint32_t t = 0u; t < PULSE_TIM_COUNT; t++)
    {
        g_pulse.freq_hz[t] = 1000u;
    }

    for (uint32_t c = 0u; c < PULSE_CH_COUNT; c++)
    {
        g_pulse.ch[c].enabled = 0u;
        g_pulse.ch[c].duty_bp = 5000u;      /* 50.00% */
        g_pulse.ch[c].phase_pct = 0u;
    }

    g_pulse.pwm_enabled = 0u;
    g_pulse.pwm_sync = 0u;

    /* 双脉冲默认值 */
    g_pulse.dual_p1_us = 10u;
    g_pulse.dual_gap_us = 10u;
    g_pulse.dual_p2_us = 10u;

    /* 清零+单脉冲默认值 */
    g_pulse.cs_enabled = 0u;        /* 进模式默认禁用清零 */
    g_pulse.cs_freq_hz = 1000u;
    g_pulse.cs_clrw_us = 10u;
    g_pulse.cs_clri_us = 10u;
    g_pulse.cs_pulse_us = 10u;
}

const pulse_param_t *pulse_param_get(void)
{
    return &g_pulse;
}

void pulse_param_set_mode(pulse_mode_t m)
{
    if (m < PULSE_MODE_COUNT)
    {
        g_pulse.mode = m;
        pulse_param_save();
    }
}

void pulse_ch_toggle(uint8_t ch_idx)
{
    if (ch_idx < PULSE_CH_COUNT)
    {
        g_pulse.ch[ch_idx].enabled = !g_pulse.ch[ch_idx].enabled;
        pulse_param_save();
    }
}

void pulse_pwm_toggle(void)
{
    g_pulse.pwm_enabled = !g_pulse.pwm_enabled;
    pulse_param_save();
}

void pulse_sync_toggle(void)
{
    g_pulse.pwm_sync = !g_pulse.pwm_sync;
    pulse_param_save();
}

void pulse_cs_toggle(void)
{
    g_pulse.cs_enabled = !g_pulse.cs_enabled;
    pulse_param_save();
}

void pulse_cs_disable(void)
{
    g_pulse.cs_enabled = 0u;
}

const char *pulse_mode_name(pulse_mode_t m)
{
    return (m < PULSE_MODE_COUNT) ? s_mode_name[m] : "?";
}

const char *pulse_ch_name(uint8_t ch_idx)
{
    return (ch_idx < PULSE_CH_COUNT) ? s_ch_name[ch_idx] : "?";
}

const char *pulse_ch_name_full(uint8_t ch_idx)
{
    return (ch_idx < PULSE_CH_COUNT) ? s_ch_name_full[ch_idx] : "?";
}

/**
 * @brief  按当前模式生成主页行列表(第一行固定为模式行)
 */
void pulse_build_main_page(pulse_page_t *page)
{
    page->count = 0u;
    page->row[page->count].type = (uint8_t)PULSE_ROW_MODE;
    page->row[page->count].idx = 0u;
    page->count++;

    if (g_pulse.mode == PULSE_MODE_PWM)
    {
        /* PWM总使能(布尔) */
        page->row[page->count].type = (uint8_t)PULSE_ROW_PWM_EN;
        page->row[page->count].idx = 0u;
        page->count++;

        /* 通道选择入口 */
        page->row[page->count].type = (uint8_t)PULSE_ROW_CHSEL;
        page->row[page->count].idx = 0u;
        page->count++;

        /* 每个 TIM: 有勾选通道才显示频率行, 之后跟各勾选通道的占空/相位 */
        if (g_pulse.pwm_sync)
        {
            /* 同步模式(T3主T4从): 两TIM共用一个频率, 只显示一行(idx=2表示共享) */
            uint8_t has_t2 = (g_pulse.ch[0].enabled != 0u) || (g_pulse.ch[1].enabled != 0u);
            uint8_t has_t3 = (g_pulse.ch[2].enabled != 0u) || (g_pulse.ch[3].enabled != 0u);

            if (has_t2 || has_t3)
            {
                page->row[page->count].type = (uint8_t)PULSE_ROW_TIM_FREQ;
                page->row[page->count].idx = 2u;    /* 共享频率行 */
                page->count++;
            }

            for (uint32_t c = 0u; c < PULSE_CH_COUNT; c++)
            {
                if (g_pulse.ch[c].enabled != 0u)
                {
                    page->row[page->count].type = (uint8_t)PULSE_ROW_CH_DUTY;
                    page->row[page->count].idx = c;
                    page->count++;

                    page->row[page->count].type = (uint8_t)PULSE_ROW_CH_PHASE;
                    page->row[page->count].idx = c;
                    page->count++;
                }
            }
        }
        else
        {
            for (uint32_t t = 0u; t < PULSE_TIM_COUNT; t++)
            {
                uint8_t ch_lo = (uint8_t)(t * 3u);      /* 每TIM 3通道 */
                uint8_t has = (g_pulse.ch[ch_lo].enabled != 0u) ||
                              (g_pulse.ch[ch_lo + 1u].enabled != 0u) ||
                              (g_pulse.ch[ch_lo + 2u].enabled != 0u);

                if (has)
                {
                    page->row[page->count].type = (uint8_t)PULSE_ROW_TIM_FREQ;
                    page->row[page->count].idx = (uint8_t)t;
                    page->count++;

                    for (uint32_t k = 0u; k < 3u; k++)
                    {
                        uint8_t c = (uint8_t)(ch_lo + k);

                        if (g_pulse.ch[c].enabled != 0u)
                        {
                            page->row[page->count].type = (uint8_t)PULSE_ROW_CH_DUTY;
                            page->row[page->count].idx = c;
                            page->count++;

                            page->row[page->count].type = (uint8_t)PULSE_ROW_CH_PHASE;
                            page->row[page->count].idx = c;
                            page->count++;
                        }
                    }
                }
            }
        }
    }
    else if (g_pulse.mode == PULSE_MODE_DUAL)   /* 双脉冲 */
    {
        static const uint8_t rows[4] = {
            (uint8_t)PULSE_ROW_DUAL_P1, (uint8_t)PULSE_ROW_DUAL_GAP,
            (uint8_t)PULSE_ROW_DUAL_P2, (uint8_t)PULSE_ROW_TRIGGER
        };

        for (uint32_t i = 0u; i < 4u; i++)
        {
            page->row[page->count].type = rows[i];
            page->row[page->count].idx = 0u;
            page->count++;
        }
    }
    else    /* 清零+单脉冲 */
    {
        static const uint8_t rows[6] = {
            (uint8_t)PULSE_ROW_CS_EN, (uint8_t)PULSE_ROW_TRIGGER,
            (uint8_t)PULSE_ROW_CS_FREQ, (uint8_t)PULSE_ROW_CS_CLRW,
            (uint8_t)PULSE_ROW_CS_CLRI, (uint8_t)PULSE_ROW_CS_PULSE
        };

        for (uint32_t i = 0u; i < 6u; i++)
        {
            page->row[page->count].type = rows[i];
            page->row[page->count].idx = 0u;
            page->count++;
        }
    }
}

/* ---- 编辑封装 ---- */

/**
 * @brief  进入编辑: 解析行类型到参数指针, 填充编辑信息并备份当前值
 * @return 1=成功进入编辑, 0=该行不可编辑
 */
uint8_t pulse_begin_edit(uint8_t row_type, uint8_t idx, pulse_edit_info_t *info)
{
    uint32_t *ptr = 0;
    const char *unit = "us";
    uint32_t min = 1u;
    uint32_t max = 65535u;

    info->bp = 0u;      /* 默认整数显示, 占空比分支单独置1(万分比) */

    switch ((pulse_row_type_t)row_type)
    {
        case PULSE_ROW_TIM_FREQ:
            if (idx > PULSE_TIM_COUNT)      /* 0=T2 1=T3 2=同步共享 */
            {
                return 0u;
            }

            if (idx == 2u)
            {
                snprintf(s_edit_name, sizeof(s_edit_name), "Freq T3=T4");
                ptr = &g_pulse.freq_hz[0];
            }
            else
            {
                snprintf(s_edit_name, sizeof(s_edit_name), "Freq T%u", (unsigned)(idx + 2u));
                ptr = &g_pulse.freq_hz[idx];
            }

            unit = "Hz";
            min = PULSE_FREQ_MIN;
            max = PULSE_FREQ_MAX;
            break;

        case PULSE_ROW_CH_DUTY:
            if (idx >= PULSE_CH_COUNT)
            {
                return 0u;
            }

            snprintf(s_edit_name, sizeof(s_edit_name), "Duty %s", pulse_ch_name(idx));
            /* 占空比走 ±1(0.01%) 线性步进, 挂到通道字段上 */
            info->name = s_edit_name;
            info->unit = "%";
            info->value = g_pulse.ch[idx].duty_bp;
            info->min = 0u;
            info->max = PULSE_DUTY_MAX;
            info->bp = 1u;                      /* 编辑页按 xx.xx 显示 */
            s_edit_ptr = 0;
            s_edit_backup = g_pulse.ch[idx].duty_bp;
            s_edit_row_kind = (uint8_t)PULSE_ROW_CH_DUTY;
            s_edit_idx = idx;
            s_edit_active = 1u;
            return 1u;

        case PULSE_ROW_CH_PHASE:
            if (idx >= PULSE_CH_COUNT)
            {
                return 0u;
            }

            snprintf(s_edit_name, sizeof(s_edit_name), "Phase %s", pulse_ch_name(idx));
            info->name = s_edit_name;
            info->unit = "%";
            info->value = g_pulse.ch[idx].phase_pct;
            info->min = 0u;
            info->max = PULSE_PHASE_MAX;
            s_edit_ptr = 0;
            s_edit_backup = g_pulse.ch[idx].phase_pct;
            s_edit_row_kind = (uint8_t)PULSE_ROW_CH_PHASE;
            s_edit_idx = idx;
            s_edit_active = 1u;
            return 1u;

        case PULSE_ROW_DUAL_P1:
            snprintf(s_edit_name, sizeof(s_edit_name), "P1");
            ptr = &g_pulse.dual_p1_us;
            break;

        case PULSE_ROW_DUAL_GAP:
            snprintf(s_edit_name, sizeof(s_edit_name), "Gap");
            ptr = &g_pulse.dual_gap_us;
            break;

        case PULSE_ROW_DUAL_P2:
            snprintf(s_edit_name, sizeof(s_edit_name), "P2");
            ptr = &g_pulse.dual_p2_us;
            break;

        case PULSE_ROW_CS_FREQ:
            snprintf(s_edit_name, sizeof(s_edit_name), "Freq");
            unit = "Hz";
            ptr = &g_pulse.cs_freq_hz;
            min = PULSE_FREQ_MIN;
            max = PULSE_FREQ_MAX;
            break;

        case PULSE_ROW_CS_CLRW:
            snprintf(s_edit_name, sizeof(s_edit_name), "ClrW");
            ptr = &g_pulse.cs_clrw_us;
            break;

        case PULSE_ROW_CS_CLRI:
            snprintf(s_edit_name, sizeof(s_edit_name), "ClrI");
            ptr = &g_pulse.cs_clri_us;
            break;

        case PULSE_ROW_CS_PULSE:
            snprintf(s_edit_name, sizeof(s_edit_name), "Pulse");
            ptr = &g_pulse.cs_pulse_us;
            break;

        default:
            return 0u;
    }

    info->name = s_edit_name;
    info->unit = unit;
    info->value = *ptr;
    info->min = min;
    info->max = max;
    s_edit_ptr = ptr;
    s_edit_backup = *ptr;
    s_edit_row_kind = row_type;
    s_edit_active = 1u;
    return 1u;
}

/**
 * @brief  编辑中步进: 线性 ±(1*mult), 钳制在[min,max]
 * @param  dir: +1增大 / -1减小
 * @param  mult: 步进倍率(按住加速用, 1/10/100); 小范围参数(百分比)自动封顶10
 */
void pulse_edit_step(int8_t dir, uint32_t mult)
{
    if (!s_edit_active)
    {
        return;
    }

    if (mult == 0u)
    {
        mult = 1u;
    }

    if (s_edit_row_kind == (uint8_t)PULSE_ROW_CH_DUTY)
    {
        /* 占空比: 万分比 0~10000, ±1=0.01%, 倍率封顶100(=1%/拍) */
        uint16_t *pv = &g_pulse.ch[s_edit_idx].duty_bp;
        int32_t m = (mult > 100u) ? 100u : (int32_t)mult;
        int32_t v = (int32_t)*pv + (int32_t)dir * m;

        if (v < 0)
        {
            v = 0;
        }

        if (v > (int32_t)PULSE_DUTY_MAX)
        {
            v = (int32_t)PULSE_DUTY_MAX;
        }

        *pv = (uint16_t)v;
        return;
    }

    if (s_edit_row_kind == (uint8_t)PULSE_ROW_CH_PHASE)
    {
        /* 相位: 百分比 0~99, ±1=1%, 倍率封顶10 */
        uint8_t *pv = &g_pulse.ch[s_edit_idx].phase_pct;
        int32_t m = (mult > 10u) ? 10u : (int32_t)mult;
        int32_t v = (int32_t)*pv + (int32_t)dir * m;

        if (v < 0)
        {
            v = 0;
        }

        if (v > (int32_t)PULSE_PHASE_MAX)
        {
            v = (int32_t)PULSE_PHASE_MAX;
        }

        *pv = (uint8_t)v;
        return;
    }

    if (s_edit_ptr != 0)
    {
        uint32_t max = (s_edit_ptr == &g_pulse.freq_hz[0] || s_edit_ptr == &g_pulse.freq_hz[1] ||
                        s_edit_ptr == &g_pulse.cs_freq_hz)
                       ? PULSE_FREQ_MAX : 65535u;
        uint32_t step = (mult > max) ? max : mult;

        if (dir > 0)
        {
            uint32_t v = *s_edit_ptr + step;

            *s_edit_ptr = (v > max) ? max : v;
        }
        else
        {
            *s_edit_ptr = (*s_edit_ptr > step) ? (*s_edit_ptr - step) : 1u;
        }
    }
}

uint32_t pulse_edit_value(void)
{
    if (!s_edit_active)
    {
        return 0u;
    }

    if ((s_edit_row_kind == (uint8_t)PULSE_ROW_CH_DUTY) ||
        (s_edit_row_kind == (uint8_t)PULSE_ROW_CH_PHASE))
    {
        return (s_edit_row_kind == (uint8_t)PULSE_ROW_CH_DUTY)
               ? (uint32_t)g_pulse.ch[s_edit_idx].duty_bp
               : (uint32_t)g_pulse.ch[s_edit_idx].phase_pct;
    }

    return (s_edit_ptr != 0) ? *s_edit_ptr : 0u;
}

void pulse_edit_cancel(void)
{
    if (!s_edit_active)
    {
        return;
    }

    if (s_edit_ptr != 0)
    {
        *s_edit_ptr = s_edit_backup;
    }
    else if (s_edit_row_kind == (uint8_t)PULSE_ROW_CH_DUTY)
    {
        g_pulse.ch[s_edit_idx].duty_bp = (uint16_t)s_edit_backup;
    }
    else if (s_edit_row_kind == (uint8_t)PULSE_ROW_CH_PHASE)
    {
        g_pulse.ch[s_edit_idx].phase_pct = (uint8_t)s_edit_backup;
    }

    s_edit_active = 0u;
}

void pulse_edit_commit(void)
{
    /* 值已实时生效: 提交即写FLASH掉电保存 */
    pulse_param_save();
    s_edit_active = 0u;
}

/* ---- 触发脉冲(硬件未实现前仅计数, main 层做LED/串口反馈) ---- */
void pulse_trigger_fire(void)
{
    s_trig_count++;
}

uint32_t pulse_trigger_count(void)
{
    return s_trig_count;
}

/* ---- 参数掉电保存(FLASH最后一页, 移植自例程19) ----
 * 页格式(半字序列):
 *   [0..1] magic = 0x53504C45 ("ELPS")
 *   [2]    payload尺寸(半字个数, 结构体布局变更防护)
 *   [3]    保留(0)
 *   [4..5] 校验和(payload半字累加和)
 *   [6..]  payload = pulse_param_t 字节流(小端, 奇数字节补0)
 * 使能类状态(pwm_enabled/cs_enabled)不持久化: 上电不自动开输出 */
#define PULSE_FLASH_ADDR    (STM32_FLASH_BASE + STM32_FLASH_SIZE - STM32_SECTOR_SIZE)
#define PULSE_SAVE_MAGIC    0x53504C45u
#define PULSE_SAVE_HDR      6u      /* 头部半字个数 */

void pulse_param_save(void)
{
    pulse_param_t snap = g_pulse;
    const uint16_t n = (uint16_t)((sizeof(pulse_param_t) + 1u) / 2u);
    uint16_t buf[PULSE_SAVE_HDR + ((sizeof(pulse_param_t) + 1u) / 2u)];
    const uint8_t *src = (const uint8_t *)&snap;
    uint16_t i;
    uint32_t sum = 0u;

    snap.pwm_enabled = 0u;      /* 使能态不持久化 */
    snap.cs_enabled = 0u;

    /* 结构体字节流 -> 小端半字(奇数字节补0) */
    for (i = 0u; i < n; i++)
    {
        uint16_t lo = src[2u * i];
        uint16_t hi = ((2u * i + 1u) < sizeof(pulse_param_t)) ? src[2u * i + 1u] : 0u;

        buf[PULSE_SAVE_HDR + i] = (uint16_t)(lo | (uint16_t)(hi << 8));
        sum += buf[PULSE_SAVE_HDR + i];
    }

    buf[0] = (uint16_t)(PULSE_SAVE_MAGIC & 0xFFFFu);
    buf[1] = (uint16_t)(PULSE_SAVE_MAGIC >> 16);
    buf[2] = n;
    buf[3] = 0u;
    buf[4] = (uint16_t)(sum & 0xFFFFu);
    buf[5] = (uint16_t)(sum >> 16);

    stmflash_write(PULSE_FLASH_ADDR, buf, (uint16_t)(PULSE_SAVE_HDR + n));
}

uint8_t pulse_param_load(void)
{
    pulse_param_t tmp;
    uint8_t *dst = (uint8_t *)&tmp;
    const uint16_t n = (uint16_t)((sizeof(pulse_param_t) + 1u) / 2u);
    uint16_t buf[PULSE_SAVE_HDR + ((sizeof(pulse_param_t) + 1u) / 2u)];
    uint16_t i;
    uint32_t sum = 0u;

    stmflash_read(PULSE_FLASH_ADDR, buf, (uint16_t)(PULSE_SAVE_HDR + n));

    /* magic / 尺寸 / 校验和 任一不符 -> 视为无有效数据, 保持默认值 */
    if ((buf[0] != (uint16_t)(PULSE_SAVE_MAGIC & 0xFFFFu)) ||
            (buf[1] != (uint16_t)(PULSE_SAVE_MAGIC >> 16)) ||
            (buf[2] != n))
    {
        return 0u;
    }

    for (i = 0u; i < n; i++)
    {
        sum += buf[PULSE_SAVE_HDR + i];
    }

    if ((buf[4] != (uint16_t)(sum & 0xFFFFu)) ||
            (buf[5] != (uint16_t)(sum >> 16)))
    {
        return 0u;
    }

    /* 半字 -> 结构体字节流 */
    for (i = 0u; i < n; i++)
    {
        dst[2u * i] = (uint8_t)(buf[PULSE_SAVE_HDR + i] & 0xFFu);

        if ((2u * i + 1u) < sizeof(pulse_param_t))
        {
            dst[2u * i + 1u] = (uint8_t)(buf[PULSE_SAVE_HDR + i] >> 8);
        }
    }

    tmp.pwm_enabled = 0u;       /* 使能态不持久化: 上电不自动开输出 */
    tmp.cs_enabled = 0u;
    g_pulse = tmp;
    return 1u;
}
