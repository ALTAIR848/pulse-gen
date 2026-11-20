/**
 * @file    pulse.h
 * @brief   脉冲参数模块: 模式管理 + 按模式独立参数 + 行模型 + 编辑封装(不碰硬件)
 */
#ifndef __PULSE_H
#define __PULSE_H

#include <stdint.h>

/* ---- 模式 ---- */
typedef enum
{
    PULSE_MODE_DUAL = 0,        /* 双脉冲 */
    PULSE_MODE_PWM,             /* PWM */
    PULSE_MODE_CLEAR_SINGLE,    /* 清零+单脉冲 */
    PULSE_MODE_COUNT
} pulse_mode_t;

/* PWM 通道(全局索引): T3为主,T4为从
 * 可用通道: TIM3 CH1(PA6)/CH3(PB0)/CH4(PB1), TIM4 CH1(PB6)/CH2(PB7)/CH3(PB8)
 * (TIM3_CH2=PA7 与 TIM4_CH4=PB9 板上不可用)
 */
typedef enum
{
    PULSE_CH_TIM3_C1 = 0,
    PULSE_CH_TIM3_C3,
    PULSE_CH_TIM3_C4,
    PULSE_CH_TIM4_C1,
    PULSE_CH_TIM4_C2,
    PULSE_CH_TIM4_C3,
    PULSE_CH_COUNT
} pulse_ch_idx_t;

#define PULSE_TIM_COUNT     2u
#define PULSE_FREQ_MIN      1u
#define PULSE_FREQ_MAX      10000u
#define PULSE_DUTY_MAX      100u    /* 占空比 0~100% */
#define PULSE_PHASE_MAX     99u     /* 相位 0~99% */

/* PWM 通道参数 */
typedef struct
{
    uint8_t enabled;        /* 勾选 */
    uint8_t duty_pct;       /* 0~100 */
    uint8_t phase_pct;      /* 0~99 */
} pulse_ch_t;

/* 参数集(按模式独立保存) */
typedef struct
{
    pulse_mode_t mode;
    uint8_t pwm_enabled;                    /* PWM模式总使能 0/1 */
    uint8_t pwm_sync;                       /* T2/T3同步: 1=共用一个频率(T2主,T3从) */
    uint32_t freq_hz[PULSE_TIM_COUNT];      /* PWM: 每个TIM一个频率 */
    pulse_ch_t ch[PULSE_CH_COUNT];          /* PWM: 通道参数 */

    /* 双脉冲模式 */
    uint32_t dual_p1_us;                    /* 脉冲1宽度 */
    uint32_t dual_gap_us;                   /* 两脉冲间隔 */
    uint32_t dual_p2_us;                    /* 脉冲2宽度 */

    /* 清零+单脉冲模式 */
    uint32_t cs_freq_hz;                    /* 清零脉冲频率 */
    uint32_t cs_clrw_us;                    /* 清零脉冲宽度 */
    uint32_t cs_clri_us;                    /* 清零脉冲间隔 */
    uint32_t cs_pulse_us;                   /* 触发后的单脉冲宽度 */
} pulse_param_t;

/* ---- 主页行模型(显示与编辑共用) ---- */
typedef enum
{
    PULSE_ROW_MODE = 0,     /* 模式行(主页第一行) */
    PULSE_ROW_PWM_EN,       /* PWM: 总使能(布尔, 确认直接翻转) */
    PULSE_ROW_CHSEL,        /* PWM: 通道选择入口 */
    PULSE_ROW_TIM_FREQ,     /* PWM: TIM频率, idx=tim(0/1) */
    PULSE_ROW_CH_DUTY,      /* PWM: 通道占空比, idx=通道全局索引 */
    PULSE_ROW_CH_PHASE,     /* PWM: 通道相位, idx=通道全局索引 */
    PULSE_ROW_DUAL_P1,      /* 双脉冲: 脉冲1宽度us */
    PULSE_ROW_DUAL_GAP,     /* 双脉冲: 间隔us */
    PULSE_ROW_DUAL_P2,      /* 双脉冲: 脉冲2宽度us */
    PULSE_ROW_CS_FREQ,      /* 清零+单脉冲: 频率Hz */
    PULSE_ROW_CS_CLRW,      /* 清零+单脉冲: 清零脉冲宽度us */
    PULSE_ROW_CS_CLRI,      /* 清零+单脉冲: 清零间隔us */
    PULSE_ROW_CS_PULSE,     /* 清零+单脉冲: 脉冲宽度us */
    PULSE_ROW_TRIGGER       /* 触发脉冲(动作行, 确认一次触发一次) */
} pulse_row_type_t;

#define PULSE_PAGE_MAX_ROWS 17u

typedef struct
{
    uint8_t count;
    struct
    {
        uint8_t type;       /* pulse_row_type_t */
        uint8_t idx;        /* TIM_FREQ: tim索引; DUTY/PHASE: 通道索引 */
    } row[PULSE_PAGE_MAX_ROWS];
} pulse_page_t;

/* ---- 编辑信息(编辑子页用) ---- */
typedef struct
{
    const char *name;       /* 参数名, 如 "Freq T1" / "Duty T1C1" */
    const char *unit;       /* "Hz" / "%" */
    uint32_t value;
    uint32_t min;
    uint32_t max;
    uint8_t seq_step;       /* 1=1/2/5序列步进, 0=±1线性步进 */
} pulse_edit_info_t;

/* ---- API ---- */
void pulse_param_init(void);
const pulse_param_t *pulse_param_get(void);
void pulse_param_set_mode(pulse_mode_t m);      /* 只切模式, 各模式参数互不影响 */
void pulse_ch_toggle(uint8_t ch_idx);           /* 通道勾选翻转 */
void pulse_pwm_toggle(void);                    /* PWM总使能翻转 */
void pulse_sync_toggle(void);                   /* T2/T3频率同步翻转 */

void pulse_build_main_page(pulse_page_t *page); /* 按当前模式生成主页行列表 */
const char *pulse_mode_name(pulse_mode_t m);
const char *pulse_ch_name(uint8_t ch_idx);      /* 短名 T1C1 */
const char *pulse_ch_name_full(uint8_t ch_idx); /* 全名 TIM1_CH1 */

uint8_t pulse_begin_edit(uint8_t row_type, uint8_t idx, pulse_edit_info_t *info); /* 1=成功 */
void pulse_edit_step(int8_t dir);               /* 编辑中修改值(步进+钳制) */
uint32_t pulse_edit_value(void);                /* 当前编辑中的值(刷新显示用) */
void pulse_edit_cancel(void);                   /* 取消, 恢复进入编辑前的值 */
void pulse_edit_commit(void);                   /* 保存(值已实时生效, 预留FLASH写入点) */

void pulse_param_save(void);                    /* 预留: FLASH掉电保存 */
void pulse_param_load(void);                    /* 预留: FLASH加载 */

void pulse_trigger_fire(void);                  /* 触发脉冲请求(硬件未实现前仅计数) */
uint32_t pulse_trigger_count(void);             /* 累计触发次数 */

#endif /* __PULSE_H */
