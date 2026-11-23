/**
 * @file    pwm.h
 * @brief   PWM 输出驱动(T3主+T4从, OC Toggle+DMA循环写CCR, 参照jxps-hipims-max-g2移植)
 * @note    纯硬件层, 不持有业务参数; main层组装 pwm_req_t 调 validate/apply
 */
#ifndef __PWM_H
#define __PWM_H

#include <stdint.h>

#define PWM_FREQ_MIN        1u
#define PWM_FREQ_MAX        100000u     /* 100kHz, 72MHz标定后实测再调 */
#define PWM_CH_TOTAL        6u          /* T3:CH1/CH3/CH4, T4:CH1/CH2/CH3 */

/* 占空比/相位常数 */
#define PWM_DUTY_0          0u          /* 常低 */
#define PWM_DUTY_100        10000u      /* 常高(占空比万分比, 1=0.01%) */
#define PWM_PHASE_MAX       99u

/* PWM 启动请求(与 pulse 参数解耦, main层负责组装) */
typedef struct
{
    uint8_t sync;                   /* 1=T4门控跟随T3共用频率, 0=T4独立 */
    uint32_t freq_hz[2];            /* [0]=TIM3 [1]=TIM4(sync=1时忽略[1]) */
    uint8_t ch_en[PWM_CH_TOTAL];
    uint16_t ch_duty[PWM_CH_TOTAL]; /* 0~10000 万分比(1=0.01%) */
    uint8_t ch_phase[PWM_CH_TOTAL]; /* 0~99, 相对周期起点偏移 */
} pwm_req_t;

uint8_t pwm_validate(const pwm_req_t *req); /* 1=配置合法(不碰硬件) */
uint8_t pwm_apply(const pwm_req_t *req);    /* 停-配-启, 1=成功(内部先validate) */
void pwm_stop(void);                        /* 全停, 引脚强制低 */
uint8_t pwm_is_running(void);

/* ---- 双脉冲单次输出 ---- */
uint8_t pwm_fire_dual(uint32_t p1_us, uint32_t gap_us, uint32_t p2_us);
    /* 1=已触发; 运行中/非法参数=0(丢弃)。固定全通道输出:
     * PWM模式的通道勾选仅作用于PWM模式, 双脉冲默认6路齐发 */
uint8_t pwm_oneshot_active(void);               /* 1=双脉冲序列运行中 */
uint8_t pwm_oneshot_poll(void);                 /* 主循环轮询, 序列刚完成返回1(内部收尾强制低) */

/* ---- 清零+单脉冲(参照 jxps-deep-osc sync_and_reset_set 架构) ----
 * T3主连续: CH1=清零信号(周期起点高电平clrw); CH3仅内部OC3REF不输出引脚,
 *           上升沿@clrw+clri 经 TRGO=OC3REF -> ITR2 触发 T4
 * T4从(TRIGGER+OPM): 触发沿出单脉冲(高电平pulse_w), 计满自停;
 *           平时卸装(SMS=DISABLE), fire时装载, 完成后(更新中断)再卸装
 *           -> 一次触发正好一个单脉冲, 与清零流同步 */
uint8_t pwm_cs_validate(uint32_t freq_hz, uint32_t clrw_us, uint32_t clri_us,
                        uint32_t pulse_us);     /* 1=合法(纯计算不碰硬件) */
void pwm_cs_apply(uint32_t freq_hz, uint32_t clrw_us, uint32_t clri_us,
                  uint32_t pulse_us, uint8_t clr_en);   /* 停-配-启; clr_en=1启动清零流 */
uint8_t pwm_cs_fire(void);      /* 1=已武装(下一清零周期出一个单脉冲); 清零未运行/武装中=0丢弃 */
uint8_t pwm_cs_poll(void);      /* 主循环轮询, 单脉冲刚完成返回1(内部已卸装) */

#endif /* __PWM_H */
