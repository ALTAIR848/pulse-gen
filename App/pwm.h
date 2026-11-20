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
#define PWM_DUTY_100        100u        /* 常高 */
#define PWM_PHASE_MAX       99u

/* PWM 启动请求(与 pulse 参数解耦, main层负责组装) */
typedef struct
{
    uint8_t sync;                   /* 1=T4门控跟随T3共用频率, 0=T4独立 */
    uint32_t freq_hz[2];            /* [0]=TIM3 [1]=TIM4(sync=1时忽略[1]) */
    uint8_t ch_en[PWM_CH_TOTAL];
    uint8_t ch_duty[PWM_CH_TOTAL];  /* 0~100 */
    uint8_t ch_phase[PWM_CH_TOTAL]; /* 0~99, 相对周期起点偏移 */
} pwm_req_t;

uint8_t pwm_validate(const pwm_req_t *req); /* 1=配置合法(不碰硬件) */
uint8_t pwm_apply(const pwm_req_t *req);    /* 停-配-启, 1=成功(内部先validate) */
void pwm_stop(void);                        /* 全停, 引脚强制低 */
uint8_t pwm_is_running(void);

#endif /* __PWM_H */
