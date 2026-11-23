/**
 * @file    pwm.c
 * @brief   PWM 输出驱动实现
 * @note    架构参照 jxps-hipims-max-g2/JXPS/Lib/Src/pwm.c 移植到 F103:
 *          - TIM3 主(TRGO=ENABLE) + CH1/CH3/CH4 输出; TIM4 门控从(ITR2) + CH1/CH2/CH3 输出
 *          - 任意相位/占空比: OC Toggle 模式 + DMA 循环写 CCR(每通道2个比较点)
 *          - 带参初始化: MX_TIM3/4_Init 拷贝参数化(psc/arr), 每次启动重配(同参考项目)
 *          - HAL 库的 HAL_TIM_OC_Start/Start_DMA 会在启动时打开定时器, 导致多通道
 *            时序不同步; 参考项目直接改 HAL 源码, 这里改为自实现"去启动版"(不动HAL文件)
 *          - 72MHz 时钟标定: MIN_CCR/MIN_ARR 为安全余量, 实测后调整
 */
#include "pwm.h"
#include "tim.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>      /* 临时: 双脉冲诊断打印, 波形验证通过后删除 */

/* ---- 常量(72MHz 标定初值) ---- */
#define PWM_TIM_CLOCK_HZ    72000000u
#define PWM_TIM_MAX_ARR     65535u
#define PWM_TIM_MIN_ARR     50u
#define PWM_MIN_CCR         15u     /* 相邻比较事件最小间隔(DMA响应余量) */
#define PWM_DUAL_LEAD_US    10u     /* 双脉冲前置量: 首个上升沿距计数起点 */

/* ---- 通道映射: 全局索引 -> (TIM, HAL通道, DMA句柄) ---- */
extern DMA_HandleTypeDef hdma_tim3_ch1_trig;
extern DMA_HandleTypeDef hdma_tim3_ch3;
extern DMA_HandleTypeDef hdma_tim3_ch4_up;
extern DMA_HandleTypeDef hdma_tim4_ch1;
extern DMA_HandleTypeDef hdma_tim4_ch2;
extern DMA_HandleTypeDef hdma_tim4_ch3;

typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t ch;                /* TIM_CHANNEL_x */
    uint32_t dma_req;           /* DIER 的 DMA 请求位掩码(TIM_DMA_CCx) */
    volatile uint32_t *ccr;     /* &TIMx->CCRy */
    DMA_HandleTypeDef *hdma;
} pwm_ch_map_t;

/* 索引顺序与 pulse 通道枚举一致: T3C1,T3C3,T3C4,T4C1,T4C2,T4C3
 * 注意: TIM_DMA_ID_CCx 是 hdma[] 数组索引(F1版 CC1=1 起, 与 UPDATE=0 错位),
 * 绝不能用它移位生成 DIER 掩码, 必须直接用 TIM_DMA_CCx 位掩码 */
static const pwm_ch_map_t s_map[PWM_CH_TOTAL] =
{
    {&htim3, TIM_CHANNEL_1, TIM_DMA_CC1, &TIM3->CCR1, &hdma_tim3_ch1_trig},
    {&htim3, TIM_CHANNEL_3, TIM_DMA_CC3, &TIM3->CCR3, &hdma_tim3_ch3},
    {&htim3, TIM_CHANNEL_4, TIM_DMA_CC4, &TIM3->CCR4, &hdma_tim3_ch4_up},
    {&htim4, TIM_CHANNEL_1, TIM_DMA_CC1, &TIM4->CCR1, &hdma_tim4_ch1},
    {&htim4, TIM_CHANNEL_2, TIM_DMA_CC2, &TIM4->CCR2, &hdma_tim4_ch2},
    {&htim4, TIM_CHANNEL_3, TIM_DMA_CC3, &TIM4->CCR3, &hdma_tim4_ch3},
};

/* 每通道计划模式(validate填写): 0=未启用 1=常低 2=常高 3=DMA翻转(不跨周期) 4=DMA翻转(跨周期) */
static uint8_t s_ch_mode[PWM_CH_TOTAL] = {0};
/* 每通道实际运行模式(apply成功后从s_ch_mode拷入, stop据此选择停止例程) */
static uint8_t s_run_mode[PWM_CH_TOTAL] = {0};
/* 每通道 CCR 序列(DMA半字传输, 必须紧凑排列) */
static uint16_t s_ccr[PWM_CH_TOTAL][2];

static uint8_t s_running = 0;

/* ---- PSC/ARR 计算(参照 cal_pwm_arr) ---- */
static uint8_t pwm_cal_arr(uint32_t freq, uint32_t *pPSC, uint32_t *pARR)
{
    uint32_t psc = 0u;
    uint32_t arr;

    do
    {
        psc++;
        arr = PWM_TIM_CLOCK_HZ / (psc * freq);
    } while (arr > PWM_TIM_MAX_ARR);

    if (arr < PWM_TIM_MIN_ARR)
    {
        return 0u;
    }

    *pPSC = psc;
    *pARR = arr;
    return 1u;
}

/* ---- 带参初始化(拷贝自生成的 MX_TIM3/4_Init, PSC/ARR 参数化) ---- */
static void pwm_tim3_init(uint32_t psc, uint32_t arr, uint32_t trgo)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = psc - 1u;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = arr - 1u;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    (void)HAL_TIM_Base_Init(&htim3);

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    (void)HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);
    (void)HAL_TIM_OC_Init(&htim3);

    /* TRGO 按模式选择: PWM/双脉冲=ENABLE(同步启动), 清零+单脉冲=OC3REF(CH3内部参考) */
    sMasterConfig.MasterOutputTrigger = trgo;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
    (void)HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

    sConfigOC.OCMode = TIM_OCMODE_FORCED_INACTIVE;
    sConfigOC.Pulse = 0u;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    (void)HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
    (void)HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);
    (void)HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4);

    HAL_TIM_MspPostInit(&htim3);
}

static void pwm_tim4_init(uint32_t psc, uint32_t arr, uint8_t slave_mode)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_SlaveConfigTypeDef sSlaveConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = psc - 1u;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = arr - 1u;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    (void)HAL_TIM_Base_Init(&htim4);

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    (void)HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig);
    (void)HAL_TIM_OC_Init(&htim4);

    /* 从模式: 0=独立 1=门控跟随T3(PWM同步) 2=触发启动(双脉冲同步起振) */
    switch (slave_mode)
    {
        case 1u:
            sSlaveConfig.SlaveMode = TIM_SLAVEMODE_GATED;
            break;

        case 2u:
            sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
            break;

        default:
            sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
            break;
    }

    sSlaveConfig.InputTrigger = TIM_TS_ITR2;
    (void)HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    (void)HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);

    sConfigOC.OCMode = TIM_OCMODE_FORCED_INACTIVE;
    sConfigOC.Pulse = 0u;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    (void)HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1);
    (void)HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2);
    (void)HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3);

    HAL_TIM_MspPostInit(&htim4);
}

/* ---- 启动辅助(自实现"去启动版", 状态机与 HAL 一致但不打开定时器) ---- */

/* 强制电平输出(常低/常高), 不使能定时器 */
static void pwm_ch_start_forced(TIM_HandleTypeDef *htim, uint32_t channel)
{
    TIM_CHANNEL_STATE_SET(htim, channel, HAL_TIM_CHANNEL_STATE_BUSY);
    TIM_CCxChannelCmd(htim->Instance, channel, TIM_CCx_ENABLE);
}

/* OC Toggle + DMA 循环写 CCR, 不使能定时器(参照 HAL_TIM_OC_Start_DMA 精简) */
static uint8_t pwm_ch_start_dma(TIM_HandleTypeDef *htim, uint32_t channel,
                                uint32_t dma_req, DMA_HandleTypeDef *hdma,
                                uint16_t *buf, uint16_t len,
                                volatile uint32_t *ccr_reg)
{
    if (TIM_CHANNEL_STATE_GET(htim, channel) != HAL_TIM_CHANNEL_STATE_READY)
    {
        return 0u;
    }

    TIM_CHANNEL_STATE_SET(htim, channel, HAL_TIM_CHANNEL_STATE_BUSY);

    if (HAL_DMA_Start(hdma, (uint32_t)buf, (uint32_t)ccr_reg, len) != HAL_OK)
    {
        TIM_CHANNEL_STATE_SET(htim, channel, HAL_TIM_CHANNEL_STATE_READY);
        return 0u;
    }

    /* HAL_DMA_Start(非IT版)成功后不释放锁, 下次启动会直接 BUSY, 手动解锁 */
    hdma->Lock = HAL_UNLOCKED;

    __HAL_TIM_ENABLE_DMA(htim, dma_req);
    TIM_CCxChannelCmd(htim->Instance, channel, TIM_CCx_ENABLE);
    return 1u;
}

/* 使能定时器(从先主后, 门控从模式使能后仍等待主门控) */
static void pwm_timer_enable(TIM_HandleTypeDef *htim)
{
    __HAL_TIM_ENABLE(htim);
}

/* ---- 校验: 纯计算不碰硬件, 合法则缓存执行计划 ---- */
typedef struct
{
    uint32_t psc[2];            /* [0]=TIM3 [1]=TIM4 */
    uint32_t arr[2];
} pwm_plan_t;

static pwm_plan_t s_plan;

/* 单通道 CCR 序列计算: 1=合法(结果写入 s_ccr/s_ch_mode 暂存) */
static uint8_t pwm_calc_ch(uint8_t ch, uint32_t arr, uint32_t duty, uint8_t phase)
{
    uint32_t w = ((uint32_t)duty * arr) / 10000u;    /* 脉宽计数(占空比万分比) */
    uint32_t r = ((uint32_t)phase * arr) / 100u;     /* 相位偏移计数 */

    if (duty == PWM_DUTY_0)
    {
        s_ch_mode[ch] = 1u;
        return 1u;
    }

    if (duty == PWM_DUTY_100)
    {
        s_ch_mode[ch] = 2u;
        return 1u;
    }

    /* DMA 相邻事件间隔余量: 脉宽与反相间隔都要够 */
    if ((w < PWM_MIN_CCR) || (arr - w < PWM_MIN_CCR))
    {
        return 0u;
    }

    if ((r + w) <= arr)
    {
        /* 脉冲不跨周期: 初始低, 上升沿r, 下降沿r+w
         * DMA序列必须降序[max,min]: 预装=min(数组末), min事件写入max, max事件写回min
         * 若存升序[min,max]且预装min, 首次写入仍是min -> 多一次min事件, 稳态极性反转
         * 注意: r=0(相位0)时预装CCR=0会与CNT=0持续匹配, 首拍DMA被预取,
         * 稳态相位错位一拍(D50看不出, 其他占空比会反相); 钳到1个tick避坑 */
        if (r == 0u)
        {
            r = 1u;
        }

        s_ch_mode[ch] = 3u;
        s_ccr[ch][0] = (uint16_t)(r + w);
        s_ccr[ch][1] = (uint16_t)r;
    }
    else
    {
        /* 脉冲跨周期: 初始高(低极性反相), 周期内事件序: fm(先) -> r(后)
         * 同样降序[r, fm], 预装=fm(数组末) */
        s_ch_mode[ch] = 4u;
        s_ccr[ch][0] = (uint16_t)r;
        s_ccr[ch][1] = (uint16_t)(r + w - arr);
    }
    return 1u;
}

uint8_t pwm_validate(const pwm_req_t *req)
{
    uint8_t i;

    if ((req->freq_hz[0] < PWM_FREQ_MIN) || (req->freq_hz[0] > PWM_FREQ_MAX))
    {
        return 0u;
    }

    if (pwm_cal_arr(req->freq_hz[0], &s_plan.psc[0], &s_plan.arr[0]) == 0u)
    {
        return 0u;
    }

    /* TIM4: 同步用TIM3的频率, 独立用自己的 */
    if (req->sync != 0u)
    {
        s_plan.psc[1] = s_plan.psc[0];
        s_plan.arr[1] = s_plan.arr[0];
    }
    else
    {
        if ((req->freq_hz[1] < PWM_FREQ_MIN) || (req->freq_hz[1] > PWM_FREQ_MAX))
        {
            return 0u;
        }

        if (pwm_cal_arr(req->freq_hz[1], &s_plan.psc[1], &s_plan.arr[1]) == 0u)
        {
            return 0u;
        }
    }

    for (i = 0u; i < PWM_CH_TOTAL; i++)
    {
        uint32_t arr = (i < 3u) ? s_plan.arr[0] : s_plan.arr[1];

        if (req->ch_en[i] == 0u)
        {
            s_ch_mode[i] = 0u;
            continue;
        }

        if (req->ch_duty[i] > PWM_DUTY_100)
        {
            return 0u;
        }

        if (pwm_calc_ch(i, arr, req->ch_duty[i], req->ch_phase[i]) == 0u)
        {
            return 0u;
        }
    }

    return 1u;
}

/* ---- 停止 ---- */
void pwm_stop(void)
{
    uint8_t i;

    /* 注意: 只按 s_run_mode(实际运行模式)选择停止例程, 不清 s_ch_mode(apply 的启动计划)。
     * 停止全部手动实现, 不用 HAL_TIM_OC_Stop/Stop_DMA:
     *  - OC_Stop_DMA 用 HAL_DMA_Abort_IT, 非中断启动的 DMA 无回调恢复状态, 会卡死在 ABORT 态
     *  - 部分版本还检查 htim->State, 与"去启动版"的状态管理不匹配 */
    for (i = 0u; i < PWM_CH_TOTAL; i++)
    {
        switch (s_run_mode[i])
        {
            case 3u:    /* DMA 翻转: 关DMA请求 -> 阻塞中止DMA(状态恢复READY) -> 关CCx */
            case 4u:
                __HAL_TIM_DISABLE_DMA(s_map[i].htim, s_map[i].dma_req);
                (void)HAL_DMA_Abort(s_map[i].hdma);
                TIM_CCxChannelCmd(s_map[i].htim->Instance, s_map[i].ch, TIM_CCx_DISABLE);
                TIM_CHANNEL_STATE_SET(s_map[i].htim, s_map[i].ch, HAL_TIM_CHANNEL_STATE_READY);
                break;

            case 1u:    /* 常低/常高 */
            case 2u:
                TIM_CCxChannelCmd(s_map[i].htim->Instance, s_map[i].ch, TIM_CCx_DISABLE);
                TIM_CHANNEL_STATE_SET(s_map[i].htim, s_map[i].ch, HAL_TIM_CHANNEL_STATE_READY);
                break;

            default:
                break;
        }

        s_run_mode[i] = 0u;
    }

    /* 引脚强制拉低(定时器已停, FORCED_INACTIVE 输出恒低) */
    {
        TIM_OC_InitTypeDef sConfigOC = {0};

        sConfigOC.OCMode = TIM_OCMODE_FORCED_INACTIVE;
        sConfigOC.Pulse = 0u;
        sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
        sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

        for (i = 0u; i < PWM_CH_TOTAL; i++)
        {
            (void)HAL_TIM_OC_ConfigChannel(s_map[i].htim, &sConfigOC, s_map[i].ch);
            /* 仅使能CCx(FORCED_INACTIVE输出恒低), 通道状态保持READY供下次启动 */
            TIM_CCxChannelCmd(s_map[i].htim->Instance, s_map[i].ch, TIM_CCx_ENABLE);
            TIM_CHANNEL_STATE_SET(s_map[i].htim, s_map[i].ch, HAL_TIM_CHANNEL_STATE_READY);
        }
    }

    /* 停机钳低: 6路CCx保持使能(FORCED_INACTIVE恒低), 但CEN必须直接清零 --
     * __HAL_TIM_DISABLE 宏会因 CCER 中 CCx 使能而拒清 CEN, 导致定时器空转 */
    TIM3->CR1 &= ~TIM_CR1_CEN;
    TIM4->CR1 &= ~TIM_CR1_CEN;
    /* 同时清OPM: 双脉冲fire曾置位, 而HAL的TIM_Base_SetConfig重配时保留CR1
     * 其他位(只清DIR/CMS/CKD/ARPE), OPM残留会使PWM模式只跑一个周期即停,
     * 通道冻结在最后一拍电平上(v3.3.4实测PWM输出全高) */
    TIM3->CR1 &= ~TIM_CR1_OPM;
    TIM4->CR1 &= ~TIM_CR1_OPM;
    /* 清T4更新中断: 清零+单脉冲模式的完成通知用UIE, 其他模式必须卸下,
     * 否则T4计满(或残留UIF)会持续进中断 */
    __HAL_TIM_DISABLE_IT(&htim4, TIM_IT_UPDATE);
    TIM4->SR = (uint16_t)~TIM_SR_UIF;
    /* 同理清T3 CC3中断(清零+单脉冲武装用), 防止残留到其他模式误触发 */
    __HAL_TIM_DISABLE_IT(&htim3, TIM_DIER_CC3IE);
    TIM3->SR = (uint16_t)~TIM_SR_CC3IF;
    s_running = 0u;
}

/* ---- 应用: 停-配-启 ---- */
uint8_t pwm_apply(const pwm_req_t *req)
{
    uint8_t i;

    if (pwm_validate(req) == 0u)
    {
        return 0u;
    }

    pwm_stop();

    /* 初始化定时器(先从后主启动前的准备) */
    pwm_tim3_init(s_plan.psc[0], s_plan.arr[0], TIM_TRGO_ENABLE);
    pwm_tim4_init(s_plan.psc[1], s_plan.arr[1], (req->sync != 0u) ? 1u : 0u);

    /* 逐通道配置 */
    for (i = 0u; i < PWM_CH_TOTAL; i++)
    {
        if (req->ch_en[i] == 0u)
        {
            continue;
        }

        switch (s_ch_mode[i])
        {
            case 1u:    /* 常低 */
            {
                TIM_OC_InitTypeDef sConfigOC = {0};

                sConfigOC.OCMode = TIM_OCMODE_FORCED_INACTIVE;
                sConfigOC.Pulse = 0u;
                sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
                sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
                (void)HAL_TIM_OC_ConfigChannel(s_map[i].htim, &sConfigOC, s_map[i].ch);
                pwm_ch_start_forced(s_map[i].htim, s_map[i].ch);
                break;
            }

            case 2u:    /* 常高 */
            {
                TIM_OC_InitTypeDef sConfigOC = {0};

                sConfigOC.OCMode = TIM_OCMODE_FORCED_ACTIVE;
                sConfigOC.Pulse = 0u;
                sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
                sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
                (void)HAL_TIM_OC_ConfigChannel(s_map[i].htim, &sConfigOC, s_map[i].ch);
                pwm_ch_start_forced(s_map[i].htim, s_map[i].ch);
                break;
            }

            case 3u:    /* 不跨周期: 初始低 */
            case 4u:    /* 跨周期: 初始高(用低极性反相实现, F103翻转输出初始恒为内部低) */
            {
                TIM_OC_InitTypeDef sConfigOC = {0};

                sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
                sConfigOC.Pulse = s_ccr[i][1];      /* 预装首个事件点(降序数组的末元素=min) */
                sConfigOC.OCPolarity = (s_ch_mode[i] == 4u) ? TIM_OCPOLARITY_LOW
                                       : TIM_OCPOLARITY_HIGH;
                sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
                (void)HAL_TIM_OC_ConfigChannel(s_map[i].htim, &sConfigOC, s_map[i].ch);

                /* 恢复循环模式(双脉冲fire曾清CIRC位做Normal单次传输) */
                s_map[i].hdma->Instance->CCR |= DMA_CCR_CIRC;

                if (pwm_ch_start_dma(s_map[i].htim, s_map[i].ch, s_map[i].dma_req,
                                     s_map[i].hdma, s_ccr[i], 2u, s_map[i].ccr) == 0u)
                {
                    pwm_stop();
                    return 0u;
                }

                break;
            }

            default:
                break;
        }
    }

    /* 先从后主(门控从模式先使能, 等待主输出使能电平) */
    pwm_timer_enable(&htim4);
    pwm_timer_enable(&htim3);

    /* 启动成功: 计划模式转正, 供下次 stop 选择停止例程 */
    for (i = 0u; i < PWM_CH_TOTAL; i++)
    {
        s_run_mode[i] = s_ch_mode[i];
    }

    s_running = 1u;
    return 1u;
}

uint8_t pwm_is_running(void)
{
    return s_running;
}

/* ---- 双脉冲单次输出(勾选通道同参同相; 全不勾选=默认所有通道) ----
 * OPM 单脉冲模式 + DMA(Normal) 三拍比较点序列, 正常极性(空闲低):
 *   预装 CCR=r1(首个事件点 = 启动后10us), 序列 [f1, r2, f2]:
 *     @r1 升(P1起) -> @f1 降(P1止) -> @r2 升(P2起) -> @f2 降(P2止)
 *   @f2 不再需要写CCR, 计到ARR由 OPM 自动停机, 输出保持低
 * v3.3.3: 极性还原(空闲低) + 前置量 r1=10us(不从0开始):
 *   上一版极性取反(使能即高), 但6路通道配置/启动的软件建立耗时全被
 *   计入首个脉宽(用户实测P1偏长)。现建立期间引脚保持低, 首个上升沿
 *   由 CEN 后10us 的比较点产生, 与建立时间无关。
 *   预装值非0同时避开 CNT=0"持续匹配"坑(v3.3错位教训, 见v3.3.1注)。
 * v3.3.2: hdma->Init.Mode 仅在 HAL_DMA_Init 中写CCR寄存器, HAL_DMA_Start
 *   不应用它(只改Init.Mode无效, DMA实际仍是Circular): 2拍走完后@f2回卷
 *   把r2写回CCR -> CCR停=r2/NDTR=1, 但f1/r2/f2三个翻转点都发生了, 波形
 *   碰巧正确, 仅诊断误导。现直接清/置CCR的CIRC位实现真Normal模式。 */
static uint16_t s_dual_seq[3];      /* [0]=f1, [1]=r2, [2]=f2 (半字紧凑) */
static uint32_t s_dual_f1 = 0;      /* 首个事件点r1, 诊断转储时机用 */
static uint8_t s_oneshot = 0;
static uint8_t s_mid_dumped = 0;

/* 临时诊断: 转储双脉冲关键寄存器(NDTR 顺序: T3C1,T3C3,T3C4,T4C1,T4C2,T4C3) */
static void pwm_dual_diag(const char *tag)
{
    printf(">> Dual %s: CNT=%u ARR=%u CCR3=%u/%u/%u SR=%04X DIER=%04X "
           "T4CCR=%u/%u/%u NDTR=%u/%u/%u/%u/%u/%u\r\n",
           tag,
           (unsigned int)TIM3->CNT, (unsigned int)TIM3->ARR,
           (unsigned int)TIM3->CCR1, (unsigned int)TIM3->CCR3, (unsigned int)TIM3->CCR4,
           (unsigned int)TIM3->SR, (unsigned int)TIM3->DIER,
           (unsigned int)TIM4->CCR1, (unsigned int)TIM4->CCR2, (unsigned int)TIM4->CCR3,
           (unsigned int)DMA1_Channel6->CNDTR, (unsigned int)DMA1_Channel2->CNDTR,
           (unsigned int)DMA1_Channel3->CNDTR, (unsigned int)DMA1_Channel1->CNDTR,
           (unsigned int)DMA1_Channel4->CNDTR, (unsigned int)DMA1_Channel5->CNDTR);
}

uint8_t pwm_fire_dual(uint32_t p1_us, uint32_t gap_us, uint32_t p2_us)
{
    /* 双脉冲固定全通道输出: PWM模式的通道勾选仅作用于PWM模式 */
    uint32_t total = PWM_DUAL_LEAD_US + p1_us + gap_us + p2_us;
    uint32_t psc = 1u;
    uint32_t arr;
    uint32_t r1;
    uint32_t f1;
    uint32_t r2;
    uint32_t f2;
    uint32_t min_gap;
    uint8_t i;

    if (total == 0u)
    {
        return 0u;
    }

    if (s_oneshot != 0u)
    {
        return 0u;      /* 序列运行中: 丢弃本次触发 */
    }

    /* PSC: 最小分频使总时长装进16位计数器(tick = psc/72 us) */
    for (;;)
    {
        arr = (total * (PWM_TIM_CLOCK_HZ / 1000000u) + psc - 1u) / psc;

        if (arr <= 65530u)
        {
            break;
        }

        psc++;

        if (psc > 65536u)
        {
            return 0u;
        }
    }

    /* 比较点(四舍五入取整): r1=前置量(10us), 之后依次 P1/Gap/P2 */
    r1 = (PWM_DUAL_LEAD_US * (PWM_TIM_CLOCK_HZ / 1000000u) + psc / 2u) / psc;

    if (r1 == 0u)
    {
        r1 = 1u;
    }

    f1 = r1 + (p1_us * (PWM_TIM_CLOCK_HZ / 1000000u) + psc / 2u) / psc;
    r2 = f1 + (gap_us * (PWM_TIM_CLOCK_HZ / 1000000u) + psc / 2u) / psc;
    f2 = r2 + (p2_us * (PWM_TIM_CLOCK_HZ / 1000000u) + psc / 2u) / psc;

    /* 事件间距(折算回时间)须 >=1us, 保证 DMA 响应; 不足说明分辨率不够 */
    min_gap = ((PWM_TIM_CLOCK_HZ / 1000000u) + psc - 1u) / psc;

    if ((f1 <= r1) || (r2 <= f1) || (f2 <= r2) ||
            (r1 < min_gap) || ((f1 - r1) < min_gap) ||
            ((r2 - f1) < min_gap) || ((f2 - r2) < min_gap))
    {
        return 0u;
    }

    if (arr <= f2)
    {
        arr = f2 + 4u;      /* 余量: 计满前完成最后一个下降沿 */
    }

    if (arr > 65535u)
    {
        return 0u;
    }

    /* 临时诊断: 打印参数与计算结果(启动前打印, 不占用序列窗口) */
    printf(">> Dual fire P1=%lu Gap=%lu P2=%lu psc=%lu arr=%lu r1=%lu f1=%lu r2=%lu f2=%lu\r\n",
           (unsigned long)p1_us, (unsigned long)gap_us, (unsigned long)p2_us,
           (unsigned long)psc, (unsigned long)arr,
           (unsigned long)r1, (unsigned long)f1, (unsigned long)r2, (unsigned long)f2);

    pwm_stop();             /* 确保干净(双脉冲模式下输出通常已停) */

    pwm_tim3_init(psc, arr, TIM_TRGO_ENABLE);
    pwm_tim4_init(psc, arr, 2u);    /* 双脉冲: T4触发从模式, 由T3的TRGO硬件同步起振 */

    s_dual_seq[0] = (uint16_t)f1;
    s_dual_seq[1] = (uint16_t)r2;
    s_dual_seq[2] = (uint16_t)f2;
    s_dual_f1 = r1;
    s_mid_dumped = 0u;

    for (i = 0u; i < PWM_CH_TOTAL; i++)
    {
        TIM_OC_InitTypeDef sConfigOC = {0};
        uint8_t ok;

        sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
        sConfigOC.Pulse = (uint16_t)r1;     /* 预装首个事件点: 启动后10us翻高 */
        sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH; /* 正常极性: 空闲低 */
        sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
        (void)HAL_TIM_OC_ConfigChannel(s_map[i].htim, &sConfigOC, s_map[i].ch);

        /* Normal模式: 直接清CCR的CIRC位(仅EN=0时可写; HAL_DMA_Start不应用
         * Init.Mode, 改hdma->Init.Mode无效): 传完3拍自动停, CNDTR=0 */
        s_map[i].hdma->Instance->CCR &= ~DMA_CCR_CIRC;

        ok = pwm_ch_start_dma(s_map[i].htim, s_map[i].ch, s_map[i].dma_req,
                              s_map[i].hdma, s_dual_seq, 3u, s_map[i].ccr);

        if (ok == 0u)
        {
            pwm_stop();
            return 0u;
        }

        s_run_mode[i] = 3u;     /* 标记运行模式, 收尾复用 pwm_stop 清理路径 */
    }

    /* 单脉冲模式: 计到ARR自动停机; UG 同步预装(参照参考项目 pwm_set) */
    TIM3->CR1 |= TIM_CR1_OPM;
    TIM4->CR1 |= TIM_CR1_OPM;
    TIM3->EGR = TIM_EGR_UG;
    TIM4->EGR = TIM_EGR_UG;

    /* 硬件同步启动: 先使能T4(触发从模式, CEN置位但等ITR2上升沿),
     * 再使能T3 -> TRGO=ENABLE同一时钟沿拉起T4(偏差~1个时钟);
     * T3的MSM=1把它自身启动延迟2拍与TRGO发出时刻对齐。
     * v3.3.4: 旧版软件先后写两个CEN, T3/T4起振有数百ns偏差(用户实测) */
    pwm_timer_enable(&htim4);
    pwm_timer_enable(&htim3);

    s_oneshot = 1u;
    return 1u;
}

uint8_t pwm_oneshot_active(void)
{
    return s_oneshot;
}

uint8_t pwm_oneshot_poll(void)
{
    if (s_oneshot == 0u)
    {
        return 0u;
    }

    if (((TIM3->CR1 & TIM_CR1_CEN) != 0u) || ((TIM4->CR1 & TIM_CR1_CEN) != 0u))
    {
        /* 临时诊断: 越过首个事件点后转储一次(判断DMA是否真实传输) */
        if ((s_mid_dumped == 0u) && (s_dual_f1 != 0u) && (TIM3->CNT >= s_dual_f1))
        {
            s_mid_dumped = 1u;
            pwm_dual_diag("mid");
        }

        return 0u;      /* 仍在运行 */
    }

    /* 序列完成: 先转储(若mid未抓到), 再收尾 */
    if (s_mid_dumped == 0u)
    {
        s_mid_dumped = 1u;
        pwm_dual_diag("done");
    }

    /* 收尾(引脚强制低/通道状态复位, 复用 pwm_stop 清理路径) */
    pwm_stop();
    s_oneshot = 0u;
    return 1u;
}

/* ---- 清零+单脉冲模式(架构参照 jxps-deep-osc sync_and_reset_set) ----
 * T3 主连续(无OPM): CH1=清零信号(PWM1: 周期起点高电平clrw, 之后低);
 *                   CH3 仅内部OC3REF(PWM2: 低->@ccr3翻高), CCR3=clrw+clri,
 *                   不使能CC3E(引脚不输出), OC3REF上升沿经TRGO=OC3REF(MMS=101)
 *                   -> ITR2 硬件触发 T4; CH4 强制低不参与
 * T4 从 TRIGGER(ITR2)+OPM: 触发沿从0计数, 1tick后输出高电平pulse_w,
 *                   计到ARR自停(CEN清零)。
 * 卸装机制: 平时SMS=DISABLE(T4不响应触发); fire时装载TRIGGER+CEN=1,
 *   完成后(更新中断, 比下一周期触发沿早得多)立刻卸装回DISABLE ->
 *   保证一次触发正好一个单脉冲, 且无主循环轮询窗口期的重触发竞态 */
#define PWM_US_TICK(us, psc)    (((us) * (PWM_TIM_CLOCK_HZ / 1000000u) + (psc) / 2u) / (psc))

static uint8_t s_cs_armed = 0;      /* 1=T4已武装(等待CC3中断启动) */
static uint8_t s_cs_clr_on = 0;     /* 1=清零信号流运行中 */
static volatile uint8_t s_cs_done = 0;  /* ISR置位: 单脉冲完成, 供主循环打印 */

uint8_t pwm_cs_validate(uint32_t freq_hz, uint32_t clrw_us, uint32_t clri_us,
                        uint32_t pulse_us)
{
    uint32_t psc3;
    uint32_t arr3;
    uint32_t w3;
    uint32_t i3;
    uint32_t p3;
    uint32_t psc4 = 1u;
    uint32_t arr4;

    if ((freq_hz < PWM_FREQ_MIN) || (freq_hz > PWM_FREQ_MAX))
    {
        return 0u;
    }

    if (pwm_cal_arr(freq_hz, &psc3, &arr3) == 0u)
    {
        return 0u;
    }

    /* 清零脉宽/间隔在T3时基上, 至少1个tick(CCR=0会持续匹配) */
    w3 = PWM_US_TICK(clrw_us, psc3);
    i3 = PWM_US_TICK(clri_us, psc3);

    if ((w3 == 0u) || (i3 == 0u))
    {
        return 0u;
    }

    /* 单脉冲脉宽在T4时基上(独立PSC); 装进周期检查用T3 tick折算 */
    p3 = PWM_US_TICK(pulse_us, psc3);

    if (p3 == 0u)
    {
        p3 = 1u;        /* 低于T3分辨率: 必然装得下, 按1tick计 */
    }

    /* 清零+间隔+脉冲+余量 必须装进一个周期, 否则下一周期触发沿截断脉冲 */
    if ((w3 + i3 + p3 + 2u) > arr3)
    {
        return 0u;
    }

    /* T4: 最小分频使 脉宽+延时1tick 装进16位计数器 */
    for (;;)
    {
        arr4 = 1u + PWM_US_TICK(pulse_us, psc4);

        if (arr4 <= 65530u)
        {
            break;
        }

        psc4++;

        if (psc4 > 65536u)
        {
            return 0u;
        }
    }

    return 1u;
}

void pwm_cs_apply(uint32_t freq_hz, uint32_t clrw_us, uint32_t clri_us,
                  uint32_t pulse_us, uint8_t clr_en)
{
    uint32_t psc3;
    uint32_t arr3;
    uint32_t psc4 = 1u;
    uint32_t arr4;
    uint32_t w3;
    uint32_t ccr3;
    TIM_OC_InitTypeDef oc = {0};

    if (pwm_cs_validate(freq_hz, clrw_us, clri_us, pulse_us) == 0u)
    {
        return;
    }

    pwm_stop();
    s_cs_armed = 0u;
    s_cs_done = 0u;

    (void)pwm_cal_arr(freq_hz, &psc3, &arr3);
    w3 = PWM_US_TICK(clrw_us, psc3);
    ccr3 = w3 + PWM_US_TICK(clri_us, psc3);

    /* 中断开销补偿: CC3比较中断从比较点到置位T4 CEN实测固定开销,
     * 按用户标定取1.5us, 触发点前移该值, 使T4实际启动落在设定的间隔点上 */
    {
        uint32_t comp = (3u * (PWM_TIM_CLOCK_HZ / 2000000u) + psc3 - 1u) / psc3;
        /* 1.5us = 108 tick@psc1, 向上取整除以psc3 */

        /* 保护边界放宽: 只要不减成负数就应用全额补偿(小间隔时触发点可
         * 落在清零脉宽内, 由ISR固定2.0us延迟自然推回, 用户确认可接受) */
        if (ccr3 > comp)
        {
            ccr3 -= comp;
        }
        else
        {
            ccr3 = 1u;
        }
    }

    for (;;)
    {
        arr4 = 1u + PWM_US_TICK(pulse_us, psc4);

        if (arr4 <= 65530u)
        {
            break;
        }

        psc4++;
    }

    printf(">> CS apply F=%luHz psc3=%lu arr3=%lu w=%lu ccr3=%lu | psc4=%lu arr4=%lu clr=%u\r\n",
           (unsigned long)freq_hz, (unsigned long)psc3, (unsigned long)arr3,
           (unsigned long)w3, (unsigned long)ccr3,
           (unsigned long)psc4, (unsigned long)arr4, clr_en);

    /* T3: TRGO=OC3REF(CH3内部参考信号作触发源) */
    pwm_tim3_init(psc3, arr3, TIM_TRGO_OC3REF);

    /* T4: 卸装态初始化(SMS=DISABLE不响应触发), 单脉冲模式计满自停 */
    pwm_tim4_init(psc4, arr4, 0u);
    TIM4->CR1 |= TIM_CR1_OPM;

    /* T3 CH1 清零信号: PWM1, 周期起点高电平w3(clrw), 之后低; CC1E沿用stop置位 */
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = (uint16_t)w3;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    (void)HAL_TIM_OC_ConfigChannel(&htim3, &oc, TIM_CHANNEL_1);

    /* T3 CH3: 仅内部OC3REF(上升沿@ccr3=clrw+clri), 不使能CC3E(引脚不输出).
     * 注: TRGO=OC3REF走T4触发从模式的方案(v3.5.1/3.5.3)实测不可靠 --
     * PWM2时触发线大部分时间为高(武装即触发), PWM1时(3.5.3)触发时刻随机,
     * F1从机触发模式对电平型内部触发源的边沿检测不可信, 弃用.
     * 最终方案: T4 SMS=DISABLE, 用T3 CC3比较中断在触发时刻软件启动T4.
     * PWM2下OC3REF在ccr3翻高, CC3IF在CNT=ccr3的比较时刻置位, 时点一致 */
    oc.OCMode = TIM_OCMODE_PWM2;
    oc.Pulse = (uint16_t)ccr3;
    (void)HAL_TIM_OC_ConfigChannel(&htim3, &oc, TIM_CHANNEL_3);
    TIM_CCxChannelCmd(TIM3, TIM_CHANNEL_3, TIM_CCx_DISABLE);
    TIM_CHANNEL_STATE_SET(&htim3, TIM_CHANNEL_3, HAL_TIM_CHANNEL_STATE_READY);
    /* CH4: stop已FORCED_INACTIVE+CCxE(恒低), 保持不参与 */

    /* T4 CH1/2/3: PWM2, CCR=1tick(延时) -> 高电平脉宽=ARR-1=pulse_w
     * 卸装态CNT=0恒低于PWM2的CCR(=1), 引脚保持低; 武装后触发沿才开始计数 */
    oc.OCMode = TIM_OCMODE_PWM2;
    oc.Pulse = 1u;

    for (uint32_t j = 3u; j < PWM_CH_TOTAL; j++)
    {
        (void)HAL_TIM_OC_ConfigChannel(s_map[j].htim, &oc, s_map[j].ch);
        TIM_CCxChannelCmd(s_map[j].htim->Instance, s_map[j].ch, TIM_CCx_ENABLE);
    }

    /* 更新中断: 单脉冲计满(OPM停机)时立即卸装, 早于下一周期触发沿 */
    TIM4->SR = (uint16_t)~TIM_SR_UIF;
    __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);
    HAL_NVIC_SetPriority(TIM4_IRQn, 1u, 0u);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);

    /* 启动: clr_en=1时T3连续运行产生清零流; T4保持卸装等待fire */
    s_cs_clr_on = clr_en;

    if (clr_en != 0u)
    {
        pwm_timer_enable(&htim3);
    }

    s_running = clr_en;
}

uint8_t pwm_cs_fire(void)
{
    if ((s_cs_clr_on == 0u) || (s_cs_armed != 0u))
    {
        return 0u;      /* 清零流未运行 / 已武装: 丢弃 */
    }

    /* v3.5.2 最终方案: 只武装不启动T4. T3 CC3 比较中断在触发时刻
     * (ccr3=clr_w+clr_i)软件启动T4计数, 见 TIM3_IRQHandler.
     * 不用T4触发从模式: 实测F1对TRGO=OC3REF电平源边沿检测不可靠
     * (v3.5.1武装即触发/v3.5.3触发时刻随机, 见apply处注释) */
    TIM3->SR = (uint16_t)~TIM_SR_CC3IF;         /* 清残留标志, 只等下一次比较 */
    __HAL_TIM_ENABLE_IT(&htim3, TIM_DIER_CC3IE);
    HAL_NVIC_SetPriority(TIM3_IRQn, 1u, 0u);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);

    s_cs_armed = 1u;
    return 1u;
}

uint8_t pwm_cs_poll(void)
{
    if (s_cs_done == 0u)
    {
        return 0u;
    }

    s_cs_done = 0u;
    return 1u;
}

/* T3 CC3比较中断: 武装状态下在触发时刻(ccr3=clr_w+clr_i)软件启动T4计数.
 * T4 SMS保持DISABLE(不依赖触发沿检测, 无"使能时已高"竞态), OPM计满自停. */
void TIM3_IRQHandler(void)
{
    if ((TIM3->SR & TIM_SR_CC3IF) != 0u)
    {
        TIM3->SR = (uint16_t)~TIM_SR_CC3IF;

        if ((s_cs_armed != 0u) && (s_cs_clr_on != 0u))
        {
            __HAL_TIM_DISABLE_IT(&htim3, TIM_DIER_CC3IE);   /* 一次触发一个脉冲 */

            TIM4->CNT = 0u;
            TIM4->SR = 0u;
            TIM4->CR1 |= TIM_CR1_CEN;       /* T4开始计数出单脉冲 */
        }
    }
}

/* T4更新中断: 单脉冲计满(OPM自动清CEN), 收尾武装状态 */
void TIM4_IRQHandler(void)
{
    if ((TIM4->SR & TIM_SR_UIF) != 0u)
    {
        TIM4->SR = (uint16_t)~TIM_SR_UIF;

        if (s_cs_armed != 0u)
        {
            s_cs_armed = 0u;
            s_cs_done = 1u;
        }
    }
}
