/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "pulse.h"
#include "key.h"
#include "display.h"
#include "pwm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define LED0_ON()       HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET)
#define LED0_OFF()      HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET)
#define LED0_BLINK()    HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin)

#define LED1_ON()       HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET)
#define LED1_OFF()      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET)
#define LED1_BLINK()    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint32_t g_last_tick = 0;          /* 上次1秒任务执行时刻 */
static uint32_t g_uptime_s = 0;           /* 开机以来的秒数 */
static char g_line[96];                   /* 串口打印缓冲 */

/* UI 状态机 */
typedef enum
{
  UI_HOME_BROWSE = 0,                     /* 主页浏览: 上下移动光标 */
  UI_HOME_MODESEL,                        /* 主页模式滚动选择 */
  UI_CH_PAGE,                             /* 通道选择子页 */
  UI_EDIT_PAGE                            /* 数值编辑子页 */
} ui_state_t;

static ui_state_t g_ui_state = UI_HOME_BROWSE;
static pulse_page_t g_page;               /* 主页行列表 */
static uint8_t g_cursor = 0;              /* 主页光标(页面行索引) */
static uint8_t g_scroll_top = 0;          /* 主页列表滚动起点 */
static uint8_t g_ch_cursor = 0;           /* 通道子页光标(0=Back,1~6通道,7=Sync) */
static uint8_t g_ch_scroll = 0;           /* 通道子页滚动起点 */
static pulse_edit_info_t g_edit_info;     /* 编辑子页信息 */
static pulse_mode_t g_mode_backup;        /* 进入模式选择前的备份 */
static pwm_req_t g_pwm_req;               /* PWM 启动请求(编辑/启停时组装) */
static uint8_t g_display_dirty = 1;       /* 需要刷新显示 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief  从业务参数组装 PWM 启动请求
 */
static void build_pwm_req(void)
{
  const pulse_param_t *p = pulse_param_get();
  uint8_t i;

  g_pwm_req.sync = p->pwm_sync;
  g_pwm_req.freq_hz[0] = p->freq_hz[0];
  g_pwm_req.freq_hz[1] = p->freq_hz[1];

  for (i = 0u; i < PWM_CH_TOTAL; i++)
  {
    g_pwm_req.ch_en[i] = p->ch[i].enabled;
    g_pwm_req.ch_duty[i] = p->ch[i].duty_bp;
    g_pwm_req.ch_phase[i] = p->ch[i].phase_pct;
  }
}

/**
 * @brief  停止 PWM 输出并同步 PWMEn 标志(UI 显示一致)
 */
static void pwm_out_stop(void)
{
  pwm_stop();

  if (pulse_param_get()->pwm_enabled)
  {
    pulse_pwm_toggle();     /* 置 OFF */
  }
}

/**
 * @brief  尝试启动 PWM 输出: 校验失败不动作(配置恒为可启动合法态)
 */
static void pwm_out_start(void)
{
  build_pwm_req();

  if (pwm_validate(&g_pwm_req) == 0u)
  {
    printf("PWM start: validate FAIL (freq=%lu/%luHz sync=%u)\r\n",
           (unsigned long)g_pwm_req.freq_hz[0],
           (unsigned long)g_pwm_req.freq_hz[1], g_pwm_req.sync);
    return;
  }

  if (pwm_apply(&g_pwm_req))
  {
    printf("PWM start OK: F=%luHz sync=%u CEN3=%u CEN4=%u\r\n",
           (unsigned long)g_pwm_req.freq_hz[0], g_pwm_req.sync,
           (unsigned)((TIM3->CR1 & TIM_CR1_CEN) != 0u),
           (unsigned)((TIM4->CR1 & TIM_CR1_CEN) != 0u));
    if (!pulse_param_get()->pwm_enabled)
    {
      pulse_pwm_toggle();   /* 置 ON */
    }
  }
  else
  {
    printf("PWM start: apply FAIL\r\n");
  }
}

/**
 * @brief  按住时长映射为编辑步进倍率: 短按=1, 按住2s后=10, 5s后=100
 */
static uint32_t edit_step_mult(key_id_t id)
{
  uint32_t hold = key_hold_ms(id);

  if (hold >= 5000u)
  {
    return 100u;
  }

  if (hold >= 2000u)
  {
    return 10u;
  }

  return 1u;
}

/**
 * @brief  重建主页行列表并钳制光标/滚动位置
 */
static void main_rebuild_page(void)
{
  pulse_build_main_page(&g_page);

  if (g_cursor >= g_page.count)
  {
    g_cursor = (uint8_t)(g_page.count - 1u);
  }
}

/**
 * @brief  主页列表滚动跟随光标: 4行窗口, 所有行(含模式行)同权滚动
 */
static void main_fix_scroll(void)
{
  if (g_page.count <= 4u)
  {
    g_scroll_top = 0u;
    return;
  }

  if (g_scroll_top > (uint8_t)(g_page.count - 4u))
  {
    g_scroll_top = (uint8_t)(g_page.count - 4u);
  }

  if (g_cursor < g_scroll_top)
  {
    g_scroll_top = g_cursor;
  }

  if (g_cursor > (uint8_t)(g_scroll_top + 3u))
  {
    g_scroll_top = (uint8_t)(g_cursor - 3u);
  }
}

/**
 * @brief  通道子页滚动跟随光标(4行窗口, 5行内容)
 */
static void ch_fix_scroll(void)
{
  if (g_ch_cursor < g_ch_scroll)
  {
    g_ch_scroll = g_ch_cursor;
  }

  if (g_ch_cursor > (uint8_t)(g_ch_scroll + 3u))
  {
    g_ch_scroll = (uint8_t)(g_ch_cursor - 3u);
  }
}

/**
 * @brief  处理一个按键事件(电视遥控式交互)
 *  主页浏览: 上/下移动光标; 确认: 模式行进滚动选择, CHSel进通道子页, 数值行进编辑子页
 *  模式选择: 上/下实时切换模式(各模式参数独立), 短按确认锁定, 长按取消恢复
 *  通道子页: 确认直接翻转勾选, Back行确认返回; 长按确认也返回
 *  编辑子页: 上/下改值, 短按确认保存返回, 长按确认取消恢复
 */
static void ui_handle_event(key_event_t ev, key_id_t id)
{
  uint8_t move = (ev == KEY_EVENT_DOWN) || (ev == KEY_EVENT_REPEAT);

  switch (g_ui_state)
  {
    case UI_HOME_BROWSE:
      if (move && id == KEY_ID_UP)
      {
        g_cursor = (g_cursor == 0u)
                   ? (uint8_t)(g_page.count - 1u) : (uint8_t)(g_cursor - 1u);
        main_fix_scroll();
      }
      else if (move && id == KEY_ID_DOWN)
      {
        g_cursor = (g_cursor >= (uint8_t)(g_page.count - 1u))
                   ? 0u : (uint8_t)(g_cursor + 1u);
        main_fix_scroll();
      }
      else if (id == KEY_ID_CONFIRM && ev == KEY_EVENT_RELEASE_SHORT)
      {
        uint8_t type = g_page.row[g_cursor].type;

        if (type == (uint8_t)PULSE_ROW_MODE)
        {
          g_mode_backup = pulse_param_get()->mode;
          g_ui_state = UI_HOME_MODESEL;
        }
        else if (type == (uint8_t)PULSE_ROW_CHSEL)
        {
          /* 进通道子页前禁用输出(用户约定: 退出后保持停止, 手动PWMEn恢复) */
          pwm_out_stop();
          g_ch_cursor = 0u;
          g_ch_scroll = 0u;
          g_ui_state = UI_CH_PAGE;
        }
        else if (type == (uint8_t)PULSE_ROW_PWM_EN)
        {
          /* 布尔项: 确认直接翻转; ON=按当前参数启动输出 */
          if (!pulse_param_get()->pwm_enabled)
          {
            pwm_out_start();
          }
          else
          {
            pwm_out_stop();
          }
        }
        else if (type == (uint8_t)PULSE_ROW_TIM_FREQ ||
                 type == (uint8_t)PULSE_ROW_CH_DUTY ||
                 type == (uint8_t)PULSE_ROW_CH_PHASE ||
                 type == (uint8_t)PULSE_ROW_DUAL_P1 ||
                 type == (uint8_t)PULSE_ROW_DUAL_GAP ||
                 type == (uint8_t)PULSE_ROW_DUAL_P2 ||
                 type == (uint8_t)PULSE_ROW_CS_FREQ ||
                 type == (uint8_t)PULSE_ROW_CS_CLRW ||
                 type == (uint8_t)PULSE_ROW_CS_CLRI ||
                 type == (uint8_t)PULSE_ROW_CS_PULSE)
        {
          if (pulse_begin_edit(type, g_page.row[g_cursor].idx, &g_edit_info))
          {
            g_ui_state = UI_EDIT_PAGE;
          }
        }
        else if (type == (uint8_t)PULSE_ROW_TRIGGER)
        {
          /* 触发脉冲: 确认一次触发一次
           * 双脉冲模式 = 真触发(固定全通道齐发 P1/Gap/P2, 运行中丢弃;
           *              PWM模式的通道勾选仅作用于PWM模式)
           * 其他模式   = 计数桩(硬件后配) */
          const pulse_param_t *pp = pulse_param_get();

          if ((pp->mode == PULSE_MODE_DUAL) &&
              pwm_fire_dual(pp->dual_p1_us, pp->dual_gap_us, pp->dual_p2_us))
          {
            pulse_trigger_fire();     /* 计数+LED反馈走原有路径 */
          }
          else if (pp->mode == PULSE_MODE_DUAL)
          {
            printf(">> Dual fire FAIL (busy/resolution)\r\n");
          }
          else
          {
            pulse_trigger_fire();
          }
        }
      }
      break;

    case UI_HOME_MODESEL:
      if (move && id == KEY_ID_UP)
      {
        pulse_mode_t old_mode = pulse_param_get()->mode;

        pulse_param_set_mode((pulse_mode_t)((pulse_param_get()->mode + PULSE_MODE_COUNT - 1u) % PULSE_MODE_COUNT));

        /* 切模式必停输出(含PWMEn标志复位, UI一致) */
        if (pulse_param_get()->mode != old_mode)
        {
          pwm_out_stop();
        }

        main_rebuild_page();
        main_fix_scroll();
      }
      else if (move && id == KEY_ID_DOWN)
      {
        pulse_mode_t old_mode = pulse_param_get()->mode;

        pulse_param_set_mode((pulse_mode_t)((pulse_param_get()->mode + 1u) % PULSE_MODE_COUNT));

        if (pulse_param_get()->mode != old_mode)
        {
          pwm_out_stop();
        }

        main_rebuild_page();
        main_fix_scroll();
      }
      else if (id == KEY_ID_CONFIRM && ev == KEY_EVENT_RELEASE_SHORT)
      {
        /* 锁定: 模式有变化则停输出(PWM参数归PWM模式私有) */
        if (pulse_param_get()->mode != g_mode_backup)
        {
          pwm_out_stop();
        }
        g_ui_state = UI_HOME_BROWSE;
      }
      else if (id == KEY_ID_CONFIRM && ev == KEY_EVENT_LONG)
      {
        /* 取消, 恢复原模式 */
        pulse_param_set_mode(g_mode_backup);
        main_rebuild_page();
        main_fix_scroll();
        g_ui_state = UI_HOME_BROWSE;
      }
      break;

    case UI_CH_PAGE:
      if (move && id == KEY_ID_UP)
      {
        g_ch_cursor = (g_ch_cursor == 0u)
                      ? (uint8_t)(PULSE_CH_COUNT + 1u) : (uint8_t)(g_ch_cursor - 1u);
        ch_fix_scroll();
      }
      else if (move && id == KEY_ID_DOWN)
      {
        g_ch_cursor = (g_ch_cursor >= PULSE_CH_COUNT + 1u)
                      ? 0u : (uint8_t)(g_ch_cursor + 1u);
        ch_fix_scroll();
      }
      else if (id == KEY_ID_CONFIRM && ev == KEY_EVENT_RELEASE_SHORT)
      {
        if (g_ch_cursor == 0u)
        {
          g_ui_state = UI_HOME_BROWSE;      /* Back行: 返回主页 */
        }
        else if (g_ch_cursor == (uint8_t)(PULSE_CH_COUNT + 1u))
        {
          /* 同步选项: T3主T4从, 勾选后共用一个频率 */
          pulse_sync_toggle();
          main_rebuild_page();      /* 主页频率行数随同步状态变化 */
          main_fix_scroll();
        }
        else
        {
          /* 布尔勾选: 确认直接翻转(映射回通道索引) */
          pulse_ch_toggle((uint8_t)(g_ch_cursor - 1u));
          main_rebuild_page();      /* 主页行数随勾选变化 */
          main_fix_scroll();
        }
      }
      else if (id == KEY_ID_CONFIRM && ev == KEY_EVENT_LONG)
      {
        g_ui_state = UI_HOME_BROWSE;
      }
      break;

    case UI_EDIT_PAGE:
      if (move && (id == KEY_ID_UP || id == KEY_ID_DOWN))
      {
        pulse_edit_step((id == KEY_ID_UP) ? -1 : 1,    /* 上=减 下=加 */
                        edit_step_mult(id));           /* 按住越久步进越大 */
        g_edit_info.value = pulse_edit_value();   /* 同步编辑页显示值 */
      }
      else if (id == KEY_ID_CONFIRM)
      {
        if (ev == KEY_EVENT_RELEASE_SHORT)
        {
          /* 方案A: 先校验整个PWM配置, 非法则拒绝本次编辑(值回滚) */
          build_pwm_req();          /* 含刚改过的值 */

          if (pwm_validate(&g_pwm_req))
          {
            pulse_edit_commit();    /* 保存返回 */
            g_ui_state = UI_HOME_BROWSE;

            /* 输出运行中: 停-配-启瞬间切换到新参数 */
            if (pwm_is_running())
            {
              pwm_stop();
              (void)pwm_apply(&g_pwm_req);
            }
          }
          else
          {
            pulse_edit_cancel();    /* 拒绝: 值恢复, 输出继续跑旧参数 */
          }
        }
        else if (ev == KEY_EVENT_LONG)
        {
          pulse_edit_cancel();      /* 取消恢复 */
          g_ui_state = UI_HOME_BROWSE;
        }
      }
      break;

    default:
      g_ui_state = UI_HOME_BROWSE;
      break;
  }

  g_display_dirty = 1;
}

/**
 * @brief  每1秒执行一次的任务: 串口打印 + LED心跳 + 显示刷新标记
 */
static void app_1s_task(void)
{
  const pulse_param_t *p = pulse_param_get();

  g_uptime_s++;

  /* LED0 每2秒翻转一次, 作为心跳指示 */
  if (g_uptime_s % 2 == 0)
  {
    LED0_BLINK();
  }

  snprintf(g_line, sizeof(g_line),
           "Hello, uptime:%lus | Mode=%s PWMEN=%u SYNC=%u TRIG=%lu | T3=%luHz T4=%luHz | "
           "C1[%u] D%u.%02u P%u C2[%u] D%u.%02u P%u C3[%u] D%u.%02u P%u "
           "C4[%u] D%u.%02u P%u C5[%u] D%u.%02u P%u C6[%u] D%u.%02u P%u\r\n",
           (unsigned long)g_uptime_s, pulse_mode_name(p->mode), p->pwm_enabled,
           p->pwm_sync, (unsigned long)pulse_trigger_count(),
           (unsigned long)p->freq_hz[0], (unsigned long)p->freq_hz[1],
           p->ch[0].enabled, p->ch[0].duty_bp / 100u, p->ch[0].duty_bp % 100u, p->ch[0].phase_pct,
           p->ch[1].enabled, p->ch[1].duty_bp / 100u, p->ch[1].duty_bp % 100u, p->ch[1].phase_pct,
           p->ch[2].enabled, p->ch[2].duty_bp / 100u, p->ch[2].duty_bp % 100u, p->ch[2].phase_pct,
           p->ch[3].enabled, p->ch[3].duty_bp / 100u, p->ch[3].duty_bp % 100u, p->ch[3].phase_pct,
           p->ch[4].enabled, p->ch[4].duty_bp / 100u, p->ch[4].duty_bp % 100u, p->ch[4].phase_pct,
           p->ch[5].enabled, p->ch[5].duty_bp / 100u, p->ch[5].duty_bp % 100u, p->ch[5].phase_pct);
  printf("%s", g_line);

  /* 模式行秒数更新, 走整页刷新 */
  g_display_dirty = 1;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  LED1_OFF();
  pulse_param_init();
  pulse_param_load();
  key_init();
  display_init();
  main_rebuild_page();
  printf("pulse-gen started\r\n");

  g_last_tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* 按键扫描: 处理本轮所有按键事件 */
    key_id_t key_id;
    key_event_t key_ev;

    while ((key_ev = key_scan(&key_id)) != KEY_EVENT_NONE)
    {
      ui_handle_event(key_ev, key_id);
    }

    /* 按需刷新显示(整页约8ms, 不脏不刷) */
    if (g_display_dirty)
    {
      g_display_dirty = 0;

      switch (g_ui_state)
      {
        case UI_HOME_BROWSE:
        case UI_HOME_MODESEL:
          display_show_main_page(&g_page, g_cursor, g_scroll_top,
                                 (g_ui_state == UI_HOME_MODESEL), g_uptime_s);
          break;

        case UI_CH_PAGE:
          display_show_ch_page(pulse_param_get(), g_ch_cursor, g_ch_scroll);
          break;

        case UI_EDIT_PAGE:
          display_show_edit_page(&g_edit_info);
          break;

        default:
          break;
      }
    }

    /* 触发脉冲即时反馈: LED1闪 + 串口打印 */
    {
      static uint32_t s_last_trig = 0;
      uint32_t tc = pulse_trigger_count();

      if (tc != s_last_trig)
      {
        s_last_trig = tc;
        LED1_BLINK();
        printf(">> Trigger #%lu (Mode=%s)\r\n", (unsigned long)tc,
               pulse_mode_name(pulse_param_get()->mode));
      }
    }

    /* 双脉冲序列完成检测(OPM 计满自动停机, 此处收尾) */
    if (pwm_oneshot_poll())
    {
      printf(">> Dual done\r\n");
    }

    /* 非阻塞1秒调度 */
    if (HAL_GetTick() - g_last_tick >= 1000)
    {
      g_last_tick += 1000;
      app_1s_task();
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
