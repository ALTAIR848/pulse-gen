/**
 * @file    stmflash.c
 * @brief   STM32 内部 FLASH 读写驱动(移植自例程19 FLASH模拟EEPROM, 适配本工程)
 *
 * 移植说明(相对例程):
 *  - 去掉 ./SYSTEM/sys/sys.h 与 usart/delay 依赖, 仅依赖 HAL;
 *  - RCT6=256KB, 页大小固定 2KB(例程按容量宏选择, 本工程直接定值);
 *  - 原理不变: 写入前检查目标区域是否为0xFFFF, 需要时读出整页->
 *    擦除页->替换目标数据->整页回写; 无需擦除时直接顺序写入。
 *
 * 注意: FLASH 擦除期间(约20~40ms)CPU取指被挂起, 中断响应会延迟;
 *       PWM 输出走 DMA 不受影响, CS 模式的 CC3 中断可能被推迟一次擦除时长。
 */
#include "stmflash.h"

/**
 * @brief 读指定地址的一个半字(16位), 地址必须是2的倍数
 */
uint16_t stmflash_read_halfword(uint32_t faddr)
{
    return *(volatile uint16_t *)faddr;
}

/**
 * @brief 从指定地址开始读出指定个数的半字
 * @param raddr  : 起始地址(须为2的倍数)
 * @param pbuf   : 数据缓冲
 * @param length : 半字(16位)个数
 */
void stmflash_read(uint32_t raddr, uint16_t *pbuf, uint16_t length)
{
    uint16_t i;

    for (i = 0u; i < length; i++)
    {
        pbuf[i] = stmflash_read_halfword(raddr);
        raddr += 2u;
    }
}

/**
 * @brief 不检查直接写入(前置条件: 目标区域已擦除为0xFFFF)
 * @param waddr  : 起始地址(须为2的倍数)
 * @param pbuf   : 数据缓冲
 * @param length : 半字(16位)个数
 */
static void stmflash_write_nocheck(uint32_t waddr, uint16_t *pbuf, uint16_t length)
{
    uint16_t i;

    for (i = 0u; i < length; i++)
    {
        (void)HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, waddr, pbuf[i]);
        waddr += 2u;
    }
}

/* 整页缓存(2KB), 与例程一致用静态区避免大栈占用 */
static uint16_t g_flashbuf[STM32_SECTOR_SIZE / 2u];

/**
 * @brief 在 FLASH 指定位置写入指定长度的数据(自动擦除)
 * @param waddr  : 起始地址(须为2的倍数, 且在FLASH范围内)
 * @param pbuf   : 数据缓冲
 * @param length : 半字(16位)个数(可跨页)
 */
void stmflash_write(uint32_t waddr, uint16_t *pbuf, uint16_t length)
{
    uint32_t secpos;        /* 页序号 */
    uint16_t secoff;        /* 页内偏移(半字单位) */
    uint16_t secremain;     /* 本页剩余(半字单位) */
    uint16_t i;
    uint32_t offaddr;       /* 相对FLASH基地址的偏移 */
    FLASH_EraseInitTypeDef flash_eraseop;
    uint32_t erase_addr;    /* 擦除错误地址 */

    if ((waddr < STM32_FLASH_BASE) ||
            (waddr >= (STM32_FLASH_BASE + STM32_FLASH_SIZE)))
    {
        return;     /* 非法地址 */
    }

    HAL_FLASH_Unlock();

    offaddr   = waddr - STM32_FLASH_BASE;
    secpos    = offaddr / STM32_SECTOR_SIZE;
    secoff    = (uint16_t)((offaddr % STM32_SECTOR_SIZE) / 2u);
    secremain = (uint16_t)(STM32_SECTOR_SIZE / 2u - secoff);

    if (length <= secremain)
    {
        secremain = length;     /* 不超出本页范围 */
    }

    for (;;)
    {
        /* 读出整页 */
        stmflash_read(secpos * STM32_SECTOR_SIZE + STM32_FLASH_BASE,
                      g_flashbuf, (uint16_t)(STM32_SECTOR_SIZE / 2u));

        /* 校验数据: 目标区域全为0xFFFF则无需擦除 */
        for (i = 0u; i < secremain; i++)
        {
            if (g_flashbuf[secoff + i] != 0xFFFFu)
            {
                break;
            }
        }

        if (i < secremain)      /* 需要擦除 */
        {
            flash_eraseop.TypeErase  = FLASH_TYPEERASE_PAGES;
            flash_eraseop.Banks      = FLASH_BANK_1;
            flash_eraseop.NbPages    = 1u;
            flash_eraseop.PageAddress = secpos * STM32_SECTOR_SIZE + STM32_FLASH_BASE;
            (void)HAL_FLASHEx_Erase(&flash_eraseop, &erase_addr);

            /* 替换目标数据 */
            for (i = 0u; i < secremain; i++)
            {
                g_flashbuf[i + secoff] = pbuf[i];
            }

            /* 整页回写 */
            stmflash_write_nocheck(secpos * STM32_SECTOR_SIZE + STM32_FLASH_BASE,
                                   g_flashbuf, (uint16_t)(STM32_SECTOR_SIZE / 2u));
        }
        else                    /* 无需擦除, 直接写入 */
        {
            stmflash_write_nocheck(waddr, pbuf, secremain);
        }

        if (length == secremain)
        {
            break;              /* 写入完成 */
        }

        /* 写入未完成, 移到下一页继续 */
        secpos++;
        secoff = 0u;
        pbuf += secremain;
        waddr += (uint32_t)secremain * 2u;
        length -= secremain;

        if (length > (STM32_SECTOR_SIZE / 2u))
        {
            secremain = (uint16_t)(STM32_SECTOR_SIZE / 2u);
        }
        else
        {
            secremain = length;
        }
    }

    HAL_FLASH_Lock();
}
