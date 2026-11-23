/**
 * @file    stmflash.h
 * @brief   STM32 内部 FLASH 读写驱动(移植自例程19 FLASH模拟EEPROM, 适配本工程)
 *          读=直接地址访问; 写=整页缓存+按需擦除+逐半字写入(自动擦除)
 */
#ifndef __STMFLASH_H
#define __STMFLASH_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

/* FLASH 容量与基地址: STM32F103RCT6 = 256KB, 页大小 2KB */
#define STM32_FLASH_SIZE        0x40000u
#define STM32_FLASH_BASE        0x08000000u
#define STM32_SECTOR_SIZE       2048u   /* 容量>=256K的F103页大小为2K字节 */

uint16_t stmflash_read_halfword(uint32_t faddr);                        /* 读半字 */
void stmflash_read(uint32_t raddr, uint16_t *pbuf, uint16_t length);    /* 连续读(半字个数) */
void stmflash_write(uint32_t waddr, uint16_t *pbuf, uint16_t length);   /* 连续写(自动擦除, 半字个数) */

#endif /* __STMFLASH_H */
