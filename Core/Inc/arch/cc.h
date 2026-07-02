/**
 ******************************************************************************
 * @file        cc.h
 * @brief       LwIP 体系结构/编译器依赖宏定义 (STM32F407 + Keil MDK)
 * @note        定义数据类型、pack 属性及 errno 支持
 ******************************************************************************
 */
#ifndef __ARCH_CC_H
#define __ARCH_CC_H

#define LWIP_NO_STDINT_H  1          /* 由本文件提供 u8_t 等类型，避免 arch.h 重复定义 */

#include <stdint.h>

typedef uint8_t     u8_t;
typedef int8_t      s8_t;
typedef uint16_t    u16_t;
typedef int16_t     s16_t;
typedef uint32_t    u32_t;
typedef int32_t     s32_t;
typedef uintptr_t   mem_ptr_t;

#define PACK_STRUCT_STRUCT  __packed    /* Keil MDK packed 属性 */

#define LWIP_PROVIDE_ERRNO  1           /* LwIP 提供 errno */

#endif /* __ARCH_CC_H */
