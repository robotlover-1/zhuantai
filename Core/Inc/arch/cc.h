/**
 ****************************************************************************************************
 * @file        arch/cc.h
 * @brief       LwIP 移植适配层 (Keil MDK-ARM)
 * @note        ① 小端序声明
 *              ② ARMCC(AC5) 下协议结构体必须紧凑对齐 (__packed),
 *                 否则 IP/TCP 头部解析错位 (arch.h 默认只处理 GCC/clang)
 ****************************************************************************************************
 */
#ifndef __CC_H__
#define __CC_H__

#include <stdint.h>
#include <string.h>

/* Cortex-M4 默认小端 */
#ifndef BYTE_ORDER
#define BYTE_ORDER LITTLE_ENDIAN
#endif

/* ARMCC (Keil AC5, 本工程使用): 协议结构体紧凑对齐.
 * 注意: __packed 只能在 struct 前出现一次 (BEGIN), STRUCT 留空,
 *       否则 ARMCC 报 #83 "type qualifier specified more than once" */
#if defined(__CC_ARM)
#define PACK_STRUCT_BEGIN    __packed
#define PACK_STRUCT_STRUCT
#define PACK_STRUCT_END
#define PACK_STRUCT_FIELD(x) x
#define PACK_STRUCT_FLD_8(x) x
#define PACK_STRUCT_FLD_S(x) x
#endif

#endif /* __CC_H__ */
