/**
 ******************************************************************************
 * @file        sys_arch.h
 * @brief       LwIP OS abstraction layer stub (bare-metal, no RTOS)
 ******************************************************************************
 */
#ifndef __ARCH_SYS_ARCH_H
#define __ARCH_SYS_ARCH_H

#include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/err.h"

/* Bare-metal (no RTOS) stub for LwIP sys_arch layer.
 * LWIP_TCPIP_CORE_LOCKING=1 and NO_SYS=1 should already be set in lwipopts.h,
 * but if not, the following stubs prevent compilation errors.
 */

typedef u8_t sys_sem_t;
typedef u8_t sys_mutex_t;
typedef u8_t sys_mbox_t;
typedef u8_t sys_thread_t;
typedef u8_t sys_prot_t;

#define SYS_MBOX_NULL   0
#define SYS_SEM_NULL    0
#define SYS_ARCH_TIMEOUT 0xFFFFFFFFUL

#define sys_sem_valid(x)    (*(x) = 0)
#define sys_sem_set_invalid(x)
#define sys_mbox_valid(x)   (*(x) = 0)
#define sys_mbox_set_invalid(x)

#define sys_mutex_new(x)
#define sys_mutex_lock(x)
#define sys_mutex_unlock(x)
#define sys_mutex_free(x)

#define sys_sem_new(c, v)   ERR_OK
#define sys_sem_signal(s)
#define sys_sem_free(s)

#define sys_arch_mbox_tryfetch(m, msg)  SYS_MBOX_EMPTY

u32_t sys_now(void);

#endif /* __ARCH_SYS_ARCH_H */
