/**
 ******************************************************************************
 * @file        sys_arch.h
 * @brief       LwIP OS abstraction layer stub (bare-metal, no RTOS)
 ******************************************************************************
 */
#ifndef __ARCH_SYS_ARCH_H
#define __ARCH_SYS_ARCH_H

/* Bare-metal (no RTOS) stub for LwIP sys_arch layer.
 * No #includes needed — this file is #included from lwip/sys.h AFTER
 * lwip/opt.h, lwip/arch.h, lwip/err.h have already been processed,
 * so u8_t, u32_t, ERR_OK, SYS_MBOX_EMPTY are all available.
 */

typedef u8_t sys_sem_t;
typedef u8_t sys_mutex_t;
typedef u8_t sys_mbox_t;
typedef u8_t sys_thread_t;
typedef u8_t sys_prot_t;

#define SYS_MBOX_NULL           0
#define SYS_SEM_NULL            0
#define SYS_ARCH_TIMEOUT        0xFFFFFFFFUL

#define sys_sem_valid(x)        (*(x) = 0)
#define sys_sem_set_invalid(x)
#define sys_mbox_valid(x)       (*(x) = 0)
#define sys_mbox_set_invalid(x)

#define sys_mutex_new(x)
#define sys_mutex_lock(x)
#define sys_mutex_unlock(x)
#define sys_mutex_free(x)

#define sys_sem_new(c, v)       ERR_OK
#define sys_sem_signal(s)
#define sys_sem_free(s)

#define sys_arch_mbox_tryfetch(m, msg)  SYS_MBOX_EMPTY
#define sys_arch_protect()      0
#define sys_arch_unprotect(x)

u32_t sys_now(void);

#endif /* __ARCH_SYS_ARCH_H */
