# IAP Bootloader 实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:subagent-driven-development（推荐）或 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。注意：任务 6/7 涉及 Keil GUI 操作，需人类配合完成编译。

**目标：** 为 F407 自研板实现 IAP bootloader（0x08000000，64KB），RES 命令置升级标志后复位进入 bootloader，经 UART5（YModem）接收 test.bin 写入 0x08010000 并跳转，实现与 F1 成品板一致的现场升级。

**架构：** 双程序结构。bootloader（新写，独立小工程）检查 0x0807C000 升级标志 → 标志=1 进 YModem 升级模式 / 标志=0 校验并跳转 APP。APP（现有工程）链接地址从 0x08000000 改为 0x08010000，`SCB->VTOR` 同步改。参数区（Sector7）与升级标志在升级时均不触碰。

**技术栈：** STM32F407ZGT6, Keil MDK-ARM (ARMCC V5.05), STM32F4xx HAL, YModem（CRC16-XMODEM）。

**设计规格：** [2026-08-25-iap-bootloader-design.md](../specs/2026-08-25-iap-bootloader-design.md)

---

## 文件结构

| 文件 | 职责 | 操作 |
|------|------|------|
| `Core/Inc/ymodem.h` | YModem 接收上下文与接口声明 | 创建 |
| `Core/Src/ymodem.c` | CRC16-XMODEM、块接收状态机、Flash 擦写 | 创建 |
| `Core/Src/boot_main.c` | bootloader 主逻辑：UART5 原始收发、APP 校验、跳转、标志处理 | 创建 |
| `Core/Src/main.c:73` | APP 的 `SCB->VTOR` 改 0x08010000 | 修改 |
| `MDK-ARM/test.uvprojx` | APP 的 IROM1 改 0x08010000 / 0x50000 | 修改 |
| `MDK-ARM/boot/boot.uvprojx` | bootloader 独立 Keil 工程 | 创建（GUI） |
| `doc/使用手册.md` | 固件升级章节 | 修改 |

bootloader 复用现有 `sys.c`（时钟）、`delay.c`（延时）、`usart.c`（UART5 + printf），不引入电机控制代码。

---

### 任务 1：创建 `Core/Inc/ymodem.h`

**文件：**
- 创建：`new/Core/Inc/ymodem.h`

- [ ] **步骤 1：写入完整文件**

```c
#ifndef __YMODEM_H
#define __YMODEM_H

#include "stm32f4xx.h"

/* 单个 YModem 文件接收上下文 */
typedef struct {
    uint32_t total_size;    /* 块0声明的文件大小（可能为0） */
    uint32_t received;      /* 已写入字节数 */
    uint32_t target_addr;   /* 固件写入起始地址（APP_BASE） */
} ymodem_ctx_t;

uint16_t crc16_xmodem(const uint8_t *data, uint32_t len);
/* 返回: 0=成功, -1=超时(未收到块0), -2=擦除失败, -3=发送方取消(CAN) */
int ymodem_receive(ymodem_ctx_t *ctx);

#endif
```

- [ ] **步骤 2：Commit**

```bash
git add Core/Inc/ymodem.h
git commit -m "feat(boot): add ymodem header"
```

---

### 任务 2：创建 `Core/Src/ymodem.c`

**文件：**
- 创建：`new/Core/Src/ymodem.c`

- [ ] **步骤 1：写入完整文件**

```c
#include "ymodem.h"
#include "stm32f4xx_hal.h"
#include <stdlib.h>

/* 底层 UART5 原始收发，由 boot_main.c 提供 */
extern void boot_uart_putc(uint8_t c);
extern int  boot_uart_getc(uint32_t timeout_ms);
extern void boot_uart_puts(const char *s);

#define C_CHR  0x43   /* 'C': 邀请/继续 (CRC模式) */
#define ACK    0x06
#define NAK    0x15
#define CAN    0x18
#define EOT    0x04
#define SOH    0x01
#define STX    0x02

#define RX_TIMEOUT   2000U  /* 单字节接收超时 ms */
#define INVITE_LIMIT 6      /* 无块0时'C'重邀上限, 6×2s≈12s 判超时 */

uint16_t crc16_xmodem(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0;
    while (len--) {
        crc ^= (uint16_t)(*data++) << 8;
        for (int i = 0; i < 8; i++) {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U)
                                  : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* 写 len 字节（4字节对齐）到 Flash */
static void flash_write_words(uint32_t addr, const uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i += 4) {
        uint32_t w = (uint32_t)data[i]
                   | ((uint32_t)data[i+1] << 8)
                   | ((uint32_t)data[i+2] << 16)
                   | ((uint32_t)data[i+3] << 24);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + i, w);
    }
}

int ymodem_receive(ymodem_ctx_t *ctx)
{
    uint8_t data[1024];
    int     exp_seq = 1;        /* 数据块序号从1开始 */
    int     got_header = 0;
    int     invite = 0;
    int     eot_handshake = 1;  /* 1=未完成EOT握手, 0=已发NAK等第二次EOT */

    boot_uart_puts("\r\nBOOT: waiting YModem...\r\n");

    while (1) {
        int c = boot_uart_getc(RX_TIMEOUT);
        if (c < 0) {
            if (!got_header) {
                if (++invite > INVITE_LIMIT) return -1;  /* 约12s无块0 */
                boot_uart_putc(C_CHR);
            } else {
                boot_uart_putc(NAK);  /* 数据阶段超时: 请求重传 */
            }
            continue;
        }
        uint8_t hdr = (uint8_t)c;

        if (hdr == SOH || hdr == STX) {
            uint32_t dlen = (hdr == SOH) ? 128U : 1024U;
            int s  = boot_uart_getc(RX_TIMEOUT);
            int ns = boot_uart_getc(RX_TIMEOUT);
            if (s < 0 || ns < 0 || (uint8_t)~s != (uint8_t)ns) { boot_uart_putc(NAK); continue; }
            uint8_t seq = (uint8_t)s;

            int ok = 1;
            for (uint32_t i = 0; i < dlen; i++) {
                int b = boot_uart_getc(RX_TIMEOUT);
                if (b < 0) { ok = 0; break; }
                data[i] = (uint8_t)b;
            }
            if (!ok) { boot_uart_putc(NAK); continue; }

            int crc_hi = boot_uart_getc(RX_TIMEOUT);  /* YModem: CRC高字节在前 */
            int crc_lo = boot_uart_getc(RX_TIMEOUT);
            if (crc_hi < 0 || crc_lo < 0) { boot_uart_putc(NAK); continue; }
            uint16_t crc = (uint16_t)((crc_hi << 8) | crc_lo);
            if (crc16_xmodem(data, dlen) != crc) { boot_uart_putc(NAK); continue; }

            if (!got_header) {
                /* 块0: "文件名\0大小\0" */
                char *fname = (char *)data;
                char *fsize = fname;
                while (*fsize) fsize++;
                fsize++;
                ctx->total_size = 0;
                if (*fsize) ctx->total_size = (uint32_t)strtoul(fsize, NULL, 10);
                got_header = 1;
                exp_seq = 1;
                ctx->received = 0;

                /* 擦除 APP 区 Sector4-6 */
                HAL_FLASH_Unlock();
                FLASH_EraseInitTypeDef f;
                uint32_t err = 0;
                f.TypeErase     = FLASH_TYPEERASE_SECTORS;
                f.Sector        = FLASH_SECTOR_4;
                f.NbSectors     = 3;
                f.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
                if (HAL_FLASHEx_Erase(&f, &err) != HAL_OK) { HAL_FLASH_Lock(); return -2; }

                boot_uart_putc(ACK);
                boot_uart_putc(C_CHR);
            }
            else if (!eot_handshake) {
                /* 结束块: 空块0 → 完成 */
                if (seq == 0 && data[0] == 0) { boot_uart_putc(ACK); HAL_FLASH_Lock(); return 0; }
                boot_uart_putc(NAK);
            }
            else {
                /* 数据块 */
                if (seq != (uint8_t)exp_seq) { boot_uart_putc(NAK); continue; }
                flash_write_words(ctx->target_addr + ctx->received, data, dlen);
                ctx->received += dlen;
                exp_seq = (exp_seq + 1) & 0xFF;
                boot_uart_putc(ACK);
                boot_uart_putc(C_CHR);
            }
        }
        else if (hdr == EOT) {
            if (eot_handshake) { boot_uart_putc(NAK); eot_handshake = 0; }  /* 第一次EOT: NAK确认 */
            else               { boot_uart_putc(ACK); boot_uart_putc(C_CHR); } /* 第二次EOT: 等结束块 */
        }
        else if (hdr == CAN) {
            HAL_FLASH_Lock();
            return -3;  /* 发送方取消 */
        }
        /* 其他字节忽略 */
    }
}
```

- [ ] **步骤 2：Commit**

```bash
git add Core/Src/ymodem.c
git commit -m "feat(boot): implement ymodem receive with CRC16 and flash write"
```

---

### 任务 3：创建 `Core/Src/boot_main.c`

**文件：**
- 创建：`new/Core/Src/boot_main.c`

- [ ] **步骤 1：写入完整文件**

```c
#include "stm32f4xx_hal.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "ymodem.h"
#include <stdio.h>

#define APP_BASE          0x08010000U
#define APP_END           0x08060000U   /* APP 区上界 = 参数区起点 */
#define UPDATE_FLAG_ADDR  0x0807C000U

/* ===== 底层 UART5 原始收发（供 ymodem.c 使用）===== */
void boot_uart_putc(uint8_t c)
{
    uint32_t t = 500000;
    while (!(UART5->SR & USART_SR_TC)) if (--t == 0) break;
    UART5->DR = c;
}

int boot_uart_getc(uint32_t timeout_ms)
{
    while (timeout_ms--) {
        if (UART5->SR & USART_SR_RXNE) return (int)(UART5->DR & 0xFF);
        if (UART5->SR & USART_SR_ORE)  { (void)UART5->DR; }  /* 清溢出错误 */
        delay_ms(1);
    }
    return -1;
}

void boot_uart_puts(const char *s)
{
    while (*s) boot_uart_putc((uint8_t)*s++);
}

/* printf 重定向由 usart.c 的 fputc 提供（UART5） */

/* ===== 全局 MSP：时钟使能 ===== */
void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
}

/* APP 有效性: 栈指针在 RAM 内, 复位向量在 APP 区 */
static int app_valid(void)
{
    uint32_t sp = *(volatile uint32_t *)APP_BASE;
    uint32_t pc = *(volatile uint32_t *)(APP_BASE + 4);
    return (sp >= 0x20000000U && sp < 0x20020000U) &&
           (pc >= APP_BASE && pc < APP_END);
}

static void jump_to_app(void)
{
    uint32_t sp = *(volatile uint32_t *)APP_BASE;
    uint32_t pc = *(volatile uint32_t *)(APP_BASE + 4);
    __disable_irq();
    SCB->VTOR = APP_BASE;
    __set_MSP(sp);
    ((void (*)(void))pc)();
}

static void clear_update_flag(void)
{
    HAL_FLASH_Unlock();
    if (*(volatile uint32_t *)UPDATE_FLAG_ADDR != 0)
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, UPDATE_FLAG_ADDR, 0);
    HAL_FLASH_Lock();
}

static void do_upgrade(void)
{
    ymodem_ctx_t ctx = {0};
    ctx.target_addr = APP_BASE;
    int r = ymodem_receive(&ctx);
    if (r == 0) {
        clear_update_flag();
        boot_uart_puts("\r\nBOOT: upgrade OK, jumping...\r\n");
        delay_ms(200);
        jump_to_app();
    } else {
        boot_uart_puts("\r\nBOOT: upgrade failed, retrying...\r\n");
    }
}

int main(void)
{
    SCB->VTOR = 0x08000000U;      /* bootloader 向量表 */
    HAL_Init();
    Stm32_Clock_Init();           /* 168MHz */
    delay_init(168);
    uart_init(115200);            /* UART5 (PC12/PD2) */
    UART5->CR1 &= ~USART_CR1_RXNEIE;  /* 关闭RX中断, YModem用轮询收原始字节 */

    printf("\r\nBOOT v1.0\r\n");

    while (1) {
        if (*(volatile uint32_t *)UPDATE_FLAG_ADDR == 1) {
            printf("BOOT: update flag set, upgrade mode\r\n");
            do_upgrade();                       /* 成功则跳转, 失败重试 */
        } else if (app_valid()) {
            printf("BOOT: jump APP\r\n");
            delay_ms(100);
            jump_to_app();
        } else {
            printf("BOOT: no valid APP, waiting YModem...\r\n");
            do_upgrade();
        }
    }
}
```

- [ ] **步骤 2：核对关键点（人工检查）**
  - `boot_uart_getc` 只轮询 RXNE，不依赖 RX 中断（bootloader 不需要行缓冲）。
  - `jump_to_app` 先关中断、再设 VTOR、再 `__set_MSP`，最后跳转。
  - `do_upgrade` 失败后回主循环重试，直到成功或断电。

- [ ] **步骤 3：Commit**

```bash
git add Core/Src/boot_main.c
git commit -m "feat(boot): add bootloader main with app jump and upgrade logic"
```

---

### 任务 4：修改 APP 的 `SCB->VTOR`

**文件：**
- 修改：`new/Core/Src/main.c:73`

- [ ] **步骤 1：编辑 `SCB->VTOR = FLASH_BASE;` → `SCB->VTOR = 0x08010000U;`**

改动前：
```c
    SCB->VTOR = FLASH_BASE;
```
改动后：
```c
    SCB->VTOR = 0x08010000U;     /* APP 已搬移到 bootloader 之后 */
```

- [ ] **步骤 2：Commit**

```bash
git add Core/Src/main.c
git commit -m "fix(app): set VTOR to APP base 0x08010000 for bootloader"
```

---

### 任务 5：修改 APP 链接地址（IROM1）

**文件：**
- 修改：`new/MDK-ARM/test.uvprojx:252-256`

- [ ] **步骤 1：编辑 IROM 元素**

改动前：
```xml
              <IROM>
                <Type>1</Type>
                <StartAddress>0x8000000</StartAddress>
                <Size>0x100000</Size>
              </IROM>
```
改动后：
```xml
              <IROM>
                <Type>1</Type>
                <StartAddress>0x8010000</StartAddress>
                <Size>0x50000</Size>
              </IROM>
```

同时把第 21 行 `<Cpu>` 字符串里的 `IROM(0x8000000-0x80FFFFF)` 改为 `IROM(0x8010000-0x805FFFF)`（保持一致，Keil 保存时会按 IROM 元素重写）。

- [ ] **步骤 2：注意**
  - **编辑前必须关闭 Keil**（否则 Keil 保存会用旧值覆盖）。
  - 改完后重新打开 Keil，确认 Target 选项里 IROM1 = 0x08010000，Size = 0x50000。

- [ ] **步骤 3：Commit**

```bash
git add MDK-ARM/test.uvprojx
git commit -m "build(app): relocate APP to 0x08010000"
```

---

### 任务 6：创建 bootloader Keil 工程并编译

**说明：** Keil 是 GUI 工具，需人工操作。工程文件 `MDK-ARM/boot/boot.uvprojx` 在 Keil 中新建（不手写 XML，避免 Keil 覆盖/校验问题）。

**文件：**
- 创建：`new/MDK-ARM/boot/boot.uvprojx`（GUI 生成）

- [ ] **步骤 1：在 Keil 新建 boot 工程**

菜单 Project → New uVision Project → 选择 `new/MDK-ARM/boot/` 目录，工程名 `boot`。Device 选 **STM32F407ZGTx**，运行时组件对话框直接点 OK（不加组件）。

- [ ] **步骤 2：配置 Target 选项（Options for Target, Alt+F7）**

| 标签页 | 设置 |
|--------|------|
| Target | Xtal(MHz): **25**；IROM1: Start **0x8000000**, Size **0x10000**；IRAM1: Start **0x20000000**, Size **0x1C000**；IRAM2: Start **0x2001C000**, Size **0x4000**；勾选 Use MicroLIB |
| Output | Name of Executable: **boot**；勾选 Create HEX File；After Build/Rebuild 加 Run#2: `fromelf --bin --output=".\boot.bin" ".\boot.axf"` |
| C/C++ | Define: `USE_HAL_DRIVER,STM32F407xx`；Optimization Level 0；Include Paths: `../Core/Inc;../Drivers/STM32F4xx_HAL_Driver/Inc;../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy;../Drivers/CMSIS/Device/ST/STM32F4xx/Include;../Drivers/CMSIS/Include` |

- [ ] **步骤 3：添加源文件到工程**

按以下组添加（路径相对 `MDK-ARM/boot/`）：

```
Boot 组:
  ../Core/Src/boot_main.c
  ../Core/Src/ymodem.c

Core 组:
  ../Core/Src/sys.c
  ../Core/Src/delay.c
  ../Core/Src/usart.c

HAL 组: (与 APP 工程同一套 HAL 源文件)
  ../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
  ../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
  ../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
  ../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
  ../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
  ../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
  ../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
  ../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
  ../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
  ../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
  ../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
  ../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
  ../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
  ../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c

CMSIS 组:
  ../Core/Src/system_stm32f4xx.c
  ../Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/arm/startup_stm32f407xx.s
```

> 注意：**不要**加入 main.c、step_motor.c、cmd_handler.c、flash_param.c、atim.c、usart2.c、led.c、key.c、rtc.c、stmflash.c、stm32f4xx_hal_msp.c（HAL_MspInit 已在 boot_main.c 定义）。

- [ ] **步骤 4：编译 boot 工程**

Project → Build Target（F7）。预期：**0 errors, 0 warnings**。产物：`MDK-ARM/boot/boot.axf`、`boot.hex`、`boot.bin`。

若报错，常见原因与处理：
- 多重定义 `fputc` → 确认没加 `Core/Src/main.c`（`fputc` 只能来自 usart.c）。
- `UART5_IRQn`/`HAL_UART_MspInit` 未定义 → 确认已加 usart.c 且 Define 含 `USE_HAL_DRIVER`。
- include 找不到 → 核对步骤 2 的 Include Paths。

---

### 任务 7：编译 APP、首次烧录部署

**文件：**
- 无代码改动（仅编译）

- [ ] **步骤 1：编译 APP 工程**

在 Keil 打开 `MDK-ARM/test.uvprojx`（已在任务 5 改过 IROM1），Rebuild。预期 0 errors。确认产物 `test.bin`（after-build fromelf 若未生效，手动跑：
`fromelf --bin --output=test.bin test.axf`，见使用手册 fromelf 一节）。

- [ ] **步骤 2：首次烧录（ST-Link/J-Link）**

用调试器把 **boot.hex 烧到 0x08000000**（Keil 打开 boot 工程 → Flash Download → 烧录）。**不需要**烧 APP——首次上电 bootloader 发现无有效 APP，会停在 YModem 等待。

- [ ] **步骤 3：验证 bootloader 启动**

连接 UART5（PC12=TX, PD2=RX）到 USB 串口模块，115200。上电，串口应打印：
```
BOOT v1.0
BOOT: no valid APP, waiting YModem...
BOOT: waiting YModem...
```

---

### 任务 8：硬件测试（升级全流程）

**前置：** bootloader 已烧入，串口终端（SecureCRT/Hterm）已连 UART5。

- [ ] **步骤 1：YModem 首次烧入 APP**

在串口工具发 YModem，选 `test.bin`。预期串口依次出现：
```
BOOT: waiting YModem...
BOOT: upgrade OK, jumping...
[时间戳] F1-Stepper on F407 ready      ← APP 启动信息
```
发送 `PARA`，确认参数默认值打印正常（参数区未被破坏）。

- [ ] **步骤 2：RES 命令进入升级模式**

在 APP 串口发 `RES`。预期设备复位，串口打印 `BOOT v1.0` 和 `BOOT: update flag set, upgrade mode`，随后 `BOOT: waiting YModem...`。

- [ ] **步骤 3：升级成功且参数保留**

（先发一条 `D:50` 改参数再升级）发 YModem 选 `test.bin`。升级完成后发 `PARA`，确认 `g_set_speed(D:)` 仍是 **50**（Sector7 参数未被擦）。

- [ ] **步骤 4：传输中断可重试**

升级时在中途断开/取消发送。断电重启，预期仍进 `BOOT: upgrade mode`。重新发 YModem 成功恢复。

- [ ] **步骤 5：坏文件被拒绝**

用编辑器改坏 `test.bin` 几个字节再发。预期串口反复出现 NAK 重传，或发送方报错，最终不写坏 APP（或设备停在 bootloader）。

- [ ] **步骤 6：APP 功能回归**

升级后测试：`D0` 回零、`L1`~`L5` 切工位、`RTF`/`RTB`、正转/急停/复位 IO——全部正常。

---

### 任务 9：更新使用手册

**文件：**
- 修改：`new/doc/使用手册.md`

- [ ] **步骤 1：在 §1.3 IO 控制说明 之后或 §3.2 命令表附近新增「固件升级」章节**

内容要点（照设计规格 §6/§7）：
- 架构：bootloader(0x08000000) + APP(0x08010000)，参数区不动
- 升级命令：发 `RES` → 保存参数、置升级标志、复位进 bootloader
- 操作：串口 115200 连 UART5 → 发 `RES` → 串口工具 Send YModem 选 `test.bin` → 完成
- 异常：传一半断电 → 再上电仍在升级模式可重试；坏文件 CRC 拒绝
- 首次量产：ST-Link 烧 bootloader，之后只走 YModem 升 APP

- [ ] **步骤 2：Commit**

```bash
git add doc/使用手册.md
git commit -m "docs: add firmware upgrade guide for IAP bootloader"
```

---

## 自检

**1. 规格覆盖度：**
- [x] Flash 布局（§2/§3）→ 任务 4/5（APP 搬移）+ 任务 3（APP_BASE/APP_END）
- [x] bootloader 状态机（§4）→ 任务 3（标志判定、跳转、升级模式）
- [x] YModem + CRC16 + Flash 擦写（§4/§7）→ 任务 2
- [x] APP 有效性检查 + 跳转（§4）→ 任务 3 `app_valid`/`jump_to_app`
- [x] 防砖（§7）→ 任务 2（先擦后写、失败不跳）、任务 3（失败重试、标志不清）
- [x] 独立 boot 工程（§10）→ 任务 6
- [x] 首次烧录 + 测试计划（§8/§10）→ 任务 7/8
- [x] 文档（§6 流程）→ 任务 9

**2. 占位符扫描：** 无 TODO/待定；所有代码块完整。

**3. 类型一致性：** `ymodem_ctx_t`（任务 1/2/3）字段一致；`boot_uart_putc/getc/puts` 在 boot_main.c 定义、ymodem.c extern 引用，签名一致；`ymodem_receive` 返回值约定在任务 1 头注释与任务 2/3 处理逻辑一致。
