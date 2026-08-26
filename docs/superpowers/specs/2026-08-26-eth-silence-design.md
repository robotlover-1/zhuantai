# 设计：静音以太网启动/运行打印

> **日期**: 2026-08-26 | **项目**: new (STM32F407 步进电机控制) | **分支**: feature/iap-bootloader

## 1. 目标

去掉以太网相关的启动/运行打印，让上电串口输出安静。**全部采用"注释掉"而非删除**，便于以后恢复。**保留参数回显**（`param_load_all()` 打印的 D:/J:/…/CS_H/total_cycles 等）。

## 2. 移除范围

| 文件 | 内容 | 处理 |
|------|------|------|
| `eth.c` | **全部** `[ETH]`/`[PHY]` printf（约 27 处：DM9000 检测/MAC、PHY 寄存器 dump、restart autoneg、LINK TIMEOUT/UP/DOWN、LwIP/netif/TCP 初始化、client 连接/断开、TCP/netif 错误等） | 注释掉 |
| `ethernetif.c:35` | `[ETH] netif init OK, MAC ...` | 注释掉 |
| `flash_param.c:125-158` | 仅 `IP:`/`MASK:`/`GW:`/`PORT:` 四处 | 注释掉 |
| `flash_param.c` 其余参数打印 | `g_set_speed(D:)`…`CS_H`、`total_cycles`（L111） | **保留** |
| `main.c:106-111` | banner 块：`[时间戳] F1-Stepper V2 on F407 ready` + `total_cycles` | 注释掉（整块含 `ts`/`rtc_get_time_str`） |

**保留功能**：串口 `PARA` 命令（cmd_handler.c）仍可查询 IP/MASK/GW/PORT 与全部参数；`CYCLE` 命令仍可查询 total_cycles——本改动不影响这些有意查询。

## 3. 实现要点

1. **空语句体处理**：eth.c 中 `printf` 是 `if`/`else`/`for` 唯一语句的地方（如 LINK UP/DOWN、`dm9000_phy_read` 循环、reject/error 分支），注释 printf 后补 `(void)0;`，避免"空语句体"告警/错误。
2. **main.c banner 块**：`ts`/`rtc_get_time_str(ts,...)` 与两行 printf 一起注释，避免未使用变量告警。
3. **提交策略**：
   - `eth.c`、`ethernetif.c`：**未跟踪新文件，只编辑不提交**。
   - `flash_param.c`：**在途改动，只编辑不提交**。
   - `main.c`：**干净，可提交**。

## 4. 验证方案

1. Keil 编译零错误/零新增告警。
2. 真机上电串口只显示 `D: … CS_H` + `total_cycles` 参数表；无 IP/MASK/GW/PORT、无 `[ETH]`/`[PHY]`、无 banner。
3. `PARA`/`CYCLE` 命令仍正常输出。

## 5. 影响与风险

- **行为变化**：以太网初始化/运行不再输出诊断打印；DM9000 找不到等错误也不再打印（仅通过串口模式仍可用判断）。用户已确认"都注释掉"。
- **风险**：空语句体若处理不当可能编译告警——实现要点 1 已覆盖。
- **范围**：仅 new 项目。
