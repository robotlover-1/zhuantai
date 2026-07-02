# 以太网口命令功能 — 设计规格

## 概述

在现有 UART5 串口命令交互基础上，新增以太网口（DM9000 + LwIP）作为并行的命令通道。PC 通过 TCP 客户端连接设备，发送与串口完全相同的文本命令，接收相同格式的响应。

---

## 1. 使用场景

串口 + 网口并存。两个通道均可发送命令、接收响应，互不干扰。

- 命令来源为 TCP 时，`printf()` 输出同时发送到 UART5 **和** TCP 发送队列（UART 始终保留输出，方便本地监控）
- 命令来源为 UART5 时，`printf()` 输出仅走 UART5（行为不变）

---

## 2. 网络协议

### 2.1 协议选型

- **协议**：原始 TCP（非 Telnet，非 WebSocket）
- **角色**：设备作为 TCP Server，PC 作为 TCP Client
- **连接数**：单客户端。第二个连接请求被拒绝（RST）
- **连接断开**：释放资源，等待新连接

### 2.2 网络配置

| 参数 | 命令 | 默认值 | 存储 |
|------|------|--------|:----:|
| 本机 IP | `IP:192.168.1.100` | `192.168.1.100` | Flash |
| 子网掩码 | `MASK:255.255.255.0` | `255.255.255.0` | Flash |
| 默认网关 | `GW:192.168.1.1` | `192.168.1.1` | Flash |
| TCP 监听端口 | `PORT:5000` | `5000` | Flash |

- 设置 IP/MASK/GW/PORT 时自动保存到 Flash（与 F/R/V/S/E 等参数行为一致）
- `PARA` 命令增加网络配置显示
- IP 地址修改后需复位生效（提示"需发送 RES 重启生效"）

---

## 3. 架构设计

### 3.1 整体架构

```
┌─────────────────────────────────────────────────────┐
│                    main.c 主循环                       │
│  ┌──────────┐   ┌──────────┐  ┌──────────────────┐   │
│  │ UART5 RX │   │ TCP RX   │  │ LwIP 协议栈轮询    │   │
│  │ (已有)    │   │ (新增)    │  │ MX_LWIP_Process() │   │
│  └────┬─────┘   └────┬─────┘  └─────────┬────────┘   │
│       │               │                  │             │
│       ▼               ▼                  │             │
│  ┌──────────────────────────┐            │             │
│  │  cmd_dispatch(buf,len)   │◄─ g_cmd_src│             │
│  │  (已有, 无需修改)         │            │             │
│  └───────────┬──────────────┘            │             │
│              │ printf()                   │             │
│              ▼                            │             │
│  ┌─────────────────────────┐             │             │
│  │  fputc() 改造            │             │             │
│  │  if (g_cmd_src==TCP)     │────────────┘             │
│  │    → tcp_send_byte()    │                            │
│  │  → UART5->DR (始终)      │                            │
│  └─────────────────────────┘                            │
│                                                         │
│  ┌─────────────────────────────────────────────────┐    │
│  │  eth.c / eth.h (新增)                             │    │
│  │  DM9000 驱动 + LwIP 初始化 + TCP Server 管理     │    │
│  └─────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────┘
```

### 3.2 核心原则：输出通道标记法（方案 A）

`cmd_dispatch()` 本身**不需要修改**。通过全局变量 `g_cmd_src` 标记当前命令来源：

```c
typedef enum {
    CMD_SRC_UART = 0,
    CMD_SRC_TCP  = 1,
} cmd_source_t;

extern cmd_source_t g_cmd_src;
```

`fputc()` 改造逻辑：
```c
int fputc(int ch, FILE *f) {
    // 始终发送到 UART5（已有逻辑不变）
    uint32_t timeout = 500000;
    while (!(UART5->SR & USART_SR_TC)) {
        if (--timeout == 0) break;
    }
    UART5->DR = (uint8_t)ch;

    // 如果命令来源是 TCP，同时写入 TCP 发送队列
    if (g_cmd_src == CMD_SRC_TCP && g_tcp_client_connected) {
        eth_tcp_send_byte((uint8_t)ch);  // 失败静默丢弃
    }
    return ch;
}
```

### 3.3 数据流（TCP 命令示例）

1. PC TCP 客户端连接设备 → TCP Server 接受连接
2. PC 发送 `"PARA\r\n"` → LwIP 接收 → 写入 `g_eth_rx_buf[]`，设 `g_eth_rx_ready = 1`
3. `main()` 循环检测到 `g_eth_rx_ready` → 设 `g_cmd_src = CMD_SRC_TCP` → 调 `cmd_dispatch(g_eth_rx_buf, len)`
4. 命令处理函数中的 `printf()` → `fputc()` 检测到 TCP 模式 → 每个字节同时写 UART5 和 TCP 发送缓冲区
5. 处理完毕 → `g_cmd_src = CMD_SRC_UART`（恢复默认）
6. LwIP 周期调用发送队列将缓冲数据发出

---

## 4. 组件设计

### 4.1 DM9000 驱动层（`Core/Src/eth.c`）

不依赖 LwIP 的底层驱动函数：

| 函数 | 说明 |
|------|------|
| `dm9000_reg_read()` | FSMC 读取 DM9000 寄存器 |
| `dm9000_reg_write()` | FSMC 写入 DM9000 寄存器 |
| `dm9000_init()` | 硬件复位（拉低 PD7 → 延时 → 拉高），读取 VID/PID 校验 |
| `dm9000_recv_packet(buf, &len)` | 从 DM9000 FIFO 读出一帧以太网数据 |
| `dm9000_send_packet(buf, len)` | 向 DM9000 FIFO 写入一帧并触发发送 |

**硬件连接**（板上已有）：

| DM9000 信号 | STM32 引脚 | 说明 |
|-------------|-----------|------|
| FSMC 数据/地址总线 | PD0-1/4-5/8-15, PE0-1/7-15, PF0-5/12-15, PG0-5/9-10 | 16位 FSMC Bank1 |
| DM9000 INT | PG6 | 中断输入 |
| DM9000 RST | PD7 | 复位输出 |

### 4.2 LwIP 移植层

- LwIP 协议栈放入 `Middlewares/LwIP/`
- `ethernetif.c` 对接 DM9000 驱动：

| 移植函数 | 说明 |
|----------|------|
| `low_level_init()` | 调用 `dm9000_init()`，配置 MAC 地址 |
| `low_level_output()` | 调用 `dm9000_send_packet()` |
| `low_level_input()` | 调用 `dm9000_recv_packet()`，将数据投喂给 LwIP |
| `ethernetif_input()` | 由主循环轮询调用，驱动 LwIP 接收处理 |

### 4.3 TCP Server（`eth.c` 内）

| 函数 | 说明 |
|------|------|
| `eth_init()` | DM9000 初始化 → LwIP 初始化 → 创建 TCP PCB → bind → listen |
| `tcp_accept_cb()` | 接受新连接（单客户端限制），注册 recv/sent/err 回调 |
| `tcp_recv_cb()` | 收到数据 → 写入 `g_eth_rx_buf[]` → 设 `g_eth_rx_ready = 1` |
| `eth_tcp_send_byte(c)` | 单字节追加到 `g_eth_tx_buf[]`（供 `fputc()` 调用） |
| `eth_reconfig()` | 配置变更后重新 bind（预留，当前需复位生效） |

### 4.4 网络参数存储（扩展 `flash_param`）

新增参数：

```c
uint8_t  g_eth_ip[4];     // 本机 IP，默认 {192, 168, 1, 100}
uint8_t  g_eth_mask[4];   // 子网掩码，默认 {255, 255, 255, 0}
uint8_t  g_eth_gw[4];     // 默认网关，默认 {192, 168, 1, 1}
uint16_t g_eth_port;      // TCP 端口，默认 5000
```

- `param_save_all()` 和 `param_load_all()` 扩展，将网络参数写入/读出 Flash 同一 Sector
- 保存时触发地址递增，在现有参数之后追加网络参数

### 4.5 命令扩展（`cmd_handler.c`）

新增 4 条参数设置命令：

| 命令 | 格式 | 处理函数 |
|------|------|----------|
| `IP` | `IP:192.168.1.100` | `cmd_set_ip()` |
| `MASK` | `MASK:255.255.255.0` | `cmd_set_mask()` |
| `GW` | `GW:192.168.1.1` | `cmd_set_gw()` |
| `PORT` | `PORT:5000` | `cmd_set_port()` |

`PARA` 命令输出扩展，增加网络配置段：
```
--- 网络配置 ---
IP: 192.168.1.100
MASK: 255.255.255.0
GW: 192.168.1.1
PORT: 5000
```

`cmd_dispatch()` 增加 IP/MASK/GW/PORT 的解析分支。

### 4.6 主循环集成（`main.c`）

在 `while(1)` 中新增两个处理块：

```c
// 1. LwIP 协议栈周期处理
MX_LWIP_Process();

// 2. TCP 接收处理（镜像 UART5 的处理逻辑）
if (g_eth_rx_ready) {
    len = g_eth_rx_len;
    g_eth_rx_ready = 0;
    g_cmd_src = CMD_SRC_TCP;
    cmd_dispatch(g_eth_rx_buf, len);
    g_cmd_src = CMD_SRC_UART;
    memset(g_eth_rx_buf, 0, len);  // 清理缓冲区
}
```

---

## 5. 错误处理与边界情况

### 5.1 连接管理

| 场景 | 处理方式 |
|------|----------|
| TCP 客户端断开 | 释放连接 PCB，清 `g_tcp_client_connected`，`g_cmd_src` 恢复默认 |
| TCP 客户端重连 | 接受新连接，覆盖旧状态 |
| 第二个客户端连接 | `tcp_accept_cb` 中拒绝（`tcp_abort`），仅保活第一个连接 |
| 网线未插 | 不影响串口正常使用。`fputc()` 中 TCP 发送失败 → 静默丢弃 |

### 5.2 发送失败保护

- `fputc()` 中 TCP 发送失败时**不阻塞、不重试**，保证电机控制的实时性
- TCP 发送队列满时丢弃新数据（UART5 不受影响）

### 5.3 缓冲区安全

| 缓冲区 | 大小 | 保护机制 |
|--------|------|----------|
| `g_eth_rx_buf[]` | 256 字节 | 超长截断，以 `\r\n` 或满时触发处理 |
| `g_eth_tx_buf[]` | TCP_MSS（~1460） | LwIP 内部流控，满则丢弃 |
| 命令响应（printf） | 无上限 | 逐字节 `fputc()`，不会整块溢出 |

### 5.4 电源与复位

- 上电：`eth_init()` 中拉 PD7 低→延时 10ms→拉高完成 DM9000 硬件复位
- 软件复位（`RES` 命令）：不主动关闭 TCP 连接，MCU 复位本身断开

### 5.5 网络故障不扩散

- DM9000 初始化失败 → 仅 `eth_init()` 返回错误，不调 `Error_Handler()`，设备可正常运行串口模式
- LwIP 内存分配失败 → TCP Server 不启动，串口正常
- TCP 中断异常 → 使用轮询方式收包，不注册 PG6 外部中断，DM9000 中断线不启用，避免干扰定时器中断

---

## 6. 文件变更清单

| 文件 | 操作 | 说明 |
|------|:----:|------|
| `Core/Src/eth.c` | **新增** | DM9000 驱动 + LwIP 初始化 + TCP Server |
| `Core/Inc/eth.h` | **新增** | 以太网模块对外接口 |
| `Middlewares/LwIP/` | **新增** | LwIP 协议栈源码 |
| `Core/Src/ethernetif.c` | **新增** | LwIP 与 DM9000 的移植对接层 |
| `Core/Src/usart.c` | 修改 | `fputc()` 增加 TCP 路由分支 |
| `Core/Src/cmd_handler.c` | 修改 | 新增 IP/MASK/GW/PORT 命令 + PARA 扩展 + dispatch 分支 |
| `Core/Inc/cmd_handler.h` | 修改 | 新增处理函数声明 |
| `Core/Src/flash_param.c` | 修改 | 新增网络参数读写 |
| `Core/Inc/flash_param.h` | 修改 | 新增网络参数 extern 声明 |
| `Core/Src/main.c` | 修改 | 主循环新增 LwIP 轮询 + TCP RX 处理 + eth_init 调用 |
| `Core/Inc/main.h` | 修改 | 新增网络相关 include（如需） |
| `lwipopts.h` | **新增** | LwIP 配置裁剪（无 UDP/DHCP/DNS，仅 TCP+IP） |

---

## 7. LwIP 配置裁剪

为控制 Flash/RAM 占用，裁剪 LwIP 功能：

| 配置项 | 值 | 说明 |
|--------|:--:|------|
| `LWIP_TCP` | 1 | 开启 TCP |
| `LWIP_UDP` | 0 | 关闭 UDP |
| `LWIP_DHCP` | 0 | 关闭 DHCP（静态 IP） |
| `LWIP_DNS` | 0 | 关闭 DNS |
| `LWIP_SNMP` | 0 | 关闭 SNMP |
| `LWIP_IGMP` | 0 | 关闭 IGMP |
| `LWIP_ICMP` | 1 | 保留 ICMP（ping 可用） |
| `MEMP_NUM_TCP_PCB` | 4 | TCP PCB 数量 |
| `MEMP_NUM_TCP_SEG` | 16 | TCP 分段数量 |
| `TCP_MSS` | 1460 | 最大分段大小 |
| `TCP_SND_BUF` | 4 * TCP_MSS | 发送缓冲区 |
| `TCP_WND` | 2 * TCP_MSS | 接收窗口 |
| `LWIP_NETIF_HOSTNAME` | 1 | 支持主机名 |
| `LWIP_STATS` | 0 | 关闭统计 |

预估资源占用：RAM ~12KB，Flash ~40KB（STM32F407ZG 有 192KB RAM + 1MB Flash，充裕）。

---

## 8. 测试策略

### 8.1 单元/集成测试

| 测试项 | 方法 |
|--------|------|
| DM9000 寄存器读写 | 读取 VID（0x0A46）/PID（0x9000）验证 FSMC 通信正常 |
| LwIP ping 通 | PC `ping 192.168.1.100`，确认 ICMP 响应 |
| TCP 连接/断开 | PC 用 `nc` 或 `Putty` 连接 → 断开 → 重连 |
| 串口命令仍正常 | 串口发送 `PARA` / `HAL` / `L1` 等，确认行为不变 |
| TCP 命令正常 | TCP 发送 `PARA`，确认返回与串口一致 |

### 8.2 边界测试

| 测试项 | 方法 |
|--------|------|
| 网络参数持久化 | 设置 `IP:192.168.1.200` → `RES` → 重启后 `PARA` 确认 |
| 网线未插 | 拔网线 → 串口发命令正常 → TCP 连不上但串口不受影响 |
| TCP 发送缓冲区满 | 连续发大量命令，确认不卡死、不丢串口响应 |
| 双通道并发 | 串口发 `L1`（长时间运动命令）的同时 TCP 连上来 |
| 超长命令 | TCP 发送超过 256 字节的单行数据，确认截断保护 |

### 8.3 压力测试

| 测试项 | 方法 |
|--------|------|
| 高频命令 | TCP 每秒发送 50 条 `PARA` 命令，持续 1 分钟，确认不死机 |
| 断连重连循环 | PC 每秒 connect → 发一个命令 → disconnect，持续 100 次 |
| 运行中网络断开 | 运动命令执行中拔网线，确认电机不异常 |

---

## 9. 实施优先级

| 优先级 | 阶段 | 内容 |
|:------:|------|------|
| P0 | 驱动验证 | DM9000 寄存器读写 + 链路层收发一帧 |
| P1 | 协议栈 | LwIP 移植 + `ping` 通 |
| P2 | TCP Server | TCP 监听 + 接收命令 + 单字节发送响应 |
| P3 | `fputc` 改造 | 输出路由 + `g_cmd_src` 标志 |
| P4 | 命令扩展 | IP/MASK/GW/PORT 命令 + Flash 存储 + PARA |
| P5 | 边界处理 | 断连重连、缓冲区保护、多通道并发 |
| P6 | 测试验证 | 按第 8 节测试策略逐项验证 |
