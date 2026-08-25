# IAP Bootloader 设计（F407 自研板固件升级）

> 日期：2026-08-25
> 状态：已确认

## 1. 背景与目标

F407 自研步进电机控制板当前无 bootloader，固件升级只能拆机接 ST-Link/J-Link。目标：实现与 F1 成品板一致的现场升级体验——发 `RES` 命令进升级模式，串口（YModem）发 `test.bin` 完成升级，不拆机、不动 BOOT0、参数保留。

## 2. 架构

双程序结构，共存于同一片 F407（1MB Flash）：

| 程序 | 地址 | 大小 | 说明 |
|------|------|------|------|
| **bootloader**（新写） | 0x08000000 | 64KB | 检查升级标志 → 跳 APP 或收 YModem 固件 |
| **APP**（现有工程改地址） | 0x08010000 | 320KB（Sector4-6） | 现有电机控制功能，逻辑不变 |

bootloader 不含电机控制逻辑，仅：时钟 / GPIO / UART5 / Flash 擦写 / YModem 接收 / 跳转。

## 3. Flash 布局

```
0x08000000  Bootloader (64KB)     Sector0-3    新写
0x08010000  APP (320KB)           Sector4-6    现有工程改链接地址
0x08060000  参数区 (Sector7)       —            升级时不动
0x0807C000  升级标志 (4B)          —            RES写1 / APP启动清0（已实现）
0x08080000  (未用)
```

- 参数区与升级标志都在 Sector7，bootloader 升级时**只擦 Sector4-6**，不碰 Sector7。
- 升级标志机制已实现：[flash_param.h](Core/Inc/flash_param.h) 的 `UPDATE_FLAG_ADDR` + `write_update_flag()`；[cmd_handler.c](Core/Src/cmd_handler.c) 的 `cmd_res` 写 1 后复位；[main.c](Core/Src/main.c) 启动清 0。

## 4. bootloader 状态机

```
上电复位
  └─ 时钟 / GPIO / UART5(115200) 初始化
     └─ 读升级标志 0x0807C000
        ├─ =1 → 升级模式
        └─ =0 → 检查 APP 有效性
                  ├─ 有效 → 跳转 APP
                  └─ 无效 → 升级模式
```

**升级模式**：
```
打印 "BOOT: upgrade mode, send YModem..."
等待 YModem 会话（PC 用 SecureCRT/Hterm 发 test.bin）
→ 逐块接收（128/1024B，CRC16 校验）
→ 擦除 APP 扇区（Sector4-6）
→ 写入 APP
→ 全部写完验证
→ 清升级标志（0x0807C000 = 0）
→ 跳转 APP
```

**跳转动作**：`SCB->VTOR = 0x08010000` → 取 APP 栈顶进 MSP → 取复位向量进 PC → 跳转。

**APP 有效性检查**：读 0x08010000 处栈顶指针，验证在 RAM 范围（0x20000000~0x2001FFFF）；读 0x08010004 复位向量，验证在 Flash 范围（0x08010000~0x0805FFFF）。都合理才跳，否则留在 bootloader。

## 5. APP 改动

现有 `new/` 工程只改两处：

1. **链接地址**：Keil Target 的 IROM1 从 `0x08000000` 改为 `0x08010000`，大小 `0x50000`（320KB，不得延伸到 0x08060000 参数区）。
2. **[main.c](Core/Src/main.c) `SCB->VTOR`**：`FLASH_BASE` 改为 `0x08010000`。

其余全部不动：参数区、升级标志、RES 命令、电机逻辑照旧。

## 6. 升级流程（现场操作）

```
1. PC 串口连 UART5（115200），打开 SecureCRT/Hterm
2. 发 "RES" → APP 保存参数、写升级标志 1、软复位
3. 进入 bootloader → 打印 "BOOT: upgrade mode..."
4. 串口工具 Send YModem，选 test.bin
5. 收完 → 校验 → 写 Flash → 清标志 → 跳 APP
6. APP 起来，打印启动信息，参数保留
```

## 7. 错误处理与防砖

| 情况 | 行为 |
|------|------|
| 传输一半断电 | 升级标志保持 1 → 下次上电仍进升级模式 → 重发即可 |
| 文件损坏 / CRC 错 | YModem 拒绝该块，重传 |
| 传输中止不想升了 | 等 10s 超时 → 跳回旧 APP（未擦时）或留在 bootloader |
| APP 无效（栈指针/复位向量异常） | 不跳，留在 bootloader |

## 8. 测试计划

1. 首次烧 bootloader → 上电打印 BOOT 提示
2. 无 APP 时发 YModem → 写入成功 → 跳转 → APP 正常跑
3. 发 `RES` → 升级成功 → APP 重启后 `PARA` 确认参数仍在
4. 传一半中止 → 断电重启 → 仍进升级模式可重试
5. 故意发坏 bin → CRC 报错被拒
6. 升级后正转/急停/回零/工位切换功能不受影响

## 9. v1 边界（YAGNI）

- **单区直写**：升级时旧 APP 被擦掉，断电无回退，靠标志重试兜底；不做双区备份。
- **不做版本检查**：不比较固件版本，bootloader 全收全写。
- **不做写保护**：不使能 Flash 读保护/写保护。
- **不做加密/签名**：YModem CRC16 仅保证传输完整。

## 10. 构建与烧录

- **bootloader**：**独立最小 Keil 工程**（新建，如 `MDK-ARM/boot/boot.uvprojx`），只编 boot 主程序 + delay/usart/sys + HAL，IROM1 = `0x08000000`，大小 `0x10000`，产出 bootloader.hex/bin。选独立工程而非在现有工程加目标，原因：避免误编入电机控制代码，且现有工程 XML 的改动会被 Keil 保存覆盖（fromelf 配置已证实），独立工程由 Keil GUI 创建更稳妥。
- **APP**：现有工程改 IROM1 后照常编译，产出 test.bin（fromelf 已配）。
- **首次量产**：ST-Link/J-Link 烧 bootloader → 上电进升级模式（无 APP 一直等）→ YModem 发 APP bin → 完成。之后只升 APP，bootloader 永不重烧。
- **PC 端**：SecureCRT / Hterm / ExtraPuTTY 的 YModem 发送，无需自写上位机。
