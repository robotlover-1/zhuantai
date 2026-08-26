# 设计：B:0 模式工位跳转改用最短路径

> **日期**: 2026-08-26 | **项目**: new (STM32F407 步进电机控制) | **分支**: feature/iap-bootloader

## 1. 目标

当前 `B:` 开关同时决定**路径方向**与**是否补偿**。用户要求 **B:0（补偿模式）** 的工位跳转也走**最短路径**，而非当前的固定方向（"高顺低逆"）。B:0 的补偿运动特性保留。

## 2. 现状

`B:` 开关对应变量 `g_no_compensate`，控制两层逻辑：

| 层级 | 位置 | B:1 (不补偿) | B:0 (补偿) |
|------|------|-------------|-----------|
| 路径方向 | `cmd_handler.c::motion_path()` | `step_motor_L1` 最短路径 | `step_motor_L2` 固定方向 |
| 单站运动 | `step_motor.c::motion_dispatch()` | `step_motor_motion` 一步到位 | `step_motor_motion_compensate` 补偿 |

`step_motor_L2` 顶部有 `receive_flag = 1` 用于锁命令并供 `cmd_gnn_test`（疲劳测试）的 `while(receive_flag == 0)` 等待运动结束；`step_motor_L1` 目前没有。

## 3. 改动设计

### 3.1 `cmd_handler.c` — `motion_path()` 始终走 L1

```c
/* 路径统一为最短路径(L1); 补偿与否由 motion_dispatch 按 B 开关决定 */
static void motion_path(int station_y, int station_z) {
    step_motor_L1(station_y, station_z);
}
```

B 开关不再选择路径方向，但继续通过 `motion_dispatch` 决定 B:0=补偿 / B:1=不补偿。

### 3.2 `step_motor.c` — `step_motor_L1()` 顶部补 `receive_flag = 1;`

镜像 L2 的锁机制。**必要性**：否则 B:0 切到 L1 后，`cmd_gnn_test` 的 `while(receive_flag == 0)` 会立即跳出，往返运动重叠、`station_x` 错乱。补上后 B:0/B:1 的 gnn 测试等待均正常，且 B:1 单命令运动期间也不再接受新跳转命令（更严格、更正确）。

### 3.3 `step_motor_L2` 保留

改后 L2 在 new 项目中无调用方，**保留**（最小改动、便于回退，保留"高顺低逆"旧逻辑作参考）。不改其代码与声明。

### 3.4 文档同步（仅 new 项目）

- `doc/使用手册.md` §3.3（L202-204）：`B:` 跳转路径说明 → B:0/B:1 均走最短路径，仅补偿不同
- `doc/使用手册.md` §4（L289-293）：补偿机制表 → 补充 B:0 同样走最短路径
- `doc/使用手册.md` §7（L366、L382-383）：F4 对比表更新
- `doc/README.md` §十三（L361-381）：路径规划说明更新

> `doc/电机进出孔策略算法说明.md` 描述的是另一套 PID 架构，与本改动无关，不修改。

## 4. 验证方案

1. **编译**：Keil MDK 工程正常编译通过（无警告/错误，L2 保留可避免未引用告警）。
2. **真机**（建议用户执行）：
   - `B:0` 下依次 `L1`~`L5` 跳转，确认走最短方向（1→5 为反转 1 站而非正转 4 站）。
   - `B:0` 下跑 `gnn` 疲劳测试，确认等待逻辑正常、无运动重叠。
   - 对照 `B:1` 行为确认路径一致、仅补偿行为有差异。

## 5. 影响与风险

- **行为变化**：B:0 跳转路径从固定方向变为最短路径（运动距离更短、更省时）。B:1 不变。
- **补偿有效性**：`step_motor_motion_compensate` 双向（dir=0/1）对称，最短路径下补偿逻辑同样适用；真机需验证进孔一致性。
- **receive_flag**：B:1 单命令期间锁由无变有（更严格），无已知冲突点；急停 `RTS`/PF7 不受影响。
- **范围**：仅 `new` 项目；F1 项目（硬编码 L2）不在本次改动内。
