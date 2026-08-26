# B:0 模式工位跳转改用最短路径 实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:subagent-driven-development（推荐）或 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** B:0（补偿）模式的工位跳转也走最短路径（`step_motor_L1`），保留补偿运动；同步更新文档。

**架构：** `cmd_handler.c::motion_path()` 不再按 `g_no_compensate` 选 L1/L2，统一调用 `step_motor_L1`；`step_motor.c::step_motor_L1()` 顶部补 `receive_flag = 1`（镜像 L2 的锁机制，保证 gnn 疲劳测试等待正常）。B 开关仍通过 `motion_dispatch()` 决定补偿与否。`step_motor_L2` 保留但不再被调用。

**技术栈：** C（STM32F407，HAL + Keil MDK），无自动化测试环境。

**规格：** `docs/superpowers/specs/2026-08-26-b0-shortest-path-design.md`

---

## ⚠️ 工作区注意事项（重要，先读）

1. **仓库 `new/` 工作区有大量未提交在途改动**（以太网/bootloader，见 `git status`）。**禁止 `git add .` / `git add -A` / `git commit -am`**。
2. `Core/Src/cmd_handler.c` 与 `Core/Src/step_motor.c` 当前已有未提交改动。**提交前先执行 `git diff <文件>` 复查**：若文件中混有在途改动，本次任务只应提交本计划的改动。无法用 `git add -p`（交互式不支持）时，**只对文档文件做独立 commit**，两个 `.c` 文件的改动留待用户确认后手动提交，不要整文件混提。
3. 本仓库无自动化测试。**每个代码步骤的验证方式是：用 `git diff` / 直接阅读改动区域复查**，编译由用户在 Keil 中执行（见任务 4）。

---

## 文件结构

| 文件 | 职责 | 改动 |
|------|------|------|
| `Core/Src/cmd_handler.c` | 命令分发 / 运动路径选择 | 修改 `motion_path()`（约 L385-392） |
| `Core/Src/step_motor.c` | 步进电机运动执行 | 修改 `step_motor_L1()` 函数头（约 L2940） |
| `doc/使用手册.md` | 用户手册（B 开关/补偿/对比表） | 更新 3 处 |
| `doc/README.md` | 技术说明（路径规划） | 更新 1 处 |

不改动：`Core/Inc/step_motor.h`（L2 保留，声明不动）、`Core/Src/step_motor.c` 的 `step_motor_L2()` 本体、`doc/电机进出孔策略算法说明.md`（另一套 PID 架构）。

---

## 任务 1：`motion_path()` 统一走最短路径

**文件：**
- 修改：`Core/Src/cmd_handler.c:385-392`

- [ ] **步骤 1：编辑 `motion_path()`**

将 [cmd_handler.c:385-392](Core/Src/cmd_handler.c#L385-L392) 的：

```c
/* 根据 B 开关选择路径: B:1 最短路径(L1), B:0 固定方向(L2) */
static void motion_path(int station_y, int station_z) {
    if (g_no_compensate) {
        step_motor_L1(station_y, station_z);
    } else {
        step_motor_L2(station_y, station_z);
    }
}
```

替换为：

```c
/* 路径统一为最短路径(L1); 补偿与否由 motion_dispatch 按 B 开关决定 */
static void motion_path(int station_y, int station_z) {
    step_motor_L1(station_y, station_z);
}
```

- [ ] **步骤 2：复查**

运行：`grep -n "motion_path\|step_motor_L1\|step_motor_L2" Core/Src/cmd_handler.c`
预期：`motion_path` 内仅剩 `step_motor_L1` 调用；`g_no_compensate` 仍被 `cmd_set_no_comp`（L153-158）使用，无未引用告警。

- [ ] **步骤 3：处理提交**

先 `git diff Core/Src/cmd_handler.c` 确认改动区域。若该文件只有本任务改动则 `git add Core/Src/cmd_handler.c && git commit -m "feat(motion): B:0 工位跳转统一走最短路径(L1)"`；若混有在途改动，**跳过提交**，在计划中记录待用户手动提交。

---

## 任务 2：`step_motor_L1()` 补 `receive_flag = 1`

**文件：**
- 修改：`Core/Src/step_motor.c:2940-2941`

- [ ] **步骤 1：编辑函数头**

在 [step_motor.c:2940-2941](Core/Src/step_motor.c#L2940-L2941) 的 `switch` 之前插入一行：

```c
void step_motor_L1(int station_y, int station_z) // 最短路径运行
{
  receive_flag = 1;   /* 与 L2 一致: 运动期间锁命令, 供 gnn 测试等待运动结束 */
  switch (station_y) {
```

`receive_flag` 已在文件顶部声明（`step_motor.c:10` `extern u8 receive_flag;`），无需新声明。

- [ ] **步骤 2：复查**

阅读 `step_motor_L1` 函数头与 `step_motor_L2` 函数头（L3092-3095）对照：两函数现在都在顶部先设 `receive_flag = 1`。确认"当前工位不运行"分支（如 L2951-2959）仍会把 `receive_flag` 清 0。

- [ ] **步骤 3：处理提交**

同任务 1 步骤 3：`git diff Core/Src/step_motor.c` 复查后决定是否提交（消息：`feat(motion): step_motor_L1 顶部补 receive_flag 锁, 保证 gnn 测试等待正常`）；混有在途改动则不提交。

---

## 任务 3：同步文档

**文件：**
- 修改：`doc/使用手册.md`（L202-204、L289-293、L382-383）
- 修改：`doc/README.md`（L361-381）

- [ ] **步骤 1：更新 `doc/使用手册.md` §3.3（L202-204）**

将：

```markdown
> 跳转路径由 `B:` 开关决定：
> - `B:0`：固定方向（`step_motor_L2`，高顺低逆轨迹）
> - `B:1`：最短路径（`step_motor_L1`，双向选择，最多 2 步）
```

替换为：

```markdown
> 跳转路径统一为**最短路径**（`step_motor_L1`，双向选择，最多 2 步）：
> - `B:0`：最短路径 + 每站运动带补偿（精度高）
> - `B:1`：最短路径 + 一步到位不补偿（省时）
```

- [ ] **步骤 2：更新 `doc/使用手册.md` §4 补偿机制说明（L289-293）**

将表格首行与补充说明改为：

```markdown
| `B:0`（补偿） | 先正向过冲 `C:` 脉冲，再反向回退，使齿轮咬合在固定一侧（跳转走最短路径） | 高，可重复 |
```

并在末尾补一条：

```markdown
- 两种模式的工位跳转均走最短路径；`B:` 仅决定是否补偿
```

- [ ] **步骤 3：更新 `doc/使用手册.md` §7.2 最短路径（L383）**

将：

```markdown
| **最短路径** | `B:1` 模式下 L1~L5 跳转使用最短路径（双向选择，最多 2 步）；F1 只有固定方向 |
```

替换为：

```markdown
| **最短路径** | `B:0`/`B:1` 模式下 L1~L5 跳转均使用最短路径（双向选择，最多 2 步），仅补偿行为不同；F1 只有固定方向 |
```

- [ ] **步骤 4：更新 `doc/README.md` §十三 路径规划逻辑（L361-381）**

将 `step_motor_L1`/`step_motor_L2` 两节替换为：

```markdown
### step_motor_L1（最短路径，唯一路径）

工位跳转统一使用最短路径（双向选择 ≤2 个工位），补偿与否由 `B:` 开关决定：

```
从工位A→工位B: 选择旋转 ≤2 个工位的方向
  例: P1→P3: 顺时针2个工位 (motion(2,0), level=6)
  例: P1→P4: 逆时针2个工位 (motion(2,1), level=6)
```

- `B:0`（补偿）: 每站运动带补偿（level=3 反向回退，精度高）
- `B:1`（一步到位）: 直接走目标距离（level=6，省时）

### step_motor_L2（已弃用，"高顺低逆"）

固定方向路径，`B:0` 改为最短路径后已不再被调用，保留作参考：

```
顺时针 (高顺):  motion(N, 0)     → level=6×N
逆时针 (低逆): compens(N, 1)   → level=6×(N-1) + level=3
```
```

- [ ] **步骤 5：复查文档**

运行：`grep -n "固定方向\|高顺低逆\|最短路径" doc/使用手册.md doc/README.md`
预期：使用手册中"固定方向（`step_motor_L2`）"表述已删除；README 中 L1/L2 两节语义与新行为一致。

- [ ] **步骤 6：提交**

`git add doc/使用手册.md doc/README.md && git commit -m "docs: B:0 工位跳转改最短路径, 同步使用手册与README"`
（文档文件工作区干净，可安全整体提交）

---

## 任务 4：编译与真机验证（用户执行）

**说明：** 本仓库无自动化测试，代码改动需在 Keil 中编译并由用户上真机验证。以下步骤供用户执行，不占用代理。

- [ ] **步骤 1：编译**

在 Keil MDK 打开 `MDK-ARM/test.uvprojx` → Build（F7）。预期零错误；`step_motor_L2` 保留故无未使用告警。产物 `MDK-ARM/test/test.hex`。

- [ ] **步骤 2：真机路径验证**

串口设 `B:0`，依次发 `L1`~`L5`，确认：
- 1→5 走反转 1 站（而非正转 4 站）；1→4 走反转 2 站；其余相邻/2 站内走正转。
- 运动完成打印 `[时间] P<工位>`，无异常。

- [ ] **步骤 3：真机补偿验证**

`B:0` 下发后观察每站运动带补偿（过冲+回退）；对比 `B:1` 无补偿。确认进孔定位精度与改前一致（无因方向改变导致的进孔不一致）。

- [ ] **步骤 4：真机 gnn 疲劳测试**

`B:0` 下执行 `PL`（或 `gnn`），观察循环往返无运动重叠、无卡死；对比改前 L2 行为。

- [ ] **步骤 5：提交代码改动（如任务 1/2 未提交）**

代码改动若因在途改动未提交，真机验证通过后由用户确认提交（或告知代理合并提交）。

---

## 自检

- **规格覆盖度**：§3.1→任务 1；§3.2→任务 2；§3.3（保留 L2，不改）→ 已注明；§3.4→任务 3；§4 验证→任务 4。全部覆盖。
- **占位符扫描**：无 TODO/待定；每个代码步骤含完整改动内容。
- **类型一致性**：`motion_path`/`step_motor_L1`/`step_motor_L2`/`g_no_compensate`/`receive_flag` 均与现状代码一致；L1/L2 声明不动。
