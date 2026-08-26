# 复位改到 PF6 实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:subagent-driven-development（推荐）或 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 把复位功能从 PE2 移到 PF6（PF6 原为正转 IO 触发，现改作复位输入），移除正转 IO 触发，删除 PE2 复位逻辑；PF7 急停不变，串口 `RTF`/`RTB` 保留。

**架构：** `step_motor.c` 初始化中 PF6 仍为 24V 上拉输入（仅改注释语义），删除 PE2 输入初始化；`main.c` 主循环 IO 轮询中 PF6 低电平由"正转"改为"复位"（清 error/解除急停锁定），删除 PE2 轮询块。串口命令路径不受影响。

**技术栈：** C（STM32F407，HAL + Keil MDK），无自动化测试环境。

**规格：** `docs/superpowers/specs/2026-08-26-reset-pin-pf6-design.md`

---

## ⚠️ 工作区注意事项（重要，先读）

1. **仓库 `new/` 工作区有大量未提交在途改动**（以太网/bootloader + 上一条 B:0 最短路径改动）。**禁止 `git add .` / `git add -A` / `git commit -am`**。
2. 提交策略（按文件是否干净）：
   - `Core/Src/step_motor.c`：**在途改动，只编辑不提交**（6208 行差异，禁止整文件混提）。
   - `Core/Src/main.c`：**干净，可提交**。
   - `doc/使用手册.md`：**已跟踪且干净，可提交**。
   - `doc/README.md`：**未跟踪新文件，只编辑不提交**。
3. 本仓库无自动化测试。**每个代码步骤的验证方式是：用 `git diff`（仅对干净文件）/ 直接阅读改动区域复查**；编译由用户在 Keil 中执行（见任务 4）。
4. **提交前**对每个待提交文件先 `git diff <文件>` 复查，确认只含本任务改动。

---

## 文件结构

| 文件 | 职责 | 改动 | 提交 |
|------|------|------|------|
| `Core/Src/step_motor.c` | 引脚初始化 | PF6 注释改复位、删除 PE2 初始化块与冗余时钟行 | **不提交**（在途） |
| `Core/Src/main.c` | 主循环 IO 轮询 | PF6 低电平改复位逻辑、删除 PE2 轮询块 | 提交 |
| `doc/使用手册.md` | 用户手册 | PF6→复位、PE2 复位表述删除 | 提交 |
| `doc/README.md` | 技术说明 | IO/引脚表更新 | **不提交**（未跟踪） |

不改动：`cmd_handler.c`（串口 `RTF`/`RTB` 保留）、`step_motor.c` 的 PF7 急停配置、`doc/电机进出孔策略算法说明.md`。

---

## 任务 1：`step_motor.c` 引脚初始化

**文件：**
- 修改：`Core/Src/step_motor.c:83-107`

- [ ] **步骤 1：改 PF6 注释**

将 [step_motor.c:87](Core/Src/step_motor.c#L87) 的注释：

```c
  /* PF6: 24V输入1 正转触发 (轮询) */
```

替换为：

```c
  /* PF6: 24V输入 复位 (轮询, 低电平=施加24V触发) */
```

GPIO 配置（`GPIO_PIN_6`、`GPIO_MODE_INPUT`、`GPIO_PULLUP`）**不变**。

- [ ] **步骤 2：删除 PE2 初始化块与冗余时钟行**

删除 [step_motor.c:85](Core/Src/step_motor.c#L85) 的：

```c
  __HAL_RCC_GPIOE_CLK_ENABLE(); /* PE2: 复位输入 */
```

（GPIOE 时钟已由 `ST1_HAL1_GPIO_CLK_ENABLE()`/`ST1_HAL2_GPIO_CLK_ENABLE()` 使能，L23-24。）

删除 [step_motor.c:104-107](Core/Src/step_motor.c#L104-L107) 的：

```c
  gpio_init_struct.Pin = GPIO_PIN_2;
  gpio_init_struct.Mode = GPIO_MODE_INPUT;
  gpio_init_struct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &gpio_init_struct);
```

（PE2 不是 24V 光耦输入，复位不再走它。）

- [ ] **步骤 3：复查**

运行：`grep -n "GPIO_PIN_2\|GPIOE\|PF6\|PF7" Core/Src/step_motor.c`
预期：PE2 的 `GPIO_PIN_2` 初始化块与 `/* PE2: 复位输入 */` 注释消失；`GPIOE` 仅剩霍尔相关（`ST1_HAL1/2_GPIO_CLK_ENABLE`、`HAL_GPIO_Init(ST1_HAL1/2_GPIO_PORT,...)`）；PF6/PF7 块仍在，PF6 注释为"复位"。

- [ ] **步骤 4：处理提交**

`step_motor.c` 在途改动，**不提交**。记录待用户手动提交。

---

## 任务 2：`main.c` IO 轮询

**文件：**
- 修改：`Core/Src/main.c:152-175`

- [ ] **步骤 1：改 PF6 块为复位逻辑**

将 [main.c:153-156](Core/Src/main.c#L153-L156) 的：

```c
        if (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_6) == 0) {      /* PF6: 正转 (LOW有效) */
            printf("IO:RTF\r\n");
            cmd_rtf();
        }
```

替换为：

```c
        if (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_6) == 0) {      /* PF6: 复位 (LOW有效) */
            time_out = 0;
            error = 0;
            g_zero_flag = 0;
            reset_flag = 0;            /* 解除急停锁定 */
            alarm_set(0);              /* 清除报警输出 */
        }
```

- [ ] **步骤 2：删除 PE2 复位块**

删除 [main.c:169-175](Core/Src/main.c#L169-L175) 的：

```c
        if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2) == 0) {      /* PE2: 复位 (LOW有效) */
            time_out = 0;
            error = 0;
            g_zero_flag = 0;
            reset_flag = 0;            /* 解除急停锁定 (恢复F1: 复位同时清reset_flag) */
            alarm_set(0);              /* 清除报警输出 */
        }
```

PF7 急停块（L157-168）**保持不变**。

- [ ] **步骤 3：复查**

运行：`grep -n "GPIO_PIN_6\|GPIO_PIN_7\|GPIO_PIN_2\|cmd_rtf\|alarm_set" Core/Src/main.c`
预期：`GPIO_PIN_6` 出现在复位块（含 `reset_flag = 0`、`alarm_set(0)`）；`GPIO_PIN_7` 急停块不变；`GPIO_PIN_2`（PE2）在主循环消失；`cmd_rtf` 不再被 main.c 引用（仍在 cmd_handler.c 被串口命令使用，无死代码）；`alarm_set` 已有声明可用。

- [ ] **步骤 4：提交**

先 `git diff Core/Src/main.c` 确认只有本任务改动，然后：

```bash
git add Core/Src/main.c
git commit -m "feat(io): 复位改到PF6, 移除正转IO触发与PE2复位"
```

---

## 任务 3：同步文档

**文件：**
- 修改：`doc/使用手册.md`（L37、L39、L60、L63、L78、L80、L82、L91、L96-98、L348-349）
- 修改：`doc/README.md`（L44、L46、L296、L298）

- [ ] **步骤 1：`使用手册.md` 引脚表（L37-39）**

将：

```markdown
| 24V 输入 1 (IN1) | PF6 | 正转信号输入（低电平触发） |
| 24V 输入 2 (IN2) | PF7 | 急停信号输入（低电平触发，下降沿中断响应，运动期间立即生效） |
| 复位输入 | PE2 | 复位信号（低电平清除报警并解除急停锁定） |
```

替换为（删除 PE2 行，PF6 改为复位）：

```markdown
| 24V 输入 1 (IN1) | PF6 | 复位信号输入（低电平清除报警并解除急停锁定） |
| 24V 输入 2 (IN2) | PF7 | 急停信号输入（低电平触发，下降沿中断响应，运动期间立即生效） |
```

- [ ] **步骤 2：`使用手册.md` 连接器引脚表（L60、L63）——需用户确认接线**

⚠️ 连接器物理引脚的实际接线不由固件决定，文档无法单方面确认。PF6 原为**板内** 24V 光耦输入（L63 注明"不在上述连接器引脚定义内"），复位改到 PF6 后，原连接器"引脚 5（复位/PE2）"的物理去向需用户确认。本步骤**只更新固件行为描述并标注待确认**：

将 L63 的注改为：

```markdown
> 注：复位（PF6）为板载 24V 光耦输入（24V 信号输入 1），原"正转 IO 触发"已移除，正转仅保留串口 `RTF` 命令。急停（PF7）为 24V 信号输入 2。原连接器引脚 5（PE2 复位）的物理去向待硬件确认。
```

连接器引脚表（L60）保持 PE2 行但标注"（已弃用，接线待确认）"：

```markdown
| 5 | 复位（输入） | PE2（已弃用） | 固件已改由 PF6（板内 24V 输入）复位；本引脚物理去向待确认 |
```

- [ ] **步骤 3：`使用手册.md` IO 触发表与流程（L78-82、L91、L96-98）**

将 L78-80 表改为：

```markdown
| 复位 | PF6 | 施加 24V（轮询检测，约 50ms 周期） | 清除报警、解除急停锁定 |
| 急停 | PF7 | 施加 24V 脉冲（约 20ms，下降沿中断） | 立即停止电机并锁定 |
```

删除 L82 注中的 PE2 表述，改为：

```markdown
> 注：复位/急停均为 **24V 高电平**触发（光耦输入，低电平生效）。
```

L91 末句"急停后需给 PE2 发复位信号解锁"改为"急停后需给 PF6 发复位信号解锁"。

L96-98 上电自检改为：

```markdown
2. 急停：给 PF7 发 24V 脉冲（≈20ms）→ 立即停电机并锁定
3. 复位：给 PF6 施加 24V → 解除锁定、清报警
4. 复位后通过串口（`D0` 回零 / `L1~L5` 切工位）继续操作
```

（删除原"正转：给 PF6 施加 24V → 转台前进一个工位"一行。）

- [ ] **步骤 4：`使用手册.md` 急停保护说明（L348-349）**

将 L348-349 中"解除锁定需 IO 复位（PE2 低电平）"与"复位信号：PE2 低电平"的 PE2 表述改为 PF6：

```markdown
2. **急停保护**：IN2（PF7）低电平时由下降沿中断立即停止电机——运动期间主循环忙等（`RTF`/`RTB`/`D0`/测试）也能响应。停止脉冲输出并置 `reset_flag` 锁定，拒绝一切运动命令；解除锁定需 IO 复位（PF6 施加 24V）或 `RES` 软复位。
3. **复位信号**：PF6 低电平时清除 `error`/`time_out`/`g_zero_flag` 并解除急停锁定（清 `reset_flag`）。急停后必须先复位，才能再次回零或切换工位。
```

- [ ] **步骤 5：`README.md` IO/引脚表（L44、L46、L296、L298）**

将：

```markdown
| PF6 | 正转触发（低电平=有24V输入→正转一个工位） |
| PF7 | 急停信号（低电平=有24V输入→立即停止） |
| PE2 | 复位/消警 |
```

替换为：

```markdown
| PF6 | 复位输入（低电平=有24V输入→清报警、解除急停锁定） |
| PF7 | 急停信号（低电平=有24V输入→立即停止） |
```

将：

```markdown
| **24V 正转** | PF6 | 上拉输入 | 光耦，低电平有效 |
| **24V 急停** | PF7 | 上拉输入 | 光耦，低电平有效 |
| **复位输入** | PE2 | 上拉输入 | 普通 GPIO |
```

替换为：

```markdown
| **24V 复位** | PF6 | 上拉输入 | 光耦，低电平有效 |
| **24V 急停** | PF7 | 上拉输入 | 光耦，低电平有效 |
```

- [ ] **步骤 6：复查文档**

运行：`grep -n "正转触发\|PE2\|PF6" doc/使用手册.md doc/README.md`
预期：`正转触发` 不再指 PF6 IO；PE2 复位表述消失；PF6 均标注为复位（除 `电机进出孔策略算法说明.md` 不查）。

- [ ] **步骤 7：提交**

`doc/使用手册.md` 干净可提交；`doc/README.md` 未跟踪**不提交**：

```bash
git add doc/使用手册.md
git commit -m "docs: 复位改到PF6, 同步使用手册"
```

---

## 任务 4：编译与真机验证（用户执行）

**说明：** 本仓库无自动化测试，代码改动需在 Keil 中编译并由用户上真机验证。以下步骤供用户执行，不占用代理。

- [ ] **步骤 1：编译**

在 Keil MDK 打开 `MDK-ARM/test.uvprojx` → Build（F7）。预期零错误；`cmd_rtf` 仍被串口命令使用，无未引用告警。

- [ ] **步骤 2：真机复位/急停验证**

- PF6 施加 24V → 复位生效：清 `error`/`time_out`/`g_zero_flag`、解除急停锁定（`reset_flag=0`）、清报警。
- PF7 施加 24V → 急停立即停电机并锁定（`reset_flag=1`）；再 PF6 复位解锁。
- 确认 PE2 不再触发复位。

- [ ] **步骤 3：真机正转/跳转验证**

- 串口 `RTF` 仍正转一个工位、`RTB` 反转一个工位。
- `L1`~`L5` 跳转正常。

- [ ] **步骤 4：提交代码改动（任务 1 未提交部分）**

`step_motor.c` 改动因在途未提交，真机验证通过后由用户确认提交（或告知代理合并提交）。

---

## 自检

- **规格覆盖度**：§3.1→任务 1；§3.2→任务 2；§3.3（串口保留）→ 已注明不改；§3.4→任务 3；§4 验证→任务 4。全部覆盖。
- **占位符扫描**：无 TODO/待定；每个代码步骤含完整替换内容。
- **类型一致性**：`GPIO_PIN_6`/`GPIO_PIN_7`/`GPIO_PIN_2`/`cmd_rtf`/`alarm_set`/`reset_flag` 均与现状代码一致；未引入新符号。
