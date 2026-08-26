# 连接器引脚表更新 + 移除 B 开关 + 注释 EL 实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:subagent-driven-development（推荐）或 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** ①注释 atim.c 的 `EL` 超时报警打印；②使用手册连接器引脚表更新为新 12 脚定义；③彻底移除 B 开关（`g_no_compensate`），系统固定为补偿 + 最短路径。

**架构：** 代码侧删除 `g_no_compensate` 的全部引用（main.c 定义、cmd_handler.c 命令/分发/PARA、step_motor.c `motion_dispatch` 分支、flash_param.c 存取），`motion_dispatch` 恒走 `step_motor_motion_compensate`；文档侧更新 使用手册 与 README 的连接器表与 B 描述。

**技术栈：** C（STM32F407，HAL + Keil MDK），无自动化测试环境。

**规格：** `docs/superpowers/specs/2026-08-26-connector-pin-b-removal-design.md`

---

## ⚠️ 工作区注意事项（重要，先读）

1. **仓库 `new/` 工作区有大量未提交在途改动**。**禁止 `git add .` / `git add -A` / `git commit -am`**。
2. 提交策略：
   - **可提交**（干净）：`Core/Src/main.c`、`doc/使用手册.md`。
   - **只编辑不提交**（在途/未跟踪）：`Core/Src/atim.c`、`Core/Src/cmd_handler.c`、`Core/Src/step_motor.c`、`Core/Src/flash_param.c`、`doc/README.md`。
3. 本仓库无自动化测试。**每个代码步骤的验证方式是：编辑后用 grep/Read 复查**；编译由用户在 Keil 中执行（见任务 5）。
4. 任务 2 的代码删除**必须跨文件一次性完成**（`g_no_compensate` 删掉定义后，其他文件若仍引用会编译失败）；提交只含 main.c。

---

## 文件结构

| 文件 | 改动 | 提交 |
|------|------|------|
| `Core/Src/atim.c` | 注释 `printf("EL\r\n")` | **不提交** |
| `Core/Src/cmd_handler.c` | 删 `cmd_set_no_comp`、`B:` 分发、PARA 显示、extern | **不提交** |
| `Core/Src/step_motor.c` | `motion_dispatch` 恒走补偿；删 extern | **不提交** |
| `Core/Src/flash_param.c` | 删 `g_no_compensate` 存取/打印/extern | **不提交** |
| `Core/Src/main.c` | 删 `u8 g_no_compensate = 0;` | 提交 |
| `doc/使用手册.md` | 连接器引脚表 + B 描述 | 提交 |
| `doc/README.md` | 路径规划 B 描述 | **不提交** |

不改动：`doc/指令速查表.md`（无 B 引用）、`PARAM_OFFSET_NO_COMP` 宏定义（保留未用）。

---

## 任务 1：注释 `EL` 报警打印（atim.c）

**文件：**
- 修改：`Core/Src/atim.c:111`

- [ ] **步骤 1：注释 printf**

将 [atim.c:111](Core/Src/atim.c#L111) 的：

```c
      printf("EL\r\n");
```

替换为：

```c
      // printf("EL\r\n");          /* 超时报警打印已注释, error=1 与报警脉冲仍生效 */
```

周围逻辑（`error = 1;`、`HAL_TIM_Base_Stop_IT`、`alarm_set(1)`→`delay_ms(50)`→`alarm_set(0)`）**保持不变**。

- [ ] **步骤 2：复查**

运行：`grep -n 'EL\\r' Core/Src/atim.c`
预期：唯一命中以 `//` 开头。

- [ ] **步骤 3：处理提交**

`atim.c` 在途改动，**不提交**。

---

## 任务 2：彻底移除 B 开关（代码，跨 4 文件）

**文件：**
- 修改：`Core/Src/cmd_handler.c`（L27、L153-158、L423、L650）
- 修改：`Core/Src/step_motor.c`（L1974、L2926-2938）
- 修改：`Core/Src/flash_param.c`（L22、L76-78、L183）
- 修改：`Core/Src/main.c`（L27）
- 提交：`Core/Src/main.c`

- [ ] **步骤 1：`main.c` 删定义**

删除 [main.c:27](Core/Src/main.c#L27) 的：

```c
u8 g_no_compensate = 0;        /* 0=补偿, 1=一步到位不补偿 */
```

- [ ] **步骤 2：`cmd_handler.c` 删命令/分发/显示/extern**

2a. 删除 [cmd_handler.c:27](Core/Src/cmd_handler.c#L27) 的 `extern u8 g_no_compensate;`。

2b. 删除 [cmd_handler.c:153-158](Core/Src/cmd_handler.c#L153-L158) 的 `cmd_set_no_comp` 整个函数：

```c
void cmd_set_no_comp(u8 *buf, int len) {
    int v = parse_uart_number(buf, len);
    g_no_compensate = (v != 0) ? 1 : 0;
    printf("g_no_compensate: %d\r\n", g_no_compensate);
    param_save_all();
}
```

2c. 删除 [cmd_handler.c:650](Core/Src/cmd_handler.c#L650) 的 `B:` 分发行：

```c
    if (buf[0]=='B' && buf[1]==':') { cmd_set_no_comp(buf, len); return 1; }
```

2d. 删除 [cmd_handler.c:423](Core/Src/cmd_handler.c#L423) 的 PARA 显示行：

```c
    printf("g_no_compensate(B:): %d\r\n", g_no_compensate);
```

- [ ] **步骤 3：`step_motor.c` motion_dispatch 恒走补偿 + 删 extern**

3a. 删除 [step_motor.c:1974](Core/Src/step_motor.c#L1974) 的 `extern u8 g_no_compensate;`。

3b. 将 [step_motor.c:2931-2938](Core/Src/step_motor.c#L2931-L2938) 的：

```c
/* 根据补偿开关选择运动方式: 0=补偿(现状) 1=一步到位不补偿 */
static void motion_dispatch(int num, int dir) {
  if (g_no_compensate) {
    step_motor_motion(num, dir);
  } else {
    step_motor_motion_compensate(num, dir);
  }
}
```

替换为：

```c
/* 运动方式: 恒为补偿模式 (B 开关已移除) */
static void motion_dispatch(int num, int dir) {
  step_motor_motion_compensate(num, dir);
}
```

> 注：`step_motor_motion`（不补偿版）仍被 `cmd_gnn_test` 直接调用（cmd_handler.c:508），不会变成死代码。

- [ ] **步骤 4：`flash_param.c` 删存取/打印/extern**

4a. 删除 [flash_param.c:22](Core/Src/flash_param.c#L22) 的 `extern u8 g_no_compensate;`。

4b. 删除 [flash_param.c:76-78](Core/Src/flash_param.c#L76-L78) 的加载+打印（含上方 `val = PARAM_READ(...)` 行），即：

```c
    val = PARAM_READ(PARAM_OFFSET_NO_COMP);
    if (val >= 0) g_no_compensate = (val == 1) ? 1 : 0;
    printf("g_no_compensate(B:): %d\r\n", g_no_compensate);
```

（若实际行内容与该文本有出入，以 Read 到的一致为准——删除整个 g_no_compensate 加载块。）

4c. 删除 [flash_param.c:183](Core/Src/flash_param.c#L183) 的保存行：

```c
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, PARAM_SECTOR_BASE + PARAM_OFFSET_NO_COMP,   (uint32_t)g_no_compensate);
```

- [ ] **步骤 5：复查无残留引用**

运行：`grep -rn 'g_no_compensate' Core/`
预期：**零命中**（main.c/cmd_handler.c/step_motor.c/flash_param.c 均无残留）。

- [ ] **步骤 6：提交 main.c**

先 `git diff Core/Src/main.c` 确认只有删定义一行，然后：

```bash
git add Core/Src/main.c
git commit -m "feat(param): 移除B开关, 固定补偿模式"
```

（其余三个文件在途，不提交。）

---

## 任务 3：使用手册连接器引脚表 + B 描述更新（提交）

**文件：**
- 修改：`doc/使用手册.md`（L57-64 连接器表、L68、L76-84、L138、L153、L202-204、L283-293、L366、L382-383）

- [ ] **步骤 1：连接器引脚表（L57-64）**

将：

```markdown
| 连接器引脚 | 功能             | MCU 引脚 | 说明                                      |
| :--------: | ---------------- | -------- | ----------------------------------------- |
|     1     | 24V 供电正       | —       | 光耦输入侧电源                            |
|     2     | 24V 供电负       | —       | 电源回路                                  |
|     3     | 到位信号（输出） | PC4      | 运动完成输出 100ms 高脉冲                 |
|     4     | 报警信号（输出） | PC2      | 运动超时报警输出 50ms 高脉冲              |
|     5     | 复位（输入）     | PF6      | —                                        |
|     6     | 急停（输入）     | PF7      | 24V 脉冲（约 20ms），下降沿中断立即停电机 |
```

替换为：

```markdown
| 连接器引脚 | 功能说明              | 信号类型   | MCU 引脚 |
| :--------: | --------------------- | :--------: | :------: |
|     1     | 24V 输入信号 COM（公共端） | 输入公共端 | —       |
|     2     | 24V 输入信号1 - 复位   | 数字输入   | PF6      |
|     3     | 24V 输入信号2 - 急停   | 数字输入   | PF7      |
|     4     | 24V 输出信号1 - 使能   | 数字输出   | PC1      |
|     5     | 24V 输出信号2 - 报警   | 数字输出   | PC2      |
|     6     | DC24V（电源正极）      | 电源输入   | —       |
|     7     | GND（电源地）          | 电源输入   | —       |
|     8-12  | 暂定                  | —          | —       |

> 注：到位信号（PC4）不在新连接器定义内（落 8-12 暂定）；使能=PC1（`ST1_EN` 开漏输出），报警=PC2（`ST1_AUX`，`alarm_set()`）。
```

- [ ] **步骤 2："施加 24V" 说明（L68）**

将"把信号引脚接通到 24V 电源（引脚 1）"中的"（引脚 1）"改为"（引脚 6，DC24V）"，并同步句意（引脚 1 现为 24V 输入 COM，不是电源正极）。最终该句为：

```markdown
"给引脚施加 24V"指通过**按钮、继电器或 PLC 触点**把信号引脚接通到 24V 电源（引脚 6，DC24V），使引脚与 24V 短接——固件经光耦感知电平变化后执行对应动作。这**不同于串口命令**（通过串口发送 `RTF`/`D0`/`L1~L5` 等文字指令），两者是独立通道，效果等价。
```

- [ ] **步骤 3：IO 输入/输出用法表（L79-82 区域）**

在现有"复位/急停"两行后补充输出两行，形成：

```markdown
| 功能 | MCU 引脚 | 触发方式 | 作用 |
| ---- | -------- | -------- | ---- |
| 复位 | PF6 | 施加 24V（轮询检测，约 50ms 周期） | 清除报警、解除急停锁定 |
| 急停 | PF7 | 施加 24V 脉冲（约 20ms，下降沿中断） | 立即停止电机并锁定 |
| 使能 | PC1 | 输出（默认释放） | 电机使能控制 |
| 报警 | PC2 | 输出（超时报警 50ms 高脉冲） | 运动超时报警 |
```

- [ ] **步骤 4：B: 参数行（L138）删除**

删除参数表里的：

```markdown
| `B:` | 补偿开关（0=补偿，1=一步到位不补偿） | 0 | 0/1 | ✓ |
```

- [ ] **步骤 5：B:1 示例（L153）删除**

删除示例块里的：

```markdown
B:1     → 开启一步到位（不补偿）
```

- [ ] **步骤 6：跳转路径说明（L202-204）**

将：

```markdown
> 跳转路径统一为**最短路径**（`step_motor_L1`，双向选择，最多 2 步）：
> - `B:0`：最短路径 + 每站运动带补偿（精度高）
> - `B:1`：最短路径 + 一步到位不补偿（省时）
```

替换为：

```markdown
> 跳转路径统一为**最短路径**（`step_motor_L1`，双向选择，最多 2 步），每站运动**带补偿**（固定模式，无 `B:` 开关）。
```

- [ ] **步骤 7：补偿机制说明（L283-293）**

将整个 §4 补偿机制说明中的 B:0/B:1 对比表与两条 bullet 替换为单模式描述：

```markdown
| 模式 | 行为 | 精度 |
|------|------|------|
| 固定补偿 | 每站运动先正向过冲 `C:` 脉冲，再反向回退，使齿轮咬合在固定一侧（跳转走最短路径） | 高，可重复 |
```

并将两条 bullet 改为：

```markdown
- 系统固定使用补偿模式（无 `B:` 开关切换）
```

- [ ] **步骤 8：F4 对比表（L366、L382-383）**

L366 删除：

```markdown
| `B:` | 参数 | 补偿开关（0=补偿，1=一步到位），F1 为固定策略 |
```

L382-383 的"补偿开关"/"最短路径"两行替换为：

```markdown
| **固定补偿** | F4 恒为补偿模式（无 `B:` 开关）；F1 固定为「顺时针不补偿、逆时针补偿」 |
| **最短路径** | `L1`~`L5` 跳转使用最短路径（双向选择，最多 2 步），每站带补偿；F1 只有固定方向 |
```

- [ ] **步骤 9：复查**

运行：`grep -n 'B:0\|B:1\|B:`\|g_no_compensate' doc/使用手册.md`
预期：无 `B:0`/`B:1`/`B:` 命令残留（仅"无 `B:` 开关"这类说明性文字可保留）。

- [ ] **步骤 10：提交**

```bash
git add doc/使用手册.md
git commit -m "docs: 更新连接器引脚表, 移除B开关说明"
```

---

## 任务 4：README B 描述更新（不提交）

**文件：**
- 修改：`doc/README.md`（L361、L369-370、L374）

- [ ] **步骤 1：路径规划 L1 节（L361、L369-370）**

将 [README.md:361](doc/README.md#L361) 的：

```markdown
工位跳转统一使用最短路径（双向选择 ≤2 个工位），补偿与否由 `B:` 开关决定：
```

替换为：

```markdown
工位跳转统一使用最短路径（双向选择 ≤2 个工位），每站运动带补偿（固定模式）：
```

并将 L369-370 的：

```markdown
- `B:0`（补偿）: 每站运动带补偿（level=3 反向回退，精度高）
- `B:1`（一步到位）: 直接走目标距离（level=6，省时）
```

替换为：

```markdown
- 每站运动带补偿（level=3 反向回退，精度高）
```

- [ ] **步骤 2：L2 节（L374）**

将：

```markdown
固定方向路径，`B:0` 改为最短路径后已不再被调用，保留作参考：
```

替换为：

```markdown
固定方向路径，已不再被调用，保留作参考：
```

- [ ] **步骤 3：复查**

运行：`grep -n 'B:0\|B:1' doc/README.md`
预期：无 `B:0`/`B:1` 残留。

- [ ] **步骤 4：处理提交**

`doc/README.md` 未跟踪，**不提交**。

---

## 任务 5：编译与真机验证（用户执行）

**说明：** 本仓库无自动化测试，需用户在 Keil 编译并上真机确认。

- [ ] **步骤 1：编译**

Keil MDK 打开 `MDK-ARM/test.uvprojx` → Build（F7）。预期**零错误**（`g_no_compensate` 无残留引用，`step_motor_motion` 仍被 gnn 测试使用故无未引用告警）。

- [ ] **步骤 2：串口验证**

- 上电参数表**无** `g_no_compensate(B:)` 行。
- `PARA` 无 `g_no_compensate(B:)`；发送 `B:0`/`B:1` **无响应**。
- 运动 `L1`~`L5` 走最短路径且带补偿。
- 超时 20s：`error=1`、报警脉冲仍输出，但**无 `EL` 打印**。

- [ ] **步骤 3：连接器验证**

- 引脚 2 施加 24V → 复位；引脚 3 施加 24V → 急停；引脚 4 使能输出正常；引脚 5 报警输出正常。

- [ ] **步骤 4：提交未提交部分（任务 1/2/4）**

`atim.c`、`cmd_handler.c`、`step_motor.c`、`flash_param.c`、`README.md` 改动在途未提交，验证通过后由用户确认提交（或告知代理合并提交）。

---

## 自检

- **规格覆盖度**：§2→任务 1；§3→任务 3；§4.1→任务 2；§4.3 使用手册→任务 3、README→任务 4（指令速查表无 B 引用，无需改）；§6 验证→任务 5。全部覆盖。
- **占位符扫描**：无 TODO/待定；代码删除给出行号与内容；文档替换给出确切新旧文本。
- **类型一致性**：`g_no_compensate`/`motion_dispatch`/`step_motor_motion_compensate`/`PARAM_OFFSET_NO_COMP` 均与现状一致；删除后无残留引用（步骤 5 的 grep 兜底）。
