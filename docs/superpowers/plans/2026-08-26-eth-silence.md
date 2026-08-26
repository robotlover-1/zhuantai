# 以太网打印静音 实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:subagent-driven-development（推荐）或 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 注释掉 eth.c / ethernetif.c 的全部 `[ETH]`/`[PHY]` 打印、flash_param.c 的 IP/MASK/GW/PORT 回显、main.c 的 banner 块；保留参数回显（D:…CS_H/total_cycles）与 `PARA`/`CYCLE` 命令查询。

**架构：** 全部采用**行注释 `//`** 逐行注释 printf（多行 printf 每行都加 `//`，避免 `/* */` 嵌套问题）。`printf` 成为块中唯一语句时补 `(void)0;`；变量变为"set but not used"时补 `(void)var;`。

**技术栈：** C（STM32F407，HAL + Keil MDK），无自动化测试环境。

**规格：** `docs/superpowers/specs/2026-08-26-eth-silence-design.md`

---

## ⚠️ 工作区注意事项（重要，先读）

1. **仓库 `new/` 工作区有大量未提交在途改动**。**禁止 `git add .` / `git add -A` / `git commit -am`**。
2. 提交策略：
   - `Core/Src/eth.c`、`Core/Src/ethernetif.c`：**未跟踪新文件，只编辑不提交**。
   - `Core/Src/flash_param.c`：**在途改动，只编辑不提交**。
   - `Core/Src/main.c`：**干净，可提交**（提交命令见任务 4）。
3. 本仓库无自动化测试。**每个代码步骤的验证方式是：编辑后用 grep/Read 复查**；编译由用户在 Keil 中执行（见任务 5）。
4. **注释规则**：一律用 `//` 行注释。单行 printf：行首加 `//`。多行 printf（参数换行）：每一行行首加 `//`。已注释的调试行（如 eth.c L271/L756）不动。

---

## 文件结构

| 文件 | 改动 | 提交 |
|------|------|------|
| `Core/Src/eth.c` | 注释掉全部 `[ETH]`/`[PHY]`/`[PHY-10H]`/`[PHY-10F]` printf（35 处） | **不提交**（未跟踪） |
| `Core/Src/ethernetif.c` | 注释掉 `[ETH] netif init OK, MAC`（L35） | **不提交**（未跟踪） |
| `Core/Src/flash_param.c` | 注释掉 `IP:`/`MASK:`/`GW:`/`PORT:`（L125-158 中的四处）；**保留**其余参数打印 | **不提交**（在途） |
| `Core/Src/main.c` | 注释掉 banner 块（L106-111） | 提交 |

不改动：`cmd_handler.c` 的 `PARA`/`CYCLE` 命令打印（有意查询）、`[TEST]`/`[REG-W-DBG]` 前缀打印（在未被调用的诊断函数里，非 `[ETH]`/`[PHY]`）。

---

## 任务 1：`eth.c` 注释掉全部 `[ETH]`/`[PHY]` printf

**文件：**
- 修改：`Core/Src/eth.c`（多处）

- [ ] **步骤 1：定位全部目标 printf**

运行：`grep -n 'printf("\[ETH\]\|printf("\[PHY\]' Core/Src/eth.c`
预期命中以下行（分组列出，便于逐处核对）。对**每一处**用 `//` 逐行注释（多行 printf 的每一行都加 `//`）：

| 行号 | printf 首串 | 备注 |
|------|------------|------|
| 191-192 | `[ETH] DM9000 @` | 多行 |
| 254 | `[ETH] DM9000 NOT found!` | |
| 255 | `[ETH] Check FSMC wiring` | |
| 277 | `[ETH] DM9000 detected` | |
| 288-290 | `[ETH] No valid MAC` | 多行 |
| 296-298 | `[ETH] MAC:` | 多行；else 块唯一语句，见步骤 2 |
| 447 | `[ETH] client disconnected` | |
| 477 | `[ETH] TCP error:` | |
| 495 | `[ETH] reject new client` | |
| 509 | `[ETH] client connected` | |
| 586 | `[ETH] LINK UP` | |
| 589 | `[ETH] LINK DOWN` | |
| 717-718 | `[PHY] EPCR address` | 多行 |
| 727 | `[PHY] BMCR  =` | |
| 728 | `[PHY] BMSR1 =` | |
| 729 | `[PHY] BMSR2 =` | |
| 730 | `[PHY] ID1   =` | |
| 731 | `[PHY] ID2   =` | |
| 732 | `[PHY] ANAR  =` | |
| 744 | `[PHY] restart autoneg` | |
| 759 | `[PHY] LINK UP` | |
| 764-765 | `[PHY] LINK TIMEOUT` | 多行 |
| 773-776 | `[PHY-10H] BMCR=` | 多行 |
| 783-786 | `[PHY-10F] BMCR=` | 多行 |
| 801 | `[PHY] reset timeout` | |
| 811-813 | `[PHY] reset complete` | 多行 |
| 843-844 | `[PHY] BMCR=` | 多行 |
| 1027-1028 | `[PHY] BMSR=` | 多行；if 块唯一语句，见步骤 3 |
| 1030 | `[PHY] internal PHY did not update EPDR` | else 块唯一语句，见步骤 3 |
| 1069 | `[ETH] LwIP init done` | |
| 1082 | `[ETH] netif_add failed!` | |
| 1088-1089 | `[ETH] netif up:` | 多行 |
| 1094 | `[ETH] TCP PCB create failed!` | |
| 1098 | `[ETH] TCP bind port` | |
| 1104 | `[ETH] TCP server listening` | |

- [ ] **步骤 2：处理 L296-298 空 else 块**

注释 L296-298 后，`else { ... }` 块变空。改为（在 else 内补 `(void)0;`）：

```c
    } else {
        /* printf("[ETH] MAC: ..."); 已注释 */
        (void)0;
    }
```

- [ ] **步骤 3：处理 L1026-1031 空 if/else 与未使用变量**

注释 L1027-1028、L1030 后，if/else 两分支变空，且 `bmsr/id1/id2`（声明于 L959）不再被读取。改为：

```c
        if (r0 == 0 && r1 == 0 && r2 == 0 && r3 == 0) {
            /* printf("[PHY] BMSR=..."); 已注释 */
            (void)0;
        } else {
            /* printf("[PHY] internal PHY ..."); 已注释 */
            (void)0;
        }
        (void)bmsr; (void)id1; (void)id2;   /* 避免 set-but-unused 告警 */
```

- [ ] **步骤 4：复查**

运行：`grep -n 'printf("\[ETH\]\|printf("\[PHY\]' Core/Src/eth.c`
预期：**无未注释的** `[ETH]`/`[PHY]` printf 残留（所有命中行均以 `//` 开头或为注释行内文本）。再运行：`grep -c '^.*//.*\[ETH\]\|^.*//.*\[PHY\]' Core/Src/eth.c`，确认注释掉的 printf 都被 `//` 覆盖。

- [ ] **步骤 5：处理提交**

`eth.c` 未跟踪，**不提交**。记录待用户手动提交。

---

## 任务 2：`ethernetif.c` 注释 netif init 打印

**文件：**
- 修改：`Core/Src/ethernetif.c:35`

- [ ] **步骤 1：注释 L35**

将 [ethernetif.c:35](Core/Src/ethernetif.c#L35) 的：

```c
    printf("[ETH] netif init OK, MAC %02X:%02X:%02X:%02X:%02X:%02X\r\n",
```

（及其后续参数行，若为多行）用 `//` 逐行注释。先 Read 该函数确认参数行范围。

- [ ] **步骤 2：复查**

运行：`grep -n 'netif init OK' Core/Src/ethernetif.c`
预期：该行以 `//` 开头。

- [ ] **步骤 3：处理提交**

`ethernetif.c` 未跟踪，**不提交**。记录待用户手动提交。

---

## 任务 3：`flash_param.c` 注释 IP/MASK/GW/PORT

**文件：**
- 修改：`Core/Src/flash_param.c:125-158`

- [ ] **步骤 1：注释四处 printf**

在 `param_load_all()` 中，用 `//` 逐行注释以下四处（多行 printf 的每一行都加 `//`）：

| 行号 | printf 首串 |
|------|------------|
| 125-126 | `IP: %d.%d.%d.%d` |
| 137-138 | `MASK: %d.%d.%d.%d` |
| 149-150 | `GW: %d.%d.%d.%d` |
| 158 | `PORT: %d` |

**其余参数打印全部保留**（含 `total_cycles` L111、D:/J:/…/CS_H）。

- [ ] **步骤 2：复查**

运行：`grep -n 'printf("IP:\|printf("MASK:\|printf("GW:\|printf("PORT:' Core/Src/flash_param.c`
预期：这四行均以 `//` 开头。运行：`grep -n 'total_cycles: %d' Core/Src/flash_param.c`
预期：L111 的 `printf("total_cycles: %d\r\n", total_cycles);` **未**被注释。

- [ ] **步骤 3：处理提交**

`flash_param.c` 在途改动，**不提交**。记录待用户手动提交。

---

## 任务 4：`main.c` 注释 banner 块并提交

**文件：**
- 修改：`Core/Src/main.c:106-111`

- [ ] **步骤 1：注释 banner 块**

将 [main.c:106-111](Core/Src/main.c#L106-L111) 的：

```c
    /* 打印启动信息 */
    {
        char ts[32];
        rtc_get_time_str(ts, sizeof(ts));
        printf("[%s] F1-Stepper V2 on F407 ready\r\n", ts);
        printf("total_cycles: %d\r\n", total_cycles);
    }
```

替换为：

```c
    /* 打印启动信息 (已注释: 静音 banner 与 total_cycles 回显, 参数表由 param_load_all 输出) */
    /* {
        char ts[32];
        rtc_get_time_str(ts, sizeof(ts));
        printf("[%s] F1-Stepper V2 on F407 ready\r\n", ts);
        printf("total_cycles: %d\r\n", total_cycles);
    } */
```

（整块注释，避免 `ts` 未使用告警。）

- [ ] **步骤 2：复查**

运行：`grep -n 'F1-Stepper V2\|total_cycles' Core/Src/main.c`
预期：`F1-Stepper V2` 与 L110 的 `total_cycles` 位于注释块内。

- [ ] **步骤 3：提交**

先 `git diff Core/Src/main.c` 确认只有本任务改动，然后：

```bash
git add Core/Src/main.c
git commit -m "chore(log): 静音以太网启动打印与banner"
```

---

## 任务 5：编译与真机验证（用户执行）

**说明：** 本仓库无自动化测试，需用户在 Keil 编译并上真机确认。

- [ ] **步骤 1：编译**

Keil MDK 打开 `MDK-ARM/test.uvprojx` → Build（F7）。预期**零错误、零新增告警**（`(void)0;` 与 `(void)` 处理了空语句体/未使用变量）。

- [ ] **步骤 2：真机确认输出**

上电串口输出应只有参数表：

```
g_set_speed(D:): 72
...
CS_H: 30
total_cycles: 55000
```

**无** IP/MASK/GW/PORT、**无** `[ETH]`/`[PHY]`、**无** `F1-Stepper V2` banner、无 `[ETH] LINK DOWN` 等运行时打印。

- [ ] **步骤 3：确认命令查询仍正常**

串口 `PARA` 仍能打印全部参数（含 IP/MASK/GW/PORT）；`CYCLE` 仍打印 `total_cycles`。

- [ ] **步骤 4：提交未提交部分（任务 1/2/3）**

`eth.c`、`ethernetif.c`、`flash_param.c` 改动在途未提交，验证通过后由用户确认提交（或告知代理合并提交）。

---

## 自检

- **规格覆盖度**：§2 eth.c→任务 1；ethernetif.c→任务 2；flash_param.c→任务 3；main.c→任务 4；§4 验证→任务 5。全部覆盖。
- **占位符扫描**：无 TODO/待定；每个 printf 位置以行号+首串精确列出；空语句体/未使用变量有具体处理（步骤 2/3）。
- **类型一致性**：变量名（bmsr/id1/id2/r0-r3）、宏（DM9000_*）、函数（rtc_get_time_str）均与现状代码一致；未引入新符号。
