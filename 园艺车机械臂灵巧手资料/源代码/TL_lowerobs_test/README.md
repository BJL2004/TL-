# 下排观测位（A/B 分侧）· 独立验证包

**用途**：在 Nano 上验证"下排观测位按 A/B 分侧 + 示教 d 键现场记录"新功能，**不改动、不影响旧 `~/Desktop/TL` 里的任何程序**。

**原理**：包里是三个包的源码**副本**（含全部依赖），四个启动脚本只从本目录 `src/` 加载程序（PYTHONPATH 前置，直跑 `python3 -m`），不需要 colcon 编译、不会覆盖旧目录。

---

## 快速上手（命令卡 · 2026-10-08 更新）

**① 种植种子（放种子）**

1. 先运行机械臂驱动（二选一，同一个驱动）：
   ```bash
   bash ~/Desktop/TL/scripts/run_driver.sh
   # 或
   ros2 launch tl_driver tl_tcb605_driver.launch.py
   ```
2. 运行抓取种子、放置种子程序：
   ```bash
   # ★ A3 排的盆必须用新版：
   bash ~/Desktop/TL_lowerobs_test/scripts/run_seed_mqtt.sh
   # 其余盆用老版（也可统一用新版）：
   bash ~/Desktop/TL/scripts/run_seed_mqtt.sh
   ```

**② 叉取花篮**

1. 先运行机械臂驱动（同上，二选一）。
2. 运行叉取花篮程序：
   ```bash
   bash ~/Desktop/TL_lowerobs_test/scripts/run_fork_mqtt.sh
   ```

**③ 示教与查点**

```bash
# 种子示教（★ A3 排必须用新版；教其余上排盆推荐老版）：
bash ~/Desktop/TL_lowerobs_test/scripts/teach_seed.sh
bash ~/Desktop/TL/scripts/teach_seed.sh
# 叉子示教（统一用新版）：
bash ~/Desktop/TL_lowerobs_test/scripts/teach_fork.sh
# 查已示教点位（只读，不动机械臂）：
bash ~/Desktop/TL_lowerobs_test/scripts/list_taught.sh
```

**★ 注意**

1. A3 排码贴反已用软件修正：发 `A3-01` 用的就是第 1 格上贴的 ID71；之前教的两个点现在叫 **A3-17 / A3-11**。
2. A3 排（以及下排盆 A1/A4/B1/B4）的教与跑都用新版 `TL_lowerobs_test`——老版没有换算。
3. 从 Windows 拷过脚本后先执行一次 `chmod +x ~/Desktop/TL_lowerobs_test/scripts/*.sh`。
4. 动作没走完不要按 Ctrl+C（等 `Arrived:` / `Returned:` 打印完再操作）。

---

## 一、部署

1. 整个文件夹拷到 Nano，例如放到 `~/Desktop/TL_lowerobs_test`（U 盘 / scp 均可；脚本自动定位自身路径，无需改任何配置）。
2. 使用前先在**另一个终端**启动驱动：`ros2 launch tl_driver tl_tcb605_driver.launch.py`。
3. 物理 E-stop 保持在手边。

## 二、命令对照（旧命令一律不动）

| 用途 | 旧命令（保留原样） | 本包命令（验证用） |
|---|---|---|
| 种子示教 | `bash ~/Desktop/TL/scripts/teach_seed.sh` | `bash ~/Desktop/TL_lowerobs_test/scripts/teach_seed.sh` |
| 叉子示教 | `bash ~/Desktop/TL/scripts/teach_fork.sh` | `bash ~/Desktop/TL_lowerobs_test/scripts/teach_fork.sh` |
| 种子抓取放置 | `bash ~/Desktop/TL/scripts/run_seed_mqtt.sh` | `bash ~/Desktop/TL_lowerobs_test/scripts/run_seed_mqtt.sh` |
| 叉子修剪 MQTT | `bash ~/Desktop/TL/scripts/run_fork_mqtt.sh` | `bash ~/Desktop/TL_lowerobs_test/scripts/run_fork_mqtt.sh` |
| 查已示教点位 | ——（老版没有此工具） | `bash ~/Desktop/TL_lowerobs_test/scripts/list_taught.sh` |

> 验证期间：同一条链路**不要**新旧程序同时跑（MQTT client-id 会冲突）；其余随时可回旧命令。
> **A3 排例外——贴反修正只在新版：A3 的教与跑都用本包**（对照表见《操作说明》第 15 章）。

## 三、本包相对旧版修改的 4 个文件

1. `src/tl_hand_deploy/tl_hand_deploy/aruco_basket_teach.py`（种子示教）
   新增：**d**=记录当前位姿为"当前侧"下排观测位（写 `lower_observation_pose_a/_b`，重复按覆盖+打印）；**g**=回到该观测位；**v** 改为 A/B 来回切换并记住当前侧；HUD 显示 side。
   ⚠️ **d 不再是"选中"别名**（选中只按 b）。
2. `src/tl_fork_teach/tl_fork_teach/fork_basket_teach.py`（叉子示教）
   同上：v 切侧（新增 `--teach-start-joint-deg/-b`，默认 A=-83.668… / B=96.332…）、d 记录、g 回位；删除了旧的固定参数 `--lower-observation-joints`。
3. `src/tl_hand/tl_hand/mock_pick_place.py`（种子抓取放置）
   下排路由：**下排盆（A1/A4/B1/B4）先去对应侧下排观测位识别，再放置**；上排盆行为与旧版完全一致。新增可选参数 `--lower-observation-joints-a/-b`（命令行优先、YAML 兜底）。
4. `src/tl_fork_teach/tl_fork_teach/fork_basket_cycle.py`（叉取，trim 节点自动继承）
   下排观测位拆 A/B、**识别挪到下排观测位做**，然后才去检索位叉取。

**判定规则**：排号 =（盆号-1）÷24；**A1/A4 → A 侧观测位，B1/B4 → B 侧观测位**（A 侧盆号 1..96，B 侧 97..192）；A2/A3/B2/B3（上排）**直接去该盆检索位**识别（无观测位步骤）。

**后续加入（2026-10-07/08，同属上述文件）**：叉子待机位统一为种球默认位；`trim_pick` 失败自动回待机位；下排 W/T 独立槽位；**A3 排码贴反换算 `--reversed-rows A3`**；B 侧下排观测位自动 = A 侧 +180°；查点工具 `list_taught`。详见《操作说明》V4。

## 四、数据文件说明（共用，不影响旧程序）

- `~/tl_mock_pick_place/basket_aruco.yaml`、`~/tl_fork_teach/fork_basket_teach.yaml`、标定文件等**沿用原路径**（这样验证的就是真实记录）。
- 新数据只有**新增键** `lower_observation_pose_a/_b` 与记录时间；旧程序不读这些键，互不干扰。
- 旧 YAML 里暂时没有这两个键属正常——在示教程序按过 d 之后才会出现。

## 五、验证步骤

### A. 记录两侧下排观测位（一次性）
```
bash …/teach_seed.sh
 ① 在 A 区把臂拖到"下排码清晰稳定"的位置 → 按 d
    终端应打印：Saved lower observation pose for side A: joint=[...]
 ② 按 g → 臂应自动回到刚才的位置（验证回位）
 ③ 按 v 切到 B 侧（打印 Current side: B）→ 拖到 B 区下排码清晰处 → 按 d
 ④ 按 g 验证 → 完成
```
叉子示教同样：`teach_fork.sh` → v / 拖 / d / g 记录两侧。

### B. 教一个下排盆（顺带回归）
到观测位附近 → 输编号 → **B** 选盆 → 照常记录 A/P（种子）或 F/I/L/W/T（叉子）→ R 返回。

### C. 运行端验证（关键）
```
bash …/run_seed_mqtt.sh   （驱动已启动）
 ① 发下排盆：{"cmd":"seed","targetId":"A1-05"}（或 B1/B4 任一已教盆）
    日志应出现：Basket number=… is on a lower row (A1/A4/B1/B4);
    moving to the side A lower observation pose to detect the code.
    → 识别 → 放置完成（retCode 0）
 ② 发上排盆（如 B2-06）：日志应是 "Moving to the taught search pose…"，
    行为与旧版逐字一致（回归确认）
 ③ 若报 "lower observation pose for side X is not recorded" → 回步骤 A 记录
```
叉子端同法：`run_fork_mqtt.sh` + `{"cmd":"trim_pick","targetId":"A4-01"}`（需该盆已教 F/I/L/W/T）。

### D. 回滚
- 想停用验证：不再跑本包脚本即可；**或直接删除整个 `TL_lowerobs_test` 文件夹**。
- 数据文件里新增的两个键无副作用；完全恢复可在旧示教程序里继续用（旧程序不读它们）。

## 六、提示

- 记录保护：臂必须停稳（/tcp_pose 新鲜）才允许 d；叉子上带盆时 d/g/v 会被拒绝。
- 记错侧了：`v` 切到正确侧 → 重新拖到位 → 再按 d 覆盖即可。
- B 侧记录时若不确定位置，可参考"A 侧值 J1±180°"先估算（现场仍建议按 d 实测记录）。
- 本包是快照副本；验证通过后如需并回 `~/Desktop/TL` 主目录（含 git 提交），说一声即可。
