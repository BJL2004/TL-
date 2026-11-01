# AGENTS.md — 天链机器人 ROS2 工作空间（Python / ARM）

## 代理协作原则

- **委托优先**：你是协调者，不总是执行者。简单实现任务（单文件修改、配置调整、typo 修复等）优先委托给子代理（`quick` / `unspecified-low` category），便宜模型足以胜任。你专注于需求分析、任务拆解、架构决策和结果审查。
- **自己动手的场景**：仅当任务涉及跨模块架构理解、口头解释/回答问题、或子代理结果需要你整合判断时，才直接操作。

## 模型输出约束

- **思考与输出语言**：模型在分析、推理、回答问题时，**必须使用中文**进行思考和输出。
- **代码注释**：所有代码注释、文档字符串（docstring）、README、配置文件说明等，**以中文为主**。代码标识符（变量名、函数名、类名）遵循项目命名规范（snake_case / PascalCase），无需翻译。
- **用户交互**：与用户的所有对话、提问、解释，均使用中文。

## 工作空间概述

天链（TianLian）机械臂 ROS2 工作空间（Python 实现，含一个 C++ 硬件接口插件，ARM 架构）。`src/` 下包含 10 个功能包（其中 `tl_moveit2_config` 内含 14 个子功能包），使用标准 `colcon build` 构建流程。纯 ROS2（ament_cmake + ament_python），无 Node.js、无前端。

## 构建命令

（必须先构建 `tl_ros2_interface`，再构建其他包）：
```bash
colcon build
source install/setup.bash
```

构建产物在 `build/`、`install/`、`log/` — 均已 gitignore。

## 代码格式化

- **格式化工具**：`black`，配置在 `pyproject.toml` 的 `[tool.black]`，行宽 100
- **git hook**：`.githooks/pre-commit` — `git commit` 时自动运行 `black` 格式化暂存的 Python 文件并重新 `git add`
- **跳过 hook**：`git commit --no-verify`
- **手动格式化**：`black .` 或 `black --check .`（仅检查）
- **不要修改 hook 脚本**：除非格式化工具有变更

## 功能包依赖关系

```
tl_ros2_interface  （基础：自定义 msg/srv，无依赖）
  └─► tl_driver       （Python 节点，通过 SWIG 链接 _tl_host.so）
  └─► tl_hardware     （C++ ros2_control 硬件接口插件）
  └─► tl_example      （使用示例，引用接口类型）
tl_description     （独立：URDF + 网格 + RViz）
  └─► tl_gazebo       （Gazebo 仿真，依赖 tl_description）
  └─► tl_moveit2_config（MoveIt2 配置集合，14 个子包）
  └─► tl_bringup      （启动聚合器：包含 tl_description）
tl_driver          （机械臂驱动，依赖 tl_ros2_interface）
  └─► tl_bringup      （启动聚合器：包含 tl_driver）
  └─► tl_hardware     （运行时通过话题/服务与 tl_driver 交互）
  └─► tl_teleop       （VR 手柄遥操作，通过话题下发运动指令）
  └─► tl_teleop_f710  （F710 手柄遥操作，通过话题下发运动指令）
```

## 功能包说明

### tl_ros2_interface
- **构建类型**：ament_cmake
- **用途**：定义所有自定义 ROS2 接口（12 个 `.msg`，45 个 `.srv`）
- **关键消息**：`ArmStatus`、`MoveCommand`、`CartesianPose`、`JobInsertMove`、`RobotDHParam`
- **关键服务**：`SetSpeed`、`GetSpeed`、`Jogging`、`SetDragMode`、`ModbusRead/Write`、`JobRun`、`TrackSave/Playback`
- **必须最先构建** — 其他包依赖其生成的 Python 模块（colcon 自动处理单包构建，但全量 build 时可能因扫描顺序导致 tl_driver 导入失败）

### tl_driver
- **构建类型**：ament_python
- **用途**：机械臂驱动 — 通过 TCP 与实体机械臂通信
- **入口**：`tl_driver/tl_driver_node.py` → 类 `TLArmNode` + 末尾 `main()`；控制台脚本 `tl_driver_node`
- **原生依赖**（`lib/arm/`）：`_tl_host.so`（ctypes 封装的 C 扩展库，预编译不可修改）+ `tl_interface.py`（ctypes 自动生成接口，928 行），以及外围库 `libtl_host.so`、`libmath_wrapper.so`、`libmodbus_wrapper.so`、`libservoJ_wrapper.so`
- **构建复制**：`setup.py` 将 `lib/arm/` 下全部文件复制到 `tl_driver/lib/`（与 x86 版硬编码文件名不同，ARM 版使用 `os.listdir` 全量复制）
- **运行时加载**：优先尝试 `tl_driver.lib.tl_interface`，失败则通过 `sys.path` 从 `src/tl_driver/lib/` 回退加载（`tl_driver_node.py` 第 15-36 行，回退路径指向 `_nrc_host.so` 所在目录）
- **接口方式**：ARM 版使用 **ctypes**（而非 SWIG）。`tl_interface.py` 文件头标注 `# Auto-generated self_interface.py using ctypes`，通过 `ctypes.CDLL` 加载 `_tl_host.so`
- **NRC 变体**：`tl_driver/lib/` 下额外包含 `_nrc_host.so`、`libnrc_host.so`，用于 NRC 控制器兼容
- **回调组架构**（`__init__` 中创建 3 组）：
  - `service_group_`（`MutuallyExclusive`）— 全部服务，保证服务回调串行
  - `topic_group_`（`MutuallyExclusive`）— 话题订阅，保证话题回调串行
  - `timer_group_`（`Reentrant`）— 状态发布定时器（10 Hz），允许并发
- **配置**：`config/` 下按臂型命名的 YAML（如 `tl_tcb605_config.yaml`）。关键参数：`arm_ip`、`arm_port`（默认 6001）、`arm_port_aux`（默认 7000）、`arm_type`、`arm_joints`
- **启动**：
  - 通用：`ros2 launch tl_driver tl_driver.launch.py arm_type:=<arm_type>`
  - 快捷：`ros2 launch tl_driver tl_tcb605_driver.launch.py`
- **话题**：发布 `/joint_states`、`/tcp_pose`、`/arm_status`；订阅 `/tl_driver/moveJ`、`/tl_driver/moveL`、`/tl_driver/set_servoj_pos`（详见下方关键话题表）
- **默认机械臂 IP**：`192.168.1.13`，端口 `6001` / `7000`（辅助）— 如需修改，改对应配置 YAML
- **安全行为**：`__init__` 中调用 `_startup_connect()`，若连接失败只记日志不崩溃

### tl_description
- **构建类型**：ament_python
- **用途**：URDF 模型 + 网格文件 + robot_state_publisher + RViz 配置
- **无编译代码** — 纯数据包（URDF、STL 网格、.rviz 配置、关节名称 YAML）
- **启动**：`ros2 launch tl_description tl_description.launch.py arm_type:=<arm_type> use_sim:=<true|false>`
- **use_sim=true**：启动 `joint_state_publisher_gui`，通过滑动条手动控制关节
- **use_sim=false**：订阅 `/joint_states`（需要 tl_driver 运行中）

### tl_bringup
- **构建类型**：ament_python
- **用途**：启动聚合器 — 同时启动 tl_driver + tl_description
- **无编译代码** — 仅启动文件
- **启动**：`ros2 launch tl_bringup tl_<arm_type>_bringup.launch.py`
- **每种臂型一个启动文件**（共 14 个，如 `tl_tcb605_bringup.launch.py`）

### tl_gazebo
- **构建类型**：ament_python
- **用途**：在 Gazebo 仿真环境中加载机械臂模型，通过 ros2_control 控制虚拟机械臂
- **启动**：`ros2 launch tl_gazebo gazebo_<arm_type>_demo.launch.py`
- **配合 MoveIt2**：`ros2 launch tl_<arm_type>_config gazebo_moveit_demo_<arm_type>.launch.py`

### tl_moveit2_config
- **构建类型**：ament_cmake（14 个子功能包集合，每个型号一套）
- **用途**：MoveIt2 运动规划配置，包含 SRDF、关节限位、运动学求解器（KDL）、控制器配置
- **启动**：`ros2 launch tl_<arm_type>_config demo.launch.py`
- **配置**：每个子包包含 `config/`（initial_positions、joint_limits、kinematics、srdf 等）和 `launch/`（demo、move_group、rviz 等）

### tl_hardware
- **构建类型**：ament_cmake（C++）
- **用途**：ros2_control `SystemInterface` 硬件接口插件，桥接 MoveIt2 控制器层与 `tl_driver`
- **插件名**：`tl_hardware/TLHardwareInterface`
- **通信**：订阅 `/joint_states`，发布 `/tl_driver/set_servoj_pos`，调用 `open_servoj`/`close_servoj` 服务
- **工作空间中唯一的 C++ 包** — 因 ros2_control 的 `pluginlib` 要求硬件接口使用 C++ 实现
- **启动**：通过 MoveIt2 配置包的 `real_hardware_demo.launch.py`，传入 `use_real_hardware:=true`

### tl_teleop
- **构建类型**：ament_python
- **用途**：VR 手柄遥操作 — 通过 xrobotoolkit_sdk（Python 绑定库）连接 VR 设备，将手柄姿态映射为机械臂运动指令
- **依赖**：`xrobotoolkit_sdk`（`setup.py` 中集成自动构建与安装，详见同包 README）

### tl_teleop_f710
- **构建类型**：ament_python
- **用途**：罗技 F710 游戏手柄遥操作，通过话题下发运动指令控制机械臂

### tl_example
- **构建类型**：ament_python
- **用途**：使用示例 — 展示如何调用 `tl_ros2_interface` 中定义的接口类型与 `tl_driver` 的服务/话题交互

## 支持的臂型

启动参数中全部小写：`tcb605`、`tcb605f`、`tcb605l`、`tcb605lv`、`tcb605v`、`tcb610`、`tcb610v`、`tcb705`、`tcb705f`、`tcb705l`、`tcb705lv`、`tcb705v`、`tcb710`、`tcb710v`

配置 YAML 中 `arm_type` 字段用大写：如 `TCB605`。关节名：6 轴为 `[joint1..joint6]`，7 轴为 `[joint1..joint7]`。

## 关键话题

| 话题 | 发布者 | 订阅者 | 类型 |
|------|--------|--------|------|
| `/joint_states` | tl_driver（真实）/ joint_state_publisher_gui（仿真） | tl_description | `sensor_msgs/JointState` |
| `/tcp_pose` | tl_driver | — | `tl_ros2_interface/CartesianPose` |
| `/arm_status` | tl_driver | — | `tl_ros2_interface/ArmStatus`（失败时回退 `std_msgs/String`） |
| `/tl_driver/moveJ` | — | tl_driver | `tl_ros2_interface/MoveCommand` |
| `/tl_driver/moveL` | — | tl_driver | `tl_ros2_interface/MoveCommand` |
| `/tl_driver/set_servoj_pos` | tl_hardware | tl_driver | `std_msgs/Float64MultiArray` |
| `/tf`、`/tf_static` | tl_description（robot_state_publisher） | — | `tf2_msgs/TFMessage` |

## 测试

测试仅存在于 `tl_driver` 的 `test/` 中：
```bash
# 需要先 source install/setup.bash 并确保 tl_ros2_interface 已构建
python3 src/tl_driver/test/test_topics_and_services.py
python3 src/tl_driver/test/run_unit_tests.py
```

另有 `test_publisher.py`（servoj 测试独立节点）和 Shell 脚本（`test_moveJ.sh`、`test_moveL.sh` 等）。

**注意**：无 colcon test 集成，`ament_lint_auto` / `ament_lint_common` 在 `test_depend` 中声明但未配置为生效关卡。

## 注意事项

- **`_tl_host.so`** 是预编译专有库（ARM 架构），通过 ctypes 加载，禁止尝试重新编译或修改。构建时从 `lib/arm/` 复制到 Python 包内。
- **外围库**：ARM 版额外包含 `libmath_wrapper.so`、`libmodbus_wrapper.so`、`libservoJ_wrapper.so`、`libtl_host.so`，构建时随 `_tl_host.so` 一并复制。缺少任一外围库会导致 ctypes 加载失败。
- **NRC 兼容**：`tl_driver/lib/` 下包含 `_nrc_host.so` + `libnrc_host.so`，用于兼容 NRC 控制器。回退加载路径也指向 `_nrc_host` 相关目录。
- **构建顺序**：必须 `colcon build --packages-select tl_ros2_interface` 先执行，否则 tl_driver 导入会静默失败。
- **启动自动连接**：`TLArmNode.__init__()` 调用 `_startup_connect()`，自动 ping 并连接机械臂 IP。无真实控制器时不会崩溃 — 节点继续运行并暴露所有服务。
- **宽泛的异常处理**：Python 版大量使用 `try/except Exception` 来优雅降级（如 msg/srv 类型不可用时跳过对应服务注册）。编辑时不要删除这些保护。
- **无类型检查**：无 pyright/mypy 配置，`try/except` 模式压住了大量类型错误。
- **启动文件模式**：所有 `*_driver.launch.py`、`*_description.launch.py` 均使用 `OpaqueFunction` + 内联字典做臂型路由。添加新型号需在多个文件的字典中添加条目。
- **服务命名空间 `/tl_driver/`**：所有驱动服务和话题都置于 `/tl_driver/` 下。
- **测试中的 7 轴硬编码**：`test_publisher.py` 硬编码了 7 自由度假定。6 轴型号使用相同代码路径但 `arm_joints` 配置不同，测试时注意适配。

## 命名规范

### Python 命名规范

| 元素 | 规范 | 示例 |
|------|------|------|
| **文件名** | snake_case | `tl_driver_node.py` |
| **类名** | PascalCase | `TLArmNode` |
| **实例变量** | snake_case | `arm_ip_`、`arm_joints_`、`is_connected_` |
| **成员函数** | snake_case | `power_on`、`disconnect`、`publish_arm_state` |
| **私有方法** | 下划线前缀 + snake_case | `_startup_connect`、`_valid_fd`、`_is_success` |
| **服务回调** | `handle_` + `{service}` + `_service` | `handle_connect_service`、`handle_set_speed_service` |
| **话题回调** | `handle_` + `{topic}` + `_topic` | `handle_movej_topic`、`handle_movel_topic` |
| **ROS 参数键** | snake_case | `'arm_ip'`、`'arm_port'`、`'arm_joints'` |
| **ROS 话题名** | snake_case（小写） | `/joint_states`、`/tcp_pose` |
| **ROS 服务名** | snake_case（小写） | `/tl_driver/connect_arm`、`/tl_driver/set_speed` |
| **入口点函数** | snake_case | `main` |
| **console_scripts** | snake_case | `tl_driver_node = tl_driver.tl_driver_node:main` |
| **发布者变量** | `{topic}_pub` 后缀 | `joint_state_pub`、`tcp_pose_pub`、`running_status_pub` |
| **订阅者变量** | `{topic}_sub` 后缀 | `movej_sub`、`movel_sub`、`set_servoj_pos_sub` |
| **回调组变量** | `{group}_group_` 后缀 | `service_group_`、`topic_group_`、`timer_group_` |

### ROS 话题/服务命名规范

- 所有话题和服务名使用 **snake_case（小写+下划线）**
- 驱动服务前缀：`/tl_driver/`
- 示例话题：`/joint_states`、`/tcp_pose`、`/arm_status`
- 示例服务：`/tl_driver/connect_arm`、`/tl_driver/set_speed`

### 文件组织规范

```
tl_driver/
├── tl_driver/tl_driver_node.py    # 主节点实现
├── tl_driver/lib/                 # 构建时从 lib/arm/ 复制（全量）
│   ├── _tl_host.so                # ctypes C 扩展（核心）
│   ├── _nrc_host.so               # NRC 控制器兼容
│   ├── tl_interface.py            # ctypes 自动生成接口
│   ├── libtl_host.so              # TL 主机库
│   ├── libmath_wrapper.so         # 数学库封装
│   ├── libmodbus_wrapper.so       # Modbus 库封装
│   ├── libservoJ_wrapper.so       # ServoJ 库封装
│   └── libnrc_host.so             # NRC 主机库
├── launch/tl_driver.launch.py     # 通用启动文件
├── launch/tl_tcbXXX_driver.launch.py  # 各臂型快捷启动
├── config/*.yaml                  # 各臂型配置
└── test/                          # 手动测试脚本
```

## ARM 与 x86 差异速查

| 项目 | x86 版 | ARM 版 |
|------|--------|--------|
| **原生接口方式** | SWIG（`import tl_interface`） | ctypes（`ctypes.CDLL` 加载） |
| **库目录** | `lib/x86/` | `lib/arm/` |
| **外围库** | 无（仅 `_tl_host.so`） | 含 `libmath_wrapper`、`libmodbus_wrapper`、`libservoJ_wrapper`、`libtl_host` |
| **NRC 兼容** | 无 | 含 `_nrc_host.so`、`libnrc_host.so` |
| **setup.py 复制方式** | 硬编码文件列表 | `os.listdir` 全量复制 |
| **package_data** | `_tl_host.*`、`tl_interface.py` | 额外包含 `libtl_host.*`、`libmath_wrapper.*`、`libmodbus_wrapper.*`、`libservoJ_wrapper.*` |
| **tl_interface.py 行数** | SWIG 生成（较短） | ctypes 生成（928 行） |

## Sisyphus 后台任务超时规避

后台 explore/librarian 任务有 **30 分钟无活动超时限制**。超大代码库搜索时容易触发。规避方法：

- **每个 explore agent 只查 1-2 个具体模式**，不要塞 5+ 个搜索需求到一个 prompt
- **已知文件位置**（如已确定路径的文件）直接用 `grep`/`read` 直接工具，不 delegation
- **大范围搜索拆成多个并行小任务**，每个小任务限定搜索范围
- 如果需要跨包搜索，拆成多个并行 agent

<!-- gitnexus:start -->
# GitNexus — 代码智能

本项目已由 GitNexus 索引，仓库名 **tl_robot_ros2_py**（6297 个符号、7618 条关系、11 条执行流）。使用 GitNexus MCP 工具来理解代码、评估影响并安全导航。

> 若任何 GitNexus 工具提示索引过期，请先在终端执行 `npx gitnexus analyze`。

## 必须遵守

- **编辑任何符号前，务必先运行影响分析。** 在修改函数、类或方法之前，运行 `gitnexus_impact({target: "symbolName", direction: "upstream"})`，并将影响范围（直接调用者、受影响流程、风险级别）报告给用户。
- **提交前务必运行 `gitnexus_detect_changes()`**，验证你的更改仅影响预期的符号和执行流。
- **若影响分析返回 HIGH 或 CRITICAL 风险级别，务必先警告用户**，再继续编辑。
- 探索不熟悉的代码时，使用 `gitnexus_query({query: "concept"})` 查找执行流，而非 grep。它返回按相关性排序的、按流程分组的结果。
- 需要某个符号的完整上下文（调用者、被调用者、参与的执行流）时，使用 `gitnexus_context({name: "symbolName"})`。

## 严禁事项

- 严禁在未运行 `gitnexus_impact` 的情况下编辑任何函数、类或方法。
- 严禁忽略影响分析返回的 HIGH 或 CRITICAL 风险警告。
- 严禁用查找替换的方式重命名符号 — 应使用 `gitnexus_rename`，它理解调用关系图。
- 严禁在未运行 `gitnexus_detect_changes()` 检查影响范围的情况下提交更改。

## 资源

| 资源 | 用途 |
|----------|---------|
| `gitnexus://repo/tl_robot_ros2_py/context` | 代码库概览，检查索引新鲜度 |
| `gitnexus://repo/tl_robot_ros2_py/clusters` | 所有功能区域 |
| `gitnexus://repo/tl_robot_ros2_py/processes` | 所有执行流 |
| `gitnexus://repo/tl_robot_ros2_py/process/{name}` | 逐步执行追踪 |

## CLI

| 任务 | 参阅技能文件 |
|------|---------------------|
| 理解架构 / "X 是怎么工作的？" | `.opencode/skills/gitnexus/gitnexus-exploring/SKILL.md` |
| 影响范围 / "改 X 会破坏什么？" | `.opencode/skills/gitnexus/gitnexus-impact-analysis/SKILL.md` |
| 追踪 Bug / "为什么 X 失败了？" | `.opencode/skills/gitnexus/gitnexus-debugging/SKILL.md` |
| 重命名 / 提取 / 拆分 / 重构 | `.opencode/skills/gitnexus/gitnexus-refactoring/SKILL.md` |
| 工具、资源、Schema 参考 | `.opencode/skills/gitnexus/gitnexus-guide/SKILL.md` |
| 索引、状态、清理、Wiki CLI 命令 | `.opencode/skills/gitnexus/gitnexus-cli/SKILL.md` |

<!-- gitnexus:end -->
