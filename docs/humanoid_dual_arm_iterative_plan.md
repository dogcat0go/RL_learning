# 人形双臂（上半身）× MoveIt2 / ros2_control 迭代学习计划

> 对《52 周学习计划》（`dual_arm_weekly_learning_plan.md`）的**人形双臂增补**。
> 原计划的 P3（W31–W46）是按「两台独立臂装在公共基座上」写的；人形机器人多出共享的腰 / 躯干、
> 随动的头部相机、7 轴冗余臂、十几个硬件组件挤在一个 `controller_manager` 里，以及移动 / 浮动基座。
> 本文**不改 P0–P2 的周安排**，只给它们加人形注释；把 **P3 的 16 周重排为 3 次迭代**；在 P4 新增一个人形专题。

---

## 0. 人形上半身与独立双臂的差异（决定了下面每一周学什么）

| 维度 | 独立双臂 | 人形上半身 | 对 MoveIt2 / ros2_control 的影响 |
|---|---|---|---|
| 运动链 | 两条独立链 | 腰（1–3 DOF）→ 胸 → 左臂 / 右臂 / 头 的**树** | 躯干归哪个 planning group、subgroup 怎么拆、单臂规划时腰锁不锁 |
| 自由度 | 6 × 2 | 7 × 2 + 腰 + 头 + 两只手，≥ 20 | KDL IK 在 7 轴上不稳；规划维度高；碰撞检查变慢 |
| 硬件 | 一条总线 | 臂 EtherCAT + 手 CAN / 串口 + 头舵机，速率不同 | 多个 `<ros2_control>` hardware component、异步 read / write、多个 controller 同步执行 |
| 相机 | 外参固定 | 头部相机随头动 | 手眼标定到胸部坐标系、TF 时间戳、octomap 拖影 |
| 基座 | 固定 | 轮式底盘或双足 | `virtual_joint` 类型、移动操作、双足需要 WBC（MoveIt 之外） |
| 安全 | 两臂互撞 | 臂–躯干–头–手 多对 | ACM 条目爆炸、工作空间软限制（别打到自己的脸） |

---

## 1. 迭代总览

迭代原则：**每次迭代都是一个 mock → 仿真 → 真机的完整闭环**，结束时有可演示的东西和一次复盘；
后一次迭代回头加厚前一次的层（螺旋），而不是另起一条线。

| 迭代 | 周 | 闭环目标 | 可演示交付 | 复用的原计划周 |
|---|---|---|---|---|
| I0 单臂切片 | W5–W30（原 P1 / P2 不变） | 人形的**一条臂**走通 MoveIt → JTC → 真机，力控 / 辨识 / 手都在单臂上做完 | 原 P1 / P2 交付物 | 全部，加 §2 注释 |
| I1 上半身建模与多组规划 | W31–W35 | 腰 + 双臂 + 头一套 `moveit_config`，任意 group 规划并由多个 controller 同步执行 | 上半身 `moveit_config` v1 + controller 分配文档 | 原 W31 |
| I2 双臂协同与头部感知 | W36–W40 | MTC 双臂任务 + 内力控制 + 头部相机进 planning scene | 真机双臂搬运且能避开相机看到的障碍 | 原 W32–W35 |
| I3 遥操作、策略与安全 | W41–W45 | VR 全上半身遥操作 → 数据 → 策略 → 执行器 → 安全层 | 端到端 pipeline + 文档 | 原 W37–W41 |
| 缓冲 | W46 | 复盘；原 W42–W45 压缩后的内容 | — | 原 W42–W46 |
| I4 专长 | W47–W52 | 原 A–D 四个方向 + 新增 E「全身 / 移动基座」 | 专题报告 | 原 P4 |

---

## 2. I0：P0–P2 的人形注释（不改周安排）

| 周 | 原内容 | 人形上多做的一件事 |
|---|---|---|
| W2 | URDF 结构 | 画出公司人形的**完整运动树**（base → 腰 → 胸 → 两臂 / 头 / 手），标出每段的 DOF 和总线 |
| W5 | Setup Assistant | 只为**一条臂**建 group，腰、头、另一臂的关节在 SRDF 里声明为 passive 或不进 group；记下「腰暂时锁死」这个决定 |
| W6 | ros2_control 架构 | 看公司 `controller_manager` 下**有几个 hardware component**、各自 `update_rate` 是否一致，先不动 |
| W7 | 驱动与总线 | 多标一张「哪条总线挂哪些关节」的表，手和头的总线一起标 |
| W12 | 规划管线 | 单臂 7 轴：观察 OMPL 在 7 维的规划时间，为 I1 做基线 |
| W14 | Planning Scene | 相机先用**头固定**的外参，I2 再处理头动 |
| W22 | IK 插件 | 7 轴臂上 KDL 的失败率先记一笔，I1 W33 会正式选型 |
| W24 | MoveIt Servo | 单臂一个 Servo 实例，I3 会扩到两臂 + 头 |
| W25 | 灵巧手 | 手的 hardware component 与臂分开写，为 I1 多组件打底 |

---

## 3. I1：上半身建模与多组规划（W31–W35）

**W31 运动树与 planning group 设计**
- 学：SRDF group 的三种定义方式（`chain` / `joints` / `subgroups`）；人形常用分组：`left_arm`、`right_arm`、`torso`、`head`、`both_arms`（= 两臂 subgroup）、`upper_body`（= 腰 + 两臂）；手挂为各臂的 `end_effector`。
- 决策：**腰的归属**三选一——(a) 腰只进 `upper_body`，单臂规划时锁死（最常见，先用这个）；(b) `left_arm_torso` / `right_arm_torso` 各含腰，单臂可借腰但两臂不能同时规划；(c) 腰独立成组，由任务层单独下发。把理由写进文档。
- 做：Setup Assistant 生成后手工整理 SRDF；`virtual_joint` 按基座类型选 `fixed` / `planar` / `floating`。
- Gate：六个 group 都能在 RViz 中规划；文档说明腰归属的选择与代价。

**W32 碰撞矩阵与碰撞体简化**
- 学：≥ 20 DOF 下 ACM 的生成（Setup Assistant 采样次数提到 10 万次以上）与**人工审核**：`Adjacent` / `Never` / `Default` 三类哪些可信；臂–头、臂–胸、手–手这些真会撞的对必须保留；`padding` / `scale`；FCL vs Bullet 后端。
- 做：用凸包 / 胶囊替换碰撞 mesh，对比简化前后单次 `checkCollision` 耗时与 OMPL 规划时间。
- Gate：单次 planning scene 碰撞检查耗时有数据并达到自定阈值；ACM 审核记录（每条被禁用的「可疑对」有理由）。

**W33 7 轴冗余 IK 与多末端 IK**
- 学：KDL 在 7 轴上的失败模式；TRAC-IK、pick_ik、bio_ik 的参数与适用场景；用 cost 把肘部推成「像人」的姿态；零空间速度 q̇ = J⁺ẋ + (I − J⁺J)q̇₀（接 W22 自写 DLS）；`upper_body` 组**双末端 + 腰**的多目标 IK（bio_ik 多 goal）。
- 做：三个插件在公司臂上 benchmark：成功率、耗时、相邻目标解的连续性（跳解会让 Servo 和策略部署炸）。
- Gate：`kinematics.yaml` 选型报告；双末端 IK 在 ≥ 95% 的采样目标上成功。

**W34 多硬件组件与 controller 分配**
- 学：一个 `controller_manager` 下多个 hardware component（臂 System、手 System / Actuator、头）；慢总线用异步 read / write 取最新值，别拖慢 1 kHz 主循环；**每组一个 JTC vs 整身一个 JTC**；`moveit_controllers.yaml` 的 controller → joints 映射；`TrajectoryExecutionManager` 选 controller 的规则（选中的 controller 集合必须**恰好覆盖**轨迹关节）；`switch_controller` 严格模式；controller chaining（为后面力控留口）。
- 做：`mock_components` 搭 20+ DOF 全身 ros2_control，配 `left_arm_controller` / `right_arm_controller` / `torso_controller` / `head_controller` / 两个手 controller；验证 `upper_body` 一条轨迹被 MoveIt 拆给多个 controller 同步执行；画 move_group → 多 controller → 多 hardware 的数据流图。
- Gate：故意让某个 controller 的 joints 与 group 不匹配，能从日志（`Unable to identify any set of controllers…`）定位；`both_arms` 轨迹两臂到达时间差在一个控制周期量级。

**W35 仿真 → 真机上线 upper_body（I1 交付）**
- 做：gz_ros2_control（或 Isaac Sim）跑整个上半身；真机按「腰锁死 → 单臂 → 双臂 → 放开腰」逐步放开；加**工作空间软限制**（头、胸、另一臂前方的禁区）。
- Gate：真机 `upper_body` 组规划 + 执行连续 10 次成功；软限制触发可复现；写 I1 复盘。

---

## 4. I2：双臂协同与头部感知（W36–W40）

**W36 双臂运动学与闭链**（原 W32）
- 学：cooperative task space（绝对 / 相对位姿）、两臂同持物体的闭链约束、约束 Jacobian；人形多一项——**腰参与**时相对位姿不变但绝对位姿改变，怎么分配给腰 vs 臂。
- 做：Pinocchio 验证含腰的约束 Jacobian。
- Gate：推导文档；仿真中满足闭链约束的双臂 + 腰运动。

**W37 MTC 双臂任务**（原 W33）
- 学：MTC stage / 容器 / 求解器；双臂并行 stage（`Alternatives` / `Merger`）；`upper_body` 组在 MTC 中作为一个 group 还是两个。
- 做：单臂 pick-and-place 跑通 → 一臂抓一臂扶 → 加入腰（例如抓低处物体先弯腰）。
- Gate：MTC 双臂 + 腰任务在仿真中成功。

**W38 内力控制**（原 W34）
- 学：运动力 / 内力分解、物体级阻抗、主从 vs 对称；用 W34 留的 controller chaining 把阻抗控制器接到两臂 JTC 之下。
- 做：仿真中双臂搬刚体，内力可调。
- Gate：内力曲线平稳，物体不掉不被捏坏。

**W39 头部相机：标定、TF 时序与 look-at**
- 学：手眼标定（`moveit_calibration` / `easy_handeye2`）把头部相机标到胸部坐标系；头在动时点云的 TF 时间戳与 `tf2` 插值；`occupancy_map_monitor` 的 `max_update_rate` 与头部运动的配合（头动时暂停更新或滤掉拖影）；`head` 组的 look-at IK（相机 z 轴对准目标）。
- 做：标定一次，验证重投影误差；头部扫视时 octomap 无拖影；写一个「看向手 / 看向物体」的 head 行为。
- Gate：标定误差有数据；头动 + octomap 更新时规划不误报碰撞。

**W40 真机双臂搬运 + 感知避障（I2 交付）**（原 W35）
- 做：带内力控制的真机搬运；桌面放置临时障碍，由头部相机看到并绕开；双臂自碰撞实时距离检查。
- Gate：搬运 10 次成功记录；障碍避让成功记录；I2 复盘。

---

## 5. I3：遥操作、策略与安全（W41–W45）

**W41 上半身遥操作**（原 W37 的遥操作部分）
- 学：VR（Quest / Vision Pro）→ 两臂末端位姿 + 头部朝向 + 手部关键点；两臂各一个 MoveIt Servo 实例（pose tracking 模式）vs 自写 `upper_body` 组整体 IK 下发；头随 HMD；手用 `dex-retargeting`；参考 Open-TeleVision、Unitree `xr_teleoperate` 的架构。
- 做：搭建遥操作链路，测量 VR → 关节指令端到端延迟。
- Gate：遥操作完成一次双手抓取；延迟与抖动有数据。

**W42 数据采集与策略训练**（原 W37 采集 + W38 对比合并）
- 做：臂 + 腰 + 头 + 手 + 多相机时间戳对齐，输出 LeRobotDataset，采 100+ episodes；同一数据集训 ACT 与 Diffusion Policy，离线评估 + 仿真。
- Gate：数据质量检查通过；对比报告。

**W43 策略 → 控制器执行器**（原 W39）
- 学：20+ DOF action chunk 的插值（线性 / 样条 / Ruckig）；10–50 Hz → 1 kHz；执行器形态二选一——自写一个「policy executor」controller 吃全部关节，或向多个 JTC 同时发 goal（注意时间基准）；延迟补偿。
- 做：实现执行器，输出 jerk 与跟踪误差。
- Gate：jerk 达标，多 controller 同步无相位差。

**W44 安全层**（原 W40）
- 学：三道闸——(1) 几何：planning scene 实时自碰撞距离 + 工作空间软限制；(2) 动力学：限幅 + 广义动量观测器；(3) 策略输出合法性（跳变、越界）。人形多一对要防的：手撞头 / 胸。
- 做：注入跳变、越界、穿障、手朝头部运动的动作。
- Gate：100% 拦截，拦截延迟可测。

**W45 真机部署 + pipeline 文档（I3 交付）**（原 W41）
- 做：采集 → 训练 → 部署端到端，文档。
- Gate：任务成功率报告；I3 复盘。

**W46 缓冲 + 压缩内容**
- 补 gate；原 W42 VLA 概览保留为 1 页纸；原 W43 / W44（Isaac Lab、Residual RL）并入 P4 方向 D；原 W45 Hybrid Planning 并入 P4 方向 E。

---

## 6. I4：P4 新增专题 E「全身 / 移动基座」（W47–W50 五选一）

- 轮式底盘：`virtual_joint` 改 `planar`，底盘作为 3 DOF 加入 `whole_body` 组做移动操作 IK；MoveIt 对移动基座支持弱，需自写底盘 controller 与规划器配合；Hybrid Planning 做动态障碍在线重规划。
- 双足：浮动基座 + 平衡约束超出 MoveIt 范围，边界是「MoveIt 规划上半身轨迹 → WBC（如 Crocoddyl / TSID）做全身任务分配」；本专题只做到接口定义与一个仿真 demo。
- 交付：全身 IK 与移动操作仿真 demo；MoveIt 与 WBC 的接口设计文档。

---

## 7. 人形上的 MoveIt2 / ros2_control 常见坑（速查）

| 现象 | 根因 | 对应周 |
|---|---|---|
| `Unable to identify any set of controllers that can actuate the specified joints` | controller 的 joints 与轨迹关节集合不是恰好覆盖 | W34 |
| `both_arms` 执行时一臂先到 | 两个 JTC 各自以收到 goal 的时刻为 t=0 | W34、W43 |
| 7 轴臂 IK 频繁 timeout / 跳解 | KDL 不适合冗余臂 | W33 |
| 真机撞了但 RViz 不报 | ACM 采样把真会撞的对标成 `Never` / `Default` | W32 |
| 头一动 octomap 出现拖影、规划误报碰撞 | 点云 TF 时间戳 / 更新率与头部运动不配合 | W39 |
| 规划一次要几秒 | 碰撞 mesh 太细、维度高、padding 大 | W32 |
| 1 kHz 主循环 jitter 变大 | 某个慢总线 hardware component 同步 read / write | W34（接 W8） |
| 肘部姿态「反人」、遥操作时手肘乱飞 | IK 无零空间偏好 | W33、W41 |
| 单臂规划时腰也动了 | 腰进了单臂 group，或未在 SRDF 声明 passive | W31 |

---

## 8. 增补资源

- MoveIt2 官方：SRDF / Setup Assistant、Kinematics plugins（含 pick_ik、bio_ik、TRAC-IK）、Controller Manager & `moveit_controllers.yaml`、MTC、Servo pose tracking、Hybrid Planning 各章节。
- ros2_control：Hardware Components 多组件写法、Controller Chaining、异步 hardware component 文档。
- 仿真：gz_ros2_control、Isaac Sim ROS2 bridge。
- 标定：moveit_calibration、easy_handeye2。
- 遥操作：Open-TeleVision、Unitree xr_teleoperate、dex-retargeting。
- 全身：Crocoddyl、TSID；Springer Handbook of Robotics 双臂协调与人形章节。
- 其余沿用原计划 §4。
