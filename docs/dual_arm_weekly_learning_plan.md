# 从平衡车到双臂灵巧手：52 周学习计划（MoveIt2 / ROS2 主线）

> 对《双臂机器人控制器方向学习路线（底层 + 算法 迭代版）》的按周细化。
> 保留原文 6 层（L0–L5）+ 4 轮螺旋迭代的骨架，新增两条原文没展开的线：
> **MoveIt2 工程栈**（公司主力框架）和 **灵巧手**（目标形态）。

---

## 0. 起点、终点与能力迁移

**起点（已具备）**：电机三环（电流/速度/位置）、VMC、LQR、PID、卡尔曼滤波、扰动观测器、
摩擦辨识（Coulomb + viscous、可辨识性检验）、死区补偿、ROS2 / Gazebo、log 驱动调试。

**终点（52 周后）**：能独立排查公司双臂机器人任意一层的问题；在 L2（关节级）或 L3（任务级）
形成专长；能把学习策略（ACT / Diffusion Policy）安全地接到 1 kHz 伺服上；灵巧手能完成基础抓取并进入数据采集闭环。

平衡车经验在机械臂上的落点（看到对应周时直接调用旧经验，不要从零学）：

| 平衡车已有 | 机械臂 / 灵巧手对应 | 出现周 |
|---|---|---|
| 电机三环 | 关节伺服：驱动器内 FOC 电流环 + CiA 402 CSP / CSV / CST 模式；ros2_control 的 command interface 选型 | W7、W9 |
| VMC（τ = Jᵀ F） | 笛卡尔阻抗控制、操作空间控制（OSC） | W3、W23、W24 |
| LQR | 关节 PD + 重力补偿的增益选取视角；后期 MPC / 轨迹优化 | W13、W47+ |
| 摩擦辨识 | 关节摩擦 + 动力学基参数辨识（回归量共线、速度覆盖、正负对称等坑原样出现） | W17–W20 |
| 扰动观测器 | 广义动量观测器碰撞检测（同源推导） | W21 |
| 卡尔曼滤波 | 关节速度估计、末端外力估计、多传感器时间对齐 | W19、W21、W27 |
| 死区补偿 | 谐波减速器回差 / 柔性关节补偿 | W20 |
| 带 gate 的分阶段计划 + log 分析 | ros2 bag + PlotJuggler + 自写分析脚本；每周 pass/fail gate | 全程 |

---

## 1. 总览

| 阶段 | 周 | 对应原文 | 目标 | 交付物 | MoveIt2 侧重 |
|---|---|---|---|---|---|
| P0 基础补齐 | W1–W4 | 入职前 L0 | 刚体变换 / 运动学 / Jacobian / 动力学，Pinocchio 上手 | 自己的 L0 笔记 + Pinocchio 验证脚本 | 跑通官方 tutorial，建立整体印象 |
| P1 单臂跑通 | W5–W16 | 第 1 轮 | 读懂公司驱动与 MoveIt 栈；位置控制 + 重力补偿；S 曲线规划；ACT 示例 | 公司机械臂建模文档 | Setup Assistant、ros2_control 对接、规划管线、Planning Scene |
| P2 力控 + 辨识 + 灵巧手入门 | W17–W30 | 第 2 轮 | 辨识工具链、拖动示教、碰撞检测、笛卡尔阻抗、灵巧手接入、单臂遥操作采数据训 ACT | 可复用的辨识工具链 | IK 插件、MoveIt Servo、手作为 end-effector group |
| P3 双臂 + 策略落地 | W31–W46 | 第 3 轮 | 双臂协同（内力控制）、双臂双手数据采集、策略部署（插值 / 安全层）、residual RL | 数据采集 → 策略部署完整 pipeline | 多 planning group、MTC、Hybrid Planning、碰撞安全层 |
| P4 专长深挖 | W47–W52 | 第 4 轮 | 选 1 个方向做 4 周专题 + 年度复盘 | 专题技术报告 | 按方向 |

**每周节奏**（工作日以公司真机和代码为主，自学时间约 10 h/周）：

- 4 h 理论（书 / 论文 / 源码），4 h 实操（仿真或真机），2 h 中文笔记 + 周复盘。
- 每周有一个 **gate**，不过不往前推，用阶段末的缓冲周（W16 / W30 / W46）补。
- 公司任务与计划撞车时，公司任务优先，把它归到对应层并记入复盘，别另起炉灶。

---

## 2. 周计划

### P0 基础补齐（W1–W4，入职前）

**W1 刚体变换**
- 学：旋转矩阵、四元数、齐次变换、SE(3)、指数坐标（Modern Robotics 第 3 章）。
- 做：装 Pinocchio（conda / pip），用 Eigen 或 Pinocchio 验证变换复合、四元数 ↔ 旋转矩阵、对数 / 指数映射。
- Gate：手推一个两级变换，与代码结果一致到 1e-9。

**W2 正运动学与 URDF**
- 学：PoE（指数积）与 DH 两套参数化（MR 第 4 章）；URDF 的 link / joint / 惯量 / 碰撞体结构。
- 做：取一个公开 7 轴臂（Panda）或 6 轴臂（UR5e）的 URDF，用 Pinocchio 做 FK，与手算 PoE 对比。
- Gate：任意 3 组关节角 FK 误差 < 1e-6。

**W3 Jacobian、速度与力映射**
- 学：空间 / 物体 Jacobian、τ = Jᵀ F（直接对照 VMC）、奇异性、可操作度（MR 第 5 章）。
- 做：解析 Jacobian 与数值差分 Jacobian 对比；画可操作度椭球随位形的变化。
- Gate：能口头解释 VMC 与机械臂力控在数学上是同一件事，并指出不同点（多自由度、奇异点）。

**W4 动力学 + MoveIt2 初体验**
- 学：拉格朗日法、牛顿–欧拉递推（RNEA）、M(q)q̈ + C(q,q̇)q̇ + g(q) = τ 中各项来源（MR 第 8 章）；Pinocchio 的 `rnea` / `crba` / `computeGeneralizedGravity`。
- 做：安装 ROS2（Humble 或 Jazzy，与公司一致）+ MoveIt2；跑官方 tutorial 的 Getting Started、RViz MotionPlanning 插件、MoveGroupInterface C++ 示例。
- Gate：用 Pinocchio 验证重力项在零位 / 水平位的量级符合直觉；RViz 中能拖动目标并执行规划。

### P1 单臂跑通（W5–W16，第 1 轮）

**W5 入职：读懂公司 URDF / SRDF / moveit_config**
- 学：MoveIt Setup Assistant 流程：自碰撞矩阵（ACM）、virtual joint、planning group、end effector、passive joint、ros2_control 标签生成。
- 做：从零用 Setup Assistant 为公司机械臂重新生成一份 moveit_config，与公司现有配置 diff。
- Gate：能解释公司 SRDF 里每一条 `disable_collisions` 的理由，并指出至少一处可改进点（或确认无误）。

**W6 ros2_control 架构**
- 学：controller_manager、`hardware_interface::SystemInterface` 的 `read / write / on_activate` 生命周期、state / command interface、`joint_trajectory_controller`（JTC）及其 `FollowJointTrajectory` action。
- 做：用 `mock_components` 把 MoveIt → JTC → mock hardware 跑通；画出数据流图（move_group → controller → hardware）。
- Gate：能定位 MoveIt 执行失败时是 planning、controller 还是 hardware 的问题（故意改坏一处验证）。

**W7 公司驱动与总线**
- 学：EtherCAT + CiA 402 状态机（Switch on disabled → Operation enabled）、PDO 映射、CSP / CSV / CST 模式各自对应「三环的哪几环在驱动器里、哪几环在控制器里」；CAN / CAN FD 作为对照。
- 做：读公司 hardware interface 源码，标注一个 1 kHz 周期内 read → update → write 的时序。
- Gate：画出时序图并说明每一步的延迟来源。

**W8 实时性与 jitter**
- 学：PREEMPT_RT、`cyclictest`、线程优先级与 CPU 亲和、ros2_control `update_rate`、内存锁定。
- 做：测控制周期 jitter 直方图（空载 / 跑 MoveIt 规划时 / 跑相机时），输出 p99 与 max。
- Gate：一份 jitter 报告，并给出至少一条可执行的改进建议。

**W9 关节伺服与安全链路**
- 学：三环复习后聚焦「机械臂与平衡车的不同」：多圈绝对编码器、双编码器（电机侧 / 输出侧）、抱闸时序、力矩 / 速度限幅、看门狗、急停链路。
- 做：梳理公司驱动的安全链路，整理成文档；在仿真中注入超速 / 超力矩指令验证限幅行为。
- Gate：安全链路文档；限幅在仿真中可复现触发。

**W10 公司机械臂建模文档（第 1 轮交付物）**
- 做：整理公司臂的 DH / PoE 参数、Jacobian、动力学方程符号形式；用 Pinocchio 加载公司 URDF，与真机 FK（示教器 / 末端测量）对比。
- Gate：建模文档 v1，FK 误差在机械公差范围内解释得通。

**W11 轨迹规划理论 + 自写 S 曲线**
- 学：梯形 / 7 段 S 曲线、多轴同步、时间最优参数化（TOTG 与 TOPP-RA 原理，用 `toppra` 库跑一遍直观感受）。
- 做：自写关节空间点到点 S 曲线规划器，直接发 `FollowJointTrajectory` 给 JTC。
- Gate：真机或仿真点到点运动，加速度曲线连续，无超调。

**W12 MoveIt2 规划管线**
- 学：`planning_pipelines` 配置、OMPL（RRTConnect / PRM 等）、Pilz Industrial Motion Planner（PTP / LIN / CIRC）、CHOMP / STOMP 概览、planner adapters（AddTimeOptimalParameterization、Ruckig 平滑、FixStartState 系列）。
- 做：同一目标分别用 OMPL 与 Pilz 规划，对比路径与时间参数化结果；`computeCartesianPath` 走一段直线。
- Gate：能解释公司 pipeline yaml 里每个 adapter 的作用及其与 W11 自写规划器的差异。

**W13 重力补偿 + 关节 PD**
- 学：PD + 重力补偿的稳定性结论；用 LQR 视角理解增益选取；effort 接口 vs 驱动器 CST 模式前馈。
- 做：用 Pinocchio `computeGeneralizedGravity` 在 ros2_control 写一个最小 effort controller（或在现有控制器加前馈）。
- Gate：低刚度下关节在任意位姿静止不下坠。

**W14 Planning Scene 与碰撞**
- 学：`CollisionObject` / `AttachedCollisionObject`、FCL 与 Bullet 后端、octomap（深度相机 → `occupancy_map_monitor`）、ACM 运行时修改。
- 做：添加桌面 / 墙面障碍物后规划；接入一台深度相机生成 octomap。
- Gate：有障碍规划成功且路径确实绕开；octomap 更新延迟可测量。

**W15 LeRobot 入门（L5）**
- 学：LeRobotDataset 格式、ACT 架构（action chunking + CVAE）、训练 / 评估脚本。
- 做：用 aloha 仿真数据集训练并评估一次 ACT。
- Gate：训练与评估跑通，能看懂 loss 曲线与成功率。

**W16 第 1 轮复盘 + 缓冲**
- 补齐未过 gate；写复盘：哪一层最卡，第 2 轮重点投哪层。

### P2 力控、辨识、灵巧手入门（W17–W30，第 2 轮）

**W17 动力学辨识理论**
- 学：线性回归形式 Y(q,q̇,q̈)·π = τ、基参数（QR 数值法求解）、物理一致性约束（惯量矩阵正定）。
- 做：推出单关节 + 平面两连杆的回归量，与 Pinocchio `computeJointTorqueRegressor` 对比。
- Gate：两种方式回归量一致。

**W18 激励轨迹设计**
- 学：有限傅里叶级数轨迹、以条件数为目标的优化（scipy）、关节限位 / 速度 / 加速度约束。
- 做：生成激励轨迹后用 MoveIt planning scene 做碰撞校验，再交给 JTC 执行。
- Gate：条件数达标且轨迹无碰撞可执行。

**W19 采集与辨识**
- 学：零相位滤波 + 数值微分、最小二乘 / 加权最小二乘、Coulomb + viscous（直接迁移平衡车做法），必要时 Stribeck。
- 做：ros2 bag 采 q、τ（电流换算），完成辨识，用验证轨迹评估 τ 预测 RMSE。
- Gate：验证集 RMSE 满足自定阈值，并复现平衡车上遇到过的「回归量共线」问题的检查步骤。

**W20 补偿、拖动示教与柔性关节**
- 学：谐波减速器回差 / 柔性建模（对照死区补偿）、柔性关节双惯量模型。
- 做：重力 + 摩擦补偿 → 零力拖动示教；录制一段拖动轨迹并回放。
- Gate：拖动顺滑无明显粘滞，回放轨迹误差可接受。

**W21 碰撞检测：广义动量观测器**
- 学：r = K(p − ∫(τ + Cᵀq̇ − g + r))，与扰动观测器对照；阈值设计、碰撞后反应（停止 / 退让 / 切换阻抗）。
- 做：真机或仿真实现观测器，轻碰触发。
- Gate：误报率与漏报率有数据，反应延迟可测。

**W22 IK 深入 + MoveIt 运动学插件**
- 学：数值 IK、阻尼最小二乘（DLS）、7 轴零空间优化；MoveIt kinematics 插件对比：KDL（默认）、TRAC-IK、pick_ik、bio_ik，`kinematics.yaml` 参数。
- 做：自写 DLS IK，与 MoveIt 插件结果对比；奇异点附近行为测试。
- Gate：自写 IK 在奇异点附近稳定；给出公司臂的插件选型建议。

**W23 笛卡尔阻抗控制**
- 学：关节阻抗 → 笛卡尔阻抗 τ = Jᵀ(K·Δx + D·Δẋ) + g(q)，与 VMC 对照；刚度 / 阻尼整定。
- 做：ros2_control 实现笛卡尔阻抗控制器（仿真先行，再上真机）。
- Gate：末端被推开后回弹无振荡，各方向刚度可独立调节。

**W24 OSC 与 MoveIt Servo**
- 学：操作空间控制理论；MoveIt Servo（Cartesian twist / joint jog、碰撞减速、奇异点处理、低通滤波参数）。
- 做：用键盘或 3D 鼠标通过 Servo 遥操作末端。
- Gate：Servo 遥操作平滑，接近障碍时自动减速。

**W25 灵巧手入门（新增）**
- 学：全驱动 / 欠驱动 / 腱驱动的差异；常见手（Inspire、LEAP、Allegro、Shadow）的自由度与通信方式（串口 / CAN / Dynamixel）；多指运动学。
- 做：为公司灵巧手写或读 ros2_control hardware interface；在 MoveIt 中配置为 end-effector group。
- Gate：RViz 与真机手指状态同步；可通过 MoveIt 设置手的预设姿态。

**W26 抓取基础**
- 学：抓取矩阵 G、形封闭 / 力封闭、Feix 抓取分类、预设抓取姿态库；基于电流 / 力矩限幅的柔顺抓取。
- 做：实现 power / precision / lateral 三种基本抓取。
- Gate：三种抓取各 10 次成功率记录。

**W27 遥操作与数据采集**
- 学：主从臂（ALOHA / SO-101 leader）或 VR 遥操作；手部重定向（`dex-retargeting`，输入来自 MediaPipe 或数据手套）；臂 + 手 + 相机的时间戳对齐。
- 做：搭建单臂 + 手的采集管线，输出 LeRobotDataset。
- Gate：采集 50 条简单抓取 episode，数据质量检查（丢帧率、时间戳抖动）通过。

**W28 ACT 训练与部署（单臂 + 手）**
- 做：训练、评估、仿真 / 真机部署。
- Gate：成功率报告 + 失败案例分类。

**W29 辨识工具链产品化（第 2 轮交付物）**
- 做：激励轨迹 → 采集 → 辨识 → 验证 封装为 ROS2 package + 中文文档。
- Gate：同事按文档能在另一台臂上复现。

**W30 第 2 轮复盘 + 缓冲**

### P3 双臂与策略落地（W31–W46，第 3 轮）

**W31 双臂建模与 MoveIt 配置**
- 学：双臂 URDF（公共基座 / 躯干）、SRDF 多 planning group（left_arm、right_arm、both_arms、left_hand、right_hand）、双臂 ACM。
- 做：生成双臂 moveit_config，both_arms 组联合规划。
- Gate：both_arms 组规划成功，两臂互为障碍时能避让。

**W32 双臂运动学与闭链**
- 学：相对 / 绝对任务空间（cooperative task space）、两臂同持物体的闭链约束、约束 Jacobian。
- 做：推导并用 Pinocchio 验证约束 Jacobian。
- Gate：推导文档，仿真中满足闭链约束的双臂运动。

**W33 双臂轨迹规划与 MTC**
- 学：MoveIt Task Constructor（stage、容器、求解器）、双臂笛卡尔同步（Pilz 或自写）。
- 做：MTC 单臂 pick-and-place 示例跑通，再改为双臂（一臂抓、一臂扶）。
- Gate：MTC 双臂任务在仿真中成功。

**W34 内力控制**
- 学：运动力 / 内力分解、物体级阻抗（object-level impedance）、主从与对称控制对比。
- 做：仿真中双臂搬运刚体，内力可调。
- Gate：内力曲线平稳，物体不掉不被捏坏。

**W35 真机双臂协同搬运（第 3 轮交付物 1）**
- 做：带内力控制的真机搬运；双臂自碰撞避免（MoveIt 碰撞 + 实时距离检查）。
- Gate：搬运一个盒子 10 次成功记录。

**W36 灵巧手深入**
- 学：多指协调、手内操作初步、触觉传感器（若有）读入、抓取稳定性在线判断。
- 做：5 种不同几何 / 材质物体抓取。
- Gate：成功率 ≥ 80%，失败原因可归类。

**W37 双臂双手遥操作数据采集**
- 做：VR（Quest / Vision Pro）或双主臂 + 手重定向，采 100+ episodes。
- Gate：数据集质量检查通过。

**W38 Diffusion Policy 与 ACT 对比**
- 学：Diffusion Policy 原理与 LeRobot 实现。
- 做：同一数据集训两种策略，离线评估 + 仿真。
- Gate：对比报告。

**W39 策略 → 控制器对接（控制器公司的核心问题）**
- 学：action chunk 插值（线性 / 样条 / Ruckig 平滑）、10–50 Hz 策略 → 1 kHz 伺服、时间对齐与延迟补偿。
- 做：实现策略输出平滑层，输出 jerk 与跟踪误差指标。
- Gate：jerk 指标达标，无明显抖动。

**W40 安全层**
- 学：限幅、工作空间约束、MoveIt planning scene 碰撞检查 / Servo 碰撞减速、动量观测器联动、策略输出合法性检查。
- 做：注入异常动作（跳变、越界、穿障）。
- Gate：安全层 100% 拦截，拦截延迟可测。

**W41 真机部署双臂策略（第 3 轮交付物 2）**
- 做：数据采集 → 训练 → 部署端到端 pipeline，写文档。
- Gate：任务成功率报告。

**W42 VLA 概览**
- 学：π0、OpenVLA、GR00T 的架构、数据格式、微调流程（了解即可）。
- Gate：一页纸对比笔记，判断公司是否值得接入。

**W43 Isaac Lab / MuJoCo RL**
- 学：PPO 回顾、Isaac Lab 操作任务结构（env / task / reward）。
- 做：跑通一个官方操作任务（优先灵巧手 in-hand 任务）。
- Gate：训练曲线收敛。

**W44 Residual RL**
- 做：在 ACT / DP 策略上做残差 RL 微调（仿真）。
- Gate：有无残差的性能对比。

**W45 Hybrid Planning 与在线重规划**
- 学：MoveIt2 Hybrid Planning（global + local planner）架构。
- 做：动态障碍物场景下局部重规划。
- Gate：障碍移动时轨迹在线修正不停机。

**W46 第 3 轮复盘 + 缓冲 + pipeline 文档整理**

### P4 专长深挖（W47–W52，第 4 轮）

**W47–W50 四周专题**（根据 3 次复盘选一个）：
- A 高性能力控：柔性关节建模、高带宽阻抗、双编码器融合。
- B 辨识自动化与在线自适应：自动激励轨迹、在线参数更新、温度漂移。
- C 策略的控制器级部署：实时性、安全性、平滑性，做成可复用的「策略执行器」组件。
- D RL 与传统控制融合 + MPC 回补：双臂轨迹优化（Crocoddyl）、接触丰富任务。

**W51 副线缓冲**：双轮足 / MPC 回补，或继续补专题。

**W52 年度复盘 + 下一年计划**。

---

## 3. 与公司 MoveIt2 栈的接口点（速查）

| 层 | MoveIt2 / ROS2 组件 | 对应周 |
|---|---|---|
| L1 | ros2_control hardware interface、controller_manager、update_rate、实时配置 | W6–W8 |
| L2 | effort / 阻抗控制器（自写 controller 插件）、JTC 参数 | W13、W23 |
| L3 | Setup Assistant、planning pipelines（OMPL / Pilz / CHOMP / STOMP）、planner adapters、kinematics 插件、Planning Scene、`computeCartesianPath`、MoveIt Servo | W5、W12、W14、W22、W24 |
| L4 | 多 planning group、MTC、Hybrid Planning、双臂 ACM | W31、W33、W45 |
| L5 | Servo / 自写执行器作为策略输出接口、planning scene 作为安全层 | W39–W41 |
| 灵巧手 | 手作为 end-effector group、独立 hardware interface、预设姿态 | W25、W36 |

---

## 4. 资源清单

- L0：Lynch & Park《Modern Robotics》（配套视频）；Siciliano《Robotics: Modelling, Planning and Control》；[Pinocchio](https://github.com/stack-of-tasks/pinocchio)。
- L1：[ros2_control 文档](https://control.ros.org)；EtherCAT / CiA 402 规范；PREEMPT_RT 与 `cyclictest`。
- L2：Khalil & Dombre《Modeling, Identification and Control of Robots》（辨识章节）；Haddadin 等关于动量观测器碰撞检测的综述。
- L3：[MoveIt2 官方 tutorials](https://moveit.picknik.ai)；[toppra](https://github.com/hungpham2511/toppra)；[pick_ik](https://github.com/PickNikRobotics/pick_ik)；[MoveIt Task Constructor](https://github.com/moveit/moveit_task_constructor)。
- L4：Caccavale & Uchiyama 双臂协调章节（Springer Handbook of Robotics）。
- L5：[LeRobot](https://github.com/huggingface/lerobot)；ACT、Diffusion Policy 原论文；[Isaac Lab](https://isaac-sim.github.io/IsaacLab)。
- 灵巧手：Murray, Li & Sastry《A Mathematical Introduction to Robotic Manipulation》（抓取章节）；[dex-retargeting](https://github.com/dexsuite/dex-retargeting)；[LEAP Hand](https://leaphand.com)。
- 个人设备（可选）：SO-101 leader/follower（遥操作 → 采数据 → 训练 → 部署全流程）；MuJoCo（CPU 即可）；Isaac Lab 需 RTX 显卡。

---

## 5. 周复盘模板

```
周次：W__      层：L_      本周 gate：pass / fail
做了什么：
卡在哪：
用到的旧经验（平衡车）：
下周调整：
```
