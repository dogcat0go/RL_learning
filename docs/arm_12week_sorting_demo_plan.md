# 12 周机械臂运控入门计划（MoveIt2 + 6D 姿态 → 分拣级 pick-and-place）

> 从《52 周学习计划》（`dual_arm_weekly_learning_plan.md`）切出的 3 个月版本。
> 目标收窄为：**单臂、位置控制、MoveIt2 规划、相机 + 6D 姿态驱动的分拣 / 演示级 pick-and-place**。
> 不做力控、辨识、自写规划器、学习策略；这些在 3 个月后按公司方向从 52 周计划 P2 续接。

---

## 0. 起点、终点与范围

**起点（已具备）**：电机三环、PID / LQR / VMC、卡尔曼滤波、扰动观测器、摩擦辨识、ROS2 / Gazebo、log 驱动调试。

**终点（12 周后）**：

- 能独立为公司机械臂配置并排查 MoveIt2 + ros2_control 栈（planning / controller / hardware 三层定位问题）。
- 手眼标定、6D 姿态估计、抓取位姿生成、planning scene 障碍、夹爪接口串成一条分拣 pipeline。
- 分拣 demo 连续 20 次成功率有记录，失败案例可归类。
- 一份「公司臂上手文档 + 坑清单」，同事按文档能复现。

**精度目标**：抓取位置误差 ±3–5 mm、姿态误差 ±3–5°（分拣 / 演示级）。误差预算按来源排序为
手眼标定 > 6D 姿态估计 > 相机内参 > 机械臂重复定位，所以 W7–W9 是精度的关键周，控制器调参不是。

**明确不在范围内**（52 周计划中对应周）：实时性 / jitter（W8）、自写 S 曲线（W11）、重力补偿与 effort 控制（W13）、LeRobot / ACT（W15）、整个 P2 力控与辨识、双臂（P3）。

**平衡车经验的落点**：

| 平衡车已有 | 本计划中的用处 | 周 |
|---|---|---|
| 三环、限幅、看门狗 | 读懂驱动器 CSP 模式与 JTC 的分工，安全链路梳理 | W3–W4 |
| LQR / VMC 的矩阵直觉 | 刚体变换、Jacobian、奇异性快速上手 | W1–W2 |
| 卡尔曼 / 时间对齐 | 相机帧、TF、关节状态的时间戳对齐 | W7、W9 |
| log 驱动调试 | ros2 bag + PlotJuggler + 自写脚本做每周 gate 验证 | 全程 |

---

## 1. 总览

| 阶段 | 周 | 目标 | 交付物 | 对应 52 周计划 |
|---|---|---|---|---|
| A 数学与 MoveIt 初体验 | W1–W2 | 变换 / FK / IK / Jacobian，Pinocchio 验证，官方 tutorial 跑通 | L0 笔记 + 验证脚本 | W1–W4 压缩 |
| B 公司栈跑通 | W3–W6 | URDF / SRDF / moveit_config、ros2_control 数据流、规划管线、IK 插件 | 公司臂数据流图 + 规划 / IK 对比报告 | W5、W6、W12、W22 |
| C 感知与标定 | W7–W9 | 相机内参、手眼标定、6D 姿态估计接入 planning scene | 标定报告 + 姿态估计误差报告 | W14 前置 + 新增 |
| D 分拣 pipeline | W10–W11 | 抓取位姿生成、状态机、夹爪接口、障碍物 | 分拣 demo + 成功率记录 | W14、W33 单臂部分 |
| E 收尾 | W12 | 缓冲、失败分类、文档 | 上手文档 + 坑清单 | W16 |

**每周节奏**：工作日以公司真机和代码为主，自学约 10 h/周（4 h 理论 / 源码、4 h 实操、2 h 笔记 + 复盘）。
每周一个 gate，不过不往前推，用 W12 补。**W1 的硬交付是仿真环境到位**（gz_ros2_control 或公司现有仿真），否则后面每周都会因真机排期打折。

---

## 2. 周计划

### A 数学与 MoveIt 初体验（W1–W2）

**W1 刚体变换 + 正运动学 + 环境搭建**
- 学：旋转矩阵、四元数、齐次变换、SE(3)（Modern Robotics 第 3 章）；PoE 与 DH（第 4 章）；URDF 的 link / joint / 惯量 / 碰撞体。
- 做：装 Pinocchio；取公司臂 URDF（或 UR5e / Panda）做 FK，与手算 PoE 对比；装 ROS2（与公司一致的发行版）+ MoveIt2 + gz_ros2_control，跑官方 Getting Started 与 RViz MotionPlanning 插件。
- Gate：3 组关节角 FK 误差 < 1e-6；RViz 中能拖动目标并执行规划；**仿真环境可用**。

**W2 Jacobian、IK 与 MoveGroupInterface**
- 学：Jacobian、奇异性、可操作度（MR 第 5 章）；数值 IK 与阻尼最小二乘（DLS）的原理，知道为什么 6 轴有解析解、7 轴需要零空间；`MoveGroupInterface` 的 `setPoseTarget` / `plan` / `execute` / `computeCartesianPath`。
- 做：解析 Jacobian 与数值差分对比；写一个 C++ 或 Python 节点，给定末端 6D 位姿规划并执行。
- Gate：能解释「末端 6D 目标 → 关节角」这条链上 IK 失败的三种常见原因（超出工作空间、奇异、碰撞）。

### B 公司栈跑通（W3–W6）

**W3 读懂公司 URDF / SRDF / moveit_config**
- 学：Setup Assistant 流程：自碰撞矩阵（ACM）、virtual joint、planning group、end effector、ros2_control 标签；`moveit_controllers.yaml`、`kinematics.yaml`、`joint_limits.yaml` 各自管什么。
- 做：用 Setup Assistant 为公司臂从零生成一份 moveit_config，与公司现有配置 diff，差异逐条弄清。
- Gate：能解释 SRDF 每条 `disable_collisions` 的理由；diff 报告。

**W4 ros2_control 数据流与安全链路**
- 学：controller_manager、`hardware_interface::SystemInterface` 的 `read / write` 生命周期、state / command interface、`joint_trajectory_controller`（JTC）与 `FollowJointTrajectory` action；驱动器 CSP 模式下「哪几环在驱动器、哪几环在控制器」；抱闸、限幅、急停链路。
- 做：`mock_components` → 仿真 → 真机三步跑通 MoveIt → JTC → 硬件；画 move_group → controller → hardware 数据流图；梳理安全链路成一页文档。
- Gate：故意改坏一处（controller joints 不匹配 / hardware 未激活 / 规划超时），能从日志定位是 planning、controller 还是 hardware 的问题。

**W5 规划管线**
- 学：`planning_pipelines` 配置；OMPL（RRTConnect、PRM）、Pilz（PTP / LIN / CIRC）各自适用场景；planner adapters（AddTimeOptimalParameterization、Ruckig、FixStartState 系列）；`computeCartesianPath` 的 `eef_step`、`jump_threshold` 与「只走了 60%」的原因。
- 做：同一目标分别用 OMPL 与 Pilz 规划，对比路径形状、规划时间、执行时间；用 `computeCartesianPath` 走一段直线并复现一次部分成功。
- Gate：能解释公司 pipeline yaml 里每个 adapter 的作用；给出「分拣场景 PTP 用什么、下压 / 提起用什么」的选型。

**W6 IK 插件与工作空间**
- 学：KDL（默认）、TRAC-IK、pick_ik、bio_ik 的差异；6 轴臂 KDL 基本够用，7 轴换 TRAC-IK / pick_ik；`kinematics.yaml` 的 timeout / 求解器参数；解的连续性（跳解）。
- 做：在分拣常用工作区域（料箱上方 + 放置区）采样 500 个 6D 位姿，统计 IK 成功率与耗时；在奇异点附近测试行为；画出可达工作空间边界。
- Gate：常用区域 IK 成功率有数据并 ≥ 95%；`kinematics.yaml` 选型结论。

### C 感知与标定（W7–W9）

**W7 相机内参与手眼标定**
- 学：针孔模型、畸变、内参标定（`camera_calibration`）；eye-in-hand vs eye-to-hand 的选择（分拣固定料箱通常 eye-to-hand）；AX = XB 问题；`moveit_calibration` / `easy_handeye2` 流程；TF 链（camera → base 或 camera → tool）与 `tf2` 时间戳。
- 做：内参标定一次；手眼标定一次，用「末端贴标定板到多个位姿」的复测法评估误差；把相机 TF 发布进 launch。
- Gate：内参重投影误差 < 0.5 px；手眼标定复测位置误差有数据（目标 < 3 mm），并说明误差来源。

**W8 6D 姿态估计接入（一）**
- 学：分拣场景的三类方案：(a) 已知 CAD + 基于模型的估计（FoundationPose、MegaPose）；(b) 点云配准（PCL / Open3D ICP，先分割再配准）；(c) 简单物体用 ArUco / AprilTag 兜底。先用现成模型，**不自己训**。
- 做：选一种方案在 1–2 种目标物体上跑通，输出 `geometry_msgs/PoseStamped`；用 AprilTag 作为真值对比估计误差。
- Gate：静态物体姿态估计误差有数据（位置 < 5 mm、姿态 < 5°）；帧率与延迟有数据。

**W9 6D 姿态估计接入（二）与 planning scene**
- 学：`CollisionObject` / `AttachedCollisionObject`；把物体姿态变成 planning scene 中的碰撞体；料箱、桌面、相机支架作为静态障碍；相机帧、关节状态、TF 的时间对齐（接卡尔曼 / 时间对齐经验）。
- 做：估计结果 → planning scene 的 `CollisionObject`，RViz 中物体位置与真机一致；多物体场景下多次估计的重复性测试；料箱 / 桌面加入 scene 后规划能绕开。
- Gate：RViz 中物体位置与真机对齐（目测 + 尺量）；有障碍规划成功且路径确实绕开。

### D 分拣 pipeline（W10–W11）

**W10 抓取位姿生成与夹爪接口**
- 学：抓取位姿 = 物体位姿 × 预设抓取偏移（按物体类型建一个小抓取库）；pre-grasp / grasp / lift 三点法；接近方向与 IK 可达性的联合检查；夹爪 / 吸盘的 ros2_control 接口（`GripperActionController` 或厂家驱动）；`attachObject` / `detachObject`。
- 做：为每种目标物体定义 2–3 个候选抓取，按 IK 可达 + 碰撞检查筛选；夹爪开合接入；单次抓取 → 提起 → 放置手动触发跑通。
- Gate：单物体抓取 10 次成功率 ≥ 80%；抓取后物体 attach 到末端，规划不与物体自碰。

**W11 分拣状态机与成功率**
- 学：状态机（`BehaviorTree.CPP` 或 `smach` / 简单 Python 状态机）：感知 → 选目标 → 规划 pre-grasp（PTP）→ 下压（LIN）→ 闭合 → 提起（LIN）→ 放置区（PTP）→ 释放 → 回 home；每步的失败处理（重新感知 / 换候选抓取 / 跳过）；运行时的 planning scene 更新。
- 做：多物体料箱分拣 demo；连续 20 次记录成功 / 失败及原因（感知错、IK 失败、规划失败、抓滑、放置偏）。
- Gate：20 次成功率有记录（目标 ≥ 80%）；失败原因按层归类。

### E 收尾（W12）

**W12 缓冲 + 文档**
- 做：补未过 gate；按失败分类做一轮针对性改进（通常是标定或抓取库）；整理「公司臂上手文档」：环境搭建、launch 顺序、配置文件说明、数据流图、安全链路、坑清单。
- Gate：同事按文档能在 1 天内跑通 demo；写 12 周复盘，决定下 3 个月从 52 周计划 P2 的哪一周续接。

---

## 3. 误差预算与排查顺序（分拣抓不准时按这个顺序查）

| 顺序 | 来源 | 典型量级 | 检查方法 | 周 |
|---|---|---|---|---|
| 1 | 手眼标定 | 2–10 mm | 末端贴标定板复测多个位姿 | W7 |
| 2 | 6D 姿态估计 | 2–10 mm / 2–10° | AprilTag 真值对比；多次估计重复性 | W8–W9 |
| 3 | 相机内参 / 畸变 | 1–3 mm（边缘更大） | 重投影误差；边缘 vs 中心对比 | W7 |
| 4 | TF 时间戳 / 相机与关节状态不同步 | 运动中才出现 | 静止 vs 运动中估计对比 | W9 |
| 5 | 抓取偏移定义 | 固定偏差 | 同一物体多次抓取偏差方向是否一致 | W10 |
| 6 | 机械臂重复定位 | < 0.1 mm | 基本可排除 | — |

---

## 4. 常见坑速查

| 现象 | 根因 | 周 |
|---|---|---|
| `Unable to identify any set of controllers that can actuate the specified joints` | `moveit_controllers.yaml` 的 joints 与 group 不匹配 | W4 |
| 规划成功但不动 / 立刻 abort | controller 未激活、hardware 未 `on_activate`、`allowed_start_tolerance` 过严 | W4 |
| `computeCartesianPath` fraction < 1.0 | 直线路径穿过奇异 / 碰撞 / 超出关节限位；`eef_step` 过大 | W5 |
| IK 频繁 timeout | 目标在工作空间边缘、姿态不可达、KDL 用在 7 轴上 | W6 |
| 相邻目标关节解差很大（跳解） | 求解器无连续性偏好 | W6 |
| RViz 中物体位置与真机差一截 | 手眼标定错、TF 父子帧搞反、相机光学帧与本体帧混用 | W7、W9 |
| 规划绕开了物体但真机撞了 | 碰撞体尺寸 / padding 不够、物体姿态估计偏 | W9 |
| 抓取后下一段规划失败 | 物体未 attach，被当作障碍 | W10 |
| 偶发抓空 | 相机帧与关节状态时间戳不同步；夹爪闭合时序 | W9、W10 |

---

## 5. 资源清单

- 数学：Lynch & Park《Modern Robotics》第 3–6 章；[Pinocchio](https://github.com/stack-of-tasks/pinocchio)。
- MoveIt2：[官方 tutorials](https://moveit.picknik.ai)（Getting Started、MoveGroup C++ / Python、Planning Scene、Pick and Place、Hand-Eye Calibration）；[pick_ik](https://github.com/PickNikRobotics/pick_ik)。
- ros2_control：[control.ros.org](https://control.ros.org)（Hardware Components、JTC、GripperActionController）。
- 标定：`camera_calibration`、`moveit_calibration`、[easy_handeye2](https://github.com/marcoesposito1988/easy_handeye2)。
- 6D 姿态：[FoundationPose](https://github.com/NVlabs/FoundationPose)、[MegaPose](https://github.com/megapose6d/megapose6d)、Open3D 配准教程、AprilTag ROS2。
- 状态机：[BehaviorTree.CPP](https://www.behaviortree.dev)。
- 仿真：gz_ros2_control；公开臂模型（UR5e、Panda）用于 W1–W2 练手。

---

## 6. 周复盘模板

```
周次：W__      阶段：_      本周 gate：pass / fail
做了什么：
卡在哪（planning / controller / hardware / 标定 / 感知）：
用到的旧经验（平衡车）：
下周调整：
```
