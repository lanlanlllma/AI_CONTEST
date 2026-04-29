# AI_CONTEST

一个基于 ROS 2 的双臂机器人项目，集成了机器人本体控制、目标检测、深度处理、抓取检测、夹爪控制、TF 工具和标定工具。仓库内各模块以独立 ROS 2 package 组织，适合在统一工作空间下按需构建和联调。

## 项目概览

该项目的核心能力包括：

- 双臂机器人运动控制与服务封装
- 基于 YOLOv8 + TensorRT 的目标检测
- 深度图与 2D 检测结果融合，生成 3D 边界框
- 基于 socket 的抓取检测服务对接
- 夹爪控制与相关驱动支持
- TF 查询、静态变换发布、手眼标定数据采集等工具

## 目录说明

| 目录 | 说明 |
| --- | --- |
| `robo_ctrl` | 机器人控制核心包，提供运动、伺服、状态发布等服务接口 |
| `detector` | 目标检测模块，基于 YOLOv8 / TensorRT，输出 2D 检测框 |
| `depth_handler` | 深度处理模块，将 2D 检测框与深度图结合生成 3D 结果 |
| `grab_detect` | 抓取检测服务包，通过 TCP socket 与外部抓取服务器通信 |
| `epg50_gripper_ros` | 夹爪控制相关 ROS 2 包 |
| `dualarm` | 双臂协同控制与流程编排相关代码 |
| `tools` | 标定、TF 查询、静态 TF 发布等辅助工具 |
| `camera_info_interceptor` | 相机信息相关的拦截与处理模块 |
| `tf_node` | TF 相关节点或占位包 |

## 环境依赖

根据各子模块文档，建议准备以下环境：

- Ubuntu + ROS 2 Humble 或更高版本
- `colcon`
- OpenCV 4.x
- `cv_bridge`
- `tf2` / `tf2_ros`
- CUDA 11.x+ 与 TensorRT 8.x+（用于 `detector`）
- 对应机器人控制库与硬件驱动

不同包的依赖略有差异，建议在开始联调前查看各子目录内已有的 `README.md`。

## 构建方式

在仓库根目录所在的 ROS 2 工作空间执行：

```bash
colcon build --symlink-install
source install/setup.bash
```

如果只想构建单个模块，可以使用：

```bash
colcon build --packages-select robo_ctrl detector depth_handler grab_detect tools dualarm
```

## 常见使用流程

一个典型流程通常是：

1. 启动机器人控制节点 `robo_ctrl`
2. 启动相机与相关 TF
3. 启动 `detector` 输出 2D 检测结果
4. 启动 `depth_handler` 生成 3D 边界框
5. 按需启动 `grab_detect` 获取抓取结果
6. 通过 `dualarm` 或自定义流程节点执行抓取与动作编排

## 运行示例

以下命令仅为常见入口，实际参数请结合各包文档调整：

```bash
ros2 launch robo_ctrl robo_ctrl.launch.py
ros2 run depth_handler depth_processor_node
ros2 launch grab_detect grasp_detect.launch.py
ros2 launch tools tf_point_query.launch.py
```

目标检测模块常见构建和运行方式：

```bash
colcon build --packages-select detector
source install/setup.bash
ros2 launch detector detector.launch.py
```

## 文档入口

仓库中已经包含若干子模块说明文档，建议优先阅读：

- `detector/README.md`
- `depth_handler/README.md`
- `grab_detect/README.md`
- `robo_ctrl/README.md`
- `tools/README.md`
- `dualarm/README_ServiceTemplate_Refactoring.md`

## 说明

当前仓库未看到统一许可证声明，若后续对外发布或协作，建议补充 `LICENSE`、总体架构图和完整启动说明。
