# ROS2-smart-car

一个面向智能移动机器人开发的项目仓库，基于 **ROS 2 + STM32** 构建，采用“**上位机（ROS 2）+ 下位机（STM32）**”的协同架构。

其中：
- **ROS2 侧**负责底盘通信、传感器接入、状态估计、建图与导航等系统集成
- **STM32 侧**负责电机驱动、执行机构控制、底层传感器读取与串口通信

整体上，这个仓库已经不是单纯的工程骨架，而是一套可以持续联调、迭代和扩展的智能车开发基础平台。

## 项目结构

- `rdk_x5_ros2/`：运行在地平线旭日 X5 / Linux 侧的 ROS 2 工作空间
- `stm32_chassis/`：运行在 STM32 侧的底盘控制与硬件驱动工程
- `docs/`：项目文档与补充资料

---

## 当前已包含的主要内容

### ROS2 上位机侧（`rdk_x5_ros2/`）

结合当前仓库中的 `PROJECT_DOC_zh.md`、`package.xml`、`launch/`、`scripts/` 等文件，ROS2 侧目前已经具备以下能力：

#### 底盘通信与控制
- `base_controller` 节点负责与 STM32 底盘进行串口通信
- 订阅 `/cmd_vel` 速度指令
- 发布 `/odom` 里程计信息

#### IMU 驱动
- `imu_driver` 节点负责惯性传感器数据采集
- 发布 `/imu/data`
- 提供姿态、角速度与加速度信息

#### 雷达接入
- 使用 `rplidar_ros` 接入 RPLIDAR
- 默认采用 `laser_link` 坐标系
- 为建图与导航提供 `/scan` 数据

#### TF 与机器人模型
- 使用 `robot_state_publisher + xacro` 统一管理机器人模型
- 描述 `base_footprint`、`imu_link`、`laser_link` 等坐标关系

#### 状态估计与融合
- 使用 `robot_localization` 的 EKF 节点
- 融合轮式里程计与 IMU 数据
- 由 EKF 统一接管 `odom -> base_footprint` 的 TF 发布

#### 建图（SLAM）
- 提供 `mapping.launch.py`
- 集成 `slam_toolbox`
- 支持 2D 地图构建与保存

#### 自主导航（Nav2）
- 提供 `navigation.launch.py`
- 对接 `nav2_bringup`
- 支持基于已有地图的自主导航

#### 自动定位辅助
- 提供 `scripts/auto_align.py`
- 用于导航启动后原地旋转，辅助 AMCL 粒子更快收敛

#### 串口设备固化
- 提供 `bind_usb.sh`
- 可将 IMU / LiDAR 固定为：
  - `/dev/robot_imu`
  - `/dev/robot_lidar`
- 便于比赛或长期部署时避免串口号漂移

> 详细说明请查看：`rdk_x5_ros2/PROJECT_DOC_zh.md`

### STM32 下位机侧（`stm32_chassis/`）

根据当前 `stm32_chassis/README.md`，下位机工程已包含：

- 直流电机驱动与速度闭环控制
- 舵机转向控制
- MPU6050 IMU 数据读取
- 通过串口与上位机 / 旭日 X5 通信
- STM32CubeMX 工程配置与 Keil 工程文件

常见目录包括：
- `Core/`：主应用代码与 HAL 回调
- `Drivers/`：HAL / CMSIS 驱动
- `MDK-ARM/`：Keil 工程
- `*.ioc`：STM32CubeMX 工程配置

> 详细说明请查看：`stm32_chassis/README.md`

---

## ROS2 侧关键入口

当前仓库中与 ROS2 侧相关的关键入口包括：

- `rdk_x5_ros2/src/ylhb_base/launch/bringup.launch.py`
- `rdk_x5_ros2/src/ylhb_base/launch/mapping.launch.py`
- `rdk_x5_ros2/src/ylhb_base/launch/navigation.launch.py`
- `rdk_x5_ros2/src/ylhb_base/scripts/auto_align.py`
- `rdk_x5_ros2/bind_usb.sh`

这些入口文件覆盖了：
- 底盘基础驱动启动
- 雷达 / IMU / EKF 联合启动
- SLAM 建图流程
- Nav2 导航流程
- 启动后的自动对齐辅助流程

---

## 快速开始

### ROS2 侧

RDK X5 侧当前工作区根目录为：

```bash
~/ROS2-smart-car/rdk_x5_ros2
```

进入该目录后，完成依赖安装与编译，再执行对应启动文件。

#### 基础驱动启动
```bash
cd ~/ROS2-smart-car/rdk_x5_ros2
source install/setup.bash
ros2 launch ylhb_base bringup.launch.py
```

#### 建图
```bash
cd ~/ROS2-smart-car/rdk_x5_ros2
source install/setup.bash
ros2 launch ylhb_base mapping.launch.py
```

#### 导航
```bash
cd ~/ROS2-smart-car/rdk_x5_ros2
source install/setup.bash
ros2 launch ylhb_base navigation.launch.py
```

> 默认会加载 `~/ROS2-smart-car/rdk_x5_ros2/my_map.yaml` 作为地图文件。若使用其他地图，可通过 `map:=/你的地图绝对路径.yaml` 覆盖。

### STM32 侧

1. 使用 STM32CubeMX 打开 `.ioc` 工程配置（如有需要）
2. 通过 **Keil MDK** 或 **STM32CubeIDE** 编译工程
3. 下载到开发板后进行联调

---

## 当前仓库状态

目前，这个仓库已经具备一套较完整的智能车开发基础框架：

- **ROS2 上位机**：负责底盘通信、IMU、雷达、TF、EKF、SLAM、Nav2 等系统集成
- **STM32 下位机**：负责电机、舵机、IMU 与底层串口通信
- **整体定位**：适合作为比赛项目、课程设计或后续功能扩展的基础工程

---

## 后续可继续完善的方向

后续还可以继续补充：

- ROS2 各节点的编译说明与依赖安装说明
- ROS2 ↔ STM32 的通信协议文档
- 地图保存、恢复与导航使用示例
- 视觉模块的代码与独立说明文档
- 更完整的系统接线图、TF 图与话题拓扑图

---

## 说明

根目录 `README.md` 主要用于提供项目总览；具体实现细节、参数说明与调试经验，建议优先查看各子目录文档：

- `rdk_x5_ros2/PROJECT_DOC_zh.md`
- `stm32_chassis/README.md`
