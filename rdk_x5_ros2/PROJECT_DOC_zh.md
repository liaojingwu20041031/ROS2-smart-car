# ROS2 机器人底层通信与融合框架文档 (实时更新版)

## 📌 项目概述
本项目 (`ROS2_ylhb`) 是一个基于 ROS 2 的机器人底层控制和多传感器驱动项目。目前主要整合了：
1. **两轮差速底盘** 的 PID 通信控制（串口下发速度指令，解析底盘里程计反馈）。
2. **IMU 单位** 的通信补偿（解析 IMU 的加速度、角速度及姿态角度数据）。
3. **传感器融合**（利用 Robot Localization 的 EKF 节点，将里程计和 IMU 数据融合以获得更精确的设备位姿表示）。

项目包主要位于 `src/ylhb_base/` 下。

---

## 🏗️ 整体系统架构
为了实现最佳的位置估计并避免 TF 树冲突，本项目中的数据与 TF 变换关系如下：

1. **底盘节点 (`base_controller`)**
   - 接收 ROS 侧的 `cmd_vel` 下发给底层串口。
   - 读取底层的速度/里程计，并发布 `/odom` 话题。
   - ⚠️ 注意：在此模式下，它主动**停止发布** `odom -> base_footprint` 的 TF (`publish_tf=False`)，让 EKF 接管。
   
2. **IMU 节点 (`imu_driver`)**
   - 读取外设传感器数据（串口通信）。
   - 发布 `/imu/data` 话题（附带协方差保证后续滤波算法的精度）。

3. **激光雷达节点 (`rplidar_node`)**
   - 读取 RPLIDAR 的激光或者点云数据。
   - 挂载为 `laser_link`，负责发布 `/scan` 话题供建图算法(Cartographer/Slam_toolbox)使用。

4. **机器人状态发布器 (`robot_state_publisher` & URDF)**
   - 替代了原先凌乱的静态 TF 树。通过读取 `src/ylhb_base/urdf/ylhb.urdf.xacro` 文件，统一管理 `base_footprint` 👉 `imu_link` / `laser_link` 的物理安装位姿。所有空间几何描述都在该 xacro 中体现。

5. **EKF 融合节点 (`ekf_filter_node` / `robot_localization`)**
   - 订阅 `/odom` 和 `/imu/data`。
   - 根据时间戳，计算最优估计，随后发布准确的 `odom` 到 `base_footprint` 的 TF 树与修正后的里程计话题。

---

## 🧩 核心节点与话题记录分析

### 1️⃣ 底盘控制节点 (`base_controller`)
- **功能**：通过串口协议与底盘 STM32/MCU 通信，下传运动速度，上传轮式里程计状态。
- **串口设备**：`/dev/ttyS1`（波特率 `115200`）
- **通信话题**：
  - 📥 **订阅者 (Subscribes)**: 
    - 话题: `/cmd_vel` 
    - 消息类型: `geometry_msgs/msg/Twist`
    - 用途: 控制机器人的线速度 (x方向) 和角速度 (z方向)。
  - 📤 **发布者 (Publishes)**: 
    - 话题: `/odom`
    - 消息类型: `nav_msgs/msg/Odometry`
    - 用途: 纯由轮子编码器推算出的自身位姿与速度。
- **参数预设**：
  - `serial_port`: 底层通信端口 (`/dev/ttyS1`)
  - `wheel_track`: 轮距 (`0.25`m)
  - `publish_tf`: 是否发布里程计TF树，默认 `True`，但在 `bringup.launch.py` 已覆盖为 `False`。

### 2️⃣ 惯性测量单元节点 (`imu_driver`)
- **功能**：处理 IMU 发出的加速度计、陀螺仪与欧拉角数据串。
- **默认串口**：`/dev/ttyUSB0`（预装 `115200` 波特率，可动态修改）
- **坐标系设定**：`imu_link`
- **通信话题**：
  - 📤 **发布者 (Publishes)**: 
    - 话题: `/imu/data`
    - 消息类型: `sensor_msgs/msg/Imu`
    - 用途: 包含姿态四元数、角速度和线性加速度信息。

### 3️⃣ 激光雷达节点 (`rplidar_node`)
- **功能**：处理 RPLIDAR 设备的数据。
- **默认串口**：`/dev/ttyUSB1`（可动态修改）
- **坐标系设定**：`laser_link`
- **主要话题**：
  - 📤 **发布者**: `/scan` (`sensor_msgs/msg/LaserScan`)

### 4️⃣ EKF 滤波融合节点 (`ekf_filter_node`)
- **功能**：使用扩展卡尔曼滤波，对具有不确定性的轮式里程计和高频漂移的 IMU 进行互补滤波。
- **通信话题**：
  - 📥 **订阅者 (Subscribes)**: `/odom`，`/imu/data`
  - 📤 **发布者 (Publishes)**: 
    - 话题: `/odom/filtered` 或者覆盖 `/odom` （视配置而定，可直接负责 TF 广播 `odom` -> `base_footprint`）。
  - 🌐 **TF变换**: 提供动态 `odom` 👉 `base_footprint` 关系。

### 5️⃣ 静态 TF 发布节点
- **功能**：描述物理设备的相互位姿。
- 坐标层级：`base_footprint` 👉 `imu_link` / `base_footprint` 👉 `laser_link`
- 关系设定：在 `bringup.launch.py` 中约定 IMU 在离地 5cm 处，雷达装在中心约离地 15cm 处。

---

## 🚀 启动指引与动态参数配置

本项目中，各个组件配置了动态启动参数 `Launch Argument`，您可以不修改源码即可切换串口绑定以应对各种变化：

1. **基本一键启动** (使用默认的内部配置: 底盘 ttyS1，IMU ttyUSB0，雷达 ttyUSB1)
   ```bash
   source install/setup.bash
   ros2 launch ylhb_base bringup.launch.py
   ```

2. **动态修改硬件串口端口启动** (如果您的设备挂载发生了串变)
   ```bash
   ros2 launch ylhb_base bringup.launch.py base_port:=/dev/ttyS1 imu_port:=/dev/ttyUSB2 lidar_port:=/dev/ttyUSB0
   ```

---

## 🗺️ 建图指南 (SLAM Toolbox)
项目已集成 `slam_toolbox` 用于构建 2D 占据栅格地图。

### 1️⃣ 开始建图
请先确保底层驱动（`bringup.launch.py`）正在后台运行，并且已连接激光雷达。随后打开一个新的终端执行：
```bash
source install/setup.bash
ros2 launch ylhb_base mapping.launch.py
```
*备注: 此命令将启动 `slam_toolbox` 的在线异步建图节点，并自动加载专门为小车匹配优化的参数 (`slam_toolbox_params.yaml`)*。

### 2️⃣ 开启键盘遥控 (Teleop)
在建图过程中，你需要控制机器人移动来扫描整个环境。可以启动键盘控制节点来向底盘下发速度指令。
打开一个全新的终端并执行：
```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
**常用控制按键说明：**
*   **`i`**：前进
*   **`,`**：后退
*   **`j`** / **`l`**：原地左转 / 右转
*   **`u`** / **`o`**：左前弯 / 右前弯
*   **`k`** 或 **`空格`**：紧急停止
*   **`q`** / **`z`**：增加/减少所有速度的 10%
*   **`w`** / **`x`**：仅增加/减少 **线速度** (前进后退的速度)
*   **`e`** / **`c`**：仅增加/减少 **角速度** (旋转的速度)

*注意：此终端窗口必须保持在最前（获取键盘焦点）时按键才能生效。一旦按键，节点就会向 `/cmd_vel` 话题发送 `Twist` 消息，驱动底盘移动，配合 SLAM 算法进行新区域的探索。*

### 3️⃣ 保存地图
建立完整地图之后，你可以使用 `nav2_map_server` 中的 `map_saver_cli` 来保存生成的地图：
```bash
ros2 run nav2_map_server map_saver_cli -f my_map
```
这将在当前目录下生成 `my_map.yaml` 和 `my_map.pgm`。

### 4️⃣ 建图核心参数解析 (SLAM Toolbox)
针对 `slam_toolbox_params.yaml` 中的核心参数解释，方便后续根据实际场地情况进行调参：
- **`odom_frame` / `map_frame` / `base_frame`**: 分别对应里程计坐标系（`odom`）、地图坐标系（`map`）和机器人基座坐标系（`base_footprint`）。
- **`scan_topic`**: 订阅的激光雷达数据话题，默认通常为 `/scan`。
- **`mode`**: 运行模式，`mapping` 为建图模式，`localization` 为纯定位模式。
- **`map_file_name`**: 在定位模式下加载的已有地图文件路径（不带后缀）。
- **`map_start_at_dock`**: 如果为 `true`，则认为机器人在充电座上启动，起始坐标固定；如果为 `false`，则在当前位置建立坐标原点。
- **`debug_logging`**: 是否开启调试日志，有助于排查问题，但会产生大量终端输出。
- **`transform_publish_period`**: 发布 `map` 到 `odom` 坐标变换的周期（秒），越小越平滑但消耗更多 CPU（例如 `0.02` 为 50Hz）。
- **`map_update_interval`**: 地图更新发布的周期（秒），例如 `1.0` 表示每秒在 Rivz 中更新一次地图显示。
- **`resolution`**: 地图的分辨率，即栅格地图中每个像素代表的实际物理大小（米），默认 `0.05`。
- **`max_laser_range`**: 激光雷达的最大有效测距（米），超出此距离的数据将被忽略，例如 A2M8 可设为 `12.0`。
- **`minimum_time_interval`**: 处理两次激光雷达扫描之间的最小时间间隔（秒）。
- **`transform_timeout`**: 坐标变换查找的超时时间（秒），默认 `0.2`。
- **`tf_buffer_duration`**: TF 缓冲区的保留时间（秒），默认 `30.0`。
- **`stack_capacity` / `queue_size`**: 处理队列的大小。如果终端出现 `Message Filter dropping message: frame 'laser_link' at time... Queue is full` 的警告，说明雷达数据发布过快而 SLAM 处理不过来，可以尝试增大这些值，或者降低雷达的扫描频率/上面提到的更新频率。
- **`minimum_travel_distance`**: 机器人必须移动多远（米）才会处理新的一帧扫描，例如 `0.5`。
- **`minimum_travel_heading`**: 机器人必须旋转多大角度（弧度）才会处理新的一帧扫描，例如 `0.5`。
- **`use_scan_matching`**: 是否使用激光雷达进行扫描匹配以优化里程计，设为 `true` 可以大大提高建图精度。
- **`use_scan_barycenter`**: 在扫描匹配中是否使用扫描重心计算。
- **`minimum_distance_penalty`**: 扫描匹配中对距离差异的惩罚权重。
- **`minimum_angle_penalty`**: 扫描匹配中对角度差异的惩罚权重。
- **`use_response_expansion`**: 在进行相关性搜索前是否扩大响应范围。
- **`correlation_search_space_dimension`**: Scan matcher 的搜索窗口大小，在里程计不准时增大该值有助于利用激光强制匹配，但会增加计算量。
- **`correlation_search_space_resolution`**: 搜索空间的分辨率。
- **`correlation_search_space_smear_deviation`**: 搜索空间的模糊偏差。
- **`loop_search_space_dimension`**: 闭环检测的搜索窗口大小，例如 `8.0`。
- **`loop_search_space_resolution`**: 闭环检测搜索空间的分辨率，例如 `0.05`。
- **`loop_search_space_smear_deviation`**: 闭环检测搜索空间的模糊偏差，例如 `0.03`。
- **`distance_variance_penalty`**: 闭环检测中对距离方差的惩罚，例如 `0.5`。
- **`angle_variance_penalty`**: 闭环检测中对角度方差的惩罚，例如 `1.0`。
- **`fine_search_angle_offset`**: 精细搜索的角度范围，例如 `0.00349`。
- **`coarse_search_angle_offset`**: 粗糙搜索的角度范围，例如 `0.349`。
- **`coarse_angle_resolution`**: 粗糙搜索的角度分辨率，例如 `0.0349`。
- **`minimum_angle_penalty`**: 根据相关性匹配计算的角度偏差惩罚系数，例如 `0.9`。
- **`minimum_distance_penalty`**: 根据相关性匹配计算的位移偏差惩罚系数，例如 `0.5`。
- **`use_response_expansion`**: 在匹配中增加一倍的分辨率来寻找响应峰值。
- **`do_loop_closing`**: 是否开启闭环检测，建图时强烈建议开启（`true`）以消除累积误差。
- **`loop_match_minimum_chain_size`**: 确认闭环前必须匹配的最小扫描链长度，例如 `10`。
- **`loop_match_maximum_variance_coarse`**: 粗糙闭环匹配的最大方差，例如 `3.0`。
- **`loop_match_minimum_response_coarse`**: 粗糙闭环匹配的最小响应值，例如 `0.35`。
- **`loop_match_minimum_response_fine`**: 精细闭环匹配的最小响应值，例如 `0.45`。

---

## 🧭 自动导航 (Nav2)
项目现已集成 `Nav2` (Navigation 2) 用于小车的自主路径规划与避障导航。

### 1️⃣ 环境准备
请确保已安装 ROS 2 Humble 的 `Nav2` 及其相关依赖。如果没有安装，可通过以下命令安装：
```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

### 2️⃣ 启动导航
在运行导航前，请确保底层节点（如：`bringup.launch.py`）正在运行，并且已经使用上文的方法建好并保存了一张地图（如 `my_map.yaml` 和 `my_map.pgm` 位于当前根目录或者指定位置）。

新开一个终端，执行：
```bash
source install/setup.bash
# 默认会加载当前目录下的 my_map.yaml 文件作为已知地图
ros2 launch ylhb_base navigation.launch.py
```
*如果你有多个地图，或者想指定其他路径的地图文件，可通过此命令行参数进行覆盖：*
*`ros2 launch ylhb_base navigation.launch.py map:=/绝对路径直到/你的地图.yaml`*

### 3️⃣ 使用可视化工具下发导航指令 (RViz2 / Foxglove)
因为 `Nav2` 的控制底层是基于 ROS 2 的标准话题通信，因此你既可以使用传统的 `RViz2` 也可以使用现代化的 `Foxglove Studio` 来下发导航指令。需要注意，根据你当前是**“边建图边导航”**还是**“已知静态地图下导航”**，下发指令的步骤会有所不同。

#### 场景A: SLAM 建图模式下的导航 (建图的同时想让小车自动探索/走目标点)
在这种模式下，地图和坐标系 `map` 是由 `slam_toolbox` 实时发布并维持的。**这个时候千万不要去发 `/initialpose`**，因为定位主导权在 SLAM，不在 AMCL。
1. **启动程序**：先运行底盘启动 `bringup.launch.py`，再运行建图 `mapping.launch.py`，最后启动无AMCL的Nav2（此指令针对自定义launch配置或者单独跑规划器）。
2. **在 Foxglove / RViz2 中确认连通**：查看 `/map` (地图)、`/scan` (雷达点云)、添加 `TF` 可视化确认 `map -> odom -> base_footprint -> laser_link` 链路存在没有断裂。
3. **下发目标点**：在确认建出部分地图以及位姿正常后，直接往 `/goal_pose` 话题发布 `geometry_msgs/PoseStamped` 消息赋予目的地。小车就会在建图的同时自动避图规划！

#### 场景B: 静态地图纯导航模式 (也就是刚才保存了地图，现在用 `navigation.launch.py`)
这是最经典的模式，基于 `map_server` 加载固定地图，并由 `AMCL` 来主导蒙特卡洛粒子滤波定位。
为实现**“上电即用”**（无需人工干预的纯自动化要求，非常适合比赛）：
1. **统一物理起点建图法**：
   - 强烈建议在建图时，每次都将小车放置在固定的起始点（如场地某一角落）作为 `[0,0,0]` 原点向同一方向启动。
   - 之后进行导航时，也必须将小车放置在同样的物理位置启动。目前 `nav2_params.yaml` 中的 `set_initial_pose: true` 已配置好了默认坐标 `[0,0,0]`，因此无需人工再用 Foxglove / RViz 发送初始大概位姿了。
2. **自动化粒子收敛对齐（已集成至程序）**：
   - 启动 `navigation.launch.py` 后，你会发现小车等待 10 秒（为了等待 AMCL 和周围节点加载完成）后，会**自动在原地顺时针旋转一圈**！
   - 这是专门为全自动定位编写的自适应匹配程序（位于 `scripts/auto_align.py`）。通过自旋转让雷达快速采集一圈墙壁特征反馈给 AMCL 算法进行高精度收敛，“咔哒”一声实现自动贴合并消除摆放的几厘米与细微偏差。
3. **下发目标点位 (Nav2 Goal)**：
   - 当小车自转停下后（此时终端提示对齐完成），比赛的上位机/流程代码即可立刻往 `/goal_pose` 话题发布 `geometry_msgs/msg/PoseStamped` 消息规划路线。全流程无需人工用鼠标干预。

### 4️⃣ Nav2 核心配置文件解析 (nav2_params.yaml)
针对目前底盘在 `/src/ylhb_base/config/nav2_params.yaml` 设定了一些核心参数：
- **`robot_model_type: "differential"`**：在 AMCL 定位中指定底盘是差速模型。
- **`base_frame_id: "base_footprint"`**：指定基座坐标系，与 URDF 一致。
- **`max_vel_x` / `max_vel_theta`**：局部规划器 DWB 中设定最大线速度和角速度，保障物理上限安全。
- **`footprint`**：配置的 `"[ [0.15, 0.15], [0.15, -0.15], [-0.15, -0.15], [-0.15, 0.15] ]"` 是用于建立代价地图 (Costmap) 计算碰撞的虚拟外框尺寸（假设小车是一台宽30cm、长30cm的方块），如果有改变需要及时修正以便更好地穿过狭窄走廊。

---

## ⚡ 重点及后续维护注意项
1. **统一 Udev 挂载固化 (防报错，比赛必备)**
   因为使用多个 USB 转串口时 (`/dev/ttyUSB0` 和 `/dev/ttyUSB1`)，每次开机系统给设备分配的名字是不固定的。如果直接使用默认配置，常常会导致驱动把雷达识别为 IMU，导致 Timeout 报错卡死系统。
   - **解决方式：运行固化脚本**。项目根目录下已提供 `bind_usb.sh`，在连接上雷达和 IMU 的状态下运行 `sudo ./bind_usb.sh`。它会将底层硬件的供应商芯片代码（比如 CP210x 和 CH340）强行改写到 `udev` 规则中。
   - 配置后，雷达会永远绑定成 `/dev/robot_lidar`，IMU 会永远叫 `/dev/robot_imu`。不管以后怎么开机，随便拔插都能启动。对应的修改已经写在 `bringup.launch.py` 中。
2. **频率配置**
   底盘状态循环大约 `20ms` （50Hz）触发一次上报发布。IMU 设定的读取循环为 `5ms` (200Hz)。在 `ekf.yaml` 中如果传感器频率调整，需同步修改 EKF 的订阅预设帧率以防止延迟抖动。
3. **坐标系与传感器安装标定约定 (前-左-上)**
   项目严格遵循 ROS 标准的 REP-103 规范建立前左上坐标系（X轴指向正前方，Y轴指向正左方，Z轴指向上方）。所有外接传感器的安装平移和旋转校准必须以此标准为参考，并写入 `ylhb.urdf.xacro` 文件中：
   - **里程计与车体方向**：小车实际前进方向为 X 轴正方向，原地左转为 Yaw 轴正方向（逆时针为正）。
   - **激光雷达 (RPLidar A2)**：安装于车体正前方 15.2cm (`xyz="0.152 0 0.172"`)。由于物理安装时，使其自身数据坐标系的 Y 轴对准了车头正方向（这也意味着属于雷达自己的 X 轴正指着小车的右方），因此在 URDF 配置中附带了顺时针 90 度的安装偏移量（`-1.5707963 弧度`），声明为：`rpy="0 0 -1.5707963"`。
   - **IMU 传感器**：安装于车体正后方 9mm 处 (`xyz="-0.009 0 0.061"`)，由于与主方向一致，朝向角无需修正，记为 `rpy="0 0 0"`。
### 6️⃣ AI 视觉与图像推流节点 (`vision_node`)
- **功能**：基于旭日 X5 (RDK X5) 和 USB 摄像头。该节点使用极低延迟的底层 `GStreamer` 原生推流管道 (MJPEG -> 解码 -> H264 编码 -> UDP)，将采集的画面推流至 PC 端，以支持后续的 AI 目标检测。在发送推流的同时，节点还会兼做 UDP 服务器，用以接收局域网内 PC 回传的视觉解析结果（坐标与类别）。
- **摄像头设备**：推荐支持 MJPEG 的 `/dev/video0`
- **默认分辨率/帧率**：`1280x720` @ `30FPS`
- **网络通信机制**：
  - 📡 **UDP 推流**: 发送给给定的 `pc_ip:5000` (单向推流)。
  - 📥 **UDP 接收**: 监听来自 PC 端本地 `5001` 端口下发的 JSON 结果请求。
- **ROS 2 通信话题**：
  - 📤 **发布者 (Publishes)**:
    - 话题: `/vision/result`
    - 消息类型: `std_msgs/msg/String`
    - 用途: 接收 PC 处理返回的纯 JSON 文本（状态与类别等信息，全状态透传供上层状态机调用）。
    - 话题: `/vision/target_pose`
    - 消息类型: `geometry_msgs/msg/PoseStamped`
    - 用途: 解析 JSON 数据中的 `x，y，z` 信息并封装成 ROS 标准位姿发送给机械臂抓取或者底盘导航规划；包含 `camera_link` 附着的 TF 标准。
- **预设参数**：
  - `pc_ip`: UDP视频流接收电脑的 IP，默认 `192.168.137.1`。
  - `video_udp_port`: 视频流发送目标电脑端口，默认 `5000`。
  - `result_udp_port`: 板端监听回传 UDP 的端口，默认 `5001`。
  - `camera_id`: 挂载摄像头地址，默认 `/dev/video0`。

---

## 📊 可视化神器 (Foxglove)
本项目推荐使用可视化神器 **Foxglove Studio** 进行数据的观测与调试（实时查看雷达点云、地图分布、TF树及各项话题数据等）。

### 启动 Foxglove Bridge
要开启对 Foxglove 的支持，只需在新终端中执行以下快捷指令将数据进行桥接：
```bash
run_bridge
```
*备注：执行该指令后，会启动 Foxglove 的 WebSocket 服务。之后即可在同一局域网内的 PC 端，使用 Foxglove 客户端直连底盘/设备的 IP，进行沉浸式可视化调试和交互。*
