# ROS2-smart-car

ROS2 智能车项目，采用单仓库双目录结构：

- `rdk_x5_ros2/`：RDK X5 侧 ROS2 / 感知 / 导航 / 上位机逻辑
- `stm32_chassis/`：STM32 侧底盘控制 / 电机驱动 / 通信协议实现

## 分支约定

- `main`：稳定主分支
- `feature/rdk-x5`：RDK X5 开发分支
- `feature/stm32`：STM32 开发分支

## 建议目录结构

```text
ROS2-smart-car/
├─ README.md
├─ .gitignore
├─ docs/
│  └─ protocol/
├─ rdk_x5_ros2/
│  ├─ src/
│  ├─ launch/
│  ├─ config/
│  └─ scripts/
└─ stm32_chassis/
   ├─ Core/
   ├─ Drivers/
   ├─ BSP/
   ├─ App/
   └─ Protocol/
```
