# ROS2-smart-car

ROS2 智能车项目，采用单仓库双目录结构：

- `rdk_x5_ros2/`：RDK X5 侧 ROS2 / 感知 / 导航 / 上位机逻辑
- `stm32_chassis/`：STM32 侧底盘控制 / 电机驱动 / 通信协议实现

## 分支约定

- `main`：稳定主分支
- `feature/rdk-x5`：RDK X5 开发分支
- `feature/stm32`：STM32 开发分支

## 目录结构

```text
ROS2-smart-car/
├─ README.md
├─ .gitignore
├─ rdk_x5_ros2/
│  └─ README.md
└─ stm32_chassis/
   ├─ Core/
   ├─ Drivers/
   ├─ MDK-ARM/
   ├─ .mxproject
   ├─ YLHB_ROS.ioc
   └─ README.md
```

## STM32 底盘部分

`stm32_chassis/` 目录保存底盘控制器工程，当前基于 `STM32F103`，用于完成小车底盘侧的实时控制与硬件驱动。

### 当前能力

- 电机 PWM 输出与方向控制
- 编码器测速与轮速反馈
- 左右轮增量式 PID 闭环调速
- 通过 `USART2` 与 RDK X5 进行控制指令和速度反馈通信
- 基于 CubeMX 生成工程，并保留 Keil / EIDE 开发配置

### 代码入口

- `stm32_chassis/Core/Src/main.c`：主控制流程、串口协议、定时调度
- `stm32_chassis/Core/mk/motor.c`：电机控制和 PID 逻辑
- `stm32_chassis/Core/mk/bmq.c`：编码器计数读取
- `stm32_chassis/MDK-ARM/`：Keil / EIDE 工程、启动文件、调试配置

### 通信说明

当前 STM32 底盘与 RDK X5 之间采用串口协议通信，控制与反馈链路已经接入 `USART2`。协议解析和反馈打包逻辑集中在 `stm32_chassis/Core/Src/main.c` 中，便于后续和 ROS2 上位机节点对接。

