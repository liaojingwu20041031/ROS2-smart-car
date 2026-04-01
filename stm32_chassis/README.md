# stm32_chassis

STM32 底盘控制工程，包含电机驱动、编码器测速、串口通信协议，以及基于 STM32F103 的底盘控制逻辑。

## 目录说明

- `Core/`：应用代码、HAL 初始化和中断处理
- `Drivers/`：STM32 HAL 与 CMSIS 驱动
- `MDK-ARM/`：Keil / EIDE 工程文件、启动文件与调试配置
- `YLHB_ROS.ioc`：CubeMX 工程配置

## 当前通信链路

当前底盘通过 `USART2` 与 RDK X5 通信，控制命令和速度反馈协议在 `Core/Src/main.c` 中实现。
