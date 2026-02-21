G4_Framework 主控框架（STM32G473VET6）

## 项目概述
本项目是基于 STM32G473VET6 的分层主控框架，使用 FreeRTOS，提供硬件抽象与应用层支持，适用于 RoboMaster 类机器人。

## 开发环境
- MCU: STM32G473VET6
- IDE: CLion + CMake
- RTOS: FreeRTOS
- Toolchain: GCC ARM None EABI
- Debug: OpenOCD + GDB + Ozone

## 目录结构

### 根目录配置
```
G4_Framework/
├── CMakeLists.txt
├── CMakePresets.json
├── G4_Framework.ioc
├── STM32G473XX_FLASH.ld
├── startup_stm32g473xx.s
├── cmake/
├── Core/
├── Drivers/
├── Middlewares/
├── User/
└── README.md
```

### Core - HAL 层
由 STM32CubeMX 生成的 HAL 初始化、时钟配置、中断处理和系统启动相关代码。

```
Core/
├── Inc/
└── Src/
```

### Drivers - 驱动库
ARM CMSIS 接口与 STM32G4 HAL 驱动库。

```
Drivers/
├── CMSIS/
└── STM32G4xx_HAL_Driver/
```

### Middlewares - 中间件层
包含 ST 与第三方中间件目录（如 FreeRTOS）。

```
Middlewares/
├── ST/
└── Third_Party/
```

---

### User - 应用代码（主要开发区）
User 目录按模块分层组织，便于维护和扩展。

#### 1) Algorithm 算法层
无硬件依赖的算法实现，便于移植。

```
User/Algorithm/
├── Inc/
│   ├── CKQ_MATH.h                     # 自封装的数学库
│   ├── controller.h                   # 控制算法接口（如 PID）
│   ├── kalman_filter.h                # 卡尔曼滤波接口
│   ├── mahony_filter.h                # Mahony 姿态滤波接口
│   └── QuaternionEKF.h                # 四元数 EKF 接口
└── Src/
    ├── CKQ_MATH.c                     # 数学库实现
    ├── controller.c                   # 控制算法实现
    ├── kalman_filter.c                # 卡尔曼滤波实现
    ├── mahony_filter.c                # Mahony 姿态滤波实现
    └── QuaternionEKF.c                # 四元数 EKF 实现
```

---

#### 2) Utils 工具层
通用工具与全局定义。

```
User/Utils/
├── Inc/
│   ├── All_define.h                   # 全局宏与类型定义
│   ├── All_Init.h                     # 系统初始化接口
│   ├── All_Motor.h                    # 统一电机接口
│   ├── user_lib.h                     # 通用工具库接口
│   └── Vofa.h                         # Vofa+ 通讯接口
└── Src/
    ├── All_Init.c                     # 系统初始化实现
    ├── user_lib.c                     # 通用工具库实现
    └── Vofa.c                         # Vofa+ 通讯实现
```

---

#### 3) BSP 板级支持层
对 HAL 的轻量封装，提供更易用的硬件访问接口。

```
User/BSP/
├── Inc/
│   ├── BSP-FDCAN.h                    # FDCAN 板级接口
│   ├── BSP_DWT.h                      # DWT 计时接口
│   ├── BSP_QSPI.h                     # QSPI 板级接口
│   ├── BSP_SPI.h                      # SPI 板级接口
│   └── WS2812.h                       # WS2812 灯带接口
└── Src/
    ├── BSP_DWT.c                      # DWT 计时实现
    ├── BSP_FDCAN.c                    # FDCAN 板级实现
    ├── BSP_QSPI.c                     # QSPI 板级实现
    ├── BSP_SPI.c                      # SPI 板级实现
    └── WS2812.c                       # WS2812 灯带实现
```

---

#### 4) Device 设备层
外设与模块驱动、协议解析与控制接口。

```
User/Device/
├── Inc/
│   ├── DBUS.h                         # 遥控接收接口
│   ├── DJI_Motor.h                    # DJI 电机接口
│   ├── DM_Motor.h                     # 达妙电机接口
│   ├── ICM42688P.h                    # ICM42688P 设备接口
│   ├── Power_CAP.h                    # 超级电容接口
│   └── W25N01GV.h                     # W25N01GV Flash 接口
└── Src/
    ├── DBUS.c                         # 遥控接收实现
    ├── DJI_Motor.c                    # DJI 电机实现
    ├── DM_Motor.c                     # 达妙电机实现
    ├── ICM42688P.c                    # ICM42688P 设备实现
    ├── Power_CAP.c                    # 超级电容实现
    └── W25N01GV.c                     # W25N01GV Flash 实现
```

---

#### 5) Middleware 中间件层
系统级数据处理与状态汇总。

```
User/Middleware/
├── Inc/
│   ├── CAN_Comm.h                     # CAN 通讯与消息定义
│   └── System_Status.h                # 系统状态机接口
└── Src/
    └── System_Status.c                # 系统状态机实现
```

---

#### 6) App 应用层
业务逻辑与任务调度层。

```
User/App/
├── Inc/
│   ├── All_Task.h                     # 任务调度与系统状态相关定义
│   ├── Chassis_Task.h                 # 底盘任务接口
│   └── IMU_Task.h                     # IMU 任务接口
└── Src/
    ├── All_Task.c                     # 任务创建与调度实现
    ├── Chassis_Task.c                 # 底盘任务实现
    └── IMU_Task.c                     # IMU 任务实现
```

---

## 分层依赖关系
```
App
 ↓ 使用
Device + Algorithm + Middleware
 ↓ 使用
BSP
 ↓ 使用
HAL
```

Utils 可被各层调用。