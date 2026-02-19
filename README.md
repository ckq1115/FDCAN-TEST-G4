RoboMaster STM32G473VET6 主控代码框架

## 项目简介
本项目是用于RoboMaster比赛的STM32G473VET6主控板代码框架，采用分层架构设计，使用FreeRTOS实时操作系统，提供完整的硬件抽象和应用开发支持。

## 开发环境
- **MCU**: STM32G473VET6
- **IDE**: CLion with CMake
- **RTOS**: FreeRTOS
- **工具链**: GCC ARM None EABI
- **调试**: OpenOCD + GDB + Ozone

## 目录结构

### 📁 核心配置文件
```
FDCAN-TEST-G4/
├── CMakeLists.txt                      # 主CMake配置文件
├── CMakePresets.json                   # CMake预设配置
├── FDCAN-TEST-G4.ioc                   # STM32CubeMX工程文件
├── STM32G473XX_FLASH.ld                # 链接脚本
├── startup_stm32g473xx.s               # 启动文件
└── G4一键启动OpenOCD.bat               # OpenOCD启动脚本
```

### 📁 Core - HAL底层代码
STM32CubeMX生成的HAL库配置和初始化代码，包括外设初始化、中断处理、系统配置等。

### 📁 Drivers - 驱动库
ARM CMSIS标准接口和STM32G4 HAL驱动库。

### 📁 Middlewares - 中间件
FreeRTOS实时操作系统源码和ST官方中间件。

---

### 📁 User - 用户应用代码（核心开发目录）

**User目录是本框架的核心开发区域，采用分层架构设计，便于代码管理和团队协作。**

#### 1️⃣ Algorithm - 算法层
**功能定位**: 纯算法实现，与硬件无关，可移植性强

```
Algorithm/
├── Inc/
│   ├── CKQ_MATH.h                     # 数学工具函数（三角函数、限幅、映射等）
│   ├── controller.h                   # 控制器算法（PID、前馈控制等）
│   ├── kalman_filter.h                # 卡尔曼滤波算法
│   ├── mahony_filter.h                # Mahony互补滤波算法
│   └── QuaternionEKF.h                # 四元数扩展卡尔曼滤波（姿态解算）
└── Src/
    ├── CKQ_MATH.c                     # 数学工具函数实现
    ├── controller.c                   # 控制器算法实现
    ├── kalman_filter.c                # 卡尔曼滤波实现
    ├── mahony_filter.c                # Mahony滤波实现
    └── QuaternionEKF.c                # 四元数EKF实现
```

**模块说明**:
- **CKQ_MATH**: 提供常用数学运算函数，如角度归一化、限幅、斜坡函数等
- **controller**: 实现PID控制器、前馈控制等控制算法
- **kalman_filter**: 卡尔曼滤波器，用于传感器数据融合和状态估计
- **mahony_filter**: Mahony互补滤波算法，适用于IMU姿态解算
- **QuaternionEKF**: 基于四元数的扩展卡尔曼滤波，用于高精度姿态估计

---

#### 2️⃣ Utils - 工具层
**功能定位**: 通用工具函数和全局定义，提供基础功能支持

```
Utils/
├── Inc/
│   ├── All_define.h                   # 全局宏定义和常量定义
│   ├── All_Init.h                     # 系统初始化函数声明
│   ├── All_Motor.h                    # 电机统一接口定义
│   ├── user_lib.h                     # 用户通用库函数
│   └── Vofa.h                         # Vofa+上位机调试工具
└── Src/
    ├── All_Init.c                     # 系统初始化实现
    ├── user_lib.c                     # 用户库函数实现
    └── Vofa.c                         # Vofa+数据发送实现
```

**模块说明**:
- **All_define.h**: 包含全局使用的宏定义、枚举类型、常量等
- **All_Init**: 系统外设和模块的统一初始化接口
- **All_Motor**: 电机统一控制接口，便于上层调用
- **user_lib**: 提供常用的工具函数，如数据处理、类型转换等
- **Vofa**: 对接Vofa+上位机，实现实时数据可视化和调试

---

#### 3️⃣ BSP - 板级支持层
**功能定位**: 对HAL库的二次封装，提供更友好的硬件操作接口

```
BSP/
├── Inc/
│   ├── BSP-FDCAN.h                    # FDCAN(CAN-FD)总线驱动
│   ├── BSP_DWT.h                      # DWT高精度延时和计时
│   ├── BSP_ICM42688P.h                # ICM42688P六轴IMU驱动
│   ├── BSP_W25N01GV.h                 # W25N01GV Flash存储器驱动
│   └── WS2812.h                       # WS2812 RGB LED驱动
└── Src/
    ├── BSP_FDCAN.c                    # FDCAN收发和过滤器配置
    ├── BSP_DWT.c                      # DWT精确延时实现
    ├── BSP_ICM42688P.c                # ICM42688P通信和数据读取
    ├── BSP_W25N01GV.c                 # W25N01GV读写擦除操作
    └── WS2812.c                       # WS2812 PWM+DMA驱动
```

**模块说明**:
- **BSP_FDCAN**: 封装FDCAN(CAN-FD)通信，支持标准帧和扩展帧，实现过滤器配置和中断接收
- **BSP_DWT**: 利用ARM Cortex-M内核的DWT单元实现微秒级精确延时和性能测量
- **BSP_ICM42688P**: 高性能6轴IMU传感器驱动，支持SPI通信和DMA传输
- **BSP_W25N01GV**: 1Gbit SPI NAND Flash驱动，用于数据日志存储
- **WS2812**: RGB LED灯带驱动，使用PWM+DMA实现，用于状态指示

---

#### 4️⃣ Device - 设备驱动层
**功能定位**: 外部设备和传感器的驱动程序，实现通信协议和控制接口

```
Device/
├── Inc/
│   ├── DBUS.h                         # 遥控器接收机（DR16等）
│   ├── DJI_Motor.h                    # 大疆电机（M3508/M2006/GM6020）
│   ├── DM_Motor.h                     # 达妙电机（DM-J4310等）
│   └── Power_CAP.h                    # 超级电容管理模块
└── Src/
    ├── DBUS.c                         # 遥控器SBUS协议解析
    ├── DJI_Motor.c                    # 大疆电机CAN通信和控制
    ├── DM_Motor.c                     # 达妙电机MIT模式通信
    └── Power_CAP.c                    # 超级电容CAN通信和功率管理
```

**模块说明**:
- **DBUS**: 遥控器接收机驱动，解析SBUS协议，获取遥控器通道值和按键状态
- **DJI_Motor**: 大疆RoboMaster电机驱动，支持M3508、M2006、GM6020等型号，实现CAN通信和电流/速度/位置控制
- **DM_Motor**: 达妙电机驱动，支持MIT模式和位置/速度/力矩混合控制
- **Power_CAP**: 超级电容模块驱动，实现功率限制、电压监测和充放电管理

---

#### 5️⃣ App - 应用层
**功能定位**: 业务逻辑实现，FreeRTOS任务和机器人功能模块

```
App/
├── Inc/
│   ├── All_Task.h                     # 所有任务的统一头文件
│   ├── IMU_Task.h                     # IMU数据采集和姿态解算任务
│   └── System_Status.h                # 系统状态监控和管理
└── Src/
    ├── All_Task.c                     # 任务创建和调度管理
    ├── IMU_Task.c                     # IMU任务实现（数据读取、滤波、解算）
    └── System_Status.c                # 系统状态机和错误处理
```

**模块说明**:
- **All_Task**: 统一管理所有FreeRTOS任务的创建、删除和调度
- **IMU_Task**: IMU数据采集任务，周期性读取IMU数据并进行姿态解算，为控制提供姿态信息
- **System_Status**: 系统状态监控，包括设备在线检测、错误诊断、模式切换等

---

## User目录分层设计理念

### 📊 依赖关系
```
App (应用层)
 ↓ 调用
Device (设备层) + Algorithm (算法层)
 ↓ 调用
BSP (板级支持层)
 ↓ 调用
HAL (硬件抽象层)
```

**Utils工具层可被各层调用**

### ✨ 设计优势
- **模块化**: 各层职责清晰，便于团队分工协作
- **可移植**: Algorithm和Utils层不依赖硬件，易于移植到其他平台
- **可维护**: 分层结构降低耦合度，修改某一层不影响其他层
- **可扩展**: 新增设备或功能只需在对应层添加模块

---

## 主要功能特性
- ✅ **高速通信**: 支持FDCAN(CAN-FD)，通信速率可达5Mbps
- ✅ **电机控制**: 支持大疆M3508/M2006/GM6020和达妙DM系列电机
- ✅ **姿态解算**: 基于ICM42688P的高精度姿态估计（卡尔曼/Mahony滤波）
- ✅ **遥控接收**: 支持DR16等SBUS协议遥控器
- ✅ **功率管理**: 超级电容功率控制和裁判系统功率限制
- ✅ **数据存储**: 板载1Gbit Flash，支持数据日志记录
- ✅ **状态指示**: WS2812 RGB灯效，实时显示系统状态
- ✅ **实时调试**: Vofa+上位机实时波形显示和参数调节

## 编译说明
### Debug版本
```bash
cmake --preset Debug
cmake --build --preset Debug
```

### Release版本
```bash
cmake --preset Release
cmake --build --preset Release
```

## 烧录调试
使用提供的`G4一键启动OpenOCD.bat`脚本启动OpenOCD进行调试。

