cmake_minimum_required(VERSION 3.22)

# 1. 项目基础设置
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_PROJECT_NAME FDCAN-TEST-G4)
project(${CMAKE_PROJECT_NAME})

# 启用 C 和 汇编语言支持
enable_language(C ASM)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# 2. 调试/发布模式差异化配置
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# --------------------------
# Debug/Release 差异化优化
# --------------------------
# Debug：优先保证调试性，关闭LTO，保留完整调试信息，轻度优化
set(CMAKE_C_FLAGS_DEBUG "-Og -g3 -fno-omit-frame-pointer -fno-lto")
# Release：极致性能，利用FPU/DSP，开启LTO和循环优化，兼顾精度
set(CMAKE_C_FLAGS_RELEASE "-O3 -ffast-math -funroll-loops -frename-registers -finline-small-functions")

# ------------------------------------------------------------------------------
# 3. 硬件架构参数
# ------------------------------------------------------------------------------
set(CPU_FLAGS
        -mcpu=cortex-m4
        -mthumb
        -mfpu=fpv4-sp-d16
        -mfloat-abi=hard    # 硬浮点ABI，最大化FPU效率
)

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/DSP/Lib/GCC)

# ------------------------------------------------------------------------------
# 4. 编译选项
# ------------------------------------------------------------------------------
add_compile_options(
        ${CPU_FLAGS}
        # 基础警告（保留关键，关闭无关）
        -Wall
        -Wextra
        -Wno-comment          # 关闭注释嵌套警告
        -Wno-unused-parameter # 关闭FreeRTOS未使用参数警告
        -Wno-unused-but-set-variable # 关闭未使用变量警告
        -Werror=implicit-function-declaration # 隐式声明转为错误（防止调用未声明函数）
        -Werror=incompatible-pointer-types    # 指针类型不匹配转为错误（避免HardFault）
        # 代码优化
        -fdata-sections
        -ffunction-sections
        # DSP/FPU 优化
        -DARM_MATH_CM4
        -D__FPU_PRESENT=1U
        -DARM_MATH_MATRIX_CHECK=0  # 关闭DSP库矩阵越界检查（提升速度）
        -DARM_MATH_ROUNDING=0      # 关闭DSP库舍入（提升速度）
)

# ------------------------------------------------------------------------------
# 5. 链接选项（精细化控制 + 性能优化）
# ------------------------------------------------------------------------------
add_link_options(
        ${CPU_FLAGS}
        -Wl,--gc-sections                     # 移除未使用段
        -Wl,--print-memory-usage              # 打印内存占用
        -Wl,--relax                           # 链接松弛（减少代码大小，提升速度）
        -Wl,--sort-common=descending          # 排序公共符号（减少内存碎片）
        # LTO：仅Release开启，Debug关闭（通过CMAKE_EXE_LINKER_FLAGS控制）
)

# --------------------------
# 关键修改2：LTO 差异化控制
# --------------------------
# Debug：关闭LTO（避免调试符号混淆）
set(CMAKE_EXE_LINKER_FLAGS_DEBUG "-fno-lto")
# Release：开启LTO并自动并行（提升编译速度+代码优化）
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "-flto=auto")

# ------------------------------------------------------------------------------
# 6. 可执行文件定义
# ------------------------------------------------------------------------------
add_executable(${CMAKE_PROJECT_NAME}
        User/App/Src/All_Task.c
        User/Utils/Inc/Vofa.h
        User/Utils/Src/Vofa.c
        User/Device/Inc/DBUS.h
        User/Device/Src/DBUS.c
        User/App/Inc/All_Task.h
        User/Algorithm/Inc/controller.h
        User/Algorithm/Src/controller.c
        User/BSP/Inc/BSP_DWT.h
        User/BSP/Src/BSP_DWT.c
        User/BSP/Inc/BSP_ICM42688P.h
        User/BSP/Src/BSP_ICM42688P.c
        User/BSP/Inc/WS2812.h
        User/BSP/Src/WS2812.c
        User/Algorithm/Inc/CKQ_MATH.h
        User/Algorithm/Src/CKQ_MATH.c
        User/App/Src/IMU_Task.c
        User/App/Inc/IMU_Task.h
        User/Algorithm/Inc/kalman_filter.h
        User/Algorithm/Src/kalman_filter.c
        User/Algorithm/Inc/QuaternionEKF.h
        User/Algorithm/Src/QuaternionEKF.c
        User/Utils/Inc/user_lib.h
        User/Utils/Src/user_lib.c
        User/Algorithm/Inc/mahony_filter.h
        User/Algorithm/Src/mahony_filter.c
        User/Utils/Inc/All_Init.h
        User/Utils/Src/All_Init.c
        User/Utils/Inc/All_define.h
        # 启动文件
        startup_stm32g473xx.s
        User/Device/Inc/DJI_Motor.h
        User/Device/Inc/DM_Motor.h
        User/Device/Src/DJI_Motor.c
        User/Device/Src/DM_Motor.c
        User/App/Src/System_Status.c
        User/App/Inc/System_Status.h
        User/Utils/Inc/All_Motor.h
        User/Device/Inc/Power_CAP.h
        User/Device/Src/Power_CAP.c
        User/BSP/Inc/BSP_W25N01GV.h
        User/BSP/Src/BSP_W25N01GV.c
        User/App/Inc/Chassis_Task.h
        User/App/Src/Chassis_Task.c
)

# 添加用户源文件
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        Core/Src/main.c
        Core/Src/gpio.c
        Core/Src/fdcan.c
        Core/Src/tim.c
        Core/Src/stm32g4xx_it.c
        Core/Src/stm32g4xx_hal_msp.c
        Core/Src/sysmem.c
        Core/Src/syscalls.c
        User/BSP/Src/BSP_FDCAN.c
)

# 添加包含路径
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        Core/Inc/
        User/App/Inc
        User/BSP/Inc
        User/Device/Inc
        User/Algorithm/Inc
        User/Utils/Inc
        Drivers/CMSIS/Include
        Drivers/CMSIS/DSP/Include
        Drivers/CMSIS/Device/ST/STM32G4xx/Include
)

# ------------------------------------------------------------------------------
# 7. 链接 STM32CubeMX 生成的 HAL 库
# ------------------------------------------------------------------------------
add_subdirectory(cmake/stm32cubemx)

target_link_libraries(${PROJECT_NAME}
        stm32cubemx
        arm_cortexM4lf_math  # DSP库
        m                    # 数学库
)

# ------------------------------------------------------------------------------
# 8. 宏定义（强化DSP/FPU）
# ------------------------------------------------------------------------------
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        STM32G473xx
        USE_HAL_DRIVER
        ARM_MATH_CM4
        __FPU_PRESENT=1U
        HAL_ASSERT_DISABLE=1
        # 开启DSP库快速模式（牺牲精度换速度）
        # ARM_MATH_FAST_MATH=1
)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE DEBUG_MODE)
else()
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE RELEASE_MODE)
endif()

# ------------------------------------------------------------------------------
# 9. 生成辅助文件 (Hex, Bin)
# ------------------------------------------------------------------------------
set(HEX_FILE ${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin)

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}\nBuilding ${BIN_FILE}"
)