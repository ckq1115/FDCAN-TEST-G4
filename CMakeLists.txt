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

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()
# 强制开启优化选项
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-Og) # -Og 是专门为调试设计的优化，比 -O0 快得多
else()
    add_compile_options(-Ofast) # 生产环境追求极致速度
endif()
# ------------------------------------------------------------------------------
# 2. 硬件架构参数 (核心配置)
# STM32G473 使用 Cortex-M4F，FPU 类型为 fpv4-sp-d16
# ------------------------------------------------------------------------------
set(CPU_FLAGS
        -mcpu=cortex-m4
        -mthumb
        -mfpu=fpv4-sp-d16
        -mfloat-abi=hard
        -flto
)

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/DSP/Lib/GCC)

# 3. 编译选项
add_compile_options(
        ${CPU_FLAGS}
        -Wall
        -Wextra
        -fdata-sections
        -ffunction-sections
)

# 4. 链接选项 (修复重复报错的关键)
# 注意：我们去掉了 -specs=nano.specs，因为它在 stm32cubemx 子目录中通常已经被定义了
add_link_options(
        ${CPU_FLAGS}
        -Wl,--gc-sections
        -Wl,--print-memory-usage
        -flto
)

# ------------------------------------------------------------------------------
# 5. 可执行文件定义
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
)

# 添加用户源文件 (请根据实际文件增减)
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
        User/Device/Src/Motor.c
        # 启动文件通常在 stm32cubemx 内部处理，或者手动添加如下：
        startup_stm32g473xx.s
)

# 添加包含路径
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        Core/Inc/
        User/App/Inc
        User/BSP/Inc
        User/Device/Inc
        User/Algorithm/Inc
        User/Utils/Inc
        # --- 新增 DSP 相关路径 ---
        Drivers/CMSIS/Include
        Drivers/CMSIS/DSP/Include
        Drivers/CMSIS/Device/ST/STM32G4xx/Include
)

# ------------------------------------------------------------------------------
# 6. 链接 STM32CubeMX 生成的 HAL 库
# ------------------------------------------------------------------------------
add_subdirectory(cmake/stm32cubemx)

target_link_libraries(${PROJECT_NAME}
        stm32cubemx
        # --- 新增 DSP 静态库和数学库 ---
        arm_cortexM4lf_math
        m
)

# ------------------------------------------------------------------------------
# [修改] 7. 宏定义 (添加 DSP 所需宏)
# ------------------------------------------------------------------------------
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        STM32G473xx
        USE_HAL_DRIVER
        # --- 新增 DSP 宏 ---
        ARM_MATH_CM4
        __FPU_PRESENT=1U
)

# 移除冗余库依赖（防止某些 GCC 版本链接冲突）
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# ------------------------------------------------------------------------------
# 8. 生成辅助文件 (Hex, Bin)
# ------------------------------------------------------------------------------
set(HEX_FILE ${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin)

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}\nBuilding ${BIN_FILE}"
)