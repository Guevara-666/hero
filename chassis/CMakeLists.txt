cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME C_project)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# ============================================================================
# CMSIS-DSP：作为子项目加入，生成 target `CMSISDSP`
# ============================================================================
# 告诉 CMSIS-DSP 去哪里找 CMSIS Core 的头文件（可选，但推荐）
set(CMSISCORE ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/components/CMSIS-DSP/Source
                 ${CMAKE_CURRENT_BINARY_DIR}/CMSISDSP)

# ============================================================================
# 用户源文件
# ============================================================================
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Tasks
    ${CMAKE_CURRENT_SOURCE_DIR}/Tasks/Src/main_task.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Tasks/Src/imu_task.cpp

    # Resources
    ${CMAKE_CURRENT_SOURCE_DIR}/Resources/Src/dm4310_drv.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Resources/Src/DT7.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Resources/Src/HW_can.cpp

    # components: ahrs
    ${CMAKE_CURRENT_SOURCE_DIR}/components/ahrs/src/ahrs_base.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/components/ahrs/src/mahony.cpp

    # components: imu
    ${CMAKE_CURRENT_SOURCE_DIR}/components/imu/src/BMI088.cpp

    # components: tick
    ${CMAKE_CURRENT_SOURCE_DIR}/components/tick/tick.cpp
)

# ============================================================================
# 用户头文件路径
# ============================================================================
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Tasks/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Resources/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/components/ahrs/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/components/imu/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/components/imu/prv
    ${CMAKE_CURRENT_SOURCE_DIR}/components/pid/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/components/tick
    ${CMAKE_CURRENT_SOURCE_DIR}/components/CMSIS-DSP/Include
)

# ============================================================================
# 用户自定义宏
# ============================================================================
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # 如果 arm_math.h 需要，可以在这里加：
    # ARM_MATH_CM4
    # ARM_MATH_MATRIX_CHECK
    # ARM_MATH_ROUNDING
)

# 移除错误的 libob.a 依赖（用 cpp 文件时）
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# ============================================================================
# 链接库
# ============================================================================
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    CMSISDSP          # ← CMSIS-DSP 静态库（由 add_subdirectory 生成）
)