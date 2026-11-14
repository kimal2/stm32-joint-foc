cmake_minimum_required(VERSION 3.22)

#
# 本文件只生成一次，
# 后续重复调用转换器时不会重新生成。
#
# 用户可按需要修改本文件
#

# 设置编译器选项
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# 设置构建类型
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# 设置项目名称
set(CMAKE_PROJECT_NAME Joint)

# 导出编译命令，便于 clangd 等工具建立代码索引
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# 项目基础配置
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# 启用 CMake 的汇编及 C 语言支持
enable_language(C ASM)

# 创建可执行目标
add_executable(${CMAKE_PROJECT_NAME})

# 添加 STM32CubeMX 生成的源文件
add_subdirectory(cmake/stm32cubemx)

# 接入 ARM GCC 的 Cortex-M3 定点数学库
add_subdirectory(ThirdParty/IQMath)

# 电流环统一使用 IQmath Q20 实现。
target_sources(${CMAKE_PROJECT_NAME} PRIVATE Components/FOC/CurrentControlIQ.c)

# 设置链接目录
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # 在此添加用户库搜索路径
)

# 为可执行目标添加源文件
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # 在此添加用户源文件
    ${CMAKE_CURRENT_SOURCE_DIR}/APP/MotorApp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/APP/CanApp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Components/CAN/CanTransport.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Components/AS5600/as5600.c
    
    ${CMAKE_CURRENT_SOURCE_DIR}/Components/FOC/FOC.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Components/FOC/MagneticSensor.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/Filter/KalmanFilter.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Components/Motor/Motorbeep.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Components/FOC/foc_utils.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Components/Flash/FlashStorage.c
)

# 添加头文件搜索路径
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # 在此添加用户头文件搜索路径
    ${CMAKE_CURRENT_SOURCE_DIR}/APP
    ${CMAKE_CURRENT_SOURCE_DIR}/Components/CAN
    ${CMAKE_CURRENT_SOURCE_DIR}/Components/AS5600
    ${CMAKE_CURRENT_SOURCE_DIR}/Components/FOC
    ${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/Filter
    ${CMAKE_CURRENT_SOURCE_DIR}/Components/Motor
    ${CMAKE_CURRENT_SOURCE_DIR}/Components/Flash
)

# 添加项目预处理宏
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # 在此添加用户预处理宏
)

# 使用 C++ 文件时移除错误的 libob.a 库依赖
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# 添加链接库
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    IQMath::IQMath

    # 在此添加用户链接库
)
