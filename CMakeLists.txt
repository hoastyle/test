cmake_minimum_required(VERSION 3.16)

# ==============================================================================
# 项目基础配置
# ==============================================================================
project(MM)

# 优先启用ccache
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    message(STATUS "启用ccache编译缓存")
    set(CMAKE_C_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
    set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
else()
    message(WARNING "ccache未找到，编译可能会较慢")
endif()

# 启用ninja构建（如果可用）
if(CMAKE_GENERATOR STREQUAL "Ninja")
    message(STATUS "使用Ninja构建系统")
endif()

message(STATUS "=== Autane 编译配置 ===")

# 设置项目根目录和构建路径
set(MM_ROOT ${PROJECT_SOURCE_DIR})
set(PREBUILT_LIB_PATH ${MM_ROOT}/lib)
set(OUTDIR ${MM_ROOT}/build/${CMAKE_BUILD_TYPE})
include(${OUTDIR}/mode.cmake)

# 设置输出目录
set(LIBRARY_OUTPUT_PATH ${OUTDIR}/install/lib)
set(EXECUTABLE_OUTPUT_PATH ${OUTDIR}/install/bin)
set(INSTALLDIR ${OUTDIR}/install)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_CLANG)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fuse-ld=lld") # or -fuse-ld=gold
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fuse-ld=lld") # or -fuse-ld=gold
    # set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fuse-ld=gold") # or -fuse-ld=gold
    # set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fuse-ld=gold") # or -fuse-ld=gold
endif()

# ==============================================================================
# 编译选项配置
# ==============================================================================

# 关闭 CMake 开发者警告
set(CMAKE_WARN_DEVELOPER FALSE)

# 配置编译选项
if(CMAKE_BUILD_TYPE MATCHES "Release")
    message(STATUS "编译模式: Release")
    # 移除 -DNDEBUG 以保留断言
    set(CMAKE_C_FLAGS_RELEASE "-O3")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3")

    # 设置优化选项
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -s")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -s")
else()
    message(STATUS "编译模式: Debug")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O3 -rdynamic")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -ggdb -rdynamic -O3")
endif()

# 设置通用编译警告
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Werror=return-type")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror=return-type")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-function")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-copy")

# ==============================================================================
# 应用程序版本配置
# ==============================================================================
set(APP_MAJOR 2)
set(APP_MINOR 0)
set(APP_PATCH 0)
add_definitions(-DAPP_MAJOR=${APP_MAJOR})
add_definitions(-DAPP_MINOR=${APP_MINOR})
add_definitions(-DAPP_PATCH=${APP_PATCH})
add_definitions(-DMM_ENABLE_DEBUG)

message(STATUS "         version : ${APP_MAJOR}.${APP_MINOR}.${APP_PATCH}")
message(STATUS "    develop mode : ${DEV_MODE}")
message(STATUS "   CMAKE_C_FLAGS : ${CMAKE_C_FLAGS}")
message(STATUS " CMAKE_CXX_FLAGS : ${CMAKE_CXX_FLAGS}")
message(STATUS "CMAKE_BUILD_TYPE : ${CMAKE_BUILD_TYPE}")

# ==============================================================================
# 日志配置
# ==============================================================================
option(ENABLE_LOGGING "enable logging for the project" on)
if(ENABLE_LOGGING)
    add_definitions("-DMM_ENABLE_LOGGING")
endif()

# ==============================================================================
# 依赖库查找和配置
# ==============================================================================

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# 函数：打印库查找结果
function(print_library_info lib_name)
    message(STATUS "")
    message(STATUS "------ ${lib_name} ------")
endfunction()

# 查找 yaml-cpp
print_library_info("yaml")
find_package(yaml-cpp REQUIRED)
message(STATUS "YAML_CPP_INCLUDE_DIR: ${YAML_CPP_INCLUDE_DIR}")
message(STATUS "YAML_CPP_LIBRARIES  : ${YAML_CPP_LIBRARIES}")

# 查找 OpenCV
print_library_info("OpenCV")
find_package(OpenCV 4.5.5 REQUIRED)
message(STATUS "OpenCV_VERSION     : ${OpenCV_VERSION}")
message(STATUS "OpenCV_LIBS        : ${OpenCV_LIBS}")
message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")

# 查找 glog
print_library_info("glog")
find_package(glog 0.5.0 REQUIRED)

# 查找 G2O
print_library_info("G2O")
find_package(G2O REQUIRED)
set(G2O_LIBS "g2o_core;g2o_stuff;g2o_types_sba")
set(G2O_LIBRARY_DIR /usr/local/lib)
message(STATUS "G2O_LIBS       : ${G2O_LIBS}")
message(STATUS "G2O_INCLUDE_DIR: ${G2O_INCLUDE_DIR}")
message(STATUS "G2O_LIBRARY_DIR: ${G2O_LIBRARY_DIR}")

# 查找 ceres
print_library_info("ceres")
find_package(Ceres REQUIRED)
message(STATUS "Ceres_INCLUDE_DIRS: ${CERES_INCLUDE_DIRS}")
message(STATUS "Ceres_LIBRARY_DIRS: ${CERES_LIBRARIES}")

# 查找 PCL
print_library_info("PCL")
find_package(PCL 1.10 REQUIRED)
message(STATUS "PCL_VERSION     : ${PCL_VERSION}")
message(STATUS "PCL_LIBRARIES   : ${PCL_LIBRARIES}")
message(STATUS "PCL_INCLUDE_DIRS: ${PCL_INCLUDE_DIRS}")
message(STATUS "PCL_LIBRARY_DIRS: ${PCL_LIBRARY_DIRS}")
message(STATUS "PCL_DEFINITIONS : ${PCL_DEFINITIONS}")
add_definitions(${PCL_DEFINITIONS})

# 查找 boost
print_library_info("boost")
find_package(Boost COMPONENTS system thread serialization timer REQUIRED)
message(STATUS "Boost_VERSION     : ${Boost_VERSION}")
message(STATUS "Boost_LIBRARIES   : ${Boost_LIBRARIES}")
message(STATUS "Boost_LIBRARY_DIRS: ${Boost_LIBRARY_DIRS}")

# 查找 TORCH
print_library_info("TORCH")
set(CMAKE_PREFIX_PATH "/usr/local/libtorch/share/cmake/Torch")
find_package(Torch REQUIRED)
set(TORCH_LIBRARY torch torch_library)
message(STATUS "TORCH_LIBRARIE  : ${TORCH_LIBRARY}")
message(STATUS "TORCH_INCLUDE_DIRS: ${TORCH_INCLUDE_DIRS}")

# 查找 CUDA
print_library_info("CUDA")
find_package(CUDA)
if(CUDA_FOUND)
    message(STATUS "CUDA ${CUDA_VERSION}")
    add_definitions(-DGPU_)
endif()

# 查找 Glib
print_library_info("Glib")
find_package(PkgConfig)
pkg_search_module(GLIB REQUIRED glib-2.0)
message(STATUS "glib-2.0 dirs:" ${GLIB_INCLUDE_DIRS})
message(STATUS "glib-2.0 lib:" ${GLIB_LIBRARIES})
include_directories(${GLIB_INCLUDE_DIRS})
link_directories(${GLIB_LIBRARY_DIRS})
link_directories(/usr/local/lib)

# 查找 cppzmq
find_package(cppzmq)

# ==============================================================================
# 预编译头文件配置
# ==============================================================================
option(USE_PCH "Use precompiled headers" ON)

if(USE_PCH AND CMAKE_VERSION VERSION_GREATER_EQUAL 3.16)
    message(STATUS "启用预编译头文件支持")

    # 设置PCH基础路径
    set(PCH_BASE_DIR "${CMAKE_SOURCE_DIR}/include")

    # 全局PCH
    if(EXISTS "${PCH_BASE_DIR}/common_pch.hpp")
        add_library(common_pch INTERFACE)
        target_precompile_headers(common_pch INTERFACE "${PCH_BASE_DIR}/common_pch.hpp")
        message(STATUS "创建全局PCH: common_pch")
    endif()

    # Service模块PCH
    if(EXISTS "${PCH_BASE_DIR}/service/service_pch.hpp")
        add_library(service_pch INTERFACE)
        target_precompile_headers(service_pch INTERFACE "${PCH_BASE_DIR}/service/service_pch.hpp")
        message(STATUS "创建服务模块PCH: service_pch")
    endif()

    # Detector模块PCH
    if(EXISTS "${PCH_BASE_DIR}/detector/detector_pch.hpp")
        add_library(detector_pch INTERFACE)
        target_precompile_headers(detector_pch INTERFACE "${PCH_BASE_DIR}/detector/detector_pch.hpp")
        message(STATUS "创建检测器PCH: detector_pch")
    endif()

    # Perception子模块PCH
    if(EXISTS "${PCH_BASE_DIR}/detector/perception/perception_pch.hpp")
        add_library(detector_perception_pch INTERFACE)
        target_precompile_headers(detector_perception_pch INTERFACE
            "${PCH_BASE_DIR}/detector/perception/perception_pch.hpp")
        message(STATUS "创建感知模块PCH: detector_perception_pch")
    endif()

    # Vision子模块PCH
    if(EXISTS "${PCH_BASE_DIR}/detector/vision/vision_pch.hpp")
        add_library(detector_vision_pch INTERFACE)
        target_precompile_headers(detector_vision_pch INTERFACE
            "${PCH_BASE_DIR}/detector/vision/vision_pch.hpp")
        message(STATUS "创建视觉模块PCH: detector_vision_pch")
    endif()
endif()

# ==============================================================================
# 路径配置
# ==============================================================================

# 包含目录
set(MM_INC ${MM_ROOT}/include)
set(MM_SRC ${MM_ROOT}/src)
set(MM_VISION ${MM_ROOT}/src/detector/vision)
set(MM_PERCEPTION ${MM_ROOT}/src/detector/perception)
set(MM_COMMON_INC ${MM_ROOT}/include/common)
set(MM_QUANERGY_INC ${MM_ROOT}/include/quanergy)
set(MM_HESAI_INC ${MM_ROOT}/thirdparty/HesaiLidar_General_SDK/include
                 ${MM_ROOT}/thirdparty/HesaiLidar_General_SDK/src/PandarGeneralRaw/include)
set(MM_MATPLOT_INC ${MM_ROOT}/thirdparty/matplotlib-cpp)

# ==============================================================================
# 库文件配置
# ==============================================================================

# 系统库
set(MM_SDK_LIB mm_sdk)
set(JSONCPP_LIB "/usr/lib/x86_64-linux-gnu/libjsoncpp.so")

# Livox SDK 库
set(LIVOXSDK_LIB "/usr/local/lib/liblivox_sdk_static.so")
set(LIVOXSDK2_LIB "/usr/local/lib/liblivox_lidar_sdk_static.so")
set(LIVOXSDK2COMMON_LIB "/usr/local/lib/liblivox_sdk_common_static.so")
set(LIVOXSDK2VEHICLE_LIB "/usr/local/lib/liblivox_sdk_vehicle_static.so")

# 查找预编译库
find_library(MSC_LIB msc HINTS ${LIBRARY_OUTPUT_PATH})
find_library(YOLOTRT_LIB yolotrt HINTS ${LIBRARY_OUTPUT_PATH})
find_library(DARKNET_LIB darknet HINTS ${LIBRARY_OUTPUT_PATH})
find_library(YOLOSEG_LIB yolov8Seg HINTS ${LIBRARY_OUTPUT_PATH})
find_library(HDICOORDENCRYPT_LIB hdicoordencrypt HINTS ${LIBRARY_OUTPUT_PATH})
find_library(QUANERGY_LIB quanergy_client HINTS ${LIBRARY_OUTPUT_PATH})
find_library(HESAI_LIB PandarGeneralSDK HINTS ${LIBRARY_OUTPUT_PATH})
find_library(SNAP7_LIB snap7 HINTS ${LIBRARY_OUTPUT_PATH})
find_library(DEEPSORT_LIB deepsort HINTS ${LIBRARY_OUTPUT_PATH})

# ==============================================================================
# 子目录添加
# ==============================================================================

# 核心模块
add_subdirectory(src/common)
add_subdirectory(src/sdk)

# 系统模块
add_subdirectory(src/plc)
add_subdirectory(src/sensor)
add_subdirectory(src/detector)
add_subdirectory(src/service)
add_subdirectory(src/tos)

# 服务器模块
add_subdirectory(server)

# ==============================================================================
# 可选模块配置
# ==============================================================================

# 测试配置
include(${MM_ROOT}/build/tests.cmake OPTIONAL)
if(MM_ENABLE_TESTS)
    add_subdirectory(test)
endif()

# 工具配置
include(${MM_ROOT}/build/tools.cmake OPTIONAL)
if(MM_ENABLE_TOOLS)
    add_subdirectory(tool)
endif()

# ==============================================================================
# 开发模式配置
# ==============================================================================

if(${DEV_MODE} STREQUAL "common")
    add_subdirectory(app)
elseif(${DEV_MODE} STREQUAL "sdk")
    execute_process(COMMAND cp -rfL ${MM_ROOT}/include ${INSTALLDIR})
else()
    message(FATAL_ERROR "unknown mode to compile, stop!!!")
endif()

# ==============================================================================
# 下载配置 (已注释)
# ==============================================================================
#include(download.cmake)
#download()
