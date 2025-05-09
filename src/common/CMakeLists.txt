cmake_minimum_required(VERSION 3.16)
include(SetProjectName.cmake)

# ==============================================================================
# 包含目录配置
# ==============================================================================

# 函数：添加一组包含目录
function(add_include_directories)
    include_directories(${ARGN})
endfunction()

# 添加基础包含目录
add_include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${MM_ROOT}/include
    ${MM_QUANERGY_INC}
    ${MM_COMMON_INC}
    ${MM_ROOT}/src
)

# 添加 darknet 相关包含目录
add_include_directories(
    ${MM_ROOT}/thirdparty/darknet/yolotrt
    ${MM_ROOT}/thirdparty/darknet/yolotrt/extra
    ${MM_ROOT}/thirdparty/darknet/src
    ${MM_ROOT}/thirdparty/darknet/include
)

# 添加 yolov8 相关包含目录
add_include_directories(
    ${MM_ROOT}/thirdparty/yolov8/include
    ${MM_ROOT}/thirdparty/yolov8/plugin
    ${MM_ROOT}/thirdparty/yolov8/
)

# 添加系统包含目录
add_include_directories(
    /usr/local/TensorRT/include/
    /usr/local/cuda/include/
)

# ==============================================================================
# 源文件收集
# ==============================================================================

# 宏：收集模块源文件
macro(collect_module_sources module_name module_dir)
    file(GLOB ${module_name} ${PROJECT_SOURCE_DIR}/${module_dir}/*.cpp)
endmacro()

# 收集核心模块源文件
collect_module_sources(MM_ADAPTOR_SRC    "adaptor")
collect_module_sources(MM_ARITH_SRC      "arith")
collect_module_sources(MM_AUDIO_SRC      "audio")
collect_module_sources(MM_COLLISION_SRC  "collision")
collect_module_sources(MM_CORE_SRC       "core")
collect_module_sources(MM_FILTER_SRC     "filter")
collect_module_sources(MM_LIC_SRC        "lic")
collect_module_sources(MM_LIDAR_SRC      "lidar")
collect_module_sources(MM_LOG_SRC        "log")
collect_module_sources(MM_PCL_SRC        "pcl")
collect_module_sources(MM_POLICY_SRC     "policy")
collect_module_sources(MM_PROJECTION_SRC "projection")
collect_module_sources(MM_RUNTIME_SRC    "runtime")
collect_module_sources(MM_TRANSFORM_SRC  "transform")
collect_module_sources(MM_UTIL_SRC       "util")
collect_module_sources(MM_WEB_SRC        "web")
collect_module_sources(MM_OBSTACLE_SRC   "obstacle")

# 收集 MMCV 模块特殊源文件(包含子目录)
file(GLOB MM_MMCV_SRC
    ${PROJECT_SOURCE_DIR}/mmcv/*.cpp
    ${PROJECT_SOURCE_DIR}/mmcv/iDetector/*.cpp
)

# ==============================================================================
# 构建共享库
# ==============================================================================

# 创建共享库
add_library(${PROJECT_NAME} SHARED
    ${MM_ADAPTOR_SRC}
    ${MM_ARITH_SRC}
    ${MM_AUDIO_SRC}
    ${MM_COLLISION_SRC}
    ${MM_CORE_SRC}
    ${MM_FILTER_SRC}
    ${MM_LIC_SRC}
    ${MM_LIDAR_SRC}
    ${MM_LOG_SRC}
    ${MM_MMCV_SRC}
    ${MM_PCL_SRC}
    ${MM_POLICY_SRC}
    ${MM_PROJECTION_SRC}
    ${MM_RUNTIME_SRC}
    ${MM_TRANSFORM_SRC}
    ${MM_UTIL_SRC}
    ${MM_WEB_SRC}
    ${MM_OBSTACLE_SRC}
)

# ==============================================================================
# 库链接配置
# ==============================================================================

target_link_directories(${PROJECT_NAME} PUBLIC
    /usr/local/TensorRT/targets/x86_64-linux-gnu/lib
)

# 配置基础库链接
target_link_libraries(${PROJECT_NAME} PUBLIC
    # 系统库
    -lcurl
    -lmysqlclient
    -lglib-2.0
    -lrdkafka

    # 第三方库
    glog::glog
    /usr/lib/x86_64-linux-gnu/libjsoncpp.so
    ${G2O_LIBS}
    ${MSC_LIB}
    ${QUANERGY_LIB}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    ${YOLOTRT_LIB}
    ${Boost_LIBRARIES}
    ${DARKNET_LIB}
    ${CERES_LIBRARIES}
    ${YOLOSEG_LIB}
    nvinfer_plugin
    nvinfer
    crypto
)

# ==============================================================================
# 版本配置
# ==============================================================================

# 设置版本号
set(MAJOR ${APP_MAJOR})
set(MINOR ${APP_MINOR})
set(PATCH ${APP_PATCH})

# 添加版本定义
add_definitions(-DMAJOR=${MAJOR} -DMINOR=${MINOR} -DPATCH=${PATCH})

# 设置库版本属性
set_target_properties(${PROJECT_NAME} PROPERTIES
    VERSION ${MAJOR}.${MINOR}.${PATCH}
    SOVERSION ${MAJOR}
)
