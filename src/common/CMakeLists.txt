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
collect_module_sources(MM_3D_DL_SRC      "3d_dl")
collect_module_sources(MM_OP_SRC         "op")

# 收集 MMCV 模块特殊源文件(包含子目录)
file(GLOB MM_MMCV_SRC
    ${PROJECT_SOURCE_DIR}/mmcv/*.cpp
    ${PROJECT_SOURCE_DIR}/mmcv/iDetector/*.cpp
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

# ==============================================================================
# 公共函数：设置库属性
# ==============================================================================
# 在文件开头添加这个函数
function(configure_common_library target_name)
    # 设置库版本属性
    set_target_properties(${target_name} PROPERTIES
        VERSION ${MAJOR}.${MINOR}.${PATCH}
        SOVERSION ${MAJOR}
    )

    # 关键：确保每个库都正确导出包含目录给其用户
    target_include_directories(${target_name} PUBLIC
        $<BUILD_INTERFACE:${MM_ROOT}/include>
        $<BUILD_INTERFACE:${MM_ROOT}/src>
        $<INSTALL_INTERFACE:include>
    )

    # 其他的通用配置...
    target_compile_features(${target_name} PUBLIC cxx_std_17)
endfunction()

function(set_common_library_properties target_name)
    # 设置库版本属性
    set_target_properties(${target_name} PROPERTIES
        VERSION ${MAJOR}.${MINOR}.${PATCH}
        SOVERSION ${MAJOR}
    )

    # 设置公共包含目录
    target_include_directories(${target_name} PUBLIC ${MM_COMMON_INCLUDE_DIRS})

    # 指定C++标准
    target_compile_features(${target_name} PUBLIC cxx_std_17)
endfunction()

# 公共函数：获取库的依赖
function(get_lib_dependencies lib_name output_var)
    # 基于库名定义依赖
    if(${lib_name} STREQUAL "mm_common_core")
        set(deps "glog::glog;${YAML_CPP_LIBRARIES}")
    elseif(${lib_name} STREQUAL "mm_common_log")
        set(deps "mm_common_core;glog::glog")
    elseif(${lib_name} STREQUAL "mm_common_pcl")
        set(deps "mm_common_core;${PCL_LIBRARIES}")
    elseif(${lib_name} STREQUAL "mm_common_util")
        set(deps "mm_common_core;mm_common_log")
    elseif(${lib_name} STREQUAL "mm_common_transform")
        set(deps "mm_common_core;mm_common_pcl;mm_common_util")
    elseif(${lib_name} STREQUAL "mm_common_mmcv")
        set(deps "mm_common_core;${OpenCV_LIBS};${DARKNET_LIB};${YOLOTRT_LIB}")
    elseif(${lib_name} STREQUAL "mm_common_lidar")
        set(deps "mm_common_core;mm_common_pcl;mm_common_transform")
    elseif(${lib_name} STREQUAL "mm_common_projection")
        set(deps "mm_common_core;mm_common_transform;${CERES_LIBRARIES}")
    elseif(${lib_name} STREQUAL "mm_common_audio")
        set(deps "mm_common_core;mm_common_util;${MSC_LIB}")
    elseif(${lib_name} STREQUAL "mm_common_collision")
        set(deps "mm_common_core;mm_common_pcl;mm_common_transform")
    elseif(${lib_name} STREQUAL "mm_common_runtime")
        set(deps "mm_common_core;mm_common_log;mm_common_util")
    else()
        set(deps "mm_common_core")
    endif()

    # 输出依赖列表
    set(${output_var} "${deps}" PARENT_SCOPE)
endfunction()

# ==============================================================================
# 创建各个子库
# ==============================================================================

# 1. 日志库
add_library(mm_common_log SHARED ${MM_LOG_SRC})
target_link_libraries(mm_common_log PUBLIC glog::glog ${JSONCPP_LIB})
configure_common_library(mm_common_log)

# 2. 工具库
add_library(mm_common_util SHARED ${MM_UTIL_SRC})
target_link_libraries(mm_common_util PUBLIC ${OpenCV_LIBS})
configure_common_library(mm_common_util)

# 3. 运行时库
add_library(mm_common_runtime SHARED ${MM_RUNTIME_SRC})
configure_common_library(mm_common_runtime)

# 4. 音频库
add_library(mm_common_audio SHARED  ${MM_AUDIO_SRC})
target_link_libraries(mm_common_audio PUBLIC ${MSC_LIB})
configure_common_library(mm_common_audio)

# 5. 基础算法库
add_library(mm_common_arith SHARED ${MM_ARITH_SRC})
target_link_libraries(mm_common_arith PUBLIC ${CERES_LIBRARIES} ${G2O_LIBS})
configure_common_library(mm_common_arith)

# 6. 视觉库
add_library(mm_common_mmcv SHARED ${MM_MMCV_SRC})
target_link_directories(mm_common_mmcv PUBLIC /usr/local/TensorRT/targets/x86_64-linux-gnu/lib)
target_link_libraries(mm_common_mmcv PUBLIC
    ${OpenCV_LIBS}
    ${DARKNET_LIB}
    ${YOLOTRT_LIB}
    ${YOLOSEG_LIB}
    nvinfer_plugin
    nvinfer
)
configure_common_library(mm_common_mmcv)

# 7. PCL库
add_library(mm_common_pcl SHARED ${MM_PCL_SRC})
# target_link_libraries(mm_common_pcl PUBLIC ${PCL_LIBRARIES} mm_common_arith mm_common_mmcv)
target_link_libraries(mm_common_pcl PUBLIC ${PCL_LIBRARIES} mm_common_arith)
configure_common_library(mm_common_pcl)

add_library(mm_common_3d_dl SHARED ${MM_3D_DL_SRC})
target_link_libraries(mm_common_3d_dl PUBLIC ${PCL_LIBRARIES} mm_common_arith mm_common_mmcv)
configure_common_library(mm_common_3d_dl)

# operation planning
add_library(mm_common_op SHARED ${MM_OP_SRC})
configure_common_library(mm_common_op)

# 8. 变换库
add_library(mm_common_transform SHARED ${MM_TRANSFORM_SRC})
configure_common_library(mm_common_transform)

# 9. 碰撞库
add_library(mm_common_collision SHARED ${MM_COLLISION_SRC})
target_link_libraries(mm_common_collision PUBLIC mm_common_pcl mm_common_transform)
configure_common_library(mm_common_collision)

# 10. 障碍物库
add_library(mm_common_obstacle SHARED ${MM_OBSTACLE_SRC})
target_link_libraries(mm_common_obstacle PUBLIC mm_common_collision mm_common_arith)
configure_common_library(mm_common_obstacle)

# 11. 核心库
add_library(mm_common_core SHARED ${MM_CORE_SRC})
target_link_libraries(mm_common_core mm_common_obstacle)
configure_common_library(mm_common_core)

# 12. 过滤器库
add_library(mm_common_filter SHARED ${MM_FILTER_SRC})
target_link_libraries(mm_common_filter PUBLIC mm_common_core)
configure_common_library(mm_common_filter)

# 13. 许可证库
add_library(mm_common_lic SHARED ${MM_LIC_SRC})
target_link_libraries(mm_common_lic PUBLIC mm_common_core)
configure_common_library(mm_common_lic)

# 16. 投影库
add_library(mm_common_projection SHARED ${MM_PROJECTION_SRC})
target_link_libraries(mm_common_projection PUBLIC mm_common_core mm_common_transform ${CERES_LIBRARIES})
configure_common_library(mm_common_projection)

# 14. 激光雷达库
add_library(mm_common_lidar SHARED ${MM_LIDAR_SRC})
target_link_libraries(mm_common_lidar PUBLIC mm_common_core mm_common_pcl mm_common_transform mm_common_projection)
configure_common_library(mm_common_lidar)

# 15. 策略库
add_library(mm_common_policy SHARED ${MM_POLICY_SRC})
target_link_libraries(mm_common_policy PUBLIC mm_common_core)
configure_common_library(mm_common_policy)

# 17. Web库
add_library(mm_common_web SHARED ${MM_WEB_SRC})
target_link_libraries(mm_common_web PUBLIC -lcurl -lmysqlclient -lglib-2.0 -lrdkafka)
configure_common_library(mm_common_web)

# 18. 适配器库
add_library(mm_common_adaptor SHARED ${MM_ADAPTOR_SRC})
configure_common_library(mm_common_adaptor)

# 创建一个接口库 - 无需源文件
add_library(mm_common_fs INTERFACE)
target_link_libraries(mm_common_fs INTERFACE ${Boost_FILESYSTEM_LIBRARY})

# 创建一个接口库 - 无需源文件
add_library(mm_common_cv INTERFACE)
target_link_libraries(mm_common_cv INTERFACE ${OpenCV_LIBS})

# ==============================================================================
# 保留向后兼容性 - 创建一个整合库 (可选)
# ==============================================================================
add_library(${PROJECT_NAME} SHARED
    $<TARGET_OBJECTS:mm_common_core>
    $<TARGET_OBJECTS:mm_common_log>
    $<TARGET_OBJECTS:mm_common_util>
    $<TARGET_OBJECTS:mm_common_pcl>
    $<TARGET_OBJECTS:mm_common_transform>
    $<TARGET_OBJECTS:mm_common_arith>
    $<TARGET_OBJECTS:mm_common_audio>
    $<TARGET_OBJECTS:mm_common_collision>
    $<TARGET_OBJECTS:mm_common_filter>
    $<TARGET_OBJECTS:mm_common_lic>
    $<TARGET_OBJECTS:mm_common_lidar>
    $<TARGET_OBJECTS:mm_common_mmcv>
    $<TARGET_OBJECTS:mm_common_policy>
    $<TARGET_OBJECTS:mm_common_projection>
    $<TARGET_OBJECTS:mm_common_runtime>
    $<TARGET_OBJECTS:mm_common_web>
    $<TARGET_OBJECTS:mm_common_obstacle>
    $<TARGET_OBJECTS:mm_common_adaptor>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    /usr/lib/x86_64-linux-gnu/libjsoncpp.so
    ${QUANERGY_LIB}
    ${Boost_LIBRARIES}
    crypto
)

set_common_library_properties(${PROJECT_NAME})

# 生成库组配置文件
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/mm_common_config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/mm_common_config.cmake
    @ONLY
)

# 安装配置文件
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/mm_common_config.cmake
    DESTINATION ${CMAKE_INSTALL_PREFIX}/share/cmake/mm_common
)
