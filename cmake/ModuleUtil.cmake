# 模块配置工具函数

# 包含库配置
if(CMAKE_BUILD_TYPE)
    include(${MM_ROOT}/build/${CMAKE_BUILD_TYPE}/install/share/cmake/mm_common/mm_common_config.cmake OPTIONAL)
else()
    # 尝试多种可能的路径
    foreach(BUILD_TYPE "Debug" "Release")
        if(EXISTS "${MM_ROOT}/build/${BUILD_TYPE}/install/share/cmake/mm_common/mm_common_config.cmake")
            include(${MM_ROOT}/build/${BUILD_TYPE}/install/share/cmake/mm_common/mm_common_config.cmake OPTIONAL)
            break()
        endif()
    endforeach()
endif()

set(CMAKE_LINK_DEPENDS_NO_SHARED TRUE)

# 全局包含目录（供所有模块使用）
set(MM_GLOBAL_INCLUDES
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${MM_ROOT}/include
    ${MM_ROOT}/src
    ${MM_QUANERGY_INC}
    ${MM_COMMON_INC}
    ${MM_HESAI_INC}
    ${MM_MATPLOT_INC}
)

# ==============================================================================
# 库组定义（如果配置未加载）
# ==============================================================================
if(NOT DEFINED MM_LIB_GROUP_CORE)
    set(MM_LIB_GROUP_CORE
        mm_common_core
        mm_common_log
        mm_common_util
    )

    set(MM_LIB_GROUP_PCL
        mm_common_core
        mm_common_pcl
        mm_common_transform
    )

    set(MM_LIB_GROUP_VISION
        mm_common_core
        mm_common_mmcv
        mm_common_util
    )

    set(MM_LIB_GROUP_PERCEPTION
        mm_common_core
        mm_common_pcl
        mm_common_transform
        mm_common_lidar
        mm_common_collision
    )

    set(MM_LIB_GROUP_FS
        mm_common_core
        mm_common_fs
        mm_common_util
    )
endif()

# 通用库列表（供所有模块使用）
set(MM_GLOBAL_LIBS
    ${Boost_LIBRARIES}
    glog::glog
)

# ==============================================================================
# 改进的模块添加宏（支持细粒度库选择）
# ==============================================================================
macro(add_detector_module)
    # 获取项目名称（原SetProjectName.cmake的逻辑）
    get_filename_component(FolderId ${CMAKE_CURRENT_SOURCE_DIR} NAME)
    string(REPLACE " " "_" FolderId ${FolderId})
    get_filename_component(PARENT_DIR0 ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)
    # 删除 DIR1 可以让命名更新为libmm_perception/vision_xxx.so
    get_filename_component(PARENT_DIR1 ${PARENT_DIR0} DIRECTORY)
    get_filename_component(ParentId ${PARENT_DIR1} NAME)
    string(REPLACE " " "_" ParentId ${ParentId})
    set(PROJECT_NAME mm_${ParentId}_${FolderId})

    # 注册项目
    project(${PROJECT_NAME})

    # 获取模块信息
    get_filename_component(MODULE_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
    get_filename_component(MODULE_TYPE ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)
    get_filename_component(MODULE_TYPE ${MODULE_TYPE} NAME)

    # 选择默认库组
    if(MODULE_TYPE STREQUAL "perception")
        set(DEFAULT_LIB_GROUP MM_LIB_GROUP_PERCEPTION)
    elseif(MODULE_TYPE STREQUAL "vision")
        set(DEFAULT_LIB_GROUP MM_LIB_GROUP_VISION)
    else()
        set(DEFAULT_LIB_GROUP MM_LIB_GROUP_CORE)
    endif()

    # # 包含通用目录
    # include_directories(${MM_GLOBAL_INCLUDES})

    # 收集当前目录的源文件
    file(GLOB MODULE_SRC *.cpp)

    # 生成共享库
    add_library(${PROJECT_NAME} SHARED ${MODULE_SRC})
    # 包含通用目录
    target_include_directories(${PROJECT_NAME} PRIVATE ${MM_GLOBAL_INCLUDES})

    # 添加PCH支持
    if(USE_PCH AND CMAKE_VERSION VERSION_GREATER_EQUAL 3.16)
        # 获取模块类型和名称
        get_filename_component(MODULE_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
        get_filename_component(MODULE_TYPE ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)
        get_filename_component(MODULE_TYPE ${MODULE_TYPE} NAME)
        
        # 确定使用哪个PCH
        set(PCH_FOUND FALSE)
        
        # 1. 尝试使用子模块专用PCH
        if(TARGET detector_${MODULE_TYPE}_pch)
            target_link_libraries(${PROJECT_NAME} PRIVATE detector_${MODULE_TYPE}_pch)
            message(STATUS "  模块 ${PROJECT_NAME} 使用 ${MODULE_TYPE} PCH")
            set(PCH_FOUND TRUE)
        # 2. 尝试使用模块PCH
        elseif(TARGET detector_pch)
            target_link_libraries(${PROJECT_NAME} PRIVATE detector_pch)
            message(STATUS "  模块 ${PROJECT_NAME} 使用 detector PCH")
            set(PCH_FOUND TRUE)
        # 3. 尝试使用全局PCH
        elseif(TARGET common_pch)
            target_link_libraries(${PROJECT_NAME} PRIVATE common_pch)
            message(STATUS "  模块 ${PROJECT_NAME} 使用全局 PCH")
            set(PCH_FOUND TRUE)
        endif()
        
        # 如果没有找到PCH，告知用户
        if(NOT PCH_FOUND AND CMAKE_VERBOSE_MAKEFILE)
            message(STATUS "  模块 ${PROJECT_NAME} 没有使用 PCH")
        endif()
    endif()

    # 关键：为源文件查找添加包含路径
    target_include_directories(${PROJECT_NAME} PRIVATE
        ${MM_ROOT}/include
        ${MM_ROOT}/src
    )

    # 定义FOLDERNAME宏，使源码中可以使用
    target_compile_definitions(${PROJECT_NAME} PRIVATE FOLDERNAME=\"${FolderId}\")

    # 读取模块配置（如果存在）
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/module.config")
        include("${CMAKE_CURRENT_SOURCE_DIR}/module.config")
    else()
        # 没有特定配置时，使用默认库组
        target_link_libraries(${PROJECT_NAME} PUBLIC ${${DEFAULT_LIB_GROUP}} ${MM_GLOBAL_LIBS})
    endif()

    # 设置版本号
    set(MAJOR 2)
    set(MINOR 0)
    set(PATCH 0)
    add_definitions(-DMAJOR=${MAJOR} -DMINOR=${MINOR} -DPATCH=${PATCH})

    set_target_properties(${PROJECT_NAME} PROPERTIES
        VERSION ${MAJOR}.${MINOR}.${PATCH}
        SOVERSION ${MAJOR}
    )

    # 调试信息
    if(CMAKE_VERBOSE_MAKEFILE)
        message(STATUS "Adding module: ${PROJECT_NAME}")
        message(STATUS "  FOLDERNAME: ${FolderId}")
        message(STATUS "  Type: ${MODULE_TYPE}")
        message(STATUS "  Sources: ${MODULE_SRC}")
    endif()

    if(ENABLE_BUILD_TIMING)
        set(LOG_FILE "${CMAKE_BINARY_DIR}/module_times.log") # 定义一个变量，避免重复

        # PRE_BUILD 命令
        add_custom_command(TARGET ${PROJECT_NAME} PRE_BUILD
            COMMAND ${CMAKE_COMMAND} -E cmake_echo_color --blue "Logging PRE_BUILD for ${PROJECT_NAME}" # 可选的调试信息
            COMMAND bash -c "echo '开始编译 ${PROJECT_NAME}: '`date +%s.%N` >> ${LOG_FILE}"
            VERBATIM # 确保特殊字符被正确传递
            COMMENT "记录 ${PROJECT_NAME} 开始编译时间"
        )

        # POST_BUILD 命令
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E cmake_echo_color --green "Logging POST_BUILD for ${PROJECT_NAME}" # 可选的调试信息
            COMMAND bash -c "echo '完成编译 ${PROJECT_NAME}: '`date +%s.%N` >> ${LOG_FILE}"
            VERBATIM # 确保特殊字符被正确传递
            COMMENT "记录 ${PROJECT_NAME} 完成编译时间"
        )
    endif()
endmacro()

# ==============================================================================
# 使用库组的模块添加宏
# ==============================================================================
macro(add_detector_module_with_group GROUP_NAME)
    # 获取项目名称
    get_filename_component(FolderId ${CMAKE_CURRENT_SOURCE_DIR} NAME)
    string(REPLACE " " "_" FolderId ${FolderId})
    get_filename_component(PARENT_DIR0 ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)
    get_filename_component(PARENT_DIR1 ${PARENT_DIR0} DIRECTORY)
    get_filename_component(ParentId ${PARENT_DIR1} NAME)
    string(REPLACE " " "_" ParentId ${ParentId})
    set(PROJECT_NAME mm_${ParentId}_${FolderId})

    # 注册项目
    project(${PROJECT_NAME})

    # 包含通用目录
    include_directories(${MM_GLOBAL_INCLUDES})

    # 收集当前目录的源文件
    file(GLOB MODULE_SRC *.cpp)

    # 生成共享库
    add_library(${PROJECT_NAME} SHARED ${MODULE_SRC})

    # 定义FOLDERNAME宏
    target_compile_definitions(${PROJECT_NAME} PRIVATE FOLDERNAME=\"${FolderId}\")

    # 链接指定的库组
    target_link_libraries(${PROJECT_NAME} PUBLIC
        ${MM_LIB_GROUP_${GROUP_NAME}}
        ${MM_GLOBAL_LIBS}
    )

    # 读取模块配置（如果存在）
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/module.config")
        include("${CMAKE_CURRENT_SOURCE_DIR}/module.config")
    endif()

    # 设置版本号
    set(MAJOR 2)
    set(MINOR 0)
    set(PATCH 0)
    add_definitions(-DMAJOR=${MAJOR} -DMINOR=${MINOR} -DPATCH=${PATCH})

    set_target_properties(${PROJECT_NAME} PROPERTIES
        VERSION ${MAJOR}.${MINOR}.${PATCH}
        SOVERSION ${MAJOR}
    )
endmacro()

# ==============================================================================
# 指定精确库依赖的模块添加宏
# ==============================================================================
macro(add_detector_module_with_libs)
    # 获取项目名称
    get_filename_component(FolderId ${CMAKE_CURRENT_SOURCE_DIR} NAME)
    string(REPLACE " " "_" FolderId ${FolderId})
    get_filename_component(PARENT_DIR0 ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)
    get_filename_component(PARENT_DIR1 ${PARENT_DIR0} DIRECTORY)
    get_filename_component(ParentId ${PARENT_DIR1} NAME)
    string(REPLACE " " "_" ParentId ${ParentId})
    set(PROJECT_NAME mm_${ParentId}_${FolderId})

    # 注册项目
    project(${PROJECT_NAME})

    # 包含通用目录
    include_directories(${MM_GLOBAL_INCLUDES})

    # 收集当前目录的源文件
    file(GLOB MODULE_SRC *.cpp)

    # 生成共享库
    add_library(${PROJECT_NAME} SHARED ${MODULE_SRC})

    # 定义FOLDERNAME宏
    target_compile_definitions(${PROJECT_NAME} PRIVATE FOLDERNAME=\"${FolderId}\")

    # 解析库参数
    set(options "")
    set(oneValueArgs "")
    set(multiValueArgs LIBS)
    cmake_parse_arguments(ARG "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    # 链接指定的库
    foreach(lib ${ARG_LIBS})
        target_link_libraries(${PROJECT_NAME} PUBLIC ${lib})
    endforeach()

    # 链接基础库
    target_link_libraries(${PROJECT_NAME} PUBLIC ${MM_GLOBAL_LIBS})

    # 读取模块配置（如果存在）
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/module.config")
        include("${CMAKE_CURRENT_SOURCE_DIR}/module.config")
    endif()

    # 设置版本号
    set(MAJOR 2)
    set(MINOR 0)
    set(PATCH 0)
    add_definitions(-DMAJOR=${MAJOR} -DMINOR=${MINOR} -DPATCH=${PATCH})

    set_target_properties(${PROJECT_NAME} PROPERTIES
        VERSION ${MAJOR}.${MINOR}.${PATCH}
        SOVERSION ${MAJOR}
    )
endmacro()

# 高级用法：允许覆盖默认行为
macro(add_detector_module_custom PROJECT_NAME SOURCES)
    # 使用自定义项目名和源文件
    project(${PROJECT_NAME})

    # 包含通用目录
    include_directories(${MM_GLOBAL_INCLUDES})

    # 生成共享库
    add_library(${PROJECT_NAME} SHARED ${SOURCES})

    # 链接通用库
    target_link_libraries(${PROJECT_NAME} ${MM_GLOBAL_LIBS})

    # 其他设置保持一致
    set(MAJOR 2)
    set(MINOR 0)
    set(PATCH 0)
    add_definitions(-DMAJOR=${MAJOR} -DMINOR=${MINOR} -DPATCH=${PATCH})

    set_target_properties(${PROJECT_NAME} PROPERTIES
        VERSION ${MAJOR}.${MINOR}.${PATCH}
        SOVERSION ${MAJOR}
    )
endmacro()
