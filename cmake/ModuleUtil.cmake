# 模块配置工具函数

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

# 通用库列表（供所有模块使用）
set(MM_GLOBAL_LIBS
    ${PCL_LIBRARIES}
    ${MM_COMMON_LIB}
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    glog::glog
)

# 统一的模块添加宏（集成了SetProjectName的功能）
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

    # 包含通用目录
    include_directories(${MM_GLOBAL_INCLUDES})

    # 收集源文件（可选的额外目录）
    set(EXTRA_SRC_DIRS "")
    if(ARGC GREATER 0)
        set(EXTRA_SRC_DIRS ${ARGV})
    endif()

    # 收集当前目录的源文件
    file(GLOB MODULE_SRC *.cpp)
    file(GLOB_RECURSE RECURSIVE_SRC *.cpp)  # 包含子目录中的源文件

    # 如果需要子目录源文件，使用recursive模式
    if(INCLUDE_SUBDIRS)
        set(MODULE_SRC ${RECURSIVE_SRC})
    endif()

    # 收集额外目录的源文件
    foreach(extra_dir ${EXTRA_SRC_DIRS})
        file(GLOB EXTRA_SRC ${extra_dir}/*.cpp)
        list(APPEND MODULE_SRC ${EXTRA_SRC})
    endforeach()

    # 生成共享库
    add_library(${PROJECT_NAME} SHARED ${MODULE_SRC})

    # 定义FOLDERNAME宏，使源码中可以使用
    target_compile_definitions(${PROJECT_NAME} PRIVATE FOLDERNAME=\"${FolderId}\")
    # target_compile_definitions(${PROJECT_NAME} PRIVATE FOLDERNAME="${FolderId}")

    # 链接通用库
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
