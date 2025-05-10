cmake_minimum_required(VERSION 3.16)

# 引入模块工具
include(${CMAKE_SOURCE_DIR}/cmake/ModuleUtil.cmake)

# ==============================================================================
# PCH应用（新增部分）
# ==============================================================================

# 应用PCH支持 - 集中管理
if(USE_PCH AND CMAKE_VERSION VERSION_GREATER_EQUAL 3.16)
    message(STATUS "为detector模块应用PCH支持")

    # 为每个模块应用适当的PCH
    foreach(target IN LISTS ADDED_MODULES)
        # 解析模块类型和名称
        string(REPLACE "/" ";" target_parts ${target})
        list(GET target_parts 0 module_type)
        list(GET target_parts 1 module_name)

        # 构建目标完整名称
        set(target_name mm_detector_${module_name})

        # 检查目标是否存在
        if(TARGET ${target_name})
            # 根据模块类型应用适当的PCH
            if(module_type STREQUAL "perception" AND TARGET detector_perception_pch)
                target_link_libraries(${target_name} PRIVATE detector_perception_pch)
                message(STATUS "  ${target_name} 使用 perception PCH")
            elseif(module_type STREQUAL "vision" AND TARGET detector_vision_pch)
                target_link_libraries(${target_name} PRIVATE detector_vision_pch)
                message(STATUS "  ${target_name} 使用 vision PCH")
            elseif(TARGET detector_pch)
                target_link_libraries(${target_name} PRIVATE detector_pch)
                message(STATUS "  ${target_name} 使用 detector PCH")
            elseif(TARGET common_pch)
                target_link_libraries(${target_name} PRIVATE common_pch)
                message(STATUS "  ${target_name} 使用 common PCH")
            endif()
        endif()
    endforeach()
endif()

# ==============================================================================
# 模块选择控制
# ==============================================================================

# 设置永久排除的模块列表
set(EXCLUDED_MODULES
    "perception/mapTruckLocator"
    # 在这里添加其他需要永久排除的模块
)

# 从环境变量读取模块选择参数（由compile.sh设置）
if(DEFINED ENV{BUILD_MODULES})
    set(BUILD_MODULES "$ENV{BUILD_MODULES}")
endif()

if(DEFINED ENV{SELECTED_MODULES})
    set(SELECTED_MODULES "$ENV{SELECTED_MODULES}")
endif()

if(DEFINED ENV{EXCLUDE_MODULES})
    string(REPLACE "," ";" ENV_EXCLUDED_MODULES "$ENV{EXCLUDE_MODULES}")
    list(APPEND EXCLUDED_MODULES ${ENV_EXCLUDED_MODULES})
endif()

# ==============================================================================
# 模块自动发现与管理
# ==============================================================================

# 函数：获取目录下的所有子模块
function(discover_modules parent_dir type_name result)
    file(GLOB children RELATIVE ${parent_dir} ${parent_dir}/*)
    set(dirlist "")
    foreach(child ${children})
        if(IS_DIRECTORY ${parent_dir}/${child})
            # 检查是否有CMakeLists.txt或源文件
            if(EXISTS "${parent_dir}/${child}/CMakeLists.txt" OR
               EXISTS "${parent_dir}/${child}/*.cpp")
                list(APPEND dirlist "${type_name}/${child}")
            endif()
        endif()
    endforeach()
    set(${result} ${dirlist} PARENT_SCOPE)
endfunction()

# 发现所有模块
discover_modules("${MM_ROOT}/src/detector/perception" "perception" perception_targets)
discover_modules("${MM_ROOT}/src/detector/vision" "vision" vision_targets)

# 合并模块列表
set(all_discovered_targets ${perception_targets} ${vision_targets})

# # 获取perception模块
# if(EXISTS "${MM_ROOT}/src/detector/perception")
#     get_subdirectories(perception_modules "${MM_ROOT}/src/detector/perception")
#     foreach(module ${perception_modules})
#         list(APPEND perception_targets "perception/${module}")
#     endforeach()
# endif()
#
# # 获取vision模块
# if(EXISTS "${MM_ROOT}/src/detector/vision")
#     get_subdirectories(vision_modules "${MM_ROOT}/src/detector/vision")
#     foreach(module ${vision_modules})
#         list(APPEND vision_targets "vision/${module}")
#     endforeach()
# endif()

# ==============================================================================
# 模块过滤逻辑
# ==============================================================================

# 根据选择模式过滤模块
if(BUILD_MODULES)
    if(BUILD_MODULES STREQUAL "perception")
        # 只编译perception模块
        set(filtered_targets ${perception_targets})
        message(STATUS "只编译 Perception 模块")
    elseif(BUILD_MODULES STREQUAL "vision")
        # 只编译vision模块
        set(filtered_targets ${vision_targets})
        message(STATUS "只编译 Vision 模块")
    else()
        set(filtered_targets ${all_discovered_targets})
    endif()
elseif(SELECTED_MODULES)
    # 使用指定的模块列表
    set(filtered_targets "")
    string(REPLACE "," ";" selected_list "${SELECTED_MODULES}")

    foreach(selected ${selected_list})
        # 清理空白字符
        string(STRIP "${selected}" selected)
        list(FIND all_discovered_targets "${selected}" index)
        if(index GREATER_EQUAL 0)
            list(APPEND filtered_targets "${selected}")
        else()
            message(STATUS "警告: 指定的模块 '${selected}' 未找到")
        endif()
    endforeach()

    list(LENGTH filtered_targets selected_count)
    message(STATUS "编译指定的 ${selected_count} 个模块")
else()
    # 编译所有模块
    set(filtered_targets ${all_discovered_targets})
    message(STATUS "编译所有模块")
endif()

# 从过滤结果中移除排除的模块
set(targets "")
foreach(target ${filtered_targets})
    list(FIND EXCLUDED_MODULES "${target}" index)
    if(index EQUAL -1)
        list(APPEND targets "${target}")
    else()
        message(STATUS "排除模块: ${target}")
    endif()
endforeach()

# ==============================================================================
# 模块处理函数
# ==============================================================================

# 函数：检查模块是否有源文件
function(has_source_files directory result)
    file(GLOB_RECURSE sources
        ${directory}/*.cpp
        ${directory}/*.c
        ${directory}/*.hpp
        ${directory}/*.h
    )
    if(sources)
        set(${result} TRUE PARENT_SCOPE)
    else()
        set(${result} FALSE PARENT_SCOPE)
    endif()
endfunction()

# 函数：处理单个模块目录
function(process_module_directory module_path)
    get_filename_component(module_name "${module_path}" NAME)
    get_filename_component(module_type "${module_path}" DIRECTORY)
    get_filename_component(module_type "${module_type}" NAME)

    set(full_path "${MM_ROOT}/src/detector/${module_path}")

    if(EXISTS "${full_path}")
        has_source_files("${full_path}" has_sources)

        if(has_sources)
            add_subdirectory(${module_path})
            list(APPEND ADDED_MODULES "${module_path}" PARENT_SCOPE)
            if(CMAKE_VERBOSE_MAKEFILE)
                message(STATUS "  Added ${module_type}/${module_name}")
            endif()
        else()
            if(CMAKE_VERBOSE_MAKEFILE)
                message(STATUS "  Skipped ${module_type}/${module_name} (no source files)")
            endif()
        endif()
    else()
        if(CMAKE_VERBOSE_MAKEFILE)
            message(STATUS "  Skipped ${module_type}/${module_name} (not found)")
        endif()
    endif()
endfunction()

# ==============================================================================
# 构建统计信息
# ==============================================================================

# 统计发现的模块数量
list(LENGTH perception_targets discovered_perception_count)
list(LENGTH vision_targets discovered_vision_count)
list(LENGTH all_discovered_targets total_discovered_count)
list(LENGTH EXCLUDED_MODULES excluded_count)
list(LENGTH targets target_count)

message(STATUS "Detector 模块配置:")
message(STATUS "  发现 Perception 模块: ${discovered_perception_count} 个")
message(STATUS "  发现 Vision 模块: ${discovered_vision_count} 个")
message(STATUS "  总发现模块: ${total_discovered_count} 个")
message(STATUS "  排除模块: ${excluded_count} 个")

# 如果启用详细模式，显示选择的模块
if(CMAKE_VERBOSE_MAKEFILE)
    if(BUILD_MODULES)
        message(STATUS "构建模式: ${BUILD_MODULES}")
    endif()
    if(SELECTED_MODULES)
        message(STATUS "选定模块: ${SELECTED_MODULES}")
    endif()
    if(EXCLUDED_MODULES)
        message(STATUS "排除的模块:")
        foreach(module ${EXCLUDED_MODULES})
            message(STATUS "  - ${module}")
        endforeach()
    endif()
endif()

# ==============================================================================
# 处理所有模块
# ==============================================================================

message(STATUS "处理模块...")
set(ADDED_MODULES "")

foreach(target IN LISTS targets)
    if(EXISTS "${MM_ROOT}/src/detector/${target}/CMakeLists.txt")
        # 如果模块有自己的CMakeLists，直接添加
        add_subdirectory(${target})
        list(APPEND ADDED_MODULES "${target}")
    elseif(EXISTS "${MM_ROOT}/src/detector/${target}/*.cpp")
        # 如果只有源文件，使用统一模块宏
        message(STATUS "使用统一配置处理模块: ${target}")
        add_subdirectory(${target})
        list(APPEND ADDED_MODULES "${target}")
    endif()
endforeach()

# ==============================================================================
# 最终统计
# ==============================================================================

list(LENGTH ADDED_MODULES added_count)
message(STATUS "成功添加模块: ${added_count}/${target_count}")

# 如果启用详细模式，显示添加的模块列表
if(CMAKE_VERBOSE_MAKEFILE AND ADDED_MODULES)
    message(STATUS "已添加的模块:")
    foreach(module ${ADDED_MODULES})
        message(STATUS "  ✓ ${module}")
    endforeach()
endif()
