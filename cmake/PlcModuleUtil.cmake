# 文件: cmake/PlcModuleUtil.cmake

# PLC模块添加宏 - 适配现有结构
macro(add_plc_module)
  # 替代SetProjectName.cmake的功能
  get_filename_component(FolderId ${CMAKE_CURRENT_SOURCE_DIR} NAME)
  string(REPLACE " " "_" FolderId ${FolderId})
  set(PROJECT_NAME mm_plc_${FolderId})
  
  # 注册项目
  project(${PROJECT_NAME})
  
  # 添加包含目录（保留原有目录）
  include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${MM_ROOT}/include
    ${MM_ROOT}/src
    ${MM_QUANERGY_INC}
    ${MM_COMMON_INC}
    ${MM_ROOT}/src/plc
  )
  
  # 收集源文件（保持原有Glob模式）
  file(GLOB PROJECT_SRC *.cpp ../*.cpp ../common/*.cpp)
  
  # 创建共享库
  add_library(${PROJECT_NAME} SHARED ${PROJECT_SRC})
  
  # 应用PCH
  if(USE_PCH AND CMAKE_VERSION VERSION_GREATER_EQUAL 3.16)
    if(TARGET common_pch)
      target_link_libraries(${PROJECT_NAME} PRIVATE common_pch)
      message(STATUS "模块 ${PROJECT_NAME} 使用 common_pch")
    endif()
  endif()
  
  # 链接通用库（保留原有依赖）
  target_link_libraries(${PROJECT_NAME} PUBLIC ${SNAP7_LIB} ${MM_COMMON_LIB})
  
  # 设置版本信息（保持原有版本配置）
  set(MAJOR 2)
  set(MINOR 0)
  set(PATCH 0)
  add_definitions(-DMAJOR=${MAJOR} -DMINOR=${MINOR} -DPATCH=${PATCH})
  
  # 设置目标属性
  set_target_properties(${PROJECT_NAME} PROPERTIES
    VERSION ${MAJOR}.${MINOR}.${PATCH}
    SOVERSION ${MAJOR}
  )
  
  # 定义FOLDERNAME宏用于源代码
  target_compile_definitions(${PROJECT_NAME} PRIVATE FOLDERNAME=\"${FolderId}\")
endmacro()
