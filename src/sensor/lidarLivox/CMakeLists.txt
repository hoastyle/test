cmake_minimum_required(VERSION 3.16)
include(../SetProjectName.cmake)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${MM_ROOT}/include
  ${MM_ROOT}/src
  ${MM_ROOT}/thirdparty/Livox-SDK/sdk_core
  ${MM_QUANERGY_INC}
  ${MM_COMMON_INC})

file(GLOB PROJECT_SRC *.cpp ../*.cpp)

add_library(${PROJECT_NAME} SHARED
  ${PROJECT_SRC})

target_link_libraries(${PROJECT_NAME}
  ${LIVOXSDK_LIB}
  mm_common_lidar)

set(MAJOR 2)
set(MINOR 0)
set(PATCH 0)
add_definitions(-DMAJOR=${MAJOR} -DMINOR=${MINOR} -DPATCH=${PATCH})

set_target_properties(${PROJECT_NAME} PROPERTIES
  VERSION ${MAJOR}.${MINOR}.${PATCH}
  SOVERSION ${MAJOR})
