cmake_minimum_required(VERSION 3.16)

include(${CMAKE_SOURCE_DIR}/cmake/PlcModuleUtil.cmake)

set(targets
  "atl"
  "my2"
  "perc"
  "sil"
  "mt3"
  "mw1"
  "printer"
  )

foreach(target IN LISTS targets)
  add_subdirectory(${target})
  set_property(DIRECTORY ${target} PROPERTY FOLDER plc)
endforeach()
