cmake_minimum_required(VERSION 3.16)

set(targets
  "my2"
  "perc"
  "sil"
  "mt3"
  "mw1"
  )

foreach(target IN LISTS targets)
  add_subdirectory(${target})
  set_property(DIRECTORY ${target} PROPERTY FOLDER tos)
endforeach()
