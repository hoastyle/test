cmake_minimum_required(VERSION 3.16)

set(targets
  "GNSS"
  "IMU"
  "IPCam"
  "lidarQ"
  "lidarR"
  "lidarLivox"
  "lidarHesai"
  "lidarLivoxs"
  "lidarPavo"
  )

foreach(target IN LISTS targets)
  add_subdirectory(${target})
  set_property(DIRECTORY ${target} PROPERTY FOLDER sensor)
endforeach()
