cmake_minimum_required(VERSION 3.16)

set(targets
  "aivGuideService"
  "antiHoistService"
  "CPS"
  "containerIdRecognizer"
  "cyLayerHeightService"
  "ETDynLandingService"
  "gantryCollisionAvoid"
  "gantryLaneKeeper"
  "gantryLocator"
  "ITLandingService"
  "LCPS"
  "loadLandCymService"
  "loadLandEtService"
  "percTruckGuideService"
  "personDetectService"
  "spreaderPoseEstimator"
  "truckIdRecognizer"
  "antiTruckLiftService"
  "truckInfoService"
  "camImgSaveService"
  "cymObsService"
  "spreaderSafetyHandler"
  "percTruckInfoService"
  "DetectorKicker"
  "LCPService"
  "camTruckLandingService"
  #"truckDynamicLoadService"
  "EncoderKFChecker"
  )

foreach(target IN LISTS targets)
    add_subdirectory(${target})
    set_property(DIRECTORY ${target} PROPERTY FOLDER service)
endforeach()

# 应用PCH支持 - 集中管理
if(USE_PCH AND CMAKE_VERSION VERSION_GREATER_EQUAL 3.16)
    message(STATUS "为service模块应用PCH支持")
    foreach(target IN LISTS SERVICE_TARGETS)
        # 构建目标完整名称
        set(target_name mm_service_${target})

        # 检查目标是否存在
        if(TARGET ${target_name})
            if(TARGET service_pch)
                # 优先使用service专用PCH
                target_link_libraries(${target_name} PRIVATE service_pch)
                message(STATUS "  ${target_name} 使用 service PCH")
            elseif(TARGET common_pch)
                # 备选使用全局PCH
                target_link_libraries(${target_name} PRIVATE common_pch)
                message(STATUS "  ${target_name} 使用 common PCH")
            endif()
        endif()
    endforeach()
endif()
