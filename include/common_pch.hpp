#pragma once

// 1. 标准库（几乎所有模块都会用到）
#include <vector>
#include <string>
#include <memory>
#include <mutex>
#include <thread>
#include <atomic>
#include <functional>
#include <map>
#include <unordered_map>
#include <algorithm>
#include <filesystem>
#include <deque>
#include <shared_mutex>

// 2. 第三方公共库
#include <Eigen/Core>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
// #include <opencv2/opencv.hpp>

// 3. PCL核心
// #include <pcl/point_types.h>
// #include <pcl/point_cloud.h>
// #include <pcl/common/common.h>

// 4. 项目公共头文件
#include "log/Log.hpp"
#include "runtime/System.hpp"

#include "core/GlobalCommonConfig.hpp"
// #include "detector/BaseDetector.hpp"
#include "util/Util.hpp"
#include "basic/CircularBuffer.hpp"
#include "core/MMMessage.hpp"
#include "core/CraneStatus.hpp"
// #include "detector/CDetectorRegister.hpp"
