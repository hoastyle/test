// src/detector/perception/perception_pch.hpp
#pragma once

#include "../detector_pch.hpp"

// perception特有的依赖
// #include <pcl/filters/extract_indices.h>
// #include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/kdtree/kdtree_flann.h>

#include <omp.h>
#include <pcl/common/intersections.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/passthrough.h>
// #include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
