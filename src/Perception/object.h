/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#ifdef RAI_PCL

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include "pclPlaneExtraction.h"

//typedef pcl::PointXYZRGB PointT;

void voxelFilter(pcl::PointCloud<PointT>::Ptr inCloud, pcl::PointCloud<PointT>::Ptr outCloud, double leafSize);

void clusterObject(pcl::PointCloud<PointT>::Ptr cloud, int numCluster, std::vector<pcl::PointCloud<PointT>::Ptr>& list_extracted_cloud, int minPoints = 200, int maxPoints = 25000);

void extractPrimitives(std::vector<pcl::PointCloud<PointT>::Ptr> list_extracted_cloud, std::vector<std::pair<pcl::ModelCoefficients::Ptr, int>>& list_primitives);

#endif
