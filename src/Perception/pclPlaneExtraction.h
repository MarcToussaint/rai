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
#include <vector>
#include "../Core/util.h"

typedef pcl::PointXYZRGB PointT;

void extractPlanes(pcl::PointCloud<PointT>::Ptr inCloud, pcl::PointCloud<PointT>::Ptr outCloud, std::vector<pcl::ModelCoefficients::Ptr>& outCoefficients, std::vector<pcl::PointIndices::Ptr>& outInliers, uint numPlanes);
//void extractPlanes(pcl::PointCloud<PointT>::Ptr inCloud, pcl::PointCloud<PointT>::Ptr outCloud, std::vector<pcl::ModelCoefficients::Ptr> &outCoefficients, std::vector<pcl::PointCloud<PointT>::Ptr> &outInliers , uint numPlanes);

void passthroughFilter(pcl::PointCloud<PointT>::Ptr inCloud, pcl::PointCloud<PointT>::Ptr outCloud, double minLimit, double maxLimit);

void normalEstimator(pcl::PointCloud<PointT>::Ptr inCloud, pcl::PointCloud<pcl::Normal>::Ptr outNormal, int knn);

void planeDetector(pcl::PointCloud<PointT>::Ptr inCloud, pcl::ModelCoefficients::Ptr outCoefficients, pcl::PointIndices::Ptr outInliersPlane);

void planeDetectorWithNormals(pcl::PointCloud<PointT>::Ptr inCloud, pcl::PointCloud<pcl::Normal>::Ptr inCloudNormal, pcl::ModelCoefficients::Ptr outCoefficients, pcl::PointIndices::Ptr outInliersPlane);

void substractPlane(pcl::PointCloud<PointT>::Ptr inCloud, pcl::PointIndices::Ptr inInliersPlane, pcl::PointCloud<PointT>::Ptr outCloud);

#endif
