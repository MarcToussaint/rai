/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PCL

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "../Core/array.h"
#include "../Geo/geo.h"

typedef pcl::PointXYZRGB PointT;

bool sphereDetector(pcl::PointCloud<PointT>::Ptr inCloud, pcl::PointCloud<pcl::Normal>::Ptr inCloudNormal, pcl::ModelCoefficients::Ptr outCoefficients, pcl::PointIndices::Ptr outInliersPlane, double min_radius, double max_radius);

bool cylinderDetector(pcl::PointCloud<PointT>::Ptr inCloud, pcl::PointCloud<pcl::Normal>::Ptr inCloudNormal, pcl::ModelCoefficients::Ptr outCoefficients, pcl::PointIndices::Ptr outInliersPlane, double min_radius, double max_radius);
//bool cylinderDetector(pcl::PointCloud<PointT>::Ptr inCloud, pcl::PointCloud<pcl::Normal>::Ptr inCloudNormal, pcl::ModelCoefficients::Ptr outCoefficients, pcl::PointIndices::Ptr outInliersPlane, double min_radius, double max_radius);

bool IsABox(pcl::PointCloud<PointT>::Ptr inCloud, pcl::PointCloud<pcl::Normal>::Ptr normal, std::vector<pcl::ModelCoefficients::Ptr>& outCoefficients, rai::Quaternion& orientation, arr& center, arr& length);

bool planeFindingWithNormals(pcl::PointCloud<PointT>::Ptr inCloud, pcl::PointCloud<pcl::Normal>::Ptr inCloudNormal);

void fittingBoundingBox(pcl::PointCloud<PointT>::Ptr inCloud, PointT& min, PointT& max);

#endif
