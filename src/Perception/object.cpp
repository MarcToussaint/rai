/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PCL

#include "object.h"
#include "object_detector.h"

void voxelFilter(pcl::PointCloud<PointT>::Ptr inCloud, pcl::PointCloud<PointT>::Ptr outCloud, double leafSize) {
  pcl::VoxelGrid<PointT> vg;
  //pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  vg.setInputCloud(inCloud);
  vg.setLeafSize(leafSize, leafSize, leafSize);
  vg.filter(*outCloud);
  std::cout << "PointCloud after filtering has: " << outCloud->points.size()  << " data points." << std::endl;  //*

}

void clusterObject(pcl::PointCloud<PointT>::Ptr cloud_filtered, int numCluster, std::vector<pcl::PointCloud<PointT>::Ptr>& list_extracted_cloud, int minPoints, int maxPoints) {
  std::vector<pcl::PointIndices> cluster_indices;

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud_filtered);

  //std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(0.02);  // 2cm
  ec.setMinClusterSize(minPoints);
  ec.setMaxClusterSize(maxPoints);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_filtered);
  ec.extract(cluster_indices);

  int j = 0;
  for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
    pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
    for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
      cloud_cluster->points.push_back(cloud_filtered->points[*pit]);  //*
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    list_extracted_cloud.push_back(cloud_cluster);

    j++;
    if(j == numCluster) break;
  }

}

void extractPrimitives(std::vector<pcl::PointCloud<PointT>::Ptr> list_extracted_cloud, std::vector<std::pair<pcl::ModelCoefficients::Ptr, int>>& list_primitives) {
  for(int num=0; num < list_extracted_cloud.size(); num++) {
    pcl::PointCloud<pcl::Normal>::Ptr normal_extracted(new pcl::PointCloud<pcl::Normal>);
    normalEstimator(list_extracted_cloud[num], normal_extracted, 50);

    // detect sphere
    pcl::ModelCoefficients::Ptr coefficients_sphere(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_sphere(new pcl::PointIndices);
    bool exist = sphereDetector(list_extracted_cloud[num], normal_extracted, coefficients_sphere, inliers_sphere, 0.05, 0.12);
    if(exist) list_primitives.push_back(std::pair<pcl::ModelCoefficients::Ptr, int>(coefficients_sphere, 0));

    // detect cylinder
    pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
    exist = cylinderDetector(list_extracted_cloud[num], normal_extracted, coefficients_cylinder, inliers_cylinder, 0.01, 0.05);
    if(exist) list_primitives.push_back(std::pair<pcl::ModelCoefficients::Ptr, int>(coefficients_cylinder, 1));
  }
}

#endif
