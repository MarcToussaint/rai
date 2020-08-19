/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PCL

#include "pclPlaneExtraction.h"

void extractPlanes(pcl::PointCloud<PointT>::Ptr inCloud, pcl::PointCloud<PointT>::Ptr outCloud, std::vector<pcl::ModelCoefficients::Ptr>& outCoefficients, std::vector<pcl::PointIndices::Ptr>& outInliers, uint numPlanes) {
  pcl::PointCloud<PointT>::Ptr cloud_be(inCloud);
  for(uint i = 0; i<numPlanes; i++) {

    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
    planeDetector(cloud_be, coefficients_plane, inliers_plane);
    substractPlane(cloud_be, inliers_plane, outCloud);

    outCoefficients.push_back(coefficients_plane);
    outInliers.push_back(inliers_plane);

    cloud_be = outCloud;

  }
  //cout << "Number of points after normal extraction: " << outCloud->size() << endl;
}
/*/
void extractPlanes(pcl::PointCloud<PointT>::Ptr inCloud, pcl::PointCloud<PointT>::Ptr outCloud, std::vector<pcl::ModelCoefficients::Ptr> &outCoefficients, std::vector<pcl::PointCloud<PointT>::Ptr> &outInliers , uint numPlanes)
{
    pcl::PointCloud<PointT>::Ptr cloud_be(inCloud);
    for (uint i = 0;i<numPlanes;i++) {
        cout<< "plane at: " <<i<<endl;
      pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
      pcl::PointCloud<PointT>::Ptr _plane (new  pcl::PointCloud<PointT>);
      planeDetector(cloud_be,coefficients_plane,inliers_plane);

      pcl::PointCloud<PointT>::Ptr temp_clound(cloud_be);
      pcl::ExtractIndices<PointT> extract;
      extract.setInputCloud (temp_clound);
      extract.setIndices (inliers_plane);
      extract.setNegative (false);
      extract.filter (*_plane);

      cout<< cloud_be->size() << endl;
      cout<< inliers_plane->indices.size() << endl;

      substractPlane(cloud_be,inliers_plane,outCloud);

      outCoefficients.push_back(coefficients_plane);
      outInliers.push_back(_plane);

      cloud_be = outCloud;

    }
}
/*/

void passthroughFilter(pcl::PointCloud<PointT>::Ptr inCloud, pcl::PointCloud<PointT>::Ptr outCloud, double minLimit, double maxLimit) {
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud(inCloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(minLimit, maxLimit);
  pass.filter(*outCloud);
  //cerr << "PointCloud after passthroughFilter: " << outCloud->points.size () << " data points." << endl;
}

void normalEstimator(pcl::PointCloud<PointT>::Ptr inCloud, pcl::PointCloud<pcl::Normal>::Ptr outNormal, int knn) {
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT> ());
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  // Estimate point normals
  ne.setSearchMethod(tree);
  ne.setInputCloud(inCloud);
  ne.setKSearch(knn);
  ne.compute(*outNormal);
  //cerr << "Normal estimation completed" << endl;
}

void planeDetector(pcl::PointCloud<PointT>::Ptr inCloud, pcl::ModelCoefficients::Ptr outCoefficients, pcl::PointIndices::Ptr outInliersPlane)

{
  pcl::SACSegmentation<PointT> seg;

  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(200);
  seg.setDistanceThreshold(0.01);
  seg.setInputCloud(inCloud);
  // Obtain the plane inliers and coefficients
  seg.segment(*outInliersPlane, *outCoefficients);
  //cerr << "Plane coefficients: " << *outCoefficients << endl;
}

void planeDetectorWithNormals(pcl::PointCloud<PointT>::Ptr inCloud, pcl::PointCloud<pcl::Normal>::Ptr inCloudNormal, pcl::ModelCoefficients::Ptr outCoefficients, pcl::PointIndices::Ptr outInliersPlane)

{
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;

  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight(0.1);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(100);
  seg.setDistanceThreshold(0.03);
  seg.setInputCloud(inCloud);
  seg.setInputNormals(inCloudNormal);
  // Obtain the plane inliers and coefficients
  seg.segment(*outInliersPlane, *outCoefficients);
  //cerr << "Plane coefficients: " << *outCoefficients << endl;
}

void substractPlane(pcl::PointCloud<PointT>::Ptr inCloud, pcl::PointIndices::Ptr inInliersPlane, pcl::PointCloud<PointT>::Ptr outCloud) {
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(inCloud);
  extract.setIndices(inInliersPlane);
  extract.setNegative(true);
  extract.filter(*outCloud);
}

#endif
