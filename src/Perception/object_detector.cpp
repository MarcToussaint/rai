#include "object_detector.h"


bool sphereDetector(pcl::PointCloud<PointT>::Ptr inCloud,pcl::PointCloud<pcl::Normal>::Ptr inCloudNormal, pcl::ModelCoefficients::Ptr outCoefficients, pcl::PointIndices::Ptr outInliersPlane,double min_radius, double max_radius)
{
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_SPHERE);
//  seg.setNormalDistanceWeight (0.1);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (2800);
  seg.setDistanceThreshold (0.01);
  seg.setInputCloud (inCloud);
  seg.setRadiusLimits(min_radius,max_radius);

  seg.setInputNormals (inCloudNormal);
  // Obtain the plane inliers and coefficients
  seg.segment (*outInliersPlane, *outCoefficients);
  std::cerr << "Sphere coefficients: " << *outCoefficients << std::endl;
  //std::cerr << "inCloud->points.size()"<<inCloud->points.size() <<"  "<< outInliersPlane->indices.size();
  if(inCloud->points.size()<=0)
      return false;
  else if((double)outInliersPlane->indices.size()/(double)inCloud->points.size() < 0.5)
      return false;
  else
      return true;

}



bool cylinderDetector(pcl::PointCloud<PointT>::Ptr inCloud,pcl::PointCloud<pcl::Normal>::Ptr inCloudNormal, pcl::ModelCoefficients::Ptr outCoefficients, pcl::PointIndices::Ptr outInliersPlane,double min_radius, double max_radius)
{
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setNormalDistanceWeight (0.1);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (2800);
  seg.setDistanceThreshold (0.01);
  seg.setInputCloud (inCloud);
  seg.setRadiusLimits(min_radius,max_radius);
  seg.setInputNormals (inCloudNormal);
  // Obtain the plane inliers and coefficients
  seg.segment (*outInliersPlane, *outCoefficients);
  std::cerr << "Cylinder coefficients: " << *outCoefficients << std::endl;

  //std::cerr << "inCloud->points.size()"<<inCloud->points.size() <<"  "<< outInliersPlane->indices.size();

  if(inCloud->points.size()<=0)
      return false;
  else if((double)outInliersPlane->indices.size()/(double)inCloud->points.size() < 0.1)
      return false;
  else
      return true;
}
