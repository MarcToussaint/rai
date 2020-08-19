/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PCL

#include "object_detector.h"
#include "../Core/array.h"
#include "../Geo/geo.h"

#include "pclPlaneExtraction.h"
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>

bool sphereDetector(pcl::PointCloud<PointT>::Ptr inCloud, pcl::PointCloud<pcl::Normal>::Ptr inCloudNormal, pcl::ModelCoefficients::Ptr outCoefficients, pcl::PointIndices::Ptr outInliersPlane, double min_radius, double max_radius) {
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_SPHERE);
//  seg.setNormalDistanceWeight (0.1);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(2800);
  seg.setDistanceThreshold(0.01);
  seg.setInputCloud(inCloud);
  seg.setRadiusLimits(min_radius, max_radius);

  seg.setInputNormals(inCloudNormal);
  // Obtain the plane inliers and coefficients
  seg.segment(*outInliersPlane, *outCoefficients);
  std::cerr << "Sphere coefficients: " << *outCoefficients << std::endl;
  //std::cerr << "inCloud->points.size()"<<inCloud->points.size() <<"  "<< outInliersPlane->indices.size();
  if(inCloud->points.size()<=0)
    return false;
  else if((double)outInliersPlane->indices.size()/(double)inCloud->points.size() < 0.5)
    return false;
  else
    return true;

}

bool cylinderDetector(pcl::PointCloud<PointT>::Ptr inCloud, pcl::PointCloud<pcl::Normal>::Ptr inCloudNormal, pcl::ModelCoefficients::Ptr outCoefficients, pcl::PointIndices::Ptr outInliersPlane, double min_radius, double max_radius) {
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_CYLINDER);
  seg.setNormalDistanceWeight(0.1);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(2800);
  seg.setDistanceThreshold(0.01);
  seg.setInputCloud(inCloud);
  seg.setRadiusLimits(min_radius, max_radius);
  seg.setInputNormals(inCloudNormal);
  // Obtain the plane inliers and coefficients
  seg.segment(*outInliersPlane, *outCoefficients);
  std::cerr << "Cylinder coefficients: " << *outCoefficients << std::endl;

  std::cerr << "inCloud->points.size()"<<inCloud->points.size() <<"  "<< outInliersPlane->indices.size();

  if(inCloud->points.size()<=0)
    return false;
  else if((double)outInliersPlane->indices.size()/(double)inCloud->points.size() < 0.025)
    return false;
  else
    return true;
}

bool planeFindingWithNormals(pcl::PointCloud<PointT>::Ptr inCloud, pcl::PointCloud<pcl::Normal>::Ptr inCloudNormal) {

  pcl::ModelCoefficients::Ptr outCoefficients;
  pcl::PointIndices::Ptr outInliersPlane;

  planeDetectorWithNormals(inCloud, inCloudNormal, outCoefficients, outInliersPlane);

  if(inCloud->points.size()<=0)
    return false;
  else if((double)outInliersPlane->indices.size()/(double)inCloud->points.size() < 0.1)
    return false;
  else
    return true;

}

bool IsABox(pcl::PointCloud<PointT>::Ptr inCloud, pcl::PointCloud<pcl::Normal>::Ptr normal, std::vector<pcl::ModelCoefficients::Ptr>& outCoefficients, rai::Quaternion& orientation, arr& center, arr& length) {

  //std::vector<pcl::ModelCoefficients::Ptr> outCoefficients;
  std::vector<pcl::PointIndices::Ptr> outInliers;
  pcl::PointCloud<PointT>::Ptr outCloud(new pcl::PointCloud<PointT>);
  uint numPlanes = 6;

  extractPlanes(inCloud, outCloud, outCoefficients,  outInliers, numPlanes);

  bool isBox = true;
  arr Matrix(3, 3);
  int index = 0;
  for(int i=0; i<outInliers.size(); i++) {
    //cout<<"this plane cloud size is "<<outInliers[i]->indices.size()<<endl;
    if((double)outInliers[i]->indices.size()/(double)inCloud->points.size()>0.1) { //proper plane

      pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
      seg.setOptimizeCoefficients(true);
      seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
      //seg.setNormalDistanceWeight (0.1);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setMaxIterations(2800);
      seg.setDistanceThreshold(0.01);
      seg.setInputCloud(inCloud);
      seg.setAxis(Eigen::Vector3f(outCoefficients[i]->values[0], outCoefficients[i]->values[1], outCoefficients[i]->values[2]));
      seg.setEpsAngle(.05);

      seg.setInputNormals(normal);
      // Obtain the plane inliers and coefficients
      //pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
      //pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
      pcl::ModelCoefficients::Ptr Coefficients(new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr InliersPlane(new pcl::PointIndices);
      seg.segment(*InliersPlane, *Coefficients);

      //cout<<InliersPlane->indices.size() <<" vs. "<<inCloud->points.size()<<endl;
      if((double)InliersPlane->indices.size()/(double)inCloud->points.size() > 0.2) {
        Matrix[index]() = ARR(outCoefficients[i]->values[0], outCoefficients[i]->values[1], outCoefficients[i]->values[2]);
        index ++;
        if(index==3) break;
      }
    }

  }

  Matrix = ~Matrix;
  cout<<" DETECTED " << index <<" ORTHO. COUPLES" <<endl;
  if(index<3) isBox = false;

  if(isBox) {
    arr U, V, D;
    svd(U, D, V, Matrix);
//          cout <<"SVD = " <<Matrix <<endl<<U <<endl<<D <<endl<<V <<endl;
    arr Transform = ~(U*~V);
//          cout <<"svd check:\n" <<Matrix - (U*diag(D)*~V) <<endl;
//          cout <<"inverse check:\n" <<Transform * (U*~V) <<endl;
    if(trace(Transform)<0.) Transform *= -1.;
    Eigen::Matrix< float, 4, 4 > m;
    m.setZero();
    m(3, 3)=1.;
    for(int k=0; k<3; k++)
      for(int j=0; j<3; j++)
        m(k, j) = Transform(k, j);

    pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>);

    pcl::transformPointCloud(*inCloud, *transformed_cloud, m);

    PointT proj_min;
    PointT proj_max;
    pcl::getMinMax3D(*transformed_cloud, proj_min, proj_max);
    //arr center(3);

    length = ARR(proj_max.x - proj_min.x, proj_max.y - proj_min.y, proj_max.z - proj_min.z);

    center(0) = 0.5*(proj_min.x + proj_max.x);
    center(1) = 0.5*(proj_min.y + proj_max.y);
    center(2) = 0.5*(proj_min.z + proj_max.z);

    Transform = ~Transform;
    center      = Transform * center;
    orientation.setMatrix(Transform.p);
    orientation.normalize();
//          cout <<"T=\n" <<Transform <<"\nquat=" <<orientation <<endl;
  }

  return isBox;
}

void fittingBoundingBox(pcl::PointCloud<PointT>::Ptr inCloud, PointT& min, PointT& max) {
  pcl::PCA< PointT > pca;
  pcl::PointCloud< PointT >::ConstPtr cloud;
  pcl::PointCloud< PointT > proj;

  //Cloud proj;
  pca.setInputCloud(cloud);
  pca.project(*cloud, proj);

  PointT proj_min;
  PointT proj_max;
  pcl::getMinMax3D(proj, proj_min, proj_max);

  pca.reconstruct(proj_min, min);
  pca.reconstruct(proj_max, max);

}

#endif
