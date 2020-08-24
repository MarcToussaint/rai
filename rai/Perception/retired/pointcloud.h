/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifndef _POINTCLOUD_H__
#define _POINTCLOUD_H__

#ifdef RAI_PCL

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include "../Core/thread.h"
//#include <devTools/logging.h>
//#include <Hardware/kinect.h>
#include "../KOMO/komo.h"

//SET_LOG(pointcloud, INFO)

typedef pcl::PointXYZRGBA PointT;
typedef rai::Array<pcl::PointCloud<PointT>::Ptr> PointCloudL;
typedef pcl::PointCloud<PointT>::Ptr FittingJob;
typedef pcl::ModelCoefficients::Ptr FittingResult;
typedef rai::Array<FittingResult> FittingResultL;
const int RADIUS = 2;
const int HEIGHT = 3;

struct ObjectBelief {

  ObjectBelief() {
    shapeParams.resize(4);
  }
  //pose
  // TODO: make pointers
  rai::Vector position;
  rai::Quaternion rotation;

  arr poseCov;

  // primitive shapes
  rai::ShapeType shapeType;
  arr shapeParams;

  // TODO: make pointer, such that the using app does not need to implicitly
  // include half of the PCL?
  //pcl::ModelCoefficients::Ptr pcl_object;

  //pcl::PointCloud<PointT>* pointCloud;
  arr vertices;
  uintA triangles;
};

typedef rai::Array<ObjectBelief*> ObjectBeliefSet;
typedef pcl::PointCloud<PointT>::Ptr PointCloudVar;
typedef PointCloudL PointCloudSet;
typedef FittingResultL ObjectSet;

// -- Processes

struct ObjectClusterer : public Module {
  ObjectClusterer();
  VAR(PointCloudVar, data_3d)
  VAR(PointCloudSet, point_clouds)

  void open();
  void step();
  void close();
};

struct ObjectFitter : public Module {
  unique_ptr<struct sObjectFitter> self;

  ObjectFitter();

  void open();
  void step();
  void close();

  VAR(PointCloudSet, objectClusters)
  VAR(ObjectSet, objects)
};

struct ObjectFilter : public Module {
  unique_ptr<struct sObjectFilter> self;
  ObjectFilter(const char* name) ;
  void open();
  void step();
  void close() {}

  VAR(ObjectSet, in_objects)
  VAR(ObjectBeliefSet, out_objects)
};

struct ObjectTransformator : public Module {
  ObjectTransformator(const char* name);
  void open();
  void step();
  void close() {}

  VAR(ObjectBeliefSet, kinect_objects)
  VAR(rai::Configuration, geoState)
  rai::Configuration geo;
};

#endif // RAI_PCL
#endif // _POINTCLOUD_H__
