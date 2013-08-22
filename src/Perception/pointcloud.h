#ifndef _POINTCLOUD_H__
#define _POINTCLOUD_H__

#ifdef PCL

#include <System/biros.h>
#include <devTools/logging.h>
#include <hardware/kinect.h>
#include <motion/motion.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>

SET_LOG(pointcloud, INFO);

typedef pcl::PointXYZRGBA PointT;
typedef MT::Array<pcl::PointCloud<PointT>::Ptr> PointCloudL;
typedef pcl::PointCloud<PointT>::Ptr FittingJob;
typedef pcl::ModelCoefficients::Ptr FittingResult;
typedef MT::Array<FittingResult> FittingResultL;

struct ObjectBeliefSet;

// -- Variables

class PointCloudVar : public Variable {
  public:
    PointCloudVar(const char* name);
    FIELD(pcl::PointCloud<PointT>::ConstPtr, point_cloud);
    pcl::PointCloud<PointT>::Ptr get_point_cloud_copy(Process *p) { readAccess(p); pcl::PointCloud<PointT>::Ptr tmp = point_cloud->makeShared(); deAccess(p); return tmp; }
};

class PointCloudSet : public Variable {
  public:
    PointCloudSet(const char* name) : Variable(name) { reg_point_clouds(); }
    FIELD(PointCloudL, point_clouds);
};

class ObjectSet : public Variable {
  public:
    ObjectSet(const char *name) : Variable(name) { reg_objects(); }
    FIELD(FittingResultL, objects);
};

// -- Processes

class ObjectClusterer : public Process {
  public:
    ObjectClusterer();
    PointCloudVar* data_3d;
    PointCloudSet* point_clouds;

    void open();
    void step();
    void close();
};

class ObjectFitter : public Process {
  struct sObjectFitter* s;
  public:
    ObjectFitter();

    void open();
    void step();
    void close();

    PointCloudSet* objectClusters;
    ObjectSet *objects;
};


class ObjectFilter : public Process {
  public:
    struct sObjectFilter *s;
    ObjectFilter(const char *name) ;
    void open();
    void step();
    void close() {};

    ObjectSet* in_objects;
    ObjectBeliefSet* out_objects;
};

class ObjectTransformator : public Process {
  public:
    ObjectTransformator(const char *name);
    void open();
    void step();
    void close() {};

    ObjectBeliefSet* kinect_objects;
    WorkingCopy<GeometricState> geo;
};

#endif // PCL
#endif // _POINTCLOUD_H__
