#pragma once

#ifdef MLR_PCL

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Geo/geo.h>
#include <Gui/opengl.h>
#include <Kin/kin.h>
#include <Core/thread.h>

#include "methods.h"

typedef pcl::PointXYZRGB PointT;

void glDrawPrimitives(void* classP);

struct Primitive{
  mlr::Transformation X;
  virtual void glDraw() = 0;
  Primitive(){ X.setZero(); }
  virtual ~Primitive(){}
};

struct Plane:Primitive{
  float nx,ny,nz,c;
  Plane(float _nx, float _ny, float _nz, float _c):nx(_nx),ny(_ny),nz(_nz),c(_c){}
  ~Plane(){}
  void glDraw();
};

struct PclCloudView:Primitive{
  pcl::PointCloud<PointT>::Ptr cloud;
  arr pts,cols;
  PclCloudView(const pcl::PointCloud<PointT>::Ptr& _cloud):cloud(_cloud){}
  void glDraw();
};

struct ArrCloudView:Primitive{
  Access<arr>& pts;
  Access<arr>& cols;
  ArrCloudView(Access<arr>& _pts,  Access<arr>& _cols):pts(_pts), cols(_cols){}
  void glDraw();
};

struct DisplayPrimitives : GLDrawer{
  mlr::Array<Primitive*> P;
  mlr::KinematicWorld G;
  arr pc[2];

  void glDraw(struct OpenGL&);
};

#endif
