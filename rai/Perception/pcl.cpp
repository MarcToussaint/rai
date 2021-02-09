/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "pcl.h"

#ifdef RAI_PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

void conv_ArrCloud_PclCloud(Pcl& cloud,
                            const arr& pts) {
  CHECK((pts.nd==3 && pts.d2==3) || (pts.nd==2 && pts.d1==3), "");
  cloud.resize(pts.N/3);
  CHECK_EQ(cloud.size(), pts.N/3, "");
  double* p=pts.p;
  for(pcl::PointXYZ& pt:cloud) {
    pt.x = (*p++); //pts.elem(i*3+0);
    pt.y = (*p++); //pts.elem(i*3+1);
    pt.z = (*p++); //pts.elem(i*3+2);
  }
}

void conv_PclCloud_ArrCloud(arr& pts,
                            const Pcl& cloud) {
  double* p=nullptr;
  if(!!pts) {
    pts.resize(cloud.size(), 3);
    p=pts.p;
  }

  for(const pcl::PointXYZ& pt:cloud) {
    if(p) {
      *(p++) = pt.x;
      *(p++) = pt.y;
      *(p++) = pt.z;
    }
  }
  if(p) CHECK_EQ(p, pts.p+pts.N, "");
}

arr conv_PclNormals_Arr(const pcl::PointCloud<pcl::Normal>::Ptr& normals){
  arr N(normals->size(), 3);
  double *p=N.p;

  for(const pcl::Normal& no:*normals) {
    if(p) {
      *(p++) = no.normal_x;
      *(p++) = no.normal_y;
      *(p++) = no.normal_z;
    }
  }
  CHECK_EQ(p, N.p+N.N, "");
  return N;
}


void conv_ArrCloud_PclCloud(PclC& cloud,
                            const arr& pts, const byteA& rgb) {
  CHECK((pts.nd==3 && pts.d2==3) || (pts.nd==2 && pts.d1==3), "");
  cloud.resize(pts.N/3);
  CHECK_EQ(cloud.size(), pts.N/3, "");
  double* p=pts.p;
  byte* c=rgb.p;
  for(pcl::PointXYZRGB& pt:cloud) {
    pt.x = (*p++); //pts.elem(i*3+0);
    pt.y = (*p++); //pts.elem(i*3+1);
    pt.z = (*p++); //pts.elem(i*3+2);
    pt.r = (*c++); //255.*cols.elem(i*3+0);
    pt.g = (*c++); //255.*cols.elem(i*3+1);
    pt.b = (*c++); //255.*cols.elem(i*3+2);
//    i++;
  }
}

Pcl::Ptr conv_ArrCloud_PclCloud(const arr& pts){
  Pcl::Ptr pcl(new Pcl);
  conv_ArrCloud_PclCloud(*pcl, pts);
  return pcl;
}

PclC::Ptr conv_ArrCloud_PclCloud(const arr& pts, const byteA& rgb){
  PclC::Ptr pcl(new PclC);
  conv_ArrCloud_PclCloud(*pcl, pts, rgb);
  return pcl;
}

void conv_PclCloud_ArrCloud(arr& pts,
                            byteA& rgb,
                            const PclC& cloud) {
  double* p=nullptr;
  byte* c=nullptr;
  if(!!pts) {
    pts.resize(cloud.size(), 3);
    p=pts.p;
  }
  if(!!rgb) {
    rgb.resize(cloud.size(), 3);
    c=rgb.p;
  }

  for(const pcl::PointXYZRGB& pt:cloud) {
    if(p) {
      *(p++) = pt.x;
      *(p++) = pt.y;
      *(p++) = pt.z;
    }
    if(c) {
      *(c++) = pt.r;
      *(c++) = pt.g;
      *(c++) = pt.b;
    }
  }
  if(p) CHECK_EQ(p, pts.p+pts.N, "");
  if(c) CHECK_EQ(c, rgb.p+rgb.N, "");
}

void conv_ArrCloud_PclCloud(PclC& cloud,
                            const arr& pts, const arr& rgb) {
  CHECK((pts.nd==3 && pts.d2==3) || (pts.nd==2 && pts.d1==3), "");
  cloud.resize(pts.N/3);
  CHECK_EQ(cloud.size(), pts.N/3, "");
  double* p=pts.p;
  double* c=rgb.p;
  for(pcl::PointXYZRGB& pt:cloud) {
    pt.x = (*p++); //pts.elem(i*3+0);
    pt.y = (*p++); //pts.elem(i*3+1);
    pt.z = (*p++); //pts.elem(i*3+2);
    pt.r = 255.*(*c++); //255.*cols.elem(i*3+0);
    pt.g = 255.*(*c++); //255.*cols.elem(i*3+1);
    pt.b = 255.*(*c++); //255.*cols.elem(i*3+2);
//    i++;
  }
}

void conv_PclCloud_ArrCloud(arr& pts,
                            arr& rgb,
                            const PclC& cloud) {
  double* p=nullptr;
  double* c=nullptr;
  if(!!pts) {
    pts.resize(cloud.size(), 3);
    p=pts.p;
  }
  if(!!rgb) {
    rgb.resize(cloud.size(), 3);
    c=rgb.p;
  }

  for(const pcl::PointXYZRGB& pt:cloud) {
    if(p) {
      *(p++) = pt.x;
      *(p++) = pt.y;
      *(p++) = pt.z;
    }
    if(c) {
      *(c++) = pt.r/255.;
      *(c++) = pt.g/255.;
      *(c++) = pt.b/255.;
    }
  }
  if(p) CHECK_EQ(p, pts.p+pts.N, "");
  if(c) CHECK_EQ(c, rgb.p+rgb.N, "");
}

#endif //RAI_PCL
