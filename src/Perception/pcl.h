/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Core/array.h"
#include "../Core/thread.h"

#ifdef RAI_PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//namespace pcl {
//struct PointXYZ;
//struct PointXYZRGB;
//template<class T> struct PointCloud;
//typedef boost::shared_ptr<PointCloud<PointT> > Ptr;
//}
typedef pcl::PointCloud<pcl::PointXYZ> Pcl;
typedef pcl::PointCloud<pcl::PointXYZRGB> PclC;

Pcl::Ptr conv_ArrCloud_PclCloud(const arr& pts);
void conv_ArrCloud_PclCloud(Pcl& cloud, const arr& pts);
void conv_PclCloud_ArrCloud(arr& pts, const Pcl& cloud);

PclC::Ptr conv_ArrCloud_PclCloud(const arr& pts, const byteA& rgb);
void conv_ArrCloud_PclCloud(PclC& cloud, const arr& pts, const byteA& rgb);
void conv_PclCloud_ArrCloud(arr& pts, byteA& rgb, const PclC& cloud);

void conv_ArrCloud_PclCloud(PclC& cloud, const arr& pts, const arr& rgb);
void conv_PclCloud_ArrCloud(arr& pts, arr& rgb, const PclC& cloud);

inline arr PclPoints(const PclC& cloud) { arr pts; conv_PclCloud_ArrCloud(pts, NoByteA, cloud); return pts; }

arr conv_PclNormals_Arr(const pcl::PointCloud<pcl::Normal>::Ptr& normals);

#endif
