/*  ------------------------------------------------------------------
    Copyright (c) 2019 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Core/array.h"
#include "../Core/thread.h"

namespace pcl {
struct PointXYZ;
struct PointXYZRGB;
template<class T> struct PointCloud;
}
typedef pcl::PointCloud<pcl::PointXYZ> Pcl;
typedef pcl::PointCloud<pcl::PointXYZRGB> PclC;

void conv_ArrCloud_PclCloud(Pcl& cloud, const arr& pts);
void conv_PclCloud_ArrCloud(arr& pts, const Pcl& cloud);

void conv_ArrCloud_PclCloud(PclC& cloud, const arr& pts, const byteA& rgb);
void conv_PclCloud_ArrCloud(arr& pts, byteA& rgb, const PclC& cloud);

void conv_ArrCloud_PclCloud(PclC& cloud, const arr& pts, const arr& rgb);
void conv_PclCloud_ArrCloud(arr& pts, arr& rgb, const PclC& cloud);

inline arr PclPoints(const PclC& cloud) { arr pts; conv_PclCloud_ArrCloud(pts, NoByteA, cloud); return pts; }

