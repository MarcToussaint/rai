#include "kinect2pointCloud.h"

REGISTER_MODULE(Kinect2PointCloud)

const unsigned int image_width = 640; //kinect resolution
const unsigned int image_height = 480; //kinect resolution
const unsigned int depth_size = image_width*image_height;

void Kinect2PointCloud::step(){
  copy(depth, kinect_depth.get()());
  rgb = kinect_rgb.get();

  if(depth.N!=image_width*image_height || rgb.N!=3*image_width*image_height){
    MT_MSG("here" <<depth.getDim() <<' ' <<kinect_depth.get()->getDim());
    return;
  }

  rgb.reshape(image_width*image_height, 3);
  pts.resize(image_width*image_height, 3);
  cols.resize(image_width*image_height, 3);

  float constant = 1.0f / 580; //focal length of kinect in pixels
  int centerX = (image_width >> 1);
  int centerY = (image_height >> 1);

  int value_idx = 0;
  int point_idx = 0;
  for (int v = -centerY; v < centerY; ++v) {
    for (int u = -centerX; u < centerX; ++u, ++value_idx, ++point_idx) {
      double d=depth.elem(value_idx);
      if (d!= 0 && d!=2047) {
        double z=(double) d * 0.001;
        pts(point_idx, 0) = z*constant*u;
        pts(point_idx, 1) = z*constant*v;
        pts(point_idx, 2) = z;

        cols(point_idx, 0) = (double)rgb(point_idx, 0)/255.;
        cols(point_idx, 1) = (double)rgb(point_idx, 1)/255.;
        cols(point_idx, 2) = (double)rgb(point_idx, 2)/255.;
      }
    }
  }

  kinect_points.set() = pts;
  kinect_pointColors.set() = cols;
}

