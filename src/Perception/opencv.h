/*  ---------------------------------------------------------------------
    Copyright 2012 Marc Toussaint
    email: mtoussai@cs.tu-berlin.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */


#ifndef MT_opencv_h
#define MT_opencv_h

//OpenCV (C++) wrappers

#ifdef MT_OPENCV

#undef COUNT
//#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#undef MIN
#undef MAX
#include <Core/array.h>
#include <Core/util.h>

inline cv::Mat cvMAT(const byteA& img){
  if(img.nd==2) return cv::Mat(img.d0, img.d1, CV_8UC1, img.p);
  if(img.nd==3) return cv::Mat(img.d0, img.d1, CV_8UC3, img.p);
  return cv::Mat();
}

inline cv::Mat cvMAT(const floatA& img){
  if(img.nd==1) return cv::Mat(img.d0, 1, CV_32FC3, img.p);
  if(img.nd==2) return cv::Mat(img.d0, img.d1, CV_32FC1, img.p);
  if(img.nd==3) return cv::Mat(img.d0, img.d1, CV_32FC3, img.p);
  return cv::Mat();
}

inline cv::Mat cvMAT(const doubleA& img){
  if(img.nd==2) return cv::Mat(img.d0, img.d1, CV_64FC1, img.p);
  if(img.nd==3) return cv::Mat(img.d0, img.d1, CV_64FC3, img.p);
  return cv::Mat();
}

inline byteA cvtMAT(const cv::Mat& mat){
  CHECK(mat.dims==2,"");
  if(mat.elemSize()==1) return byteA(mat.data, mat.total());
  if(mat.elemSize()==3) return byteA(mat.data, 3*mat.total()).reshape(mat.rows, mat.cols, 3);
  NIY;
  return byteA();
}

char cvShow(const byteA& img, const char *window="opencv", bool wait=false);
char cvShow(const floatA& img, const char *window="opencv", bool wait=false);
void getDiffProb(floatA& diff, const byteA& img0, const byteA& img1, float pixSdv, uint range);

#else

inline cv::Mat cvMAT(const byteA& img){ NICO }
inline cv::Mat cvMAT(const floatA& img){ NICO }
inline cv::Mat cvMAT(const doubleA& img){ NICO }
inline byteA cvtMAT(const cv::Mat& mat){ NICO }

#endif //MT_OPENCV

#endif
