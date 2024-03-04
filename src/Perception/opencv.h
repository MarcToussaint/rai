/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

//OpenCV (C++) wrappers

#ifdef RAI_OPENCV

#undef COUNT
#include <opencv2/opencv.hpp>
//#include "opencv2/highgui.hpp"
//#include "opencv2/imgcodecs.hpp"
//#include "opencv2/imgproc.hpp"
#undef MIN
#undef MAX

#include "../Core/array.h"
#include "../Core/util.h"

extern ::Mutex cvMutex;

inline cv::Mat CV(const byteA& img) {
  if(img.nd==2) return cv::Mat(img.d0, img.d1, CV_8UC1, img.p);
  if(img.nd==3) return cv::Mat(img.d0, img.d1, CV_8UC3, img.p);
  if(img.nd==4) return cv::Mat(img.d0, img.d1, CV_8UC4, img.p);
  return cv::Mat();
}

inline cv::Mat CV(const floatA& img) {
  if(img.nd==1) return cv::Mat(img.d0, 1, CV_32FC3, img.p);
  if(img.nd==2) return cv::Mat(img.d0, img.d1, CV_32FC1, img.p);
  if(img.nd==3) return cv::Mat(img.d0, img.d1, CV_32FC3, img.p);
  return cv::Mat();
}

inline cv::Mat CV(const arr& img) {
  if(img.nd==2) return cv::Mat(img.d0, img.d1, CV_64FC1, img.p);
  if(img.nd==3) return cv::Mat(img.d0, img.d1, CV_64FC3, img.p);
  return cv::Mat();
}

inline byteA conv_cvMat2byteA(const cv::Mat& mat) {
  CHECK_EQ(mat.dims, 2, "");
  if(mat.elemSize()==1) return byteA().referTo(mat.data, mat.total());
  if(mat.elemSize()==3) return byteA().referTo(mat.data, 3*mat.total()).reshape(mat.rows, mat.cols, 3);
  NIY;
  return byteA();
}

inline floatA conv_cvMat2floatA(const cv::Mat& mat) {
  CHECK_EQ(mat.dims, 2, "");
  floatA X(mat.rows, mat.cols);
  if(mat.isContinuous()) {
    X.setCarray((float*)mat.data, X.N);
  } else {
    for(int i=0; i<mat.rows; i++) X[i].setCarray((float*)mat.ptr<uchar>(i), mat.cols);
  }
  return X;
}

char cvShow(const byteA& img, const char* window="opencv", bool wait=false);
char cvShow(const floatA& img, const char* window="opencv", bool wait=false);
void getDiffProb(floatA& diff, const byteA& img0, const byteA& img1, float pixSdv, uint range);

#else

#include "../Core/array.h"
inline char cvShow(const byteA& img, const char* window="opencv", bool wait=false) { NICO }
inline char cvShow(const floatA& img, const char* window="opencv", bool wait=false) { NICO }
inline void getDiffProb(floatA& diff, const byteA& img0, const byteA& img1, float pixSdv, uint range) { NICO }

#endif //RAI_OPENCV
