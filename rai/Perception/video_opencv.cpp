/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_OPENCV

#undef COUNT
#include <opencv2/opencv.hpp>
#undef MIN
#undef MAX
#include "videoEncoder.h"
#include "opencv.h"

struct sVideoEncoder_OpenCV {
  CvVideoWriter* video;
  const char* filename;
  uint fps;
  uint numFrames; //,width,height;
  sVideoEncoder_OpenCV():video(nullptr) {}
  void open(uint width, uint height);
};

void sVideoEncoder_OpenCV::open(uint width, uint height) {
  numFrames=0;
  //self->width = width;
//  self->height = height;
//  video = cvCreateVideoWriter(filename, CV_FOURCC('X','V','I','D'), fps , cvSize(width, height), true);
  HALT("CV_FOURCC is obsolete - needs fix");
}

VideoEncoder_OpenCV::VideoEncoder_OpenCV(const char* filename, uint fps) {
  self = make_unique<sVideoEncoder_OpenCV>();
  self->filename = filename;
  self->fps = fps;
}

void VideoEncoder_OpenCV::addFrame(const byteA& img) {
  if(!self->video) self->open(img.d1, img.d0);
#if 1
  HALT("IplImage is obsolete needs fix");
#else
  IplImage ipl_img;
  cv::Mat ref=conv_Arr2CvRef(img);
  cvGetImage(&ref, &ipl_img);
  cvWriteFrame(self->video, &ipl_img);
//  self->video <<conv_Arr2CvRef(img);
  self->numFrames++;
#endif
}

void VideoEncoder_OpenCV::close() {
#if 1
  HALT("cvReleaseVideoWriter is obsolete needs fix");
#else
  cvReleaseVideoWriter(&self->video);
#endif
}

#else //RAI_OPENCV

#include "../Core/util.h"
#include "videoEncoder.h"

struct sVideoEncoder_OpenCV {};

VideoEncoder_OpenCV::VideoEncoder_OpenCV(const char* filename, uint fps) { RAI_MSG("WARNING - using dummy Revel module"); };
void VideoEncoder_OpenCV::addFrame(const byteA& img) {};
void VideoEncoder_OpenCV::close() {};

#endif //RAI_OPENCV
