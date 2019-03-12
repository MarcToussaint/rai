/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

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
  CvVideoWriter *video;
  const char* filename;
  uint fps;
  uint numFrames; //,width,height;
  sVideoEncoder_OpenCV():video(NULL) {}
  void open(uint width, uint height);
};

void sVideoEncoder_OpenCV::open(uint width, uint height) {
  numFrames=0;
  //s->width = width;
//  s->height = height;
//  video = cvCreateVideoWriter(filename, CV_FOURCC('X','V','I','D'), fps , cvSize(width, height), true);
  HALT("CV_FOURCC is obsolete - needs fix");
}

VideoEncoder_OpenCV::VideoEncoder_OpenCV(const char* filename, uint fps) {
  s = new sVideoEncoder_OpenCV;
  s->filename = filename;
  s->fps = fps;
}

void VideoEncoder_OpenCV::addFrame(const byteA& img) {
  if(!s->video) s->open(img.d1, img.d0);
#if 1
  HALT("IplImage is obsolete needs fix");
#else
  IplImage ipl_img;
  cv::Mat ref=conv_Arr2CvRef(img);
  cvGetImage(&ref, &ipl_img);
  cvWriteFrame(s->video, &ipl_img);
//  s->video <<conv_Arr2CvRef(img);
  s->numFrames++;
#endif  
}

void VideoEncoder_OpenCV::close() {
#if 1
  HALT("cvReleaseVideoWriter is obsolete needs fix");
#else
  cvReleaseVideoWriter(&s->video);
#endif
}

#else //RAI_OPENCV

#include <Core/util.h>
VideoEncoder_OpenCV::VideoEncoder_OpenCV(const char* filename, uint fps) { RAI_MSG("WARNING - using dummy Revel module"); };
void VideoEncoder_OpenCV::addFrame(const byteA& img) {};
void VideoEncoder_OpenCV::close() {};

#endif //RAI_OPENCV
