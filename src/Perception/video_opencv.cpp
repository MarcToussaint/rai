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


#include "videoEncoder.h"
#ifdef MT_OPENCV

#undef COUNT
#include <opencv2/opencv.hpp>
#undef MIN
#undef MAX
#include "opencv.h"

struct sVideoEncoder_OpenCV{
  CvVideoWriter *video;
  const char* filename;
  uint fps;
  uint numFrames; //,width,height;
  sVideoEncoder_OpenCV():video(NULL){}
  void open(uint width, uint height);
};

void sVideoEncoder_OpenCV::open(uint width, uint height){
  numFrames=0;
  //s->width = width;
//  s->height = height;
  video = cvCreateVideoWriter(filename, CV_FOURCC('X','V','I','D'), fps , cvSize(width, height), true);
}

VideoEncoder_OpenCV::VideoEncoder_OpenCV(const char* filename, uint fps){
  s = new sVideoEncoder_OpenCV;
  s->filename = filename;
  s->fps = fps;
}

void VideoEncoder_OpenCV::addFrame(const byteA& img){
  if(!s->video) s->open(img.d1, img.d0);
  IplImage ipl_img;
  cv::Mat ref=cvMAT(img);
  cvGetImage(&ref, &ipl_img);
  cvWriteFrame(s->video, &ipl_img);
//  s->video <<cvMAT(img);
  s->numFrames++;
}

void VideoEncoder_OpenCV::close(){
  cvReleaseVideoWriter(&s->video);
}

#else //MT_OPENCV

#include <Core/util.h>
  VideoEncoder_OpenCV::VideoEncoder_OpenCV(const char* filename, uint fps){ MT_MSG("WARNING - using dummy Revel module"); };
  void VideoEncoder_OpenCV::addFrame(const byteA& img){};
  void VideoEncoder_OpenCV::close(){};

#endif //MT_OPENCV
