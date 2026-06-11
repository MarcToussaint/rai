/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_OPENCV

#include "opencv.h"
#include "opencvCamera.h"

struct sOpencvCamera {  cv::VideoCapture capture;  };

OpencvCamera::OpencvCamera(const char* _name, int _cameraID)
  : Thread(STRING("OpencvCamera_"<<_name), 0.),
    cameraID(_cameraID) {
  self = make_unique<sOpencvCamera>();
  threadLoop();
}

OpencvCamera::~OpencvCamera() {
  threadClose();
}

void OpencvCamera::open() {
  self->capture.open(cameraID);
  // for(std::map<int, double>::const_iterator i = properties.begin(); i != properties.end(); ++i) {
  //   if(!self->capture.set(i->first, i->second)) {
  //     cout << "could not set property " << i->first << " to value " << i->second << endl;
  //   }
  // }

  // self->capture.set(cv::CAP_PROP_CONVERT_RGB, 1);
  self->capture.set(cv::CAP_PROP_FRAME_WIDTH,  2560);
  self->capture.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
  // self->capture.get(cv::CAP_PROP_FPS);
  //    cout <<"FPS of opened OpenCV VideoCapture = " <<capture.get(CV_CAP_PROP_FPS) <<endl;;
}

void OpencvCamera::close() {
  self->capture.release();
}

void OpencvCamera::step() {
  cv::Mat img;
  self->capture.read(img);
  if(!img.empty()) {
    if(flip_bgr) cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
    image.set() = conv_cvMat2byteA(img);
  }
}

#else //OPENCV

#include "opencvCamera.h"

struct sOpencvCamera {};

OpencvCamera::OpencvCamera(const char* _name, int _cameraID)
  : Thread(STRING("OpencvCamera_"<<_name), 0.),
    cameraID(_cameraID),
    image(this) { NICO }
OpencvCamera::~OpencvCamera() { NICO }
void OpencvCamera::open() { NICO }
void OpencvCamera::step() { NICO }
void OpencvCamera::close() { NICO }

#endif
