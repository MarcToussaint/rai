/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_OPENCV

#include "opencv.h"
#include "opencvCamera.h"

struct sOpencvCamera {  cv::VideoCapture capture;  };

OpencvCamera::OpencvCamera(const Var<byteA>& _rgb)
  : Thread(STRING("OpencvCamera_"<<_rgb.name()), 0.)
  , rgb(this, _rgb) {
  self = make_unique<sOpencvCamera>();
  threadLoop();
}

OpencvCamera::~OpencvCamera() {
  threadClose();
}

void OpencvCamera::open() {
  self->capture.open(0);
  for(std::map<int, double>::const_iterator i = properties.begin(); i != properties.end(); ++i) {
    if(!self->capture.set(i->first, i->second)) {
      cerr << "could not set property " << i->first << " to value " << i->second << endl;
    }
  }
  //    capture.set(CV_CAP_PROP_CONVERT_RGB, 1);
  //    cout <<"FPS of opened OpenCV VideoCapture = " <<capture.get(CV_CAP_PROP_FPS) <<endl;;
}

void OpencvCamera::close() {
  self->capture.release();
}

void OpencvCamera::step() {
  cv::Mat img; //,imgRGB;
  self->capture.read(img);
  if(!img.empty()) {
//    cv::cvtColor(img, imgRGB, CV_BGR2RGB);
    rgb.set() = conv_cvMat2byteA(img);
  }
}

bool OpencvCamera::set(int propId, double value) {
  if(self)
    return self->capture.set(propId, value);
  else {
    properties[propId] = value;
    return true; // well, can't really do anything else here...
  }
}

#endif
