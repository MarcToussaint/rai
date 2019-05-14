#include "opencv.h"

#include "opencvCamera.h"

#ifdef RAI_OPENCV

struct sOpencvCamera {  cv::VideoCapture capture;  };

OpencvCamera::OpencvCamera(const Var<byteA>& _rgb)
  : Thread(STRING("OpencvCamera_"<<_rgb.name()), 0.)
  , rgb(this, _rgb) {
  s = make_shared<sOpencvCamera>();
  threadLoop();
}

OpencvCamera::~OpencvCamera(){
  threadClose();
}


void OpencvCamera::open() {
  s->capture.open(0);
  for(std::map<int,double>::const_iterator i = properties.begin(); i != properties.end(); ++i) {
    if(!s->capture.set(i->first, i->second)) {
      cerr << "could not set property " << i->first << " to value " << i->second << endl;
    }
  }
  //    capture.set(CV_CAP_PROP_CONVERT_RGB, 1);
  //    cout <<"FPS of opened OpenCV VideoCapture = " <<capture.get(CV_CAP_PROP_FPS) <<endl;;
}

void OpencvCamera::close() {
  s->capture.release();
}

void OpencvCamera::step() {
  cv::Mat img; //,imgRGB;
  s->capture.read(img);
  if(!img.empty()) {
//    cv::cvtColor(img, imgRGB, CV_BGR2RGB);
    rgb.set() = conv_cvMat2byteA(img);
  }
}

bool OpencvCamera::set(int propId, double value) {
  if(s)
    return s->capture.set(propId, value);
  else {
    properties[propId] = value;
    return true; // well, can't really do anything else here...
  }
}

#endif
