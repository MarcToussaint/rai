/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "perception.h"
#include "audio.h"
#include "videoEncoder.h"
#include "../Core/util.ipp"
#include "../Gui/opengl.h"

void lib_Perception() { RAI_MSG("loading"); }

//REGISTER_MODULE (PointCloudViewer)
//REGISTER_MODULE (VideoEncoder)
//REGISTER_MODULE (VideoEncoderX264)
//REGISTER_MODULE (AudioReader)
//REGISTER_MODULE (AudioWriter)

//===========================================================================
//
// VideoEncoder
//

struct sVideoEncoder {
  rai::String filename;
  VideoEncoder_libav_simple video;
  ofstream timeTagFile;
  byteA buffer;

  sVideoEncoder(const char* _filename, double fps, bool is_rgb=false):filename(_filename), video(filename.p, fps, 0, is_rgb) {
    timeTagFile.open(STRING(filename <<".times"));
  }
};

void VideoEncoder::open() {
  self = make_unique<sVideoEncoder>(STRING("z." <<img.data->name <<'.' <<rai::getNowString() <<".avi"), fps, is_rgb);
}

void VideoEncoder::close() {
  self->video.close();
  self.reset();
}

void VideoEncoder::step() {
  //-- grab from shared memory (necessary?)
  uint rev = img.readAccess();
  double time = img.data->write_time;
  self->buffer = img();
  img.deAccess();

  //save image
  self->video.addFrame(self->buffer);

  //save time tag
  rai::String tag;
  tag.resize(30, false);
  sprintf(tag.p, "%6i %13.6f", rev, time);
  self->timeTagFile <<tag <<endl;
}

//===========================================================================
//
// VideoEncoder
//

#if 0
struct sVideoEncoderX264 {
  rai::String filename;
  VideoEncoder_x264_simple video;
  ofstream timeTagFile;
  byteA buffer;
  int revision;
  sVideoEncoderX264(const char* _filename, double fps, bool is_rgb) : filename(_filename), video(filename.p, fps, 0, is_rgb), revision(-1) {
    timeTagFile.open(STRING(filename <<".times"));
  }
};

void VideoEncoderX264::open() {
  s = new sVideoEncoderX264(STRING("z." <<img.data->name <<'.' <<rai::getNowString() <<".264"), fps, is_rgb);
}

void VideoEncoderX264::close() {
  std::clog << "Closing VideoEncoderX264...";
  self->video.close();
  self.reset();
  std::clog << "done" << endl;
}

void VideoEncoderX264::step() {
  //-- grab from shared memory (necessary?)
  int nextRevision = img.readAccess();
  double time = img.data->write_time;
  self->buffer = img();
  img.deAccess();

  //save image
  self->video.addFrame(self->buffer);

  //save time tag
  rai::String tag;
  tag.resize(30, false);
  sprintf(tag.p, "%6i %13.6f", self->revision, time);
  self->timeTagFile <<tag <<endl;
  self->revision = nextRevision;
}
#endif

//===========================================================================
//
// AudioReader and Writer
//

#if 0
void AudioReader::open() {
#ifdef HAVE_PULSEAUDIO
  poller = new AudioPoller_PA();
#else
  poller = nullptr;
#endif
}
void AudioReader::close() {
  if(poller != nullptr) {
    delete poller;
    poller = nullptr;
  }
}
void AudioReader::step() {
  if(poller == nullptr) {
    return;
  }
  pcms16ne2c.writeAccess();
  pcms16ne2c().resize(4096);
  poller->read(pcms16ne2c());
  pcms16ne2c.deAccess();
}

void AudioWriter::open() {
#ifdef HAVE_LIBAV
  writer = new AudioWriter_libav(STRING("z.audio" <<'.' <<rai::getNowString() <<".wav"));
#else
  writer = nullptr;
#endif
}
void AudioWriter::close() {
  if(writer != nullptr) {
    delete writer;
    writer = nullptr;
  }
}
void AudioWriter::step() {
  if(writer == nullptr) {
    return;
  }
  writer->writeSamples_R48000_2C_S16_NE(pcms16ne2c.get());
}

#ifdef RAI_OPENCV

#include "opencv.h"
#include "colorseg.h"

#undef COUNT
#include <opencv2/opencv.hpp>
//#include <opencv2/features2d/features2d.hpp>
//#include <opencv2/gpu/gpu.hpp>
#ifdef ARCH_LINUX
#include <opencv2/nonfree/nonfree.hpp>
#endif
#undef MIN
#undef MAX

//REGISTER_MODULE (OpencvCamera)
//REGISTER_MODULE (CvtGray)
//REGISTER_MODULE (CvtHsv)
//REGISTER_MODULE (HsvFilter)
//REGISTER_MODULE (MotionFilter)
//REGISTER_MODULE (DifferenceFilter)
//REGISTER_MODULE (CannyFilter)
//REGISTER_MODULE (Patcher)
//REGISTER_MODULE (SURFer)
//REGISTER_MODULE (HoughLineFilter)
//REGISTER_MODULE (ShapeFitter)

//===========================================================================
//
// Camera
//

struct sOpencvCamera {  cv::VideoCapture capture;  };

void OpencvCamera::open() {
  self = make_unique<sOpencvCamera>();
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
  self.reset();
}

void OpencvCamera::step() {
  cv::Mat img, imgRGB;
  self->capture.read(img);
  if(!img.empty()) {
    cv::cvtColor(img, imgRGB, CV_BGR2RGB);
    rgb.set()=conv_cvMat2byteA(imgRGB);
  }
}

bool OpencvCamera::set(int propId, double value) {
  if(s)
    return self->capture.set(propId, value);
  else {
    properties[propId] = value;
    return true; // well, can't really do anything else here...
  }
}

//===========================================================================
//
// CvtGray
//

void CvtGray::open() {}
void CvtGray::close() {}
void CvtGray::step() {
  byteA _rgb, _gray;
  _rgb = rgb.get();

  _gray.resize(_rgb.d0, _rgb.d1);

  if(!_rgb.N) return;
  cv::Mat ref=conv_Arr2CvRef(_gray);
  cv::Mat src=conv_Arr2CvRef(_rgb);
  cv::cvtColor(src, ref, CV_RGB2GRAY);

  gray.set() = _gray;
}

//===========================================================================
//
// CvtHsv
//

void CvtHsv::open() {}
void CvtHsv::close() {}
void CvtHsv::step() {
  byteA rgbA, hsvA;
  rgbA = rgb.get();

  hsvA.resizeAs(rgbA);

  if(!rgbA.N) return;
  cv::Mat ref=conv_Arr2CvRef(hsvA);
  cv::Mat src=conv_Arr2CvRef(rgbA);
  cv::cvtColor(src, ref, CV_RGB2HSV);

  hsv.set() = hsvA;
}

//===========================================================================
//
// HsvFilter
//

struct sHsvFilter {
  floatA hsvMean, hsvDeviation;
  float hsvDifference(const byteA& hsv) {
    float difference = 0.f;

    //measure hue distance circularly
    float tmp = hsvMean(0) - hsv(0);
    if(tmp < -128.f) tmp += 255.f;
    if(tmp > 128.f) tmp -= 255.f;

    // calculate squared z scores and sum up
    difference += (tmp / hsvDeviation(0)) * (tmp / hsvDeviation(0));
    difference += ((hsvMean(1) - hsv(1)) / hsvDeviation(1)) * ((hsvMean(1) - hsv(1)) / hsvDeviation(1));
    difference += ((hsvMean(2) - hsv(2)) / hsvDeviation(2)) * ((hsvMean(2) - hsv(2)) / hsvDeviation(2));
    return difference;
  }
};

void HsvFilter::open() {
  self = make_unique<sHsvFilter>();
  self->hsvMean      = rai::getParameter<floatA>("hsvMean");
  self->hsvDeviation = rai::getParameter<floatA>("hsvDeviation");
}

void HsvFilter::close() {
  self.reset();
}

void HsvFilter::step() {
  self->hsvMean      = rai::getParameter<floatA>("hsvMean");
  self->hsvDeviation = rai::getParameter<floatA>("hsvDeviation");

  byteA hsvA;
  hsvA = hsv.get();
  uint w=hsvA.d1, h=hsvA.d0;

  floatA evidence;
  evidence.resize(w*h);

  hsvA.reshape(w*h, 3);

  for(uint i = 0; i < evidence.N; ++i) {
    if(hsvA(i, 0) > 0 || hsvA(i, 1) > 0 || hsvA(i, 2) > 0) {
      evidence(i) = exp(-.5 * self->hsvDifference(hsvA[i]));
    }
  }

  evidence.reshape(1, h, w);
  evi.set() = evidence;
}

//===========================================================================
//
// MotionFilter
//

struct sMotionFilter {
  byteA old_rgb;
};

void MotionFilter::open() {
  self = make_unique<sMotionFilter>();
}

void MotionFilter::close() {
  self.reset();
}

void MotionFilter::step() {
  byteA rgbA, grayA;
  rgbA = rgb.get();
  uint H=rgbA.d0, W=rgbA.d1;

  if(self->old_rgb.N!=rgbA.N) {
    self->old_rgb=rgbA;
    return;
  }

  grayA.resize(rgbA.d0*rgbA.d1);
  rgbA.reshape(grayA.N, 3);
  self->old_rgb.reshape(grayA.N, 3);
  for(uint i=0; i<grayA.N; i++) {
    uint diff
      = abs((int)rgbA(i, 0)-(int)self->old_rgb(i, 0))
        + abs((int)rgbA(i, 1)-(int)self->old_rgb(i, 1))
        + abs((int)rgbA(i, 2)-(int)self->old_rgb(i, 2));
    grayA(i) = diff/3;
  }

  grayA.reshape(H, W);
  self->old_rgb=rgbA;

  motion.set() = grayA;
}

//===========================================================================
//
// DifferenceFilter
//

void DifferenceFilter::open() {}
void DifferenceFilter::close() {}
void DifferenceFilter::step() {
  uint threshold = 50;
  byteA rgb1, rgb2, diff;
  rgb1 = i1.get();
  rgb2 = i2.get();

  if(rgb1.N!=rgb2.N) {
    rgb2=rgb1;
    i2.set() = rgb2;
  }

  uint d0=rgb1.d0, d1=rgb1.d1;
  rgb1.reshape(rgb1.N/3, 3);
  rgb2.reshape(rgb2.N/3, 3);
  diff.resizeAs(rgb1);
  diff.setZero();

  for(uint i=0; i<diff.d0; i++) {
    uint d = abs(rgb1(i, 0)-rgb2(i, 0)) + abs(rgb1(i, 1)-rgb2(i, 1)) + abs(rgb1(i, 2)-rgb2(i, 2));
    if(d>threshold) diff[i] = rgb1[i];
  }

  diff.reshape(d0, d1, 3);
  diffImage.set() = diff;
}

//===========================================================================
//
// CannyFilter
//

void CannyFilter::open() {}
void CannyFilter::close() {}
void CannyFilter::step() {
  float cannyThreshold = 50.f;
  byteA gray, canny;
  gray = grayImage.get();
  if(!gray.N) return;
  canny.resizeAs(gray);
  cv::Mat ref = conv_Arr2CvRef(canny);
  cv::Canny(conv_Arr2CvRef(gray), ref, cannyThreshold, 4.f*cannyThreshold, 3);
  cannyImage.set() = canny;
}

//===========================================================================
//
// Patcher
//

void Patcher::open() {}
void Patcher::close() {}
void Patcher::step() {
  byteA rgb, display;
  uintA patching; //for each pixel an integer
  arr pch_cen;    //patch centers
  uintA pch_edges; //patch Delauney edges
  floatA pch_rgb; //patch mean colors

  rgb = rgbImage.get();
  uint np=get_single_color_segmentation(patching, rgb, 3.f, 64.f, 400);
  np=incremental_patch_ids(patching);
  get_patch_centroids(pch_cen, patching, np);
  get_patch_colors(pch_rgb, rgb, patching, np);
  pch2img(display, patching, pch_rgb);
  //getDelaunayEdges(pch_edges, pch_cen);

  patchImage.writeAccess();
  patchImage().patching=patching;
  patchImage().pch_cen=pch_cen;
  patchImage().pch_rgb=pch_rgb;
  patchImage().pch_edges=pch_edges;
  patchImage().display=display;
  patchImage.deAccess();
}

//===========================================================================
//
// SURFer
//

struct sSURFer {
//  cv::SURF *surf;
};

void SURFer::open() {
  self = make_unique<sSURFer>();
//  self->surf = new cv::SURF(500);
  HALT("something in opencv changed... please upate the code")
}

void SURFer::close() {
  //  delete self->surf;
  self.reset();
}

void SURFer::step() {
  byteA gray, display;
  gray = grayImage.get();
  if(!gray.N) return;

  std::vector<cv::KeyPoint> keypoints;
  std::vector<float> descriptors;
  //(*surf)(conv_Arr2CvRef(gray), cv::Mat(), keypoints, descriptors);

  display=gray;
  cv::Mat ref = conv_Arr2CvRef(display);
  for(uint i=0; i<keypoints.size(); i++) {
    circle(ref, keypoints[i].pt, 3, cv::Scalar(255));
  }

  features.writeAccess();
  features().keypoints = keypoints;
  features().descriptors = descriptors;
  features().display = display;
  features.deAccess();
}

//===========================================================================
//
// HoughLineFilter
//

void HoughLineFilter::open() {}
void HoughLineFilter::close() {}
void HoughLineFilter::step() {
  byteA gray, display;
  gray = grayImage.get();
  if(!gray.N) return;

  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(conv_Arr2CvRef(gray), lines, 1, CV_PI/180, 50, 30, 10);
  display = gray;
  cv::Mat ref=conv_Arr2CvRef(display);
  for(uint i=0; i<lines.size(); i++) {
    cv::line(ref, cv::Point(lines[i][0], lines[i][1]),
             cv::Point(lines[i][2], lines[i][3]), cv::Scalar(255), 3);
  }

  houghLines.writeAccess();
  houghLines().lines = lines;
  houghLines().display = display;
  houghLines.deAccess();
}

//===========================================================================
//
// ShapeFitter
//

//struct ShapeFitter: Thread{
//  unique_ptr<struct sShapeFitter> self;

//  FloatImage *eviL, *eviR;
//  PerceptionOutput *percOut;

//  ShapeFitter(FloatImage& _eviL, FloatImage& _eviR, PerceptionOutput &_perc);
//  void open();
//  void step();
//  void close() {}
//};

//===========================================================================
//
// helpers
//

byteA evidence2RGB(const floatA& evidence) {
  byteA tmp;
  if(!evidence.N) return tmp;
  tmp.resize(evidence.N, 3);

  for(uint i = 0; i < evidence.N; i++)
    tmp(i, 0) = tmp(i, 1) = tmp(i, 2) = 255.f * evidence.elem(i);

  tmp.reshape(evidence.N/evidence.d2, evidence.d2, 3);

  return tmp;
}

#endif //RAI_OPENCV

#ifdef RAI_PCL
// Pointcloud stuff
//
//ModuleL newPointcloudProcesses() {
//  ModuleL processes;
//  processes.append(new ObjectClusterer);
//  processes.append(new ObjectFitter);

//  processes.append(new ObjectFilter("Object Filter"));
//  processes.append(new ObjectTransformator("Object Transformator"));
//  return processes;
//}

//VariableL newPointcloudVariables() {
//  VariableL variables;
//  variables.append(new PointCloudVar("KinectData3D"));
//  variables.append(new PointCloudSet("ObjectClusters"));
//  variables.append(new ObjectSet("Objects"));
//  variables.append(new ObjectBeliefSet("filteredObjects"));
//  return variables;
//}
#endif

void openGlLock();
void openGlUnlock();

void draw1(void*) {
  glStandardLight(nullptr);
  glColor(1, 0, 0);
  glFrontFace(GL_CW);
//  glutSolidTeapot(1.);
  glDrawAxes(1.);
  glFrontFace(GL_CCW);
}

//void AllViewer::open() {
//  gl.add(glStandardScene, 0);
//  gl.add(kinect);
//  gl.add(glDrawPlanes, &planes_now_copy);
//}

//void AllViewer::step(){
//  kinect.V = kinect_points.get();
//  kinect.C = kinect_pointColors.get();
//  planes_now_copy = planes_now.get();
//  gl.update();
//}

#endif

struct sOpencvCamera {  };
