#include "aruco.h"

#ifdef RAI_OPENCV

#include "opencv.h"

#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/charuco_detector.hpp>
// #include <opencv2/objdetect/objdetect.hpp>
// #include <opencv2/calib.hpp>
#include <opencv2/geometry/3d.hpp>

namespace rai {

byteA getArucoImage(int id, int borderBits){
  cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);
  int width = 5+2*borderBits;
  cv::Mat _img;
  dictionary.generateImageMarker(id, width, _img, borderBits);
  byteA img;
  img = conv_cvMat2byteA(_img).reshape(width, width);
  if(borderBits>1){ //make extra border white
    for(int i=0;i<borderBits-1;i++){ img[i]=255; img[width-i-1]=255; }
    for(int i=0;i<width;i++) for(int j=0;j<borderBits-1;j++) { img(i,j)=255; img(i,width-j-1)=255; }
    // for(int i=0;i<borderBits-1;i++) for(int j=0;j<borderBits-1;j++) img(i,j)=0;
  }
  return img;
}

ArucoFinder::ArucoFinder(){
  dictionary = make_shared<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50));
  cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
  // detectorParams.cornerRefinementMethod = cv::aruco::CornerRefineMethod::CORNER_REFINE_SUBPIX;
  detector = make_shared<cv::aruco::ArucoDetector>(*dictionary, detectorParams);
}

void ArucoFinder::find(const byteA& rgb){
  if(!rgb.N){ ids.clear(); pts.clear(); return; }

  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
  cv::Mat inputImage = CV(rgb);

  detector->detectMarkers(inputImage, markerCorners, markerIds, rejectedCandidates);

  ids = as_arr<int>(markerIds, false);
  pts.resize(ids.N, 4, 2);
  if(ids.N){
    // cv::Size winSize = cv::Size( 5, 5 );
    // cv::Size zeroZone = cv::Size( -1, -1 );
    // cv::TermCriteria criteria = cv::TermCriteria( cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 40, 0.001 );
    // cv::Mat grayImage;
    // cv::cvtColor(inputImage, grayImage, cv::COLOR_RGB2GRAY);
    // cv::cornerSubPix(grayImage, markerCorners[0], winSize, zeroZone, criteria);

    for(uint i=0;i<pts.d0;i++){
      CHECK_EQ(markerCorners[i].size(), 4, "");
      for(uint j=0;j<4;j++){
        pts(i, j, 0) = markerCorners[i][j].x;
        pts(i, j, 1) = markerCorners[i][j].y;
      }
    }

    if(verbose>0){
      cv::Mat outputImage = inputImage.clone();
      cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
      rgb_annotated = conv_cvMat2byteA(outputImage);
      if(verbose>1){
        if(!gl) gl = make_shared<OpenGL>();
        gl->watchImage(rgb_annotated, false);
      }
    }
  }

}

str ArucoFinder::report(){
  str msg;
  msg <<"aruco finder report: " <<ids.N <<" markers, pts shape: " <<pts.dim();
  return msg;
}

//===========================================================================

std::tuple<intAA, arrA> findArucos(const byteAA& imgs){
  ArucoFinder finder;
  finder.verbose=1;

  intAA ids(imgs.d0, imgs.d1);
  arrA pts(imgs.d0, imgs.d1);
  for(uint t=0;t<imgs.d0;t++) for(uint c=0;c<imgs.d1;c++){
      finder.find(imgs(t,c));
      cout <<finder.report() <<endl;
      ids(t,c) = finder.ids;
      pts(t,c) = finder.pts;
    }
  return std::tuple{ids, pts};
}

//===========================================================================

ArucoThread::ArucoThread(uint _cam_id, Var<byteA>& _input, double beatIntervalSec)
    : Thread(STRING("aruco_thread_" <<_cam_id), beatIntervalSec),
    input(_input),
    cam_id(_cam_id){
  finder.verbose=0;
  LOG(0) <<"launching aruco thread cam_id: " <<_cam_id;
  // status.listenTo(input);
  // threadOpen();
  filter.set()->K = 2;
  filter.set()->threshold = 30.;
  threadLoop();
}

ArucoThread::~ArucoThread(){
  LOG(0) <<"DTOR cam_id: " <<cam_id <<" - " <<timer.report();
  threadClose();
}

void ArucoThread::step() {
  double data_time;
  {
    auto get = input.get();
    rgb = get.data;
    data_time = get.var.data_time;
  }

  if(!rgb.N) return;
  finder.find(rgb);

  if(use_filter){
    flat.resize(50, 8);
    for(double& d:flat) d=std::nan("");
    for(uint i=0;i<finder.ids.N;i++){
      uint id = finder.ids.elem(i);
      flat[id] = finder.pts[i];
    }
    flat.reshape(-1);
    {
      auto set = filter.set();
      set->update(data_time, flat);
      set.var.data_time = data_time;
    }
  }

  // LOG(0) <<"aruco " <<cam_id <<" found #" <<finder.ids.N <<" points in image rev " <<input_revision;
  {
    auto set = output.set();
    set->cam_id = cam_id;
    set->ids = finder.ids;
    set->pts = finder.pts;
    set.var.data_time = data_time;
  }
  timer.tic(1);
}

//===========================================================================

void undistort_point(arr& p, const arr& fxycxy, const arr& distortion) {
  arr K = arr{{3,3}, {fxycxy(0), 0., fxycxy(2), 0., fxycxy(1), fxycxy(3), 0., 0., 1.}};
  std::vector<cv::Point2d> p_in = { cv::Point2d{p(0), p(1)} };
  std::vector<cv::Point2d> p_out;
  cv::undistortImagePoints(p_in, p_out, CV(K), CV(distortion));
  p = arr{p_out[0].x, p_out[0].y};
}

byteA undistort_image(const byteA& img, const arr& fxycxy, const arr& distortion)  {
  arr K = arr{{3,3}, {fxycxy(0), 0., fxycxy(2), 0., fxycxy(1), fxycxy(3), 0., 0., 1.}};
  byteA imgU = img;
  cv::undistort(CV(img), CV(imgU), CV(K), CV(distortion));
  return imgU;
}

//===========================================================================

std::tuple<arrA, arrA> calibrateIntrinsicsWithCharuco(const byteAA& imgs, uint distortionDofs, float square_len_m, float marker_len_m){
  shared_ptr<cv::aruco::Dictionary> dictionary;
  shared_ptr<cv::aruco::CharucoBoard> board;
  shared_ptr<cv::aruco::CharucoDetector> detector;

  //output
  arrA K;
  arrA Fxycxy;
  arrA Distortion;

  //user
  int verbose=2;
  shared_ptr<OpenGL> gl;
  byteA rgb_annotated;

  dictionary = make_shared<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50));
  board = make_shared<cv::aruco::CharucoBoard>(cv::Size{5, 7}, square_len_m, marker_len_m, *dictionary, cv::noArray());
  auto detectorParams = cv::aruco::DetectorParameters();
  detectorParams.cornerRefinementMethod = cv::aruco::CornerRefineMethod::CORNER_REFINE_SUBPIX;
  detector = make_shared<cv::aruco::CharucoDetector>(*board, cv::aruco::CharucoParameters(), detectorParams, cv::aruco::RefineParameters());

  uint n_cam = imgs.d1;

  Distortion.resize(n_cam);
  K.resize(n_cam);
  Fxycxy.resize(n_cam);

  auto fil = ofstream("calib_cams_intrinsics.yml");
  for(uint c=0;c<n_cam;c++){
    std::vector<std::vector<cv::Point2f>> allImagePoints;
    std::vector<std::vector<cv::Point3f>> allObjectPoints;
    cv::Size imageSize;

    for(uint t=0; t<imgs.d0; t++){
      byteA& img = imgs(t,c);
      if(!gl) gl = make_shared<OpenGL>();
      gl->watchImage(img, false);
      {
        std::vector<int> charucoIds;
        std::vector<cv::Point2f> charucoCorners;
        cv::Mat inputImage = CV(img);

        detector->detectBoard(inputImage, charucoCorners, charucoIds);

	if(charucoIds.size()>20){
	  intA ids = as_arr<int>(charucoIds, false);
	  arr pts(ids.N, 2);
	  if(ids.N>=10){
	    for(uint i=0;i<pts.d0;i++){
	      pts(i, 0) = charucoCorners[i].x;
	      pts(i, 1) = charucoCorners[i].y;
	    }

	    if(verbose>0){
	      cv::Mat outputImage = inputImage.clone();
	      cv::aruco::drawDetectedCornersCharuco(outputImage, charucoCorners, charucoIds);
	      rgb_annotated = conv_cvMat2byteA(outputImage);
	      if(verbose>1) gl->watchImage(rgb_annotated, verbose>2);
	    }
	  }

	  std::vector<cv::Point3f> currentObjectPoints;
	  std::vector<cv::Point2f> currentImagePoints;
	  board->matchImagePoints(charucoCorners, charucoIds, currentObjectPoints, currentImagePoints);

	  if(currentImagePoints.empty() || currentObjectPoints.empty()) {
	    cout << "Point matching failed, try again." << endl;
	  }

	  allImagePoints.push_back(currentImagePoints);
	  allObjectPoints.push_back(currentObjectPoints);

	  // cout <<" added " <<charucoIds.size() <<" " <<currentImagePoints.size() <<' ' <<currentObjectPoints.size() <<endl;
	  imageSize = inputImage.size();
	}
      }
    }

    cv::Mat cameraMatrix, distCoeffs;
    int flags;
    if(distortionDofs==5) flags = 0;
    else if(distortionDofs==4) flags = cv::CALIB_FIX_K3;
    else if(distortionDofs==2) flags = cv::CALIB_FIX_K3 | cv::CALIB_FIX_TANGENT_DIST;
    else HALT("distortionDofs needs to be 5, 4, or 2");
    double repErr = cv::calibrateCamera(allObjectPoints, allImagePoints, imageSize, cameraMatrix, distCoeffs,
                                        cv::noArray(), cv::noArray(), flags);

    // cout <<"calibration rep error: " <<repErr <<endl;
    // cout <<"org: " <<cameraMatrix <<endl;
    // cout <<"optimal0 ?: " <<cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 0, imageSize) <<endl;
    // cout <<"optimal1 ?: " <<cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize) <<endl;
    K(c) = cv_asArr(cameraMatrix);
    Distortion(c) =   cv_asArr(distCoeffs);
    Fxycxy(c) = {cameraMatrix.at<double>(0,0), cameraMatrix.at<double>(1,1), cameraMatrix.at<double>(0,2), cameraMatrix.at<double>(1,2)};

    fil <<"   Edit(camera_" <<c <<"): { fxycxy: " <<Fxycxy(c) <<", distortion: " <<Distortion(c) <<" }  #err: " <<repErr <<" count: " <<allImagePoints.size() <<endl;
  }

  { str fn = "calib_cams_intrinsics.yml"; auto fil = ifstream(fn); cout <<"#--- " <<fn <<endl <<str(fil) <<endl; }

  return std::tuple(Fxycxy, Distortion);
}

} //namespace

#else //OPENCV

namespace rai {
  byteA getArucoImage(int id, int borderBits){ NICO }
  byteA getFullArucoDict(){ NICO }
  void undistort_point(arr& p, const arr& fxycxy, const arr& distortion) { NICO }
  ArucoFinder::ArucoFinder(){ NICO }
  void ArucoFinder::find(const byteA& rgb){ NICO }
  str ArucoFinder::report(){ NICO }

  ArucoThread::ArucoThread(uint k_id, Var<byteA>& _input, double beatIntervalSec)
      : Thread(STRING("aruco_thread_" <<k_id), beatIntervalSec), input(_input), cam_id(k_id) { NICO }
  ArucoThread::~ArucoThread(){ NICO }
  void ArucoThread::step(){ NICO }
}

#endif //OPENCV

