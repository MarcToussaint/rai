#include "aruco.h"

#ifdef RAI_OPENCV

#include "opencv.h"

#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/charuco_detector.hpp>
// #include <opencv2/objdetect/objdetect.hpp>
// #include <opencv2/calib.hpp>
// #include <opencv2/geometry/3d.hpp>

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

byteA getFullArucoDict(){
  cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);
  int borderBits = 1;
  int width = 5+2*borderBits;
  cv::Mat _img;
  byteA dict(50, width, width);
  for(uint id=0;id<dict.d0;id++){
    dictionary.generateImageMarker(id, width, _img, 1);
    dict[id] = conv_cvMat2byteA(_img).reshape(width, width);
  }
  return dict;
}

FindArucos::FindArucos(){
  dictionary = make_shared<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50));
  cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
  // detectorParams.cornerRefinementMethod = cv::aruco::CornerRefineMethod::CORNER_REFINE_SUBPIX;
  detector = make_shared<cv::aruco::ArucoDetector>(*dictionary, detectorParams);
}

void FindArucos::find(const byteA& rgb){

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

str FindArucos::report(){
  str msg;
  msg <<"aruco finder report: " <<ids.N <<" markers, pts shape: " <<pts.dim();
  return msg;
}

//===========================================================================

void findCharuco(CalibrateIntrinsicsWithCharuco &CI,
		 const byteA& rgb,
		 std::vector<std::vector<cv::Point2f>>& allImagePoints,
		 std::vector<std::vector<cv::Point3f>>& allObjectPoints,
		 cv::Size& imageSize){
  std::vector<int> charucoIds;
  std::vector<cv::Point2f> charucoCorners;
  cv::Mat inputImage = CV(rgb);

  CI.detector->detectBoard(inputImage, charucoCorners, charucoIds);

  if(charucoIds.size()>5){
    intA ids = as_arr<int>(charucoIds, false);
    arr pts(ids.N, 2);
    if(ids.N>=10){
      for(uint i=0;i<pts.d0;i++){
        pts(i, 0) = charucoCorners[i].x;
        pts(i, 1) = charucoCorners[i].y;
      }

      if(CI.verbose>0){
        cv::Mat outputImage = inputImage.clone();
        cv::aruco::drawDetectedCornersCharuco(outputImage, charucoCorners, charucoIds);
        CI.rgb_annotated = conv_cvMat2byteA(outputImage);
        if(CI.verbose>1){
          if(!CI.gl) CI.gl = make_shared<OpenGL>();
          CI.gl->watchImage(CI.rgb_annotated, CI.verbose>2);
        }
      }
    }


    std::vector<cv::Point3f> currentObjectPoints;
    std::vector<cv::Point2f> currentImagePoints;
    CI.board->matchImagePoints(charucoCorners, charucoIds, currentObjectPoints, currentImagePoints);

    if(currentImagePoints.empty() || currentObjectPoints.empty()) {
      cout << "Point matching failed, try again." << endl;
    }

    allImagePoints.push_back(currentImagePoints);
    allObjectPoints.push_back(currentObjectPoints);

    imageSize = inputImage.size();
  }
}

CalibrateIntrinsicsWithCharuco::CalibrateIntrinsicsWithCharuco(str path, uint t_start, uint t_stop, uint n_cams, float square_len_m, float marker_len_m){
  dictionary = make_shared<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50));
  board = make_shared<cv::aruco::CharucoBoard>(cv::Size{5, 7}, square_len_m, marker_len_m, *dictionary, cv::noArray());
  auto detectorParams = cv::aruco::DetectorParameters();
  // detectorParams.cornerRefinementMethod = cv::aruco::CornerRefineMethod::CORNER_REFINE_SUBPIX;
  detector = make_shared<cv::aruco::CharucoDetector>(*board, cv::aruco::CharucoParameters(), detectorParams, cv::aruco::RefineParameters());

  for(uint c=0;c<n_cams;c++){
    std::vector<std::vector<cv::Point2f>> allImagePoints;
    std::vector<std::vector<cv::Point3f>> allObjectPoints;
    cv::Size imageSize;

    for(uint t=t_start; t<t_stop; t++){
      byteA img = readImage(STRING(path <<"img_" <<std::setw(4) <<std::setfill('0') <<t <<'_' <<c<<".png"));
      if(!gl) gl = make_shared<OpenGL>();
      gl->watchImage(img, false);
      findCharuco(*this, img, allImagePoints, allObjectPoints, imageSize);
    }

    cv::Mat cameraMatrix, distCoeffs;
    double repErr = cv::calibrateCamera(allObjectPoints, allImagePoints, imageSize, cameraMatrix, distCoeffs,
                                        cv::noArray(), cv::noArray(), cv::noArray(), cv::noArray(), cv::noArray(), 0);

    arr P = conv_cvMat2arr(cameraMatrix);
    arr dist = conv_cvMat2arr(distCoeffs);

    arr fxycxy = {P(0,0), P(1,1), P(0,2), P(1,2)};

    cout <<"camera: " <<c <<", count: " <<allImagePoints.size() <<", fxycxy: " <<fxycxy <<", distortion: " <<dist <<endl;
  }
}

byteA CalibrateIntrinsicsWithCharuco::readImage(str filename){
  byteA img;
  read_png(img, filename, false);
  uint H=img.d0, W=img.d1;
  img.reshape(W*H,4);
  img.delColumns(3);
  img.reshape(H,W,3);
  swap_RGB_BGR(img);
  // make_grey(img);
  return img;
}



#else //OPENCV

byteA getArucoImage(int id, int borderBits){ NICO }
byteA getFullArucoDict(){ NICO }
FindArucos::FindArucos(){ NICO }
void FindArucos::find(const byteA& rgb){ NICO }

#endif //OPENCV
