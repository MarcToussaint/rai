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

CalibrateIntrinsicsWithCharuco::CalibrateIntrinsicsWithCharuco(const byteAA& imgs, float square_len_m, float marker_len_m){
  dictionary = make_shared<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50));
  board = make_shared<cv::aruco::CharucoBoard>(cv::Size{5, 7}, square_len_m, marker_len_m, *dictionary, cv::noArray());
  auto detectorParams = cv::aruco::DetectorParameters();
  detectorParams.cornerRefinementMethod = cv::aruco::CornerRefineMethod::CORNER_REFINE_SUBPIX;
  detector = make_shared<cv::aruco::CharucoDetector>(*board, cv::aruco::CharucoParameters(), detectorParams, cv::aruco::RefineParameters());

  uint n_cam = imgs.d1;

  Distortion.resize(n_cam);
  K.resize(n_cam);
  Fxycxy.resize(n_cam);

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
    int flags = cv::CALIB_FIX_K3 | cv::CALIB_FIX_TANGENT_DIST;
    double repErr = cv::calibrateCamera(allObjectPoints, allImagePoints, imageSize, cameraMatrix, distCoeffs,
                                        cv::noArray(), cv::noArray(), flags);

    // cout <<"calibration rep error: " <<repErr <<endl;
    // cout <<"org: " <<cameraMatrix <<endl;
    // cout <<"optimal0 ?: " <<cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 0, imageSize) <<endl;
    // cout <<"optimal1 ?: " <<cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize) <<endl;
    K(c) = cv_asArr(cameraMatrix);
    Distortion(c) =   cv_asArr(distCoeffs);
    Fxycxy(c) = {cameraMatrix.at<double>(0,0), cameraMatrix.at<double>(1,1), cameraMatrix.at<double>(0,2), cameraMatrix.at<double>(1,2)};

    cout <<"Edit(camera_" <<c <<"): { fxycxy: " <<Fxycxy(c) <<", distortion: " <<Distortion(c) <<" }  #err: " <<repErr <<" count: " <<allImagePoints.size() <<endl;
  }
}



#else //OPENCV

byteA getArucoImage(int id, int borderBits){ NICO }
byteA getFullArucoDict(){ NICO }
FindArucos::FindArucos(){ NICO }
void FindArucos::find(const byteA& rgb){ NICO }

#endif //OPENCV
