#include "aruco.h"

#ifdef RAI_OPENCV

#include "opencv.h"

#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>

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

  ids = as_arr<int>(markerIds, true);
  pts.resize(ids.N, 4, 2);
  if(ids.N){
    // cv::Size winSize = cv::Size( 5, 5 );
    // cv::Size zeroZone = cv::Size( -1, -1 );
    // cv::TermCriteria criteria = cv::TermCriteria( cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 40, 0.001 );
    // cv::Mat grayImage;
    // cv::cvtColor(inputImage, grayImage, cv::COLOR_RGB2GRAY);
    // cv::cornerSubPix(grayImage, markerCorners[0], winSize, zeroZone, criteria);

    cout <<ids <<endl;
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
      // gl.watchImage(rgb_annotated, verbose>1);
    }
  }

}

#else //OPENCV

#endif //OPENCV
