#include "aruco.h"

#include "../Perception/opencv.h"

int aruco_dict_5X5_50 [] = {0, 10, 52, 12, 42, 14, 0,
			   0, 32, 6, 32, 58, 24, 0,
			   0, 22, 30, 48, 26, 46, 0,
			   0, 2, 56, 40, 60, 58, 0,
			   0, 22, 46, 44, 36, 8, 0,
			   0, 46, 4, 16, 32, 44, 0,
			   0, 44, 56, 42, 62, 12, 0,
			   0, 28, 8, 40, 48, 52, 0,
			   0, 2, 22, 6, 36, 18, 0,
			   0, 50, 16, 60, 46, 40, 0,
			   0, 50, 38, 54, 2, 48, 0,
			   0, 22, 40, 26, 26, 0, 0,
			   0, 30, 12, 20, 6, 34, 0,
			   0, 40, 14, 14, 52, 24, 0,
			   0, 62, 38, 62, 40, 4, 0,
			   0, 40, 48, 6, 54, 62, 0,
			   0, 36, 60, 36, 22, 6, 0,
			   0, 52, 46, 4, 50, 60, 0,
			   0, 60, 12, 50, 28, 8, 0,
			   0, 2, 12, 56, 60, 4, 0,
			   0, 18, 54, 26, 50, 42, 0,
			   0, 42, 32, 38, 16, 0, 0,
			   0, 26, 26, 48, 40, 32, 0,
			   0, 52, 10, 8, 26, 30, 0,
			   0, 38, 38, 10, 32, 48, 0,
			   0, 22, 52, 24, 52, 18, 0,
			   0, 14, 56, 50, 10, 52, 0,
			   0, 16, 8, 2, 18, 24, 0,
			   0, 48, 58, 40, 50, 18, 0,
			   0, 16, 4, 4, 34, 54, 0,
			   0, 16, 28, 44, 54, 28, 0,
			   0, 48, 44, 16, 48, 50, 0,
			   0, 8, 32, 10, 24, 28, 0,
			   0, 8, 42, 20, 26, 4, 0,
			   0, 8, 12, 2, 46, 22, 0,
			   0, 56, 10, 54, 62, 20, 0,
			   0, 36, 50, 16, 42, 12, 0,
			   0, 4, 40, 10, 4, 32, 0,
			   0, 36, 42, 52, 4, 46, 0,
			   0, 4, 12, 60, 48, 18, 0,
			   0, 20, 22, 20, 32, 40, 0,
			   0, 20, 20, 52, 14, 30, 0,
			   0, 44, 2, 18, 60, 58, 0,
			   0, 12, 24, 16, 28, 38, 0,
			   0, 44, 10, 62, 34, 34, 0,
			   0, 28, 18, 6, 50, 22, 0,
			   0, 28, 50, 28, 12, 58, 0,
			   0, 60, 18, 18, 0, 24, 0,
			   0, 60, 52, 8, 36, 40, 0,
			   0, 60, 44, 44, 30, 58, 0};

#if 1

inline byteA intA2img(const intA& I){
  byteA img(I.N, I.N);
  img.setZero();
  for(uint i=0;i<img.d0;i++) for(uint j=0;j<img.d1;j++) if(I(i)&(1<<j)) img(i,j)=255;
  return img;
}

byteA getArucoImage(int id){
  intA I;
  I.referTo(aruco_dict_5X5_50 + 7*id, 7);
  return intA2img(I);
}

#else //OPENCV

#include <opencv2/objdetect/aruco_dictionary.hpp>

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

#endif //OPENCV
