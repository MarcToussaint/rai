
#pragma once

#include <Core/array.h>
#include <Gui/opengl.h>

//forward
namespace cv{
class Mat;
namespace aruco{
class Dictionary;
class ArucoDetector;
class CharucoBoard;
class CharucoDetector;
}
}

inline byteA intA2img(const intA& I){
  byteA img(I.N, I.N);
  img.setZero();
  for(uint i=0;i<img.d0;i++) for(uint j=0;j<img.d1;j++) if(I(i)&(1<<j)) img(i,j)=255;
  return img;
}

inline intA img2intA(const byteA& img){
  intA I(img.d0);
  I.setZero();
  for(uint i=0;i<img.d0;i++) for(uint j=0;j<img.d1;j++) if(img(i,j)) I(i) |= 1<<j;
  return I;
}
byteA getArucoImage(int id, int borderBits = 2);

//===========================================================================

std::tuple<intAA, arrA> findArucos(const byteAA& imgs);
std::tuple<arrA, arrA> calibrateIntrinsicsWithCharuco(const byteAA& imgs, uint distortionDofs, float square_len_m=0.055, float marker_len_m=0.041);

//===========================================================================

struct FindArucos {
  shared_ptr<cv::aruco::Dictionary> dictionary;
  shared_ptr<cv::aruco::ArucoDetector> detector;

  std::shared_ptr<OpenGL> gl;
  int verbose=1;

  //outputs!
  intA ids;
  arr pts;
  byteA rgb_annotated;

  FindArucos();

  void find(const byteA& rgb);
  str report();
};

//===========================================================================

