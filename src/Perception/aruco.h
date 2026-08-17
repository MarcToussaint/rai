
#pragma once

#include <Core/array.h>
#include <Gui/opengl.h>

//forward
namespace cv{
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
byteA getFullArucoDict();

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

struct CalibrateIntrinsicsWithCharuco {
  shared_ptr<cv::aruco::Dictionary> dictionary;
  shared_ptr<cv::aruco::CharucoBoard> board;
  shared_ptr<cv::aruco::CharucoDetector> detector;

  //user
  int verbose=2;
  shared_ptr<OpenGL> gl;
  byteA rgb_annotated;

  //TODO: should get an array of images directly as input, not deal with reading from file
  CalibrateIntrinsicsWithCharuco(str path="../data/board/", uint t_start=0, uint t_stop=40, uint n_cams=3, float square_len_m=0.055, float marker_len_m=0.041);

  byteA readImage(str filename);
};
