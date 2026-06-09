#pragma once

#include <Core/array.h>
#include <Gui/opengl.h>

//forward
namespace cv{
namespace aruco{
class Dictionary;
class ArucoDetector;
}
}

byteA getArucoImage(int id, int borderBits = 2);


struct FindArucos {
  shared_ptr<cv::aruco::Dictionary> dictionary;
  shared_ptr<cv::aruco::ArucoDetector> detector;

  // OpenGL gl;
  int verbose=1;

  //outputs!
  intA ids;
  arr pts;
  byteA rgb_annotated;

  FindArucos();

  void find(const byteA& rgb);
};
