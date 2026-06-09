#include <Perception/opencv.h> //always include this first! OpenCV headers define stupid macros
#include <Perception/opencvCamera.h>
#include <Perception/aruco.h>

#include <Gui/opengl.h>
#include <Kin/kin.h>
#include <Kin/frame.h>
#include <Kin/viewer.h>

#include <opencv2/features2d.hpp>
#include "opencv2/imgproc.hpp"
#include <opencv2/objdetect/aruco_detector.hpp>


void test(){
  rai::Configuration C;
  C.addFile("$RAI_PATH/scenarios/pandaSingle.g");
  // C.view(true);
  // rai::params()->set("Render/renderShadow", false);

  byteA img(5,5,1);
  for(uint i=0;i<img.N;i++) img.elem(i) = 255.*i/(img.N-1);

  img = getArucoImage(10);
  img.reshape(img.d0, img.d1, 1);

  rai::Frame& f = *C.addFrame("quad");
  f.setParent(C.getFrame("l_gripper"));
  f.setRelativePosition({.05, 0., -.05});
  f.setRelativeQuaternion({1., 0., 1., 0.});
  f.setQuad(img, {.1, .1});
  rai::ConfigurationViewer& V = *C.get_viewer();

  OpenGL gl;

  for(uint k=0;k<20;k++){

    img = getArucoImage(k);
    img.reshape(img.d0, img.d1, 1);
    f.setQuad(img, {.1, .1});

    C.setRandom();
    C.view(true);
    auto rgb = V.getRgb();


    auto finder = FindArucos();
    finder.find(rgb);
    if(finder.rgb_annotated.N){
      gl.watchImage(finder.rgb_annotated, false);
    }

    C.view(true);
  }
}

int main(int argc,char **argv){
  rai::initCmdLine(argc,argv);

  test();
  
  return 0;
}
