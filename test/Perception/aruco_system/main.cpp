#include <Perception/opencv.h> //always include this first! OpenCV headers define stupid macros
#include <Perception/opencvCamera.h>
#include <Perception/aruco.h>
#include <Perception/MultiViewProblems.h>

#include <Gui/opengl.h>
#include <Kin/kin.h>
#include <Kin/frame.h>
#include <Kin/viewer.h>
#include <Kin/cameraview.h>

#ifdef RAI_OPENCV

#include <opencv2/features2d.hpp>
#include "opencv2/imgproc.hpp"
#include <opencv2/objdetect/aruco_detector.hpp>

void test(){
  rai::Configuration C;
  C.addFile("scene.yml");

  // rai::ConfigurationViewer& V = *C.get_viewer();
  // V.setWindow("bla", 800, 600);

  rai::CameraView V(C);
  byteA rgb;
  floatA depth;

  FrameL arucos(12);
  for(uint i=0;i<arucos.N;i++) arucos(i) = C.getFrame(STRING("ar" <<i));

  FrameL tests(12);
  for(uint i=0;i<arucos.N;i++){
    tests(i) = C.addFrame(STRING("test_"<<i));
    tests(i)-> setShape(rai::ST_marker, {.2});
  }

  FrameL cams(4);
  for(uint i=0;i<cams.N;i++) cams(i) = C.getFrame(STRING("cam" <<i));

  for(uint k=0;k<20;k++){
    C.setRandom();
    C.view(true);
    // auto rgb = V.getRgb();

    MultiViewSolver D(12, 4);

    V.updateConfiguration(C);
    for(uint k=0;k<cams.N;k++){
      V.selectSensor(cams(k));
      V.computeImageAndDepth(rgb, depth);
      D.setCamera(k, V.getFxycxy(), V.currentCamera->cam.X);

      auto finder = FindArucos();
      finder.verbose=2;
      finder.find(rgb);
      C.get_viewer()->setQuad(k, finder.rgb_annotated, 0., k*.25, .25);

      for(uint j=0;j<finder.ids.N;j++){
        D.addDataPoint(finder.ids(j), k, finder.pts[j]);
      }
    }

    D.subSelectObservedPoints();
    D.solveColinearityForPoints();
    cout <<D.X <<endl;
    for(uint i=0;i<D.J;i++) tests(i)->setPosition(D.X[i]);

    int key = C.view(true);
    if(key=='q') break;
  }
}

#else //opencv
void test(){ NICO }
#endif

int main(int argc,char **argv){
  rai::initCmdLine(argc,argv);

  test();
  
  return 0;
}
