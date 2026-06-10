#include <Perception/opencv.h> //always include this first! OpenCV headers define stupid macros
#include <Perception/opencvCamera.h>

#include <Gui/opengl.h>

void test(){
  OpencvCamera cam("test", 4);
  cam.flip_bgr=true;

  byteA rgb;
  OpenGL gl;
  uint i=0, key=0;

  CycleTimer ct;

  for(;;){
    ct.cycleStart();
    i=cam.image.waitForRevisionGreaterThan(i);
    rgb = cam.image.get();
    ct.cycleDone();
    if(rgb.N) key=gl.watchImage(rgb, false);
    if(key=='q') break;

  }
  cout <<ct.report() <<endl;
}

int main(int argc,char **argv){
  rai::initCmdLine(argc,argv);

  test();
  
  return 0;
}
