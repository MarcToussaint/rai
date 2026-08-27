#include <Perception/KomoArucoTracker.h>

#include <Kin/frame.h>
#include <Kin/cameraview.h>
#include <Perception/aruco.h>
#include <Optim/NLP_Solver.h>

void testKomoTracker(){
  rai::Configuration C;
  C.addFile("station.g");

  rai::KomoArucoTracker K(C, "obj");
  cout <<K.CS.report() <<endl;

  rai::CameraView V(C);
  byteA rgb;
  floatA depth;

  for(uint k=0;k<20;k++){
    C.setRandom();
    K.reset();
    // auto rgb = V.getRgb();

    C.setRandom();
    V.updateConfiguration(C);
    arr q_true = C.getJointState()({-7,0});

    for(uint c=0;c<K.CS.cams.N;c++){
      V.selectSensor(K.CS.cams(c));
      V.computeImageAndDepth(rgb, depth);
      // gl.watchImage(rgb, true);

      auto finder = rai::FindArucos();
      finder.verbose=1;
      finder.find(rgb);
      if(finder.rgb_annotated.N){
        C.get_viewer()->setQuad(c, finder.rgb_annotated, 0., c*.25, .25);
      }else{
        C.get_viewer()->setQuad(c, rgb, 0., c*.25, .25);
      }

      for(uint i=0;i<finder.pts.d0;i++){
        uint id = finder.ids(i);
        if(K.CS.obj_aruco_ids.contains(id)){
          for(uint j=0;j<finder.pts.d1;j++){
            K.addPointView(finder.pts(i, j, {}), c, id, j);
          }
        }
      }

      // if(verbose){
      cout <<"FINDER " <<c <<":" <<finder.report() <<endl;
      // }
    }

    K.solve(0);

    rai::Transformation X(K.ret->x);

    cout <<"error: " <<::sqrt(rai::sqrDistance(X, K.CS.obj->get_Q())) <<" time: " <<K.ret->time <<endl;
    cout <<*K.ret <<endl;
    cout <<K.ret->x <<endl <<q_true <<endl;

    K.komo->view(false);
    int key = C.view(true);
    if(key=='q') break;
  }
}

int main(int argc,char **argv){
  rai::initCmdLine(argc,argv);

  testKomoTracker();
  
  return 0;
}
