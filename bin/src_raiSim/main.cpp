#include <Kin/frame.h>
#include <Kin/kin_bullet.h>
#include <Kin/viewer.h>
#include <Core/thread.h>

//===========================================================================

void simGfile(){
  rai::String file=rai::getParameter<rai::String>("file",STRING("none"));
  if(rai::argc>=2 && rai::argv[1][0]!='-') file=rai::argv[1];
  LOG(0) <<"opening file `" <<file <<"'" <<endl;

  //-- load configuration
  rai::Configuration C;
  C.addFile(file);

  //-- create bullet
  BulletInterface bull_rai(C);
  C.view(true);
  C.get_viewer()->_resetPressedKey();

  //-- simulate
  double tau = .01;
  double glTau = .025;
  Metronome tic(tau);
  for(uint t=0;;t++){
    bull_rai.step(tau);
    bull_rai.pullDynamicStates(C);

    if(tau>glTau || !(t%int(glTau/tau))){
      int key = C.view(false, STRING("time t="<<tau*t));
      if(key==13 || key==27 || key=='q') break;
    }

    tic.waitForTic();
  }

}

//===========================================================================

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  simGfile();

  return 0;
}

