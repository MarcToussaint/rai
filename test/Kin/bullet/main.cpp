#include <Kin/frame.h>
#include <Kin/kin_bullet.h>
#include <Kin/kin_physx.h>
#include <Kin/viewer.h>
#include <Core/thread.h>

//===========================================================================

void dropRandomScene(){
  rai::Configuration C;
  rai::Frame *f = C.addFrame("base");
  f->setShape(rai::ST_ssBox, {8., 8., .2, .02});
  f->setColor({.5, .5, .5});
  f->setPosition({0,0,.1});
  f->shape->cont=1;

  for(uint k=0;k<10;k++){
    rai::Frame *f = C.addFrame(STRING("obj" <<k));
    if(rnd.uni()<.5){
      f->setShape(rai::ST_ssBox, {1.,.2,.1, .02});
    }else{
      f->setConvexMesh(.2*randn(10,3), {}, .2);
    }
    f->setMass(1.);
    f->inertia->matrix.deleteOffDiagonal();
    f->set_X()->setRandom();
    f->set_X()->pos.z += 2.;
  }

  C.getJointState();

//  BulletInterface sim(C);
  PhysXInterface sim(C);

  for(uint t=0; t<300; t++) {
    rai::wait(.01);
    sim.step(.01);
    sim.pullDynamicStates(C);
    C.view(false, STRING("t="<<t));
  }

  C.view(true);
}

//===========================================================================

void simGfile(){
    rai::String file=rai::getParameter<rai::String>("file",STRING("none"));
    if(rai::argc>=2 && rai::argv[1][0]!='-') file=rai::argv[1];
    LOG(0) <<"opening file `" <<file <<"'" <<endl;

    //-- load configuration
    rai::Configuration C;
    C.addFile(file);
    C.optimizeTree();
    C.ensure_q();

#if 0
    SimInteract sim(C);
    sim.loop();
#else
//    BulletInterface bull_rai(C);
    PhysXInterface bull_rai(C);
//    FrameL bots = C.getParts();
//    for(auto& f:bots) if(f->ats && (*f->ats)["motors"]){
//      bull_rai.motorizeMultiBody(f);
//    }
    C.view(true);
    C.get_viewer()->_resetPressedKey();

    double tau = .01;
    double glTau = .025;
    Metronome tic(tau);
    for(uint t=0;t<5./tau;t++){
      bull_rai.step(tau);
      bull_rai.pullDynamicStates(C);
      cout <<C.getJointState() <<endl;

      if(tau>glTau || !(t%int(glTau/tau))){
        int key = C.view(false, STRING("time t="<<tau*t));
        if(key==13 || key==27 || key=='q') break;
      }

      tic.waitForTic();
    }
#endif

}

//===========================================================================

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  dropRandomScene();
//  simGfile();

  return 0;
}

