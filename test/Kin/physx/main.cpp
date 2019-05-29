#include <Kin/kin.h>
#include <Kin/frame.h>
#include <Kin/kin_bullet.h>
#include <Kin/kin_physx.h>

//===========================================================================

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  rai::KinematicWorld C;
  rai::Frame *f = C.addFrame("base");
  f->setShape(rai::ST_ssBox, {8., 8., .2, .02});
  f->setColor({.5, .5, .5});
  f->X.pos.set(0,0,.1);
  f->shape->cont=1;

  for(uint k=0;k<10;k++){
    rai::Frame *f = C.addFrame(STRING("obj" <<k));
    if(rnd.uni()<.5){
      f->setShape(rai::ST_ssBox, {1.,.2,.1, .02});
    }else{
      f->setConvexMesh(.2*randn(10,3), {}, .02);
    }
    (new rai::Inertia(*f)) -> type=rai::BT_dynamic;
    f->shape->cont=1;
    f->X.setRandom();
    f->X.pos.z += 2.;
  }

//  BulletInterface BT(C);
  PhysXInterface PY(C);

  for(uint t=0; t<300; t++) {
    PY.step(.01);
//        BT.step();
//        BT.pullDynamicStates(C.frames);
    C.watch(false, STRING("t="<<t));
  }

  C.watch(true);
  return 0;
}

