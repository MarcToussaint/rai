#include <Kin/kin.h>
#include <Kin/frame.h>
#include <Kin/kin_bullet.h>
#include <Kin/kin_physx.h>

//===========================================================================

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

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
    f->shape->cont=1;
    f->set_X()->setRandom();
    f->set_X()->pos.z += 2.;
  }

  BulletInterface sim(C, true);
//  PhysXInterface sim(C, true);

  for(uint t=0; t<300; t++) {
    rai::wait(.01);
    sim.step(.01);
    sim.pullDynamicStates(C.frames);
    C.watch(false, STRING("t="<<t));
  }

  C.watch(true);
  return 0;
}

