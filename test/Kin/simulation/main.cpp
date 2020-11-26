#include <Kin/kin.h>
#include <Kin/frame.h>
#include <Kin/feature.h>
#include <Kin/simulation.h>
#include <Kin/viewer.h>

#include <iomanip>

//===========================================================================

void testPushes(){
  rai::Configuration C;
  C.addFile("model.g");
  C.watch(true);

  rai::Simulation S(C, S._bullet, true);
  //rai::Simulation S(C, S._physx, true);

  double tau=.01;
  Metronome tic(tau);
  byteA rgb;
  floatA depth;

  arr Xstart = C.getFrameState();

  for(uint k=0;k<5;k++){

    //restart from the same state multiple times
    S.setState(Xstart);

    for(uint t=0;t<300;t++){
      tic.waitForTic();
      if(!(t%10)) S.getImageAndDepth(rgb, depth); //we don't need images with 100Hz, rendering is slow

      //some good old fashioned IK
      arr q = C.getJointState();
      Value diff = C.feature(FS_positionDiff, {"gripper", "box"})->eval(C);
      diff.y *= .005/length(diff.y);
      q -= pseudoInverse(diff.J, NoArr, 1e-2) * diff.y;
//      C.setJointState(q);

      S.step(q, tau, S._position);

      //a crazy disturbance, lifting the box suddenly
      if(!(t%100)){
        arr p = C["box"]->getPosition();
        p(0) += .05;
        p(2) += .2;
        C["box"]->setPosition(p);

        S.setState(C.getFrameState());
      }
    }
  }
}

//===========================================================================

void testGrasp(){
  rai::Configuration C;
  C.addFile("model.g");

  C.selectJointsByName({"finger1", "finger2"}, true);

  rai::Simulation S(C, S._bullet, true);
  //rai::Simulation S(C, S._physx, true);

  byteA rgb;
  floatA depth;
  double tau=.01;
  Metronome tic(tau);

  for(uint t=0;;t++){
    tic.waitForTic();
//    rai::wait(.05);

    C.stepSwift();
//    C.reportProxies();

    if(!(t%10)) S.getImageAndDepth(rgb, depth); //we don't need images with 100Hz, rendering is slow

    arr q = C.getJointState();

    //some good old fashioned IK
    if(t<=300){
      Value diff = C.feature(FS_oppose, {"finger1", "finger2", "ring4"})->eval(C);
      diff.y *= rai::MIN(.008/length(diff.y), 1.);
      q -= pseudoInverse(diff.J, NoArr, 1e-2) * diff.y;
    }

    if(t==300){
      S.closeGripper("gripper");
    }

    if(S.getGripperIsGrasping("gripper")){
      Value diff = C.feature(FS_position, {"gripper"})->eval(C);
      q -= pseudoInverse(diff.J, NoArr, 1e-2) * ARR(0.,0.,-2e-4);
    }

    if(t==900){
      S.openGripper("gripper");
    }

    if(t>1200 && S.getGripperWidth("gripper")>=.02){ //that's the upper limit of this gripper
      break;
    }

    S.step(q, tau, S._position);
  }
}

//===========================================================================

void testOpenClose(){
  rai::Configuration RealWorld;
  RealWorld.addFile("../../../../rai-robotModels/scenarios/pandasTable.g");
  rai::Simulation S(RealWorld, S._bullet, true);
  //rai::Simulation S(RealWorld, S._physx, true);

  rai::Configuration C;
  C.addFile("../../../../rai-robotModels/scenarios/pandasTable.g");
  C.watch(false, "initial");

  double tau = .01;

  S.closeGripper("R_gripper");
  for(uint t=0;;t++){
    rai::wait(tau);

    arr q = S.get_q();
    C.setJointState(q);
    C.watch();

    S.step({}, tau, S._none);
    if(S.getGripperIsClose("R_gripper")) break;
  }

  S.openGripper("R_gripper");
  for(uint t=0;;t++){
    rai::wait(tau);

    arr q = S.get_q();
    C.setJointState(q);
    C.watch();

    S.step({}, tau, S._none);
    if(S.getGripperIsOpen("R_gripper")) break;
  }
}

//===========================================================================

void makeRndScene(){
  rai::Configuration C;

  for(uint i=0;i<30;i++){
    rai::Frame *obj = C.addFrame(STRING("obj" <<i));
    arr size = {rnd.uni(.2,.8), rnd.uni(.1,.4), rnd.uni(.05,.2), .01};
    obj->setShape(rai::ST_ssBox, size);
    rai::Transformation pose;
    pose.setRandom();
    pose.pos.y *= .3;
    pose.pos.y += .5;
    pose.pos.z += 2.;
    obj->setPose(pose);
    obj->setMass(.2);
  }

  FILE("z.rndObjects.g") <<C; //write configuration into a file

  C.addFile("../../../../rai-robotModels/scenarios/pandasTable.g");

  rai::Simulation S(C, S._bullet, true);
  //rai::Simulation S(C, S._physx, true);
  S.cameraview().addSensor("camera");

  byteA rgb;
  floatA depth;
  double tau=.01;
  Metronome tic(tau);

  for(uint t=0;t<300;t++){
    tic.waitForTic();
    if(!(t%10)) S.getImageAndDepth(rgb, depth); //we don't need images with 100Hz, rendering is slow

    S.step({}, tau, S._none);

    cout <<"depth in range: " <<depth.min() <<' ' <<depth.max() <<endl;
  }

  C.sortFrames();
  FILE("z.g") <<C; //write configuration into a file

  rai::wait();
}

//===========================================================================

void testFriction(){
  rai::Configuration C;

  for(int i=0;i<10;i++){
    rai::Frame *obj = C.addFrame(STRING("obj" <<i));
    arr size = {.1,.1,.1, .01};
    obj->setShape(rai::ST_ssBox, size);
    obj->setPosition({(i-5)*.2,0.,1.});
    obj->setMass(.2);
    obj->addAttribute("friction", .02*i);
  }

  C.addFile("../../../../rai-robotModels/scenarios/pandasTable.g");

  C["table"]->setQuaternion({1.,-.1,0.,0.}); //tilt the table!!

  rai::Simulation S(C, S._bullet, true);
  //rai::Simulation S(C, S._physx, true);
  S.cameraview().addSensor("camera");

  double tau=.01;
  Metronome tic(tau);

  int ppmCount=0;
  rai::system("mkdir -p z.vid/; rm -f z.vid/*.ppm");

  for(uint t=0;t<300;t++){
    tic.waitForTic();

    S.step({}, tau, S._none);
    write_ppm(S.getScreenshot(), STRING("z.vid/"<<std::setw(4)<<std::setfill('0')<<(ppmCount++)<<".ppm"));
  }

  rai::wait();
}

//===========================================================================

void testStackOfBlocks(){
  rai::Configuration C;

  for(int i=0;i<7;i++){
    rai::Frame *obj = C.addFrame(STRING("obj" <<i));
    arr size = {.2,.2,.2, .02};
    obj->setShape(rai::ST_ssBox, size);
    obj->setPosition({0.,0.,1.+i*.25});
    obj->setMass(1.); //does not seem to have much effect?
//    obj->addAttribute("friction", 1.);
//    obj->addAttribute("restitution", .01);
  }

  C.addFile("../../../../rai-robotModels/scenarios/pandasTable.g");

  rai::Simulation S(C, S._bullet, true);
  //rai::Simulation S(C, S._physx, true);

  double tau=.01;  //jumps a bit for tau=.01
  Metronome tic(tau);

  for(uint t=0;t<4./tau;t++){
    tic.waitForTic();

    S.step({}, tau, S._none);
  }

  rai::wait();
}

//===========================================================================

int main(int argc,char **argv){
  rai::initCmdLine(argc, argv);

  makeRndScene();
  testFriction();
  testStackOfBlocks();
  testPushes();
  testGrasp();
  testOpenClose();

  return 0;
}
