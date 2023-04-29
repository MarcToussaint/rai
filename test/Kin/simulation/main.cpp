#include <Kin/kin.h>
#include <Kin/frame.h>
#include <Kin/feature.h>
#include <Kin/simulation.h>
#include <Kin/viewer.h>
#include <Kin/F_geometrics.h>
#include <Kin/kin_physx.h>

#include <iomanip>

//===========================================================================

void testPushes(){
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/liftRing.g"));
  C["box"]->set_Q()->setText("<t(.3 -.1 .25) d(40 1 1 0)>");
  C["stick"]->set_Q()->setText("<t(-.3 .6 1.1) d(90 1 0 0) d(20 1 1 0)>");
  C.view(true);

  rai::Simulation S(C, S._physx, 2);

  double tau=.01;
  Metronome tic(tau);
  byteA rgb;
  floatA depth;

  arr Xstart = C.getFrameState();

  for(uint k=0;k<2;k++){

    //restart from the same state multiple times
    S.setState(Xstart);

    for(uint t=0;t<300;t++){
      tic.waitForTic();
      if(!(t%10)) S.getImageAndDepth(rgb, depth); //we don't need images with 100Hz, rendering is slow

      //some good old fashioned IK
      arr q = C.getJointState();
      arr diff = C.feature(FS_positionDiff, {"gripper", "box"})->eval(C);
      diff *= .02/length(diff);
      q -= pseudoInverse(diff.J(), NoArr, 1e-2) * diff;
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
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/liftRing.g"));
  C["box"]->set_Q()->setText("<t(.3 .1 .25) d(40 1 1 0)>");
  C["stick"]->set_Q()->setText("<t(-.3 .6 1.1) d(90 1 0 0) d(20 1 1 0)>");

  arr q = C.getJointState();

  rai::Simulation S(C, S._physx, 2);

//  OpenGL gl;  gl.add(glStandardScene);  gl.add(*S.hidden_physx());

  byteA rgb;
  floatA depth;
  double tau=.01;
  Metronome tic(tau);

  for(uint t=0;;t++){
    tic.waitForTic();
//    rai::wait(.05);

    C.stepFcl();
//    C.reportProxies();

    if(!(t%10)) S.getImageAndDepth(rgb, depth); //we don't need images with 100Hz, rendering is slow

    //q = C.getJointState();

    //some good old fashioned IK
    if(t<=500){
      arr diff = F_GraspOppose()
//                 .setCentering()
                 .eval(C.getFrames({"finger1", "finger2", "ring4"}));
//                 C.feature(FS_oppose, {"finger1", "finger2", "ring4"})->eval(C);
      diff *= rai::MIN(.01/length(diff), 1.);
      q -= pseudoInverse(diff.J(), NoArr, 1e-2) * diff;
    }

    if(t==500){
      S.closeGripper("gripper");
    }

    if(S.getGripperIsGrasping("gripper")){
      arr diff = C.feature(FS_position, {"gripper"})->eval(C);
      q -= pseudoInverse(diff.J(), NoArr, 1e-2) * arr{0.,0.,-1e-3};
    }

    if(t==900){
      S.openGripper("gripper");
    }

    if(t>1200 && S.getGripperWidth("gripper")>=.02){ //that's the upper limit of this gripper
      break;
    }

    S.step(q, tau, S._position);
    //gl.update();
  }
}

//===========================================================================

void testOpenClose(){
  rai::Configuration RealWorld;
//  RealWorld.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));
  RealWorld.addFile(rai::raiPath("../rai-robotModels/scenarios/liftRing.g"));
//  RealWorld["box"]->set_Q()->setText("<t(.3 -.1 .25) d(40 1 1 0)>");
//  RealWorld["stick"]->set_Q()->setText("<t(-.3 .6 1.1) d(90 1 0 0) d(20 1 1 0)>");
  rai::Simulation S(RealWorld, S._physx, 2);

  rai::Configuration C;
//  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/liftRing.g"));
  C.view(false, "initial");
  arr q0 = C.getJointState();

  double tau = .01;
  S.step(q0, tau, S._position);

  rai::wait();
  S.closeGripper("gripper");
  for(uint t=0;;t++){
    rai::wait(tau);

    arr q = S.get_q();
    C.setJointState(q);
    C.view();

    S.step(q0, tau, S._position);
    cout <<"closing finger pos:" <<S.getGripperWidth("gripper") <<endl;
    if(S.getGripperIsClose("gripper")) break;
  }

  rai::wait();
  S.openGripper("gripper");
  for(uint t=0;;t++){
    rai::wait(tau);

    arr q = S.get_q();
    C.setJointState(q);
    C.view();

    S.step(q0, tau, S._position);
    cout <<"opening finger pos:" <<S.getGripperWidth("gripper") <<endl;
    if(S.getGripperIsOpen("gripper")) break;
  }
}

//===========================================================================

void testRndScene(){
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

  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandasTable.g"));

  rai::Simulation S(C, S._physx, 2);
  S.cameraview().addSensor("camera");

  byteA rgb;
  floatA depth;
  double tau=.01;
  Metronome tic(tau);

  for(uint t=0;t<300;t++){
    tic.waitForTic();
    if(!(t%10)) S.getImageAndDepth(rgb, depth); //we don't need images with 100Hz, rendering is slow

    S.step({}, tau, S._none);
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

  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandasTable.g"));

  C["table"]->setQuaternion({1.,-.1,0.,0.}); //tilt the table!!

  rai::Simulation S(C, S._physx, 2);
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

  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandasTable.g"));

  rai::Simulation S(C, S._physx, 2);

  double tau=.01;  //jumps a bit for tau=.01
  Metronome tic(tau);

  for(uint t=0;t<4./tau;t++){
    tic.waitForTic();

    S.step({}, tau, S._none);
  }

  rai::wait();
}

//===========================================================================

void testCompound(){
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/tests/compound.g"));

  rai::Simulation S(C, S._physx, 4);

  double tau=.01;
  Metronome tic(tau);

  rai::system("mkdir -p z.vid/; rm -f z.vid/*.ppm");

  for(uint t=0;t<4./tau;t++){
    tic.waitForTic();

    S.step({}, tau, S._none);
    write_ppm(S.getScreenshot(), STRING("z.vid/"<<std::setw(4)<<std::setfill('0')<<t<<".ppm"));
  }

  rai::wait();

}

//===========================================================================

void testMotors(){
  rai::Configuration C;
  C.addFile("../bullet/bots.g");
//  C.addFile("../kin/arm3.g");
  arr q0 = C.getJointState();
  arr v0 = zeros(q0.N);

  rai::Simulation S(C, S._physx, 4);
  rai::wait();

  double tau=.01;
  Metronome tic(tau);

  rai::system("mkdir -p z.vid/; rm -f z.vid/*.ppm");

  for(uint t=0;t<4./tau;t++){
    tic.waitForTic();

    S.step((q0,v0).reshape(2,-1), tau, S._posVel);

    write_ppm(S.getScreenshot(), STRING("z.vid/"<<std::setw(4)<<std::setfill('0')<<t<<".ppm"));
  }

  rai::wait();

}

//===========================================================================

int MAIN(int argc,char **argv){
  rai::initCmdLine(argc, argv);

  testMotors();
  testRndScene();
  testFriction();
  testStackOfBlocks();
  testCompound();
  testPushes();
  testOpenClose();
  testGrasp();

  return 0;
}
