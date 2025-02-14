#include <DataGen/shapenetGrasps.h>
#include <Core/h5.h>

//===========================================================================

void testLowLevel() {
  ShapenetGrasps SG;
  // SG.opt.verbose = 2;

  SG.loadObject(3);
  arr pcl = SG.getPointCloud();
  cout <<"point cloud size: " <<pcl.d0 <<endl;

  arr pose = SG.sampleGraspPose();
  cout <<"candidate pose: " <<pose <<endl;

  arr scores = SG.evaluateGrasp();
  cout <<"scores: " <<scores <<endl;
}

//===========================================================================

void testBatchGeneration() {
  ShapenetGrasps SG;
  arr X,S;
  uintA Z;
  SG.getSamples(X, Z, S, 10);

  SG.displaySamples(X, Z, S);

  //reevaluate samples - just to check
  for(uint i=0;i<X.d0;i++){
    arr scores = SG.evaluateSample(X[i], Z(i));
    cout <<"scores " <<i <<"\n  data: " <<S[i] <<"\n  eval: " <<scores.reshape(-1) <<' ' <<(min(scores)>0.) <<endl;
  }
}

//===========================================================================

void generateGraspsFiles(uint N=10) {
  ShapenetGrasps SG;

  if(SG.opt.endShape<0) SG.opt.endShape = SG.files.N;

  for(uint shape=SG.opt.startShape;shape<(uint)SG.opt.endShape;shape++){

    bool succ = SG.loadObject(shape);
    if(!succ) continue;

    floatA Xsucc, Xfail;

    for(uint i=0;i<N;i++){
      SG.resetObjectPose();
      arr pose = SG.sampleGraspPose();
      if(!pose.N) continue;

      arr scores = SG.evaluateGrasp();

      if(min(scores)>0.) Xsucc.append(convert<float>(pose).reshape(1,-1));
      else Xfail.append(convert<float>(pose).reshape(1,-1));

      cout <<"  -- shape: " <<shape <<" eval: " <<i+1 <<" #succ: " <<Xsucc.d0 <<
	" #fail: " <<Xfail.d0 <<endl;
    }

    if(!Xsucc.d0) continue;

    rai::system("mkdir -p z");
    str filename = "z/" + SG.files(shape);
    filename.removePostfix(".shape.h5");
    filename <<".grasps.h5";
    LOG(0) <<"writing grasps to " <<filename;

    rai::H5_Writer h5(filename);
    h5.addGroup("grasps");
    if(Xsucc.N) h5.add("grasps/success", Xsucc);
    if(Xfail.N) h5.add("grasps/fail", Xfail);
  }
}

void displayGraspsFiles(uint N=10) {
  ShapenetGrasps SG;
  floatA X;
  uintA shapes;

  for(uint shape=SG.opt.startShape;shape<(uint)SG.opt.endShape;shape++){
    str filename = "shapenet/grasps/" + SG.files(shape);
    filename.removePostfix(".shape.h5");
    filename <<".grasps.h5";

    if(!rai::FileToken(filename).exists()) continue;
    rai::H5_Reader h5(filename);
    floatA Xsucc = h5.read<float>("grasps/success");
    X.append(Xsucc);
    shapes.append(shape, Xsucc.d0);
  }

  SG.displaySamples(convert<double>(X), shapes);
}


//===========================================================================

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  rnd.clockSeed();

  // testLowLevel();
  // testBatchGeneration();
  // generateGraspsFiles(100);
  displayGraspsFiles();

  return 0;
}

