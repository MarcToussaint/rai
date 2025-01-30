#include <DataGen/shapenetGrasps.h>

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

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  rnd.clockSeed();

  testLowLevel();
  testBatchGeneration();

  return 0;
}

