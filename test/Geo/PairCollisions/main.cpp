#include <stdlib.h>
#include <GL/gl.h>

#include <Geo/mesh.h>
#include <Gui/opengl.h>
#include <Geo/pairCollision.h>
#include <Kin/kin.h>
#include <Kin/frame.h>

extern bool orsDrawWires;

//===========================================================================

void TEST(PairCollision){
  uint n=50;
  MeshA meshes(n);
  OpenGL gl;
  gl.add(glStandardScene);
  for(uint i=0;i<n;i++){
    rai::Mesh &m=meshes(i);
    m.setRandom(20);
    m.scale(1.);
    m.translate(rnd.uni(-5.,5.), rnd.uni(-5.,5.), rnd.uni(1.,10.));
    m.buildGraph();
    m.C = {.8,.8,.8,.4};

    gl.add(m);
  }
//  gl.watch();

  arr D(n,n);
  D.setZero();
  rai::timerStart();
  for(uint i=0;i<n;i++) for(uint j=i+1;j<n;j++){
    PairCollision pc(meshes(i), meshes(j), 0, 0);
    D(i,j)=pc.distance;
#if 1 //turn off for timing
    cout <<pc <<endl;
    gl.add(pc);
    if(pc.distance<0) gl.watch(); else gl.timedupdate(.1);
    gl.remove(pc);
#endif
  }
  cout <<"time: " <<rai::timerRead() <<"sec" <<endl;

//  arr q0,q;
//  q0 = C.getJointState();
//  rai::timerStart();
//  for(uint t=0;t<3;t++){
//    for(rai::Frame *a:C.frames){
//      a->setPose(rai::Transformation().setRandom());
//      a->set_X()->pos.z += 1.;
//      a->set_X()->pos *= 5.;
//    }

////    C.stepSwift();
//    C.stepFcl();
//    cout <<"#proxies: " <<C.proxies.N <<endl; //this also calls pair collisions!!
//    cout <<"time: " <<rai::timerRead() <<endl;
//    cout <<"total penetration: " <<C.totalCollisionPenetration() <<endl; //this also calls pair collisions!!
//    cout <<"time: " <<rai::timerRead() <<endl;
//     C.reportProxies(FILE("z.col"), 0.);
//  }
//  cout <<" query time: " <<rai::timerRead(true) <<"sec" <<endl;
}

//===========================================================================

int MAIN(int argc, char** argv){
  rai::initCmdLine(argc, argv);

//  rnd.clockSeed();

  testPairCollision();

  return 0;
}
