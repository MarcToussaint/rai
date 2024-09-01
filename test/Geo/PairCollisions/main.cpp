#include <Geo/mesh.h>
#include <Gui/opengl.h>
#include <Gui/RenderData.h>
#include <Geo/pairCollision.h>
#include <Kin/kin.h>
#include <Kin/frame.h>

extern bool orsDrawWires;

//===========================================================================

void TEST(PairCollision){
  uint n=50;
  MeshA meshes(n);
  rai::RenderData R;
  R.addStandardScene();
  for(uint i=0;i<n;i++){
    rai::Mesh &m=meshes(i);
    m.setRandom(20);
    m.scale(.4);
    m.translate(rnd.uni(-2.,2.), rnd.uni(-2.,2.), rnd.uni(1.,4.));
//    m.buildGraph();
    m.C = {.8,.8,.8,.4};

    R.add().mesh(m, 0);
  }
  OpenGL gl;
  gl.add(&R);
//  gl.update(true);

  arr D(n,n);
  D.setZero();
  rai::timerStart();
  for(uint i=0;i<n;i++) for(uint j=i+1;j<n;j++){
    rai::PairCollision pc(meshes(i), meshes(j), 0, 0);
    D(i,j)=pc.distance;
#if 1 //turn off for timing
    cout <<pc <<endl;
    {
      auto lock = R.dataLock(RAI_HERE);
      R.addDistMarker(pc.p1, pc.p2);
    }
    if(pc.distance<0) gl.update(true); else gl.timedupdate(.01);
//    gl.update(true);
    {
      auto lock = R.dataLock(RAI_HERE);
      R.distMarkers.pos.clear();
      R.distMarkers.slices.clear();
    }
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

////    C.stepFcl();
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
