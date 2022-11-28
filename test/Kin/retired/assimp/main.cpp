#include <Kin/kin.h>
#include <Kin/frame.h>
#include <Gui/opengl.h>

//===========================================================================
//
// test laod save
//

void TEST(LoadAssimp){
  rai::Configuration K;

  const char* filename = "/home/mtoussai/git/bullet3/data/kuka_lwr/meshes_arm/arm_wrist.dae";
  if(rai::argc>1 && rai::argv[1][0]!='-') filename = rai::argv[1];
  K.addAssimp(filename);
  K.view(true);

  for(auto *f:K.frames){
    f->shape->mesh().fuseNearVertices();
    f->shape->mesh().C = id2color(f->ID);
  }
  K.view(true);

  makeConvexHulls(K.frames, false);
  K.view(true);
}

// =============================================================================

int MAIN(int argc,char **argv){
  rai::initCmdLine(argc, argv);

  testLoadAssimp();

  return 0;
}
