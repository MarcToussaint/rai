#include <Kin/kin.h>
#include <Kin/frame.h>
#include <Gui/opengl.h>

//===========================================================================
//
// test laod save
//

void TEST(LoadAssimp){
  rai::KinematicWorld K;

  const char* filename = "/home/mtoussai/CAD/LG14-Modell.dae";
  if(rai::argc>1) filename = rai::argv[1];
  K.addAssimp(filename);
  K.watch(true);

  for(auto *f:K.frames){
    f->shape->mesh().fuseNearVertices();
    f->shape->mesh().C = id2color(f->ID);
  }
  K.watch(true);

  makeConvexHulls(K.frames, false);
  K.watch(true);
}

// =============================================================================

int MAIN(int argc,char **argv){
  rai::initCmdLine(argc, argv);

  testLoadAssimp();

  return 0;
}
