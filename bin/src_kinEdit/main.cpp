#include <Kin/kin.h>
#include <Gui/opengl.h>
#include <Core/graph.h>

void mesh_readAssimp(char const*){}

const char *USAGE=
"\n\
Usage:  kinEdit <g-filename>\n\
\n\
Iterate between editing the file (with an external editor) and\n\
viewing the model in the OpenGL window (after pressing ENTER).\n\
\n\
Use the number keys 1 2 3 4 5 to toggle display options.\n\
";

int MAIN(int argc,char **argv){
  rai::initCmdLine(argc, argv);

    cout <<USAGE <<endl;

    rai::String file=rai::getParameter<rai::String>("file",STRING("test.ors"));
    if(rai::argc==2 && rai::argv[1][0]!='-') file=rai::argv[1];
    cout <<"opening file `" <<file <<"'" <<endl;

    rai::KinematicWorld K(file);

    K.checkConsistency();
    K >>FILE("z.g");
    //some optional manipulations
    K.optimizeTree(false);
    K.calc_q();
    K.checkConsistency();
    if(K.fwdActiveSet.N == K.frames.N) K.frames=K.fwdActiveSet; //adopt sorting of forward..
    K >>FILE("z.g");
//    makeConvexHulls(G.frames);
//    computeOptimalSSBoxes(G.shapes);
//    G >>FILE("z.ors");
//    K.watch(true);
//    return;

    K.writeURDF(FILE("z.urdf"));

    if(rai::checkParameter<bool>("cleanOnly")) return 0;

    editConfiguration(file, K, K.gl());

  return 0;
}
