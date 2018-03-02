#include <Kin/kin.h>
#include <Gui/opengl.h>
#include <Core/graph.h>

const char *USAGE=
"\n\
Usage:  ors_edit <ors-filename>\n\
\n\
Iterate between editing the file (with an external editor) and\n\
viewing the model in the OpenGL window (after pressing ENTER).\n\
\n\
Use the number keys 1 2 3 4 5 to toggle display options.\n\
";


int MAIN(int argc,char **argv){
  mlr::initCmdLine(argc, argv);

    cout <<USAGE <<endl;

    mlr::String file=mlr::getParameter<mlr::String>("file",STRING("test.ors"));
    if(mlr::argc==2 && mlr::argv[1][0]!='-') file=mlr::argv[1];
    cout <<"opening file `" <<file <<"'" <<endl;

    mlr::KinematicWorld K(file);

    K.checkConsistency();
    K >>FILE("z.g");
    //some optional manipulations
    K.optimizeTree(false);
    K.calc_q();
    K.checkConsistency();
    K >>FILE("z.g");
//    makeConvexHulls(G.frames);
//    computeOptimalSSBoxes(G.shapes);
//    G >>FILE("z.ors");
//    K.watch(true);
//    return;

    K.writeURDF(FILE("z.urdf"));

    if(mlr::checkParameter<bool>("cleanOnly")) return 0;

    editConfiguration(file, K);

  return 0;
}
