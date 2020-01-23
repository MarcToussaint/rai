#include <Kin/kin.h>
#include <Gui/opengl.h>
#include <Core/graph.h>

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

    rai::String file=rai::getParameter<rai::String>("file",STRING("test.g"));
    if(rai::argc>=2 && rai::argv[1][0]!='-') file=rai::argv[1];
    cout <<"opening file `" <<file <<"'" <<endl;

    rai::Configuration C;
    for(;;){
    Inotify ino(file);
    try {
      rai::lineCount=1;
      C.init(file);
      C.report();
      break;
    } catch(std::runtime_error& err) {
      cout <<"line " <<rai::lineCount <<": " <<err.what() <<" -- please check the file and press ENTER" <<endl;
      for(;;) {
        if(ino.poll(false, true)) break;
        rai::wait(.02);
      }
    }
    }

    C.checkConsistency();
    C >>FILE("z.g");
    //some optional manipulations
    if(rai::checkParameter<bool>("prune")){
      C.optimizeTree(true, true, true);
    }else{
      C.optimizeTree(false, false, false);
    }
    C.ensure_q();
    C.checkConsistency();
    C.sortFrames();

//    makeConvexHulls(G.frames);
//    computeOptimalSSBoxes(G.shapes);

    C >>FILE("z.g");

    if(rai::checkParameter<bool>("dot")) C.displayDot();
    C.writeURDF(FILE("z.urdf"));

    if(rai::checkParameter<bool>("cleanOnly")) return 0;

    editConfiguration(file, C);

  return 0;
}
