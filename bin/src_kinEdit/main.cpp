#include <Kin/kin.h>
#include <Gui/opengl.h>
#include <Core/graph.h>

const char *USAGE =
    "\nUsage:  kinEdit <g-filename>"
    "\n"
    "\nIterate between editing the file (with an external editor) and"
    "\nviewing the model in the OpenGL window (after pressing ENTER)."
    "\nUse the number keys 1 2 3 4 5 to toggle display options.";

int main(int argc,char **argv){
  rai::initCmdLine(argc, argv);

  cout <<USAGE <<endl;

  rai::String file=rai::getParameter<rai::String>("file",STRING("test.g"));
  if(rai::argc>=2 && rai::argv[1][0]!='-') file=rai::argv[1];
  LOG(0) <<"opening file `" <<file <<"'" <<endl;

  //-- load configuration
  rai::Configuration C;
  for(;;){
    Inotify ino(file);
    try {
      rai::lineCount=1;
      C.readFromGraph(file);
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

  //-- report collisions
//  C.stepSwift();
//  LOG(0) <<"total penetration: " <<C.totalCollisionPenetration();
//  LOG(0) <<"collision report: ";
//  C.reportProxies(cout, 0.);

  //-- some optional manipulations
  if(rai::checkParameter<bool>("prune")){
    cout <<"PRUNING STRUCTURE" <<endl;
    C.optimizeTree(true, true, false);
  }else{
    C.optimizeTree(false, false, false);
  }
  C.ensure_q();
  C.checkConsistency();
  C.sortFrames();

  //    makeConvexHulls(G.frames);
  //    computeOptimalSSBoxes(G.shapes);

  //-- save file in different formats
  FILE("z.g") <<C;
  C.writeURDF(FILE("z.urdf"));
  C.writeCollada("z.dae");
  if(rai::checkParameter<bool>("dot")) C.displayDot();

  if(rai::checkParameter<bool>("cleanOnly")) return 0;

  //-- continuously animate
  editConfiguration(file, C);

  return 0;
}
