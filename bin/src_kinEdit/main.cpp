#include <Kin/kin.h>
#include <Gui/opengl.h>
#include <Core/graph.h>

const char *USAGE =
    "\nUsage:  kinEdit <g-filename>"
    "\n"
    "\n  -file <g-file>"
    "\n  -prune           to optimize the tree structure and prune useless frames"
    "\n  -writeMeshes     to write all meshes in a folder"
    "\n  -dot             to illustrate the tree structure as graph"
    "\n  -cleanOnly       to skip the animation/edit loop\n";

int main(int argc,char **argv){
  rai::initCmdLine(argc, argv);

  cout <<USAGE <<endl;

  rai::String file=rai::getParameter<rai::String>("file",STRING("none"));
  if(rai::argc>=2 && rai::argv[1][0]!='-') file=rai::argv[1];
  LOG(0) <<"opening file `" <<file <<"'" <<endl;

  if(file=="none") return 0;

  //-- load configuration
  rai::Configuration C;
  for(;;){
    Inotify ino(file);
    try {
      rai::lineCount=1;
      C.clear();
      if(file.endsWith(".dae")){
        C.addAssimp(file);
      }else{
        C.addFile(file);
      }
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
  }
//    C.optimizeTree(false, false, false);
  C.ensure_q();
  C.checkConsistency();
  C.sortFrames();

  if(rai::checkParameter<bool>("writeMeshes")){
    rai::system("mkdir -p meshes");
    C.writeMeshes();
  }

  //    makeConvexHulls(G.frames);
  //    computeOptimalSSBoxes(G.shapes);

  //-- save file in different formats
  FILE("z.g") <<C;
  C.writeURDF(FILE("z.urdf"));
  C.writeCollada("z.dae");
  if(rai::checkParameter<bool>("dot")) C.displayDot();

  if(rai::checkParameter<bool>("cleanOnly")) return 0;

  //-- continuously animate
  if(file.endsWith(".dae")){
    C.watch(true);
  }else{
    editConfiguration(file, C);
  }

  return 0;
}
