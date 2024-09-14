#include <Kin/kin.h>
#include <Gui/opengl.h>
#include <Core/graph.h>
#include <Kin/frame.h>
#include <Kin/simulation.h>

const char *USAGE =
    "\nUsage:  kinEdit <g-filename>"
    "\n"
    "\n  -file <g-file>"
    "\n  -prune           optimize the tree structure and prune useless frames"
    "\n  -makeConvexHulls   make all meshes convex"
    "\n  -collisions      compute collisions in the scene and report proxies"
    "\n  -writeMeshes     write all meshes in a folder"
    "\n  -convert         write various export formats (urdf, collada)"
    "\n  -dot             illustrate the tree structure as graph"
    "\n  -cleanOnly       skip the animation/edit loop\n";

int main(int argc,char **argv){
  rai::initCmdLine(argc, argv);

  cout <<USAGE <<endl;

  rai::String file=rai::getParameter<rai::String>("file",STRING("none"));
  if(rai::argc>=2 && rai::argv[1][0]!='-') file=rai::argv[1];
  LOG(0) <<"== opening file `" <<file <<"'" <<endl;

  if(file=="none") return 0;

  if(rai::checkParameter<bool>("process")){
    //-- load configuration
    rai::Configuration C;
    try {
      rai::lineCount=1;
      C.clear();
      if(file.endsWith(".dae")){
        C.addAssimp(file);
      }else{
        C.addFile(file);
      }
      C.report();
    } catch(std::runtime_error& err) {
      cout <<"line " <<rai::lineCount <<": " <<err.what() <<" -- please check the file and press ENTER" <<endl;
      return 0;
    }

    C.checkConsistency();

    //-- some optional manipulations
    if(rai::checkParameter<bool>("prune")){
      LOG(0) <<"PRUNING STRUCTURE";
      C.optimizeTree(true, true, false);
    }

    //-- sort frames
    //  if(rai::checkParameter<bool>("sort")){
    LOG(0) <<"SORTING FRAMES";
    C.sortFrames();
    //  }

    //    C.optimizeTree(false, false, false);
    C.ensure_q();
    C.checkConsistency();
    C.checkUniqueNames(true);

    //-- make convex
    if(rai::checkParameter<bool>("makeConvexHulls")){
      LOG(0) <<"creating convex hulls";
      makeConvexHulls(C.frames, false);
    }

    //-- report collisions
    if(rai::checkParameter<bool>("collisions")){
      C.ensure_proxies();
      LOG(0) <<"total penetration: " <<C.getTotalPenetration();
      LOG(0) <<"collision report: ";
      C.reportProxies(cout, 0.);
    }

    //-- save meshes
    if(rai::checkParameter<bool>("writeMeshes")){
      LOG(0) <<"writing meshes";
      rai::system("mkdir -p meshes");
      C.writeMeshes();
    }

    //-- save file in different formats
    FILE("z.g") <<C;
    if(rai::checkParameter<bool>("convert")){
      LOG(0) <<"saving urdf and dae files";
      C.writeURDF(FILE("z.urdf"));
      C.writeMesh("z.ply");
      C.writeCollada("z.dae");
    }

    if(rai::checkParameter<double>("scale")){
      for(rai::Frame *f:C.frames) if(f->shape){
        f->shape->mesh().scale(rai::getParameter<double>("scale"));
        f->shape->mesh().C = id2color(f->ID);
      }
      C.writeMeshes("meshes/");
      FILE("z.ex.g") <<C;
    }

    if(rai::checkParameter<bool>("dot")) C.displayDot();
  }

  if(rai::checkParameter<bool>("cleanOnly")) return 0;

  if(rai::checkParameter<bool>("sim")){
    rai::Configuration C;
    C.addFile(file);
    rai::Simulation S(C, S._physx, 2);
    S.loadTeleopCallbacks();

    double tau=.01;
    Metronome tic(tau);
    while(!S.teleopCallbacks->stop){
      tic.waitForTic();
      S.step({}, tau, S._position);
      //C.ensure_proxies();
      //C.getTotalPenetration();
      //C.reportProxies();
      //C.view();
    }
  }
  else if(file.endsWith(".dae")){
    LOG(0) <<"opening dae file";
    rai::Configuration C;
    C.addAssimp(file);
    C.view(true);
  }
  else{
    rai::Configuration C;
    C.watchFile(file);
  }

  return 0;
}
