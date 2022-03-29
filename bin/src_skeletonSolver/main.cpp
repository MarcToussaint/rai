#include <KOMO/skeleton.h>

//===========================================================================

const char *USAGE =
    "\nUSAGE:  skeletonSolver <sktFile> <confFile>  -mode [path|sequence] -collisions [true|false]"
    "\n        (set parameters in rai.cfg alternatively, see z.log.global for a log of all used options)"
    "\n";

//===========================================================================

int main(int argc,char **argv){
  rai::initCmdLine(argc, argv);

//  rnd.clockSeed();
  rnd.seed(0);

  cout <<USAGE <<endl;

  rai::String sktFile = rai::getParameter<rai::String>("sktFile", STRING("none"));
  rai::String confFile=rai::getParameter<rai::String>("confFile", STRING("none"));
  rai::ArgWord mode = rai::getParameter<rai::Enum<rai::ArgWord>>("mode", rai::_path);
  bool collisions = rai::getParameter<bool>("collisions", false);
  if(rai::argc>=2 && rai::argv[1][0]!='-') sktFile=rai::argv[1];
  if(rai::argc>=3 && rai::argv[1][0]!='-' && rai::argv[2][0]!='-') confFile=rai::argv[2];

  LOG(0) <<"using sktFile '" <<sktFile <<"'";
  LOG(0) <<"using confFile '"  <<confFile <<"'";
  LOG(0) <<"using mode: " <<mode;
  LOG(0) <<"using collisions: " <<collisions;

  rai::Configuration C;
  C.addFile(confFile);
  C.watch();

  rai::Skeleton S;
  S.read(FILE(sktFile));
  S.collisions = collisions;
  cout <<S <<endl;

  S.setConfiguration(C);
//  S.solve(mode, rai::getParameter<int>("verbose", 3));
  S.solve3(true, rai::getParameter<int>("verbose", 3));

  rai::wait();

  return 0;
}

