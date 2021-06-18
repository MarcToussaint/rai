#include <Core/graph.h>

void read(const char* filename){
  rai::Graph G(filename, false);  //including parse info?
  cout <<G <<endl;
  G.checkConsistency();
  cout <<"\ndone" <<endl;
//  G.writeParseInfo(cout);
  cout <<"read graph=\n--------------------\n" <<G <<"\n--------------------" <<endl;

  G.writeHtml(FILE("z.html"), FILE(filename));
  G.writeDot(FILE("z.dot").getOs());

//  rai::system("firefox z.html &");
  rai::system("dot -Tpdf z.dot > z.pdf");
  rai::system("evince z.pdf &");
}

//===========================================================================

int MAIN(int argc, char** argv){
  rai::initCmdLine(argc, argv);

  rai::String file=rai::getParameter<rai::String>("file",STRING("none"));
  if(rai::argc>=2 && rai::argv[1][0]!='-') file=rai::argv[1];
  LOG(0) <<"opening file `" <<file <<"'" <<endl;

  if(file=="none") return 1;

  read(file);

  return 0;
}
