#include <Core/graph.h>

void read(const char* filename){
  Graph G;
  G.read(FILE(filename), true); //including parse info
  cout <<G <<endl;
  G.checkConsistency();
  cout <<"\ndone" <<endl;
  G.writeParseInfo(cout);
  cout <<"read graph=\n--------------------\n" <<G <<"\n--------------------" <<endl;

  G.writeHtml(FILE("z.html"), FILE(filename));
  G.writeDot(FILE("z.dot").getOs());

  rai::system("firefox z.html &");
  rai::system("dot -Tpdf z.dot > z.pdf");
  rai::system("evince z.pdf &");
}

//===========================================================================

int MAIN(int argc, char** argv){
  rai::initCmdLine(argc, argv);

  if(argc<2){
    cout <<"need g file as argument" <<endl;
    return 1;
  }

  read(argv[1]);

  return 0;
}
