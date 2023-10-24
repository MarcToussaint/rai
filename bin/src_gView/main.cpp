#include <yaml-cpp/yaml.h>
#include <Core/graph.h>

//===========================================================================

void read(const char* filename){
  rai::Graph G(filename, false);  //including parse info?
  G.checkConsistency();
  cout <<"--------- read graph -----------\n" <<G <<"\n--------------------" <<endl;

  //-- display html
  //G.writeHtml(FILE("z.html"), FILE(filename));
  //rai::system("firefox z.html &");

  //-- output
  FILE("z.g") <<G;
  G.writeYaml(FILE("z.yaml"));
  return;

  //-- test yaml consistency
  G.writeYaml(FILE("z1.yaml"));
  try {
    YAML::Node yaml = YAML::LoadFile(filename);
    FILE("z2.yaml") <<yaml;
  }catch(YAML::ParserException& e){
    cout <<"*** file '" <<filename <<"' is not yaml compatible:\n    " <<e.what() <<endl;
  }
  try {
    YAML::Node yaml = YAML::LoadFile("z1.yaml");
    FILE("z3.yaml") <<yaml;
  }catch(YAML::ParserException& e){
    cout <<"*** Graph::writeYaml('z1.yaml') is not yaml compatible!!!\n    " <<e.what() <<endl;
  }
  //rai::system("diff z2.yaml z3.yaml");

  //-- display dot
  G.writeDot(FILE("z.dot").getOs());
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
