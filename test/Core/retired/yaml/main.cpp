#include <yaml-cpp/yaml.h>
#include <Core/graph.h>

//===========================================================================

void testRead(){
  rai::String file = rai::getParameter<rai::String>("gfile");
  YAML::Node config = YAML::LoadFile(file.p);
  cout <<config;

  //rai::Graph G(file.p);
  //FILE("z.g") <<G;
}

//===========================================================================

namespace rai{

void graph2yamlNode(Node* n, YAML::Node& root){
  auto y = root[std::string(n->key.p)];
  if(n->is<Graph>()){
    Graph& g = n->graph();
    g.checkUniqueKeys(true);
    for(Node* p:n->parents) y["edge"].push_back(p->key.p);
    for(Node *ch:g) graph2yamlNode(ch, y);
  } else if(n->is<String>()) { y = n->as<String>().p;
  } else if(n->is<FileToken>()) { y = n->as<FileToken>().autoPath().p;
  } else if(n->is<arr>()) { for(double& x:n->as<arr>()) y.push_back(x);
  } else if(n->is<floatA>()) { for(float& x:n->as<floatA>()) y.push_back(x);
  } else if(n->is<uintA>()) { for(uint& x:n->as<uintA>()) y.push_back(x);
  } else if(n->is<intA>()) { for(int& x:n->as<intA>()) y.push_back(x);
  } else if(n->is<byteA>()) { for(byte& x:n->as<byteA>()) y.push_back(x);
  } else if(n->is<intAA>()) {
    NIY;//n->as<intAA>()->write(os, ", ", nullptr, "[]");
  } else if(n->is<StringA>()) { for(String& x:n->as<StringA>()) y.push_back(x.p);
  } else if(n->is<NodeL>()) { for(Node* x:n->as<NodeL>()) y.push_back(x->key.p);
  } else if(n->is<bool>()) { y = n->as<bool>();
  } else if(n->is<int>()) { y = n->as<int>();
  } else if(n->is<uint>()) { y = n->as<uint>();
  } else if(n->is<float>()) { y = n->as<float>();
  } else if(n->is<double>()) { y = n->as<double>();
  } else{
    THROW("type conversion not implemented: " <<rai::niceTypeidName(n->type))
    NIY;
  }
}

} //namespace

//===========================================================================

void testWrite(){
  rai::Graph G(rai::getParameter<rai::String>("gfile"));

  G.checkUniqueKeys(true);

  YAML::Node root;
  for(rai::Node* n:G) rai::graph2yamlNode(n, root);

  cout <<G <<endl;

  YAML::Emitter out;
  out.SetIndent(2);
//  out.SetMapFormat(YAML::Flow);
  out.SetSeqFormat(YAML::Flow);
//  out.SetStringFormat(YAML::DoubleQuoted);
  out <<root;

  std::cout <<out.c_str();
}

//===========================================================================

int main(int argn, char** argv){
  rai::initCmdLine(argn, argv);

  testRead();

  testWrite();
//  std::cout <<root;

  return 0;
}
