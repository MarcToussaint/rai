#include <Core/graph.h>

//===========================================================================

void TEST(Read){
  rai::Graph G;

  G.checkConsistency();
  cout <<"\n** reading graph..." <<std::flush;
  // G.read("example.g", true); //including parse info
  G.read(FILE("example.yml"), true, true); //including parse info
  cout <<G <<endl;
  G.checkConsistency();
  cout <<"\ndone" <<endl;
  cout <<"read graph=\n--------------------\n" <<G <<"\n--------------------" <<endl;

  cout <<"\n** access to individual items:" <<endl;
  cout <<G.findNode("k") <<endl;
  cout <<G.findNode("k")->graph() <<endl;
  //  cout <<G["val"]->graph()(0)->get<double>() <<endl;
  cout <<G.findNode("k")->graph().findNode("z")->as<rai::String>() <<endl;
  cout <<"DONE" <<endl;

  // G.checkUniqueKeys(true);
  cout <<"--- as yaml serial ---\n" <<G.asYaml(true) <<"---" <<endl;
  cout <<"--- as yaml normal ---\n" <<G.asYaml(false) <<"---" <<endl;
}

//===========================================================================

void TEST(Init){
  rai::Graph G = {"x", "b", {"a", 3.}, {"b", {"x"}, 5.}, {"c", rai::String("BLA")} };
  cout <<G <<endl;
  G.checkConsistency();

  rai::Graph B;

  B <<"x" <<"b" <<rai::NodeInitializer("a", 3.) <<rai::NodeInitializer("b", {"x"}, arr{1.,2.,3.});
  cout <<B <<endl;
}

//===========================================================================

const rai::Graph& rndContainer(const rai::Graph& G){
  const rai::Graph *g=&G;
  while(rnd.uni()<.8){
    if(!g->isNodeOfGraph) break;
    g = &g->isNodeOfGraph->container;
  }
  return *g;
}

rai::Graph& rndSubgraph(rai::Graph& G){
  rai::Graph *g=&G;
  while(rnd.uni()<.8){
    rai::NodeL subgraphs = g->getNodesOfType<rai::Graph>(nullptr);
    if(!subgraphs.N) break;
    rai::Node *subgraph=subgraphs.rndElem();
    g = &subgraph->graph();
  }
  return *g;
}

rai::NodeL rndParents(const rai::Graph& G){
  if(!G.N) return {};
  uint nparents=rnd.uni_int(0,10);
  rai::NodeL par;
  for(uint i=0;i<nparents;i++){
    par.append(rndContainer(G).rndElem());
  }
  return par;
}

void rndModify(rai::Graph& G){
  switch(rnd(4)){
    case 0://add bool item
      G.add<bool>(rai::String().setRandom(), true) ->setParents(rndParents(G));
      break;
    case 1://add Subgraph item
      G.addSubgraph(rai::String().setRandom(), rndParents(G));
      break;
    case 2://delete item
      if(G.N) delete G.rndElem();
      break;
    case 3://clone an item
      if(G.N) G.rndElem()->newClone(G);
      break;
    default:HALT("");
  }
}

//===========================================================================

void TEST(Random){
  rai::Graph A,B;

  rai::ArrayG<int> Ax(A);

  for(uint k=0;k<10;k++){
    rndModify(rndSubgraph(A));
    rai::Graph& C = rndSubgraph(A).addSubgraph();

    rai::Graph& D = rndSubgraph(A);
    if(D.N){
      D.getRenderingInfo(D.rndElem()).dotstyle.setRandom();
    }

    cout <<"---" <<endl <<A <<endl; // <<Ax <<endl;

    A.checkConsistency();
    C.checkConsistency();
    B = A;
    B.checkConsistency();
    delete C.isNodeOfGraph;
    A.checkConsistency();

  }
  A.clear();
  A.checkConsistency();
  B.clear();
  B.checkConsistency();
}

//===========================================================================

void TEST(Dot){
  rai::Graph G;
  G <<FILE("coffee_shop.fg");
  G.checkConsistency();
  //  G.sortByDotOrder();
  //  G.checkConsistency();
  G.writeDot(FILE("z.dot").getOs());
}

//===========================================================================

struct Something{
  Something(double y=0.){ x=y; }
  double x;
};

void operator<<(ostream& os, const Something& s){ os <<s.x; }
//the following 2 lines are optional: they enable naming the type and typed reading from file
void operator>>(istream& is, Something& s){ is >>s.x; }
bool operator==(const Something&, const Something&){ return false; }

void TEST(Manual){
  rai::Graph G;
  G.add<Something>("hallo", Something(3));
  cout <<G <<endl;
}

//===========================================================================

int MAIN(int argc, char** argv){
  rai::initCmdLine(argc, argv);

  // testRandom();
  testRead(); return 0;
  testInit();
  testDot();

  testManual();

  return 0;
}
