#include <Core/graph.h>

//const char *filename="/home/mtoussai/git/3rdHand/documents/USTT/14-meeting3TUD/box.g";
const char *filename=nullptr;

//===========================================================================

void TEST(Read){
  rai::Graph G;

  G.checkConsistency();
  cout <<"\n** reading graph..." <<flush;
  G.read(FILE(filename?filename:"example.g"), true); //including parse info
  cout <<G <<endl;
  G.checkConsistency();
  cout <<"\ndone" <<endl;
  G.writeParseInfo(cout);
  cout <<"read graph=\n--------------------\n" <<G <<"\n--------------------" <<endl;

//  Node *m = G["modify"];
//  G.merge(m);
//  cout <<"'k modify' merged with 'k':" <<*G["k"] <<endl;

//  rai::Node *n = G.first();

  G.checkConsistency();
  if(filename) return; //below only for "example.g"
  cout <<"\n** access to individual items:" <<endl;
  cout <<*G["k"] <<endl;
  cout <<G["k"]->graph() <<endl;
//  cout <<G["val"]->graph()(0)->get<double>() <<endl;
  cout <<G["k"]->graph()["z"]->get<rai::String>() <<endl;
  cout <<"DONE" <<endl;

  G.writeHtml(FILE("z.html"), FILE("example.g"));
}

//===========================================================================

void TEST(Init){
  rai::Graph G = {"x", "b", {"a", 3.}, {"b", {"x"}, 5.}, {"c", rai::String("BLA")} };
  cout <<G <<endl;
  G.checkConsistency();

  rai::Graph B;

  B <<"x" <<"b" <<rai::NodeInitializer("a", 3.) <<rai::NodeInitializer("b", {"x"}, ARR(1.,2.,3.));
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
  uint nparents=rnd(0,10);
  rai::NodeL par;
  for(uint i=0;i<nparents;i++){
    par.append(rndContainer(G).rndElem());
  }
  return par;
}

void rndModify(rai::Graph& G){
  switch(rnd(4)){
    case 0://add bool item
      G.newNode<bool>(rai::String().setRandom(), rndParents(G), true);
      break;
    case 1://add Subgraph item
      G.newSubgraph(rai::String().setRandom(), rndParents(G));
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
    rai::Graph& C = rndSubgraph(A).newSubgraph({}, {});

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
  G <<FILE(filename?filename:"coffee_shop.fg");
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
  G.newNode<Something>({"hallo"}, {}, Something(3));
  cout <<G <<endl;
}

//===========================================================================

int MAIN(int argc, char** argv){
  rai::initCmdLine(argc, argv);

  if(argc>1 && argv[1][0]!='-') filename=argv[1];

  testRandom();
  testRead();
  testInit();
  testDot();

  testManual();

  return 0;
}
