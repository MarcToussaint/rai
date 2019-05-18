#include <Core/graph.h>

//const char *filename="/home/mtoussai/git/3rdHand/documents/USTT/14-meeting3TUD/box.g";
const char *filename=NULL;

//===========================================================================

void TEST(Read){
  Graph G;

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
  Graph G = {"x", "b", {"a", 3.}, {"b", {"x"}, 5.}, {"c", rai::String("BLA")} };
  cout <<G <<endl;
  G.checkConsistency();

  Graph B;

  B <<"x" <<"b" <<NodeInitializer("a", 3.) <<NodeInitializer("b", {"x"}, ARR(1.,2.,3.));
  cout <<B <<endl;
}

//===========================================================================

const Graph& rndContainer(const Graph& G){
  const Graph *g=&G;
  while(rnd.uni()<.8){
    if(!g->isNodeOfGraph) break;
    g = &g->isNodeOfGraph->container;
  }
  return *g;
}

Graph& rndSubgraph(Graph& G){
  Graph *g=&G;
  while(rnd.uni()<.8){
    NodeL subgraphs = g->getNodesOfType<Graph>(NULL);
    if(!subgraphs.N) break;
    Node *subgraph=subgraphs.rndElem();
    g = &subgraph->graph();
  }
  return *g;
}

NodeL rndParents(const Graph& G){
  if(!G.N) return {};
  uint nparents=rnd(0,10);
  NodeL par;
  for(uint i=0;i<nparents;i++){
    par.append(rndContainer(G).rndElem());
  }
  return par;
}

void rndModify(Graph& G){
  switch(rnd(4)){
    case 0://add bool item
      G.newNode<bool>({rai::String().setRandom(), rai::String().setRandom()}, rndParents(G), true);
      break;
    case 1://add Subgraph item
      G.newSubgraph({rai::String().setRandom(), rai::String().setRandom()}, rndParents(G));
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
  Graph A,B;

  ArrayG<int> Ax(A);

  for(uint k=0;k<10;k++){
    rndModify(rndSubgraph(A));
    Graph& C = rndSubgraph(A).newSubgraph({}, {});

    Graph& D = rndSubgraph(A);
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
  Graph G;
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
  Graph G;
  G.newNode<Something>({"hallo"}, {}, Something(3));
  cout <<G <<endl;
}

//===========================================================================

int MAIN(int argc, char** argv){
  rai::initCmdLine(argc, argv);

  cout <<"GLOBAL LATENT REGISTRY:\n" <<registry() <<endl;

  if(argc>=2) filename=argv[1];

  testRandom();
  testRead();
  testInit();
  testDot();

  testManual();

  return 0;
}
