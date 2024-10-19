#include <Algo/minimalSpanningTree.h>
#include <Gui/plot.h>
#include <Algo/ann.h>
#include <math.h>

void testMinimalSpanningTree(){

  uint k=20;
  arr X = 2.*rand(k, 2) - 1.;

  plot()->Points(X);
  plot()->update(true);

  rai::Array<DoubleEdge> G;

  ANN ann;
  ann.setX(X);
  uint neigh=5;
  uintA idx;
  arr sqrDists;
  for(uint i=0;i<X.d0;i++){
    ann.getkNN(sqrDists, idx, X[i], neigh);
    for(uint nn=1;nn<idx.N;nn++){
      uint j = idx(nn);
      double d = ::sqrt(sqrDists(nn));
      G.append(DoubleEdge{i,j,d});
    }
  }

  for(DoubleEdge& e: G){
    plot()->Line((X[e.i], X[e.j]).reshape(2,2));
  }
  plot()->update(true);

  auto T = minimalSpanningTree(X.d0, G);

  plot()->Clear();
  for(uint e: std::get<1>(T)){
    plot()->Line((X[G(e).i], X[G(e).j]).reshape(2,2));
  }
  plot()->update(true);

}

// =============================================================================

int MAIN(int argc,char **argv){
  rai::initCmdLine(argc, argv);

  testMinimalSpanningTree();

  return 0;
}
