#include "minimalSpanningTree.h"

struct DisjointSetUnion {
  intA parent;
  intA rank;

  DisjointSetUnion(int n) : parent(n), rank(n){
    parent = -1;
    rank = 1;
  }

  int findRoot(int i) {
    if (parent.p[i] == -1) return i;
    return parent.p[i] = findRoot(parent.p[i]);
  }

  void unite(int i, int j) {
    int root1 = findRoot(i);
    int root2 = findRoot(j);

    if (root1 != root2) {
      if (rank.p[root1] < rank.p[root2]) {
        parent.p[root1] = root2;
      } else if (rank.p[root1] > rank.p[root2]) {
        parent.p[root2] = root1;
      } else {
        parent.p[root2] = root1;
        rank.p[root1]++;
      }
    }
  }
};

uintA minimalSpanningTree(uint num_vertices, rai::Array<DoubleEdge>& edges){

  //sort edges
  std::sort(edges.p, edges.p+edges.N,
            [](const DoubleEdge& a, const DoubleEdge& b) -> bool { return a.w <= b.w; }
  );

  DisjointSetUnion s(num_vertices);
  uintA selected;

  //add edges when not in same component (which are identified by root)
  for(DoubleEdge& e : edges) {
    if(s.findRoot(e.i) != s.findRoot(e.j)) {
      s.unite(e.i, e.j);
      selected.append(&e - edges.p);
      //cost += e.w;
    }
  }
  return selected;
}
