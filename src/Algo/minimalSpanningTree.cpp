/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "minimalSpanningTree.h"

struct DisjointSetUnion {
  intA parent;
  intA rank;

  DisjointSetUnion(int n) : parent(n), rank(n) {
    parent = -1;
    rank = 1;
  }

  int findRoot(int i) {
    if(parent.p[i] == -1) return i;
    return parent.p[i] = findRoot(parent.p[i]);
  }

  void unite(int i, int j) {
    int root1 = findRoot(i);
    int root2 = findRoot(j);

    if(root1 != root2) {
      if(rank.p[root1] < rank.p[root2]) {
        parent.p[root1] = root2;
      } else if(rank.p[root1] > rank.p[root2]) {
        parent.p[root2] = root1;
      } else {
        parent.p[root2] = root1;
        rank.p[root1]++;
      }
    }
  }
};

std::tuple<double, uintA> minimalSpanningTree(uint num_vertices, const rai::Array<DoubleEdge>& edges) {

  //sort edges
  intA sorting; sorting.setStraightPerm(edges.N);

  std::sort(sorting.p, sorting.p+sorting.N,
            [&](const int& a, const int& b) -> bool { return edges.elem(a).w < edges.elem(b).w; });

  DisjointSetUnion s(num_vertices);
  uintA selected;
  double cost = 0.;

  //add edges when not in same component (which are identified by root)
  for(uint a:sorting) {
    const DoubleEdge& e = edges.elem(a);
    if(s.findRoot(e.i) != s.findRoot(e.j)) {
      s.unite(e.i, e.j);
      selected.append(&e - edges.p);
      cost += e.w;
    }
  }
  return {cost, selected};
}
