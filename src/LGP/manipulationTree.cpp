#include "manipulationTree.h"

void ManipulationTree_Node::dump(int level){
  for(uint i=0;i<level;i++) cout <<"--";
  if(decision) cout <<' ' <<*decision <<endl;
  for(uint i=0;i<level;i++) cout <<"  ";
  cout <<' ';
  folState->write(cout, " ");
  cout <<endl;
  for(ManipulationTree_Node *n:children) n->dump(level+1);
}
