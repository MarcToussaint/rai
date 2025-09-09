#include "NLP_Descriptor.h"
// #include "../KOMO/komo_NLP.h"
#include "../KOMO/objective.h"
#include "../Kin/feature.h"

rai::Graph getDescriptor(KOMO& komo, int verbose){
  if(verbose>2){
    LOG(0) <<"=== Descriptor for following KOMO problem:";
    cout <<komo.report(true, true, false) <<endl;
  }
  rai::Graph G;
  for(shared_ptr<Objective>& ob:komo.objectives) {
    if(ob->feat->frameIDs.N<=2 && ob->feat->order<=1){
      for(GroundedObjective* go:ob->groundings){
        int timeSlice = go->timeSlices.elem(-1);

        if(verbose>1) cout <<"\n** objective " <<*ob <<' ' <<timeSlice;

	str o_name; o_name <<go->feat->typeString() <<'_' <<go->type <<go->feat->order;
	if(go->feat->target.N) o_name <<'_' <<go->feat->target;
	if(verbose>1) cout <<"\n  feature:" <<o_name;
	rai::Node* o_node = G.add(o_name, true);

	for(rai::Frame *f: go->frames){
	  str f_name; f_name <<f->name <<'_' <<f->time;
	  if(verbose>1) cout <<"\n  frame:" <<f_name;
	  rai::Node* f_node = G.findNode(f_name);
	  if(!f_node) f_node = G.add(f_name, true);
	  o_node->addParent(f_node);
	}
	if(verbose>1) cout <<endl;
      }
    }
  }

  if(verbose>2){
    LOG(0) <<"=== done";
  }

  return G;
}
