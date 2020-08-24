/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "filter.h"
#include "../Control/taskControl.h"
#include "../Kin/frame.h"
#include "../Kin/TM_default.h"
#include <unordered_set>

Filter::Filter()
  : Thread("Filter", -1.),
    percepts_input(this, true), //listens!!
    percepts_filtered(this),
    modelWorld(this) {
  threadOpen();
}

Filter::~Filter() {
  threadClose();
}

void Filter::open() {
  Var<rai::Configuration> modelWorld(this, "modelWorld");
  modelWorld.readAccess();
  for(rai::Frame* b:modelWorld().frames) {
    if(b->ats["percept"]) {
      //first check if it already is in the percept list
      bool done=false;
      for(const PerceptPtr& p:percepts_filtered.get()()) if(p->bodyId==(int)b->ID) done=true;
      if(!done) {
        LOG(0) <<"ADDING this body " <<b->name <<" to the percept database, which ats:" <<endl;
        LOG(0) <<*b <<"--" <<b->ats <<endl;
        rai::Shape* s=b->shape;
        switch(s->type()) {
          case rai::ST_box: {
            PerceptPtr p = make_shared<PercBox>(b->ensure_X(), s->size(), s->mesh().C);
            p->id = nextId++;
            p->bodyId = b->ID;
            percepts_filtered.set()->append(p);
          } break;
          default: NIY
        }
      }
    }
  }
  modelWorld.deAccess();
}

void Filter::step() {

//  createNewPercepts = true;
//  verbose = 1;

  if(false) { //just copy!
    percepts_filtered.set() = percepts_input.get();
    return;
  }

  percepts_input.writeAccess();
  percepts_filtered.writeAccess();

  if(verbose>0) cout <<"FILTER: #inputs=" <<percepts_input().N <<" #database=" <<percepts_filtered().N <<endl;
  if(verbose>1) {
    cout <<"INPUTS:" <<endl;
    for(PerceptPtr& p:percepts_input()) cout <<(*p) <<endl;
    cout <<"DATABASE:" <<endl;
    for(PerceptPtr& p:percepts_filtered()) cout <<(*p) <<endl;
  }

  // If empty inputs, do nothing.
  if(percepts_input().N == 0 && percepts_input().N == 0) {
    percepts_filtered.deAccess();
    percepts_input.deAccess();
    return;
  }

  PerceptL newCreations;

  //-- step 1: discount precision of old percepts
  // in forward models, the variance of two Gaussians is ADDED -> precision is 1/variance
  for(PerceptPtr& p:percepts_filtered()) p->precision = 1./(1./p->precision + 1./precision_transition);

  //-- step 2: compute matches within types and fuse
  // For each type of inputs, run the matching
  for(int t=0; t<=Percept::Type::PT_end; t++) {
    Percept::Type type = Percept::Type(t);
    //collect input and database percepts of given type
    PerceptL input_ofType, database_ofType;
    for(PerceptPtr& p : percepts_input())     if(p->type==type)  input_ofType.append(p);
    for(PerceptPtr& p : percepts_filtered())  if(p->type==type)  database_ofType.append(p);

    //no percepts at all of this type..
    if(!input_ofType.N && !database_ofType.N) continue;

    //create bipartite costs
    costs = createCostMatrix(input_ofType, database_ofType);

    //run Hungarian algorithm
    Hungarian ha(costs);

#if 0
    // Now we have the optimal matching. Assign values.
    PerceptL assignedObjects = assign(input_ofType, database_ofType, ha);
    filtered.append(assignedObjects);
#else
    for(uint i=0; i<input_ofType.N; i++) {
      PerceptPtr& perc = input_ofType(i);
      int j=ha.getMatch_row(i);
      if(j>=(int)database_ofType.N) j=-1;
      if(j==-1) { //nothing to merge with
        if(createNewPercepts) { //add this percept as a new object to the database
          perc->id = nextId++;
          perc->precision = 1.;
          newCreations.append(perc);
        } else {
          perc->id = 0; //will be deleted
        }
      } else {
        PerceptPtr& obj = database_ofType(j);
        perc->id = obj->id;
        obj->fuse(perc);
      }
    }
#endif
  }

  //-- step 3: remove all database objects with too low precision and no body match; and append new creations
  for(uint i=percepts_filtered().N; i--;) {
    PerceptPtr& p = percepts_filtered()(i);
    if(p->precision<precision_threshold && p->bodyId>=0) {
      percepts_filtered().remove(i);
    }
  }
  percepts_filtered().append(newCreations);
  newCreations.clear();

  //-- step 4: clean up remaining percepts
  percepts_input().clear();

#if 0
  //-- delete objects in percepts and database that are not filtered
  for(Percept* p : percepts_input()) {
    if(!filtered.contains(p))
      delete p;
  }
  percepts_input().clear();

  for(Percept* p : percepts_filtered()) {
    if(!filtered.contains(p))
      delete p;
  }
  percepts_filtered().clear();
  percepts_filtered() = filtered;
#endif

  //-- step 5: sync with modelWorld using inverse kinematics
  modelWorld.writeAccess();
  modelWorld->selectJointsByName({"S1"});

  TaskControlMethods taskController(rai::getParameter<double>("Hrate", .1)*modelWorld->getHmetric());
  CtrlObjectiveL tasks;
  arr q=modelWorld().q;

  // create task costs on the modelWorld for each percept
  for(PerceptPtr& p:percepts_filtered()) {
    if(p->bodyId>=0) {
      rai::Frame* b = modelWorld->frames(p->bodyId);
      if(p->type==Percept::PT_box) {
        rai::Shape* s=b->shape;
//        s->size() = (static_cast<PercBox*>(p))->size;
//        s->mesh().setSSBox(s->size(0), s->size(1), s->size(2), 0.0001);
        s->mesh().C = std::dynamic_pointer_cast<PercBox>(p)->color;
        if(s->mesh().C.d0 > 3)
          s->mesh().C=ARR(0., 0., 0.);
      }

      CtrlObjective* t;

      t = new CtrlObjective(STRING("syncPos_" <<b->name), make_shared<TM_Default>(TMT_pos, b->ID));
      t->ref = make_shared<CtrlTargetFeed_Const>(p->pose.pos.getArr());
      tasks.append(t);

      t = new CtrlObjective(STRING("syncQuat_" <<b->name), make_shared<TM_Default>(TMT_quat, b->ID));
      t->ref = make_shared<CtrlTargetFeed_Const>(p->pose.rot.getArr4d(), true);
      tasks.append(t);
    }
  }

  double cost=0.;
  for(CtrlObjective* t: tasks) t->update(.0, modelWorld()); //computes their values and Jacobians
  arr dq = taskController.inverseKinematics(tasks, NoArr, NoArr, NoArr, &cost);
  q += dq;

  if(verbose>0) {
    LOG(0) <<"FILTER: IK cost=" <<cost <<" perc q vector = " <<q <<endl;
    taskController.reportCurrentState(tasks);
  }

  listDelete(tasks); //cleanup tasks

  modelWorld->setJointState(q);
  modelWorld->selectJointsByName({}, true);
  modelWorld.deAccess();

  //-- done

  if(verbose>1) {
    cout <<"AFTER FILTER: DATABASE:" <<endl;
    for(PerceptPtr& p:percepts_filtered()) cout <<(*p) <<endl;
  }

  percepts_filtered.deAccess();
  percepts_input.deAccess();
}

PerceptL Filter::assign(const PerceptL& inputs, const PerceptL& database, const Hungarian& ha) {
  PerceptL new_database;

  uint num_old = database.N;
  uint num_new = inputs.N;

  for(uint i = 0; i < ha.starred.dim(0); ++i) {   //index over inputs
    uint col = ha.starred[i]().argmax();        //index over database
    // 3 cases:
    // 1) Existed before, no longer exists. If i > num_new
    // 2) Existed before and still exists. If costs < distannce_threshold
    // 3) Didn't exist before. This happens iff col >= num_old

    // Existed before, doesn't exist now.
    if(i >= num_new) {
      //std::cout<< "Existed before, doesn't now." << std::endl;
      PerceptPtr& old_obj = database(col);
      old_obj->precision *= relevance_decay_factor;
      if(old_obj->precision > precision_threshold)
        new_database.append(old_obj);
      //otherwise the object is lost/discarded
    } else {
      if((col < num_old) && (costs(i, col) < distance_threshold)) {      // Existed before
        //std::cout<< "Existed before, does now" << std::endl;
        PerceptPtr& new_obj = inputs(i);
        PerceptPtr& old_obj = database(col);
        if(new_obj->type != Percept::Type::PT_alvar)
          new_obj->id = database(col)->id;
        old_obj->fuse(new_obj);
        new_database.append(old_obj);
      } else { // This didn't exist before. Add it in
        //std::cout<< "Didn't exist before, or not close enough." << std::endl;
        PerceptPtr& new_obj = inputs(i);
        if(new_obj->type != Percept::Type::PT_alvar) {
          new_obj->id = nextId;
          nextId++;
        }
        new_database.append(new_obj);
        //std::cout << "Didn't exist before. Col >= num_old: " << col << ' ' << num_old << std::endl;
      }
    }
    //std::cout << "Assigning \t" << i << "\tMatches:\t" << matched_ids(i).id << "\t Relevance: " << perceps(i).relevance << std::endl;
  }

  //mt: this is already done in case 1) above!
//  // For each of the old objects, update the relevance factor.
//  for ( uint i = 0; i < database.N; ++i ) {
//    if ( matched_ids.find(database(i)->id) == matched_ids.end() ) {
//      Percept *new_obj = database(i);
//      new_obj->relevance *= relevance_decay_factor;
//      new_database.append(new_obj);
//    }
//  }

  return new_database;
}

arr Filter::createCostMatrix(const PerceptL& inputs, const PerceptL& database) {
  // First, make a padded out matrix to ensure it is bipartite.
  uint num_new = inputs.N;
  uint num_old = database.N;
  uint dims = std::max(num_old, num_new);
  arr costs = -ones(dims, dims);

  // Assign costs
  for(uint i=0; i<num_new; ++i) for(uint j=0; j<num_old; ++j) {
      costs(i, j) = inputs(i)->idMatchingCost(*database(j));
    }

  // For every element that hasn't been set, set the costs to the max.
  double max_costs = costs.max();

  for(uint i=0; i<dims; ++i) for(uint j=0; j<dims; ++j) {
      if((i >= num_new) || (j >= num_old))
        costs(i, j) = max_costs;
    }
  return costs;
}
