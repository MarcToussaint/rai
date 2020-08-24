/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "towerProblem.h"
#include "LGP.h"

void TowerProblem::setRandom() {
  symbols.checkConsistency();
  Node* CYLIN = symbols["Cylin"];
  Node* BOARD = symbols["Board"];
  Node* DEPTH = symbols["depth"];
  Graph& state = symbols["STATE"]->graph();

  uint n = 10+rnd(20);
  double x=-1.6, y=-1.;
  for(uint i=0; i<n; i++) {
    //add an object to the geometry
    rai::Body* b = new rai::Body(world);
    rai::Shape* s = new rai::Shape(world, *b);
    s->cont=true;
    b->X.addRelativeTranslation(x, y, .62);
    //randomize type and size
    if(rnd.uni()<.6) {
      s->type = rai::ST_cylinder;
      s->size(0)=s->size(1)=0.;
      s->size(2)=.2;
      s->size(3)=.05;
      s->name <<"cyl_" <<i;
    } else {
      s->type = rai::ST_box;
      s->size(0)=.1 + .3*rnd.uni();
      s->size(1)=.1 + .6*rnd.uni();
      s->size(2)=.02;
      s->size(3)=0.;
      s->name <<"boa_" <<i;
    }
    b->name = s->name;
    //position on grid
    b->X.addRelativeTranslation(0, .5*s->size(1), .5*s->size(2));
    y += .1 + s->size(1)+s->size(3);
    if(y>1.) { x+=.4; y=-1.; }

    //add symbols
    Node* o = symbols.newNode<bool>({"Object", s->name}, {}, true);
    if(s->type==rai::ST_cylinder) {
      state.newNode<bool>({}, {CYLIN, o}, true);
    } else {
      state.newNode<bool>({}, {BOARD, o}, true);
    }
    state.newNode<double>({}, {DEPTH, o}, 0.);
  }

  symbols.checkConsistency();

  //HACK: move the actionSequence item to the end...
  Node* ss = symbols["STATE"];
  symbols.NodeL::append(ss);
  symbols.NodeL::remove(ss->index);
  symbols.index();

  Node* as = symbols["actionSequence"];
  symbols.NodeL::append(as);
  symbols.NodeL::remove(as->index);
  symbols.index();

  world.calc_fwdPropagateShapeFrames();
}

double TowerProblem::reward(const rai::Configuration& world, const Graph& symbols) {
  //-- find max depth
  double depth=0.;
  Node* depthSymbol=symbols["depth"];
  Graph& state =symbols["STATE"]->graph();

  for(Node* dep:depthSymbol->children) if(&dep->container==&state) {
      double d = dep->get<double>();
      if(d>depth) depth=d;
    }

  //-- count supports below
  double supp=0.;
  Node* supportSymbol=symbols["supports"];
  NodeL objs=symbols.getNodes("Object");
  for(Node* obj:objs) {
    NodeL supporters;
    for(Node* constraint:obj->children) {
      if(constraint->parents.N==3 && constraint->parents(0)==supportSymbol && constraint->parents(2)==obj) {
        supporters.append(constraint->parents(1));
      }
    }
    supp += .2 * (supporters.N * supporters.N);
  }

  return 10.*depth + supp;
}

void TowerProblem_new::setRandom() {
  fol_root.KB.checkConsistency();
  Node* CYLIN = fol_root.KB["Cylin"];
  Node* BOARD = fol_root.KB["Board"];
  Node* OBJECT = fol_root.KB["Object"];
  Graph& state = *fol_root.state;

  uint n = 2; //10+rnd(20);
  double x=-1.6, y=-1.;
  for(uint i=0; i<n; i++) {
    //add an object to the geometry
    rai::Body* b = new rai::Body(world_root);
    rai::Shape* s = new rai::Shape(world_root, *b);
    s->cont=true;
    b->X.addRelativeTranslation(x, y, .62);
    //randomize type and size
    if(rnd.uni()<.6) {
      s->type = rai::ST_ssBox;
      s->size(0)=s->size(1)=0.;
      s->size(2)=.2;
      s->size(3)=.05;
      s->name <<"cyl_" <<i;
    } else {
      s->type = rai::ST_ssBox;
      s->size(0)=.1 + .3*rnd.uni();
      s->size(1)=.1 + .6*rnd.uni();
      s->size(2)=.02;
      s->size(3)=.01;
      s->name <<"boa_" <<i;
    }
    s->sscCore.setBox();
    s->sscCore.scale(s->size(0), s->size(1), s->size(2));
    s->mesh.setSSCvx(s->sscCore, s->size(3));
    s->mesh_radius = s->mesh.getRadius();
    b->name = s->name;
    //position on grid
    b->X.addRelativeTranslation(0, .5*s->size(1), .5*s->size(2));
    y += .1 + s->size(1)+s->size(3);
    if(y>1.) { x+=.4; y=-1.; }

    //add symbols
    Node* o = fol_root.KB.newNode<bool>({s->name}, {}, true);
    //add predicates
    state.newNode<bool>({}, {OBJECT, o}, true);
    if(!s->size(0)) {
      state.newNode<bool>({}, {CYLIN, o}, true);
    } else {
      state.newNode<bool>({}, {BOARD, o}, true);
    }
  }

  fol_root.KB.checkConsistency();

  world_root.calc_fwdPropagateShapeFrames();
}
