/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#include "effectivePoseProblem.h"
#include <Gui/opengl.h>
#include <Kin/taskMaps.h>
#include <Optim/constrained.h>

//===========================================================================

EffectivePoseProblem::EffectivePoseProblem(mlr::KinematicWorld& effKinematics_before,
                                           const Graph& KB, const Graph& symbolicState_before, const Graph& symbolicState_after,
                                           int verbose)
  : effKinematics(effKinematics_before),
    KB(KB),
    symbolicState_before(symbolicState_before),
    symbolicState_after(symbolicState_after),
    verbose(verbose){

  CHECK(symbolicState_before.isNodeOfGraph && &symbolicState_before.isNodeOfGraph->container==&KB,"");
  CHECK(symbolicState_after.isNodeOfGraph && &symbolicState_after.isNodeOfGraph->container==&KB,"");

  /* LATER COMMENT: This is to create the effective kinematics, which is now more standardized with the ICRA'16 LGP submission
   */

  // ConstrainedProblem::operator=(
  //       [this](arr& phi, arr& J, arr& H, ObjectiveTypeA& tt, const arr& x) -> void {
  //   return this -> phi(phi, J, H, tt, x);
  // } );

//  Node *glueSymbol  = KB["glued"];
//  for(Node *s:glueSymbol->parentOf) if(&s->container==&symbolicState_before){
//    //-- create a joint between the object and the target
//    mlr::Shape *ref1 = effKinematics.getShapeByName(s->parents(1)->keys.last());
//    mlr::Shape *ref2 = effKinematics.getShapeByName(s->parents(2)->keys.last());

  Node *glueSymbol  = KB["glued"];
  for(Node *s:glueSymbol->parentOf) if(&s->container==&symbolicState_before){
    //-- create a joint between the object and the target
    mlr::Shape *ref1 = effKinematics.getShapeByName(s->parents(1)->keys.last());
    mlr::Shape *ref2 = effKinematics.getShapeByName(s->parents(2)->keys.last());

    //TODO: this may generate multiple joints! CHECK IF IT EXISTS ALREADY
    mlr::Joint *j = new mlr::Joint(effKinematics, ref1->body, ref2->body);
    j->type = mlr::JT_free;
    j->Q.setDifference(ref1->body->X, ref2->body->X);
  }

  Node *supportSymbol  = KB["Gsupport"];
  for(Node *s:supportSymbol->parentOf) if(&s->container==&symbolicState_after){
    //-- create a joint between the object and the target
    mlr::Shape *ref2 = effKinematics.getShapeByName(s->parents(1)->keys.last());
    mlr::Shape *ref1 = effKinematics.getShapeByName(s->parents(2)->keys.last());

    if(!ref2->body->inLinks.N){ //object does not yet have a support -> add one; otherwise NOT!
      mlr::Joint *j = new mlr::Joint(effKinematics, ref1->body, ref2->body);
      j->type = mlr::JT_transXYPhi;
      j->A.addRelativeTranslation(0, 0, .5*ref1->size(2));
      j->B.addRelativeTranslation(0, 0, .5*ref2->size(2));
      j->Q.addRelativeTranslation(rnd.uni(-.1,.1), rnd.uni(-.1,.1), 0.);
      j->Q.addRelativeRotationDeg(rnd.uni(-180,180), 0, 0, 1);
    }
  }


  effKinematics.topSort();
  effKinematics.checkConsistency();
  effKinematics.getJointState(x0);
}

void EffectivePoseProblem::phi(arr& phi, arr& phiJ, arr& H, ObjectiveTypeA& tt, const arr& x){
  effKinematics.setJointState(x);
  if(verbose>1) effKinematics.gl().timedupdate(.1);
  if(verbose>2) effKinematics.gl().watch();

  phi.clear();
  if(&phiJ) phiJ.clear();
  if(&H) H.clear();
  if(&tt) tt.clear();

  arr y,J;

  //-- regularization
  double prec=1e+0;
  phi.append(prec*(x-x0));
  if(&phiJ) phiJ.append(prec*eye(x.N));
  if(&tt) tt.append(OT_sumOfSqr, x.N);

  //-- touch symbols -> constraints of being inside!
  //LATER: This is not yet transferred to the new LGP!
  Node *touch=symbolicState_after["touch"];
  for(Node *constraint:touch->parentOf) if(&constraint->container==&symbolicState_after){
    mlr::Shape *s1=effKinematics.getShapeByName(constraint->parents(1)->keys(0));
    mlr::Shape *s2=effKinematics.getShapeByName(constraint->parents(2)->keys(0));

    TM_GJK gjk(s1, s2, true);

    gjk.phi(y, (&phiJ?J:NoArr), effKinematics);
    phi.append(y);
    if(&phiJ) phiJ.append(J);
    if(&tt) tt.append(OT_eq, y.N);
  }

  //-- support symbols -> constraints of being inside!
  //LATER: This is is now done by the TM_AboveBox (as used in place)
  Node *support=symbolicState_after["Gsupport"];
  for(Node *constraint:support->parentOf) if(&constraint->container==&symbolicState_after){
    mlr::Body *b1=effKinematics.getBodyByName(constraint->parents(1)->keys.last());
    mlr::Body *b2=effKinematics.getBodyByName(constraint->parents(2)->keys.last());
    if(b2->shapes(0)->type==mlr::ST_cylinder){
      mlr::Body *z=b1;
      b1=b2; b2=z;
    }//b2 should be the board
    arr y,J;
    effKinematics.kinematicsRelPos(y, J, b1, NoVector, b2, NoVector);
    arr range(3);
    double d1 = .5*b1->shapes(0)->size(0) + b1->shapes(0)->size(3);
    double d2 = .5*b2->shapes(0)->size(0) + b2->shapes(0)->size(3);
    range(0) = fabs(d1 - d2);
    d1 = .5*b1->shapes(0)->size(1) + b1->shapes(0)->size(3);
    d2 = .5*b2->shapes(0)->size(1) + b2->shapes(0)->size(3);
    range(1) = fabs(d1 - d2);
    range(2)=0.;
    if(verbose>2) cout <<y <<range
                      <<y-range <<-y-range
                     <<"\n 10=" <<b1->shapes(0)->size(0)
                    <<" 20=" <<b2->shapes(0)->size(0)
                   <<" 11=" <<b1->shapes(0)->size(1)
                  <<" 21=" <<b2->shapes(0)->size(1)
                 <<endl;
    prec = 1e1;
    phi.append(prec*(  y(0) - range(0) ));
    phi.append(prec*( -y(0) - range(0) ));
    phi.append(prec*(  y(1) - range(1) ));
    phi.append(prec*( -y(1) - range(1) ));
    if(&phiJ){
      phiJ.append(prec*( J[0]));
      phiJ.append(prec*(-J[0]));
      phiJ.append(prec*( J[1]));
      phiJ.append(prec*(-J[1]));
    }
    if(&tt) tt.append(OT_ineq, 4);
  }

  //-- supporters below object -> maximize their distances to center
  //LATER: TODO:  we'd need to transfer this when multiple supporters in a tower - not done yet!
  NodeL objs=symbolicState_after.getNodes("Object");
  for(Node *obj:objs){
    NodeL supporters;
    for(Node *constraint:obj->parentOf){
      if(constraint->parents.N==3 && constraint->parents(0)==support && constraint->parents(2)==obj){
        supporters.append(constraint->parents(1));
      }
    }
    if(supporters.N>=2){
      if(verbose>1){ cout <<"Adding cost term Object" <<*obj <<" above "; listWrite(supporters, cout); cout <<endl; }
      //-- compute center
      uint n=effKinematics.getJointStateDimension();
      arr cen(3),cenJ(3,n);  cen.setZero(); cenJ.setZero();
      mlr::Body *b;
      arr y,J;
      for(Node *s:supporters){
        b=effKinematics.getBodyByName(s->keys.last());
        effKinematics.kinematicsPos(y, J, b);
        cen += y;
        cenJ += J;
      }
      cen  /= (double)supporters.N;
      cenJ /= (double)supporters.N;

      //-- max distances to center
      prec=3e-1;
      for(Node *s:supporters){
        b=effKinematics.getBodyByName(s->keys.last());
        effKinematics.kinematicsPos(y, J, b);
        y -= cen;
        double d = length(y);
        arr normal = y/d;
        phi.append( prec*(1.-d) );
        if(&phiJ) phiJ.append( prec*(~normal*(-J+cenJ)) );
        if(&tt) tt.append(OT_sumOfSqr, 1);
      }

      //-- align center with object center
      prec=1e-1;
      b=effKinematics.getBodyByName(obj->keys.last());
      effKinematics.kinematicsPos(y, J, b);
      phi.append( prec*(y-cen) );
      if(&phiJ) phiJ.append( prec*(J-cenJ) );
      if(&tt) tt.append(OT_sumOfSqr, 3);
    }

    prec=1e-0;
    //TODO: ALIGN transfer!
    if(supporters.N==1){ // just one-on-one: align
      arr y1,J1,y2,J2;
      mlr::Body *b1=effKinematics.getBodyByName(obj->keys.last());
      mlr::Body *b2=effKinematics.getBodyByName(supporters(0)->keys.last());
      if(b1->shapes(0)->type==mlr::ST_box){
        if(verbose>1){ cout <<"Adding cost term Object" <<*obj <<" below "; listWrite(supporters, cout); cout <<endl; }
        effKinematics.kinematicsPos(y1, J1, b1);
        effKinematics.kinematicsPos(y2, J2, b2);
        phi.append( prec*(y1-y2) );
        if(&phiJ) phiJ.append( prec*(J1-J2) );
        if(&tt) tt.append(OT_sumOfSqr, 3);
      }
    }
  }

  //-- supporters above object
  for(Node *obj:objs){
    NodeL supporters;
    for(Node *constraint:obj->parentOf){
      if(constraint->parents.N==3 && constraint->parents(0)==support && constraint->parents(1)==obj){
        supporters.append(constraint->parents(2));
      }
    }

    if(supporters.N>=2){
      if(verbose>1){ cout <<"Adding cost term Object" <<*obj <<" below "; listWrite(supporters, cout); cout <<endl; }
      //-- compute center
      uint n=effKinematics.getJointStateDimension();
      arr cen(3),cenJ(3,n);  cen.setZero(); cenJ.setZero();
      mlr::Body *b;
      arr y,J;
      for(Node *s:supporters){
        b=effKinematics.getBodyByName(s->keys.last());
        effKinematics.kinematicsPos(y, J, b);
        cen += y;
        cenJ += J;
      }
      cen  /= (double)supporters.N;
      cenJ /= (double)supporters.N;

      //-- max distances to center
      prec=1e-1;
      for(Node *s:supporters){
        b=effKinematics.getBodyByName(s->keys.last());
        effKinematics.kinematicsPos(y, J, b);
        y -= cen;
        double d = length(y);
        arr normal = y/d;
        phi.append( prec*(1.-d) );
        if(&phiJ) phiJ.append( prec*(~normal*(-J+cenJ)) );
        if(&tt) tt.append(OT_sumOfSqr, 1);
      }

      //-- align center with object center
      prec=1e-0;
      b=effKinematics.getBodyByName(obj->keys.last());
      effKinematics.kinematicsPos(y, J, b);
      phi.append( prec*(y-cen) );
      if(&phiJ) phiJ.append( prec*(J-cenJ) );
      if(&tt) tt.append(OT_sumOfSqr, 3);
    }

    prec=1e-0;
    if(supporters.N==1){ // just one-on-one: align
      arr y1,J1,y2,J2;
      mlr::Body *b1=effKinematics.getBodyByName(obj->keys.last());
      mlr::Body *b2=effKinematics.getBodyByName(supporters(0)->keys.last());
      if(b1->shapes(0)->type==mlr::ST_box){
        if(verbose>1){ cout <<"Adding cost term Object" <<*obj <<" below "; listWrite(supporters, cout); cout <<endl; }
        effKinematics.kinematicsPos(y1, J1, b1);
        effKinematics.kinematicsPos(y2, J2, b2);
        phi.append( prec*(y1-y2) );
        if(&phiJ) phiJ.append( prec*(J1-J2) );
        if(&tt) tt.append(OT_sumOfSqr, 3);
      }
    }
  }

  if(&phiJ) phiJ.reshape(phi.N, x.N);
}

//===========================================================================

double EffectivePoseProblem::optimize(arr& x){
  x = effKinematics.getJointState();
  rndGauss(x, .1, true);

//  checkJacobianCP(*this, x, 1e-4);
  OptConstrained opt(x, NoArr, *this, OPT(verbose=2));
  opt.run();
  //  checkJacobianCP(f, x, 1e-4);
  effKinematics.setJointState(x);
  return opt.L.get_costs();
}

//===========================================================================
