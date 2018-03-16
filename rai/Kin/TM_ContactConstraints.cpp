/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "TM_ContactConstraints.h"
#include "TM_PairCollision.h"
#include "frame.h"
#include "contact.h"

void TM_ContactConstraints::phi(arr &y, arr &J, const mlr::KinematicWorld &K, int t){
  uint C=0;
  arr yc, Jc;
  y.clear();
  if(&J) J.clear();
  for(mlr::Frame *f:K.frames) if(f->contacts.N) for(mlr::Contact *c:f->contacts) if(&c->a==f){
//    TaskMap *map = c->getTM_ContactNegDistance();
    TaskMap *map = new TM_PairCollision(c->a.ID, c->b.ID, true);
    map->phi(yc, (&J?Jc:NoArr), K, t);
    c->y = yc.scalar();
    y.append( c->y );
    if(&J) J.append( Jc );
    C++;
    delete map;
  }
//  if(!C){
//    y.append(-1.);
//    if(&J) J.append(zeros(K.q.N));
//  }
  if(&J) J.reshape(y.N, K.q.N);
}

uint TM_ContactConstraints::dim_phi(const mlr::KinematicWorld &K){
  uint C=0;
  for(mlr::Frame *f:K.frames) if(f->contacts.N) for(mlr::Contact *c:f->contacts) if(&c->a==f){
    C++;
  }
//  if(!C){
//    C=1;
//  }
  return C;
}
