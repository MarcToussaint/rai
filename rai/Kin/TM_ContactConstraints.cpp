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

void TM_ContactConstraints::phi(arr &y, arr &J, const rai::KinematicWorld &K) {
  y.clear();
  if(&J) J.clear();
  for(rai::Frame *f:K.frames) if(f->contacts.N) for(rai::Contact *con:f->contacts) if(&con->a==f) {

    if(!con->soft){
      //-- needs to touch!!
      arr d, Jd;
      TM_PairCollision dist(con->a.ID, con->b.ID, TM_PairCollision::_negScalar, false);
      dist.phi(d, (&J?Jd:NoArr), K);
      con->y = d.scalar();
      con->setFromPairCollision(*dist.coll);

      y.append(con->y);
      if(&J) J.append(Jd);
    }else{
      arr d, Jd;
      TM_PairCollision dist(con->a.ID, con->b.ID, TM_PairCollision::_negScalar, false);
      dist.phi(d, (&J?Jd:NoArr), K);
      con->y = d.scalar();
      con->setFromPairCollision(*dist.coll);

      //soft! complementarity
      double s = 1e-1;

      //get force
      arr ferr, Jferr;
      K.kinematicsForce(ferr, Jferr, con);

      y.append(s*d.scalar() * ferr);
      if(&J) J.append( (s*d.scalar())*Jferr + (s*ferr) * Jd );

//      y.append(ferr);
//      if(&J) J.append(Jferr);
    }

    //-- non-aligned force
    //get collision normal
    arr c, Jc;
    TM_PairCollision cvec(con->a.ID, con->b.ID, TM_PairCollision::_normal, true);
    cvec.phi(c, (&J?Jc:NoArr), K);
    if(length(c)<1e-6) continue;
    //get force
    arr ferr, Jferr;
    K.kinematicsForce(ferr, Jferr, con);
    //subtract c-aligned projection
    if(&J) Jferr -= (c*~c*Jferr + c*~ferr*Jc + scalarProduct(c,ferr)*Jc);
    ferr -= c*scalarProduct(c,ferr);

    y.append(ferr);
    if(&J) J.append(Jferr);
  }
  if(&J) J.reshape(y.N, K.q.N);
}

uint TM_ContactConstraints::dim_phi(const rai::KinematicWorld &K) {
  uint C=0;
  for(rai::Frame *f:K.frames) if(f->contacts.N) for(rai::Contact *c:f->contacts) if(&c->a==f) {
    if(!c->soft)  C += 4;
    else C += 6;
  }
  return C;
}
