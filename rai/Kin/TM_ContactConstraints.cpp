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

    //-- needs to touch!!
    arr d, Jd;
    TM_PairCollision dist(con->a.ID, con->b.ID, true, false);
    dist.phi(d, (&J?Jd:NoArr), K);
    con->y = d.scalar();
    y.append(con->y);
    if(&J) J.append(Jd);

    con->setFromPairCollision(*dist.coll);

    //-- non-aligned force
    arr c, Jc;
    TM_PairCollision cvec(con->a.ID, con->b.ID, false, true);
    cvec.phi(c, (&J?Jc:NoArr), K);
    if(length(c)<1e-6) continue;
    normalizeWithJac(c, Jc);

    //just get force
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
    C += 4;
  }
  return C;
}
