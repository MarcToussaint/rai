/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "proxy.h"
#include "kin.h"
#include "frame.h"
#include "../Gui/opengl.h"
#include "../Geo/i_coal.h"

//===========================================================================
//
// Proxy
//

void rai::Proxy::calc_coll(const Array<Frame*>& frames) {
  rai::Frame* f1 = frames.elem(A);
  rai::Frame* f2 = frames.elem(B);
  CHECK(f1 && f2, "ill-defined proxies!");

  double r1=0., r2=0.;
  arr m1=zeros(1,3), m2=zeros(1,3);
  if(f1->shape){  m1.referTo( f1->shape->sscCore() );  r1=f1->shape->coll_cvxRadius;  }
  if(f2->shape){  m2.referTo( f2->shape->sscCore() );  r2=f2->shape->coll_cvxRadius;  }

  if(collision) collision.reset();
  if(f2->shape && f2->shape->_type==rai::ST_pointCloud){
    CHECK_EQ(m1.d0, 1, "collision against PCL only work for points (=spheres)");
    Mesh* m2isPCL = f2->shape->_mesh.get();
    r2=0.;
    collision = make_shared<rai::PairCollision_PtPcl>(m1, m2isPCL->ensure_ann(), f1->ensure_X(), f2->ensure_X(), r1, r2);
  }else if(f2->shape && f2->shape->_mesh && f2->shape->_mesh->cvxParts.N){
    collision = make_shared<rai::PairCollision_CvxDecomp>(m1, *f2->shape->_mesh, f1->ensure_X(), f2->ensure_X(), r1, r2);
  }else{
    collision = make_shared<PairCollision_CvxCvx>(m1, m2, f1->ensure_X(), f2->ensure_X(), r1, r2);
    CHECK(f1->shape && f2->shape, "")
    // collision = make_shared<PairCollision_Coal>(f1->shape.get(), f2->shape.get(), f1->ensure_X(), f2->ensure_X(), r1, r2);
  }

  d = collision->distance-collision->rad1-collision->rad2;
  if(collision->p1.N){
    normal = collision->normal;
    posA = collision->p1;
    posB = collision->p2;
    if(collision->rad1>0.) posA -= collision->rad1*normal;
    if(collision->rad2>0.) posB += collision->rad2*normal;
  }
}

typedef rai::Array<rai::Proxy*> ProxyL;

void rai::Proxy::write(std::ostream& os, bool brief) const {
  os <<" [" <<A <<',' <<B <<"] \td=" <<d;
  if(!brief)
    os <<" |A-B|=" <<(posB-posA).length()
       //        <<" d^2=" <<(posB-posA).lengthSqr()
       <<" v=" <<(posB-posA)
       <<" normal=" <<normal
       <<" posA=" <<posA
       <<" posB=" <<posB;
}

