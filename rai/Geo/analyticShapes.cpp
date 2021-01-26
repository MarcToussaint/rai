#include "analyticShapes.h"

DistanceFunction_Sphere::DistanceFunction_Sphere(const rai::Transformation& _pose, double _r):pose(_pose), r(_r) {
  ScalarFunction::operator=([this](arr& g, arr& H, const arr& x)->double{ return f(g, H, x); });
}

double DistanceFunction_Sphere::f(arr& g, arr& H, const arr& x) {
  arr d = x-conv_vec2arr(pose.pos);
  double len = length(d);
  if(!!g) g = d/len;
  if(!!H) H = 1./len * (eye(3) - (d^d)/(len*len));
  return len-r;
}

//===========================================================================

//double DistanceFunction_InfCylinder::fs(arr& g, arr& H, const arr& x){
//  z = z / length(z);
//  arr a = (x-c) - scalarProduct((x-c), z) * z;
//  arr I(x.d0,x.d0);
//  uint i;
//  double na = length(a);

//  if(!!g) g = s*a/na;
//  if(!!H){
//    I.setZero();
//    for(i=0;i<x.d0;++i) I(i,i)=1;
//    H = s/na * (I - z*(~z) - 1/(na*na) * a*(~a));
//  }
//  return s*(na-r);
//}

//===========================================================================

DistanceFunction_Cylinder::DistanceFunction_Cylinder(const rai::Transformation& _pose, double _size_z, double _r):pose(_pose), size_z(_size_z), r(_r) {
  ScalarFunction::operator=([this](arr& g, arr& H, const arr& x)->double{ return f(g, H, x); });
}

double DistanceFunction_Cylinder::f(arr& g, arr& H, const arr& x) {
  arr z = conv_vec2arr(pose.rot.getZ());
  arr c = conv_vec2arr(pose.pos);
  arr b = scalarProduct(x-c, z) * z;
  arr a = (x-c) - b;
  arr I(3, 3);
  double la = length(a);
  double lb = length(b);
  arr aaTovasq = 1/(la*la) * (a^a);
  arr zzT = z^z;

  if(lb < size_z/2.) {   // x projection on z is inside cyl
    if(la<r && (size_z/2.-lb)<(r-la)) { // x is INSIDE the cyl and closer to the lid than the wall
      if(!!g) g = 1./lb*b; //z is unit: s*z*|z|*sgn(b*z) = s*b/nb
      if(!!H) { I.setZero(); H=I; }
      return lb-size_z/2.;
    } else { // closer to the side than to a lid (inc. cases in- and outside the tube, because (r-na)<0 then)
      if(!!g) g = a/la;
      if(!!H) {
        I.setId(3);
        H = 1./la * (I - zzT - aaTovasq);
      }
      return la-r;
    }
  } else { // x projection on z is outside cylinder
    if(la < r) {  // inside the infinite cylinder
      if(!!g) g = b/lb;
      if(!!H) H.resize(3, 3).setZero();
      return lb-size_z/2.;
    } else { // outside the infinite cyl
      arr v =  b/lb * (lb-size_z/2.)  + a/la * (la-r); //MT: good! (note: b/nb is the same as z) SD: well, b/nb is z or -z.
      double nv=length(v);
      if(!!g) g = v/nv;
      if(!!H) {
        I.setId(3);
        arr dvdx = (la-r)/la*(I - zzT - aaTovasq)
                   + aaTovasq + zzT;
        H = 1./nv* (dvdx - 1/nv/nv * (v^v) * (~dvdx));
      }
      return nv;
    }
  }
  HALT("You shouldn't be here!");
}

//===========================================================================

DistanceFunction_Capsule::DistanceFunction_Capsule(const rai::Transformation& _pose, double _size_z,  double _r):pose(_pose), size_z(_size_z), r(_r) {
  ScalarFunction::operator=([this](arr& g, arr& H, const arr& x)->double{ return f(g, H, x); });
}

double DistanceFunction_Capsule::f(arr& g, arr& H, const arr& x) {
  arr z = conv_vec2arr(pose.rot.getZ());
  arr c = conv_vec2arr(pose.pos);
  double zcoord = scalarProduct(x-c, z);
  arr b = zcoord * z;
  arr a = (x-c) - b;
  arr I(3, 3);
  double la = length(a);
  arr aaTovasq = 1/(la*la) * (a^a);
  arr zzT = z^z;

  if(zcoord < .5*size_z && zcoord > -.5*size_z) {   // x projection on z is inside line
    if(!!g) g = a/la;
    if(!!H) {
      I.setId(3);
      H = 1./la * (I - zzT - aaTovasq);
    }
    return la-r;
  } else { // x projection on z is outside line
    arr v;
    if(zcoord>0.) v=c+(0.5*size_z)*z;
    else  v=c-(0.5*size_z)*z;
    arr d = x-v;
    double len=length(d);
    if(!!g) g = d/len;
    if(!!H) H = 1./len * (eye(3) - (d^d)/(len*len));
    return len-r;
  }
  HALT("You shouldn't be here!");
}

//===========================================================================

/// dx, dy, dz are box-wall-coordinates: width=2*dx...; t is box transform; x is query point in world
void closestPointOnBox(arr& closest, arr& signs, const rai::Transformation& t, double dx, double dy, double dz, const arr& x) {
  arr rot = t.rot.getArr();
  arr a_rel = (~rot)*(x-conv_vec2arr(t.pos)); //point in box coordinates
  arr dim = {dx, dy, dz};
  signs.resize(3);
  signs.setZero();
  closest = a_rel;
  arr del_abs = fabs(a_rel)-dim;
  if(del_abs.max()<0.) { //inside
    uint side=del_abs.argmax(); //which side are we closest to?
    //in positive or neg direction?
    if(a_rel(side)>0) { closest(side) = dim(side);  signs(side)=+1.; }
    else             { closest(side) =-dim(side);  signs(side)=-1.; }
  } else { //outside
    for(uint side=0; side<3; side++) {
      if(closest(side)<-dim(side)) { signs(side)=-1.; closest(side)=-dim(side); }
      if(closest(side)> dim(side)) { signs(side)=+1.; closest(side)= dim(side); }
    }
  }
  closest = rot*closest + t.pos.getArr();
}

//===========================================================================

DistanceFunction_ssBox::DistanceFunction_ssBox(const rai::Transformation& _pose, double _size_x, double _size_y, double _size_z, double _r)
  : pose(_pose), size_x(_size_x), size_y(_size_y), size_z(_size_z), r(_r) {
  ScalarFunction::operator=([this](arr& g, arr& H, const arr& x)->double{ return f(g, H, x); });
}

double DistanceFunction_ssBox::f(arr& g, arr& H, const arr& x) {
  arr rot = pose.rot.getArr();
  arr x_rel = (~rot)*(x-conv_vec2arr(pose.pos)); //point in box coordinates
  arr box = {.5*size_x-r, .5*size_y-r, .5*size_z-r};

  arr closest = x_rel;
  arr del_abs = fabs(x_rel)-box;
  bool inside=true;
  //-- find closest point on box
  if(del_abs.max()<0.) { //inside
    uint side=del_abs.argmax(); //which side are we closest to?
    if(x_rel(side)>0) closest(side) = box(side);  else  closest(side)=-box(side); //in positive or neg direction?
  } else { //outside
    inside = false;
    closest = elemWiseMax(-box, closest);
    closest = elemWiseMin(box, closest);
  }

 //-- distance to closest point
  arr del = x_rel-closest;
  double d = length(del);
  if(inside) d *= -1.;
  if(!!g) g = rot*del/d; //transpose(R) rotates the gradient back to world coordinates
  if(!!H) {
    if(inside) { //inside
      H.resize(3, 3).setZero();
    } else { //outside
      if(del_abs.min()>0.) { //outside on all 3 axis
        H = 1./d * (eye(3) - (del^del)/(d*d));
      } else {
        arr edge=del_abs;
        for(double& z: edge) z=(z<0.)?0.:1.;
        if(sum(edge)<=1.1) { //closest to the plane (equals 1.)
          H.resize(3, 3).setZero();
        } else { //closest to an edge
          edge = 1.-edge;
          H = 1./d * (eye(3) - (del^del)/(d*d) - (edge^edge));
        }
      }
      H = rot*H*(~rot);
    }
  }

  return d-r;
}

ScalarFunction DistanceFunction_SSBox = [](arr& g, arr& H, const arr& x) -> double{
  // x{0,2} are box-wall-coordinates, not width!
  CHECK_EQ(x.N, 14, "query-pt + abcr + pose");
  rai::Transformation t;
  t.pos.set(x({7, 9}));
  t.rot.set(x({10, 13}));
  t.rot.normalize();
  arr closest, signs;
  closestPointOnBox(closest, signs, t, x(3), x(4), x(5), x({0, 2}));
  arr grad = x({0, 2}) - closest;
  double d = length(grad);
  grad /= d;
  d -= x(6);
  if(!!g) {
    g.resize(14);
    g.setZero();
    g({0, 2}) = grad;
    g({7, 9}) = - grad;
    g({3, 5}) = - signs%(t.rot / rai::Vector(grad)).getArr();
    g(6) = -1.;
    g({10, 13}) = ~grad*crossProduct(t.rot.getJacobian(), (x({0, 2})-t.pos.getArr()));
    g({10, 13})() /= -sqrt(sumOfSqr(x({10, 13}))); //account for the potential non-normalization of q
  }
  return d;
};
