#include "signedDistanceFunctions.h"

#include "../Gui/opengl.h"
#include "../Optim/newton.h"

#include <math.h>

//===========================================================================

arr SDF::eval(const arr& samples){
  CHECK_EQ(samples.nd, 2, "");
  CHECK_EQ(samples.d1, 3, "");
  arr y(samples.d0);
  for(uint i=0;i<y.N;i++) y.elem(i) = f(NoArr, NoArr, samples[i]);
  return y;
}

floatA SDF::evalFloat(const arr& samples){
  CHECK_EQ(samples.nd, 2, "");
  CHECK_EQ(samples.d1, 3, "");
  floatA y(samples.d0);
  for(uint i=0;i<y.N;i++) y.elem(i) = f(NoArr, NoArr, samples[i]);
  return y;
}

void SDF::viewSlice(OpenGL& gl, double z, const arr& lo, const arr& hi){
  gl.resize(505, 505);

  arr samples = ::grid({lo(0), lo(1), z}, {hi(0), hi(1), z}, {100, 100, 0});
  arr values = eval(samples);
  values.reshape(101,101);
  gl.displayRedBlue(values, false, 5.);
}

void SDF::animateSlices(const arr& lo, const arr& hi, double wait){
  OpenGL gl;
  for(double z=lo(2);z<=hi(2);z += (hi(2)-lo(2))/20.){
    viewSlice(gl, z, lo, hi);
    gl.text <<" z=" <<z;
    if(wait<0.) gl.watch();
    else{
      gl.update();
      if(wait) rai::wait(wait);
    }
  }
}

arr SDF::projectNewton(const arr& x0, double maxStep, double regularization){
  ScalarFunction distSqr = [this, &x0, regularization](arr& g, arr& H, const arr& x){
    double d = f(g, H, x);
    if(!!H) H *= 2.*d;
    if(!!H) H += 2.*(g^g);
    if(!!g) g *= 2.*d;


    double w=regularization;
    arr c = (x-x0);
    if(!!g) g += (2.*w)*c;
    if(!!H) H += (2.*w)*eye(3);

    return d*d + w*sumOfSqr(c);
  };

  arr y = x0;
//  checkGradient(distSqr, y, 1e-6);
//  checkHessian(distSqr, y, 1e-6);
//  checkGradient(*this, y, 1e-6);
//  checkHessian(*this, y, 1e-6);

  OptNewton newton(y, distSqr, rai::OptOptions()
                   .set_verbose(0)
                   .set_maxStep(maxStep)
                   .set_damping(1e-10) );
  newton.run();

  checkGradient(distSqr, y, 1e-4);
//  checkHessian(distSqr, y, 1e-6);
//  checkGradient(*this, y, 1e-6);
//  checkHessian(*this, y, 1e-6);

  return y;
}

//===========================================================================

double SDF_Sphere::f(arr& g, arr& H, const arr& x) {
  arr d = x-conv_vec2arr(pose.pos);
  double len = length(d);
  double eps=1e-10;
  if(!!g) g = d/(len+eps);
  if(!!H) H = 1./(len+eps) * (eye(3) - (g^g));
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

double SDF_Cylinder::f(arr& g, arr& H, const arr& x) {
  arr z = conv_vec2arr(pose.rot.getZ());
  arr c = conv_vec2arr(pose.pos);
  double zcoord = scalarProduct(x-c, z);
  arr b = zcoord * z;
  arr a = (x-c) - b;
  arr I(3, 3);
  double la = length(a);
  double lb = length(b);
  arr aaTovasq = 1/(la*la) * (a^a);
  arr zzT = z^z;

  if(la<1e-10){
    if(!!H) H.resize(x.N, x.N).setZero();
    if(zcoord > .5*size_z){
      if(!!g) g = z;
      return zcoord - .5*size_z;
    }else if(-zcoord > .5*size_z) {
      if(!!g) g = -z;
      return (-zcoord - .5*size_z);
    }else{
      if(!!g) g.resize(x.N).setZero();
      return -r;
    }
  }

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

double SDF_Capsule::f(arr& g, arr& H, const arr& x) {
  arr z = conv_vec2arr(pose.rot.getZ());
  arr c = conv_vec2arr(pose.pos);
  double zcoord = scalarProduct(x-c, z);
  arr b = zcoord * z;
  arr a = (x-c) - b;
  arr I(3, 3);
  double la = length(a);

  if(la<1e-10){
    if(!!H) H.resize(x.N, x.N).setZero();
    if(zcoord > .5*size_z){
      if(!!g) g = z;
      return zcoord - .5*size_z - r;
    }else if(-zcoord > .5*size_z) {
      if(!!g) g = -z;
      return (-zcoord - .5*size_z) - r;
    }else{
      if(!!g) g.resize(x.N).setZero();
      return -r;
    }
  }

  arr aaTovasq = 1/(la*la) * (a^a);
  arr zzT = z^z;

  if(zcoord <= .5*size_z && zcoord >= -.5*size_z) {   // x projection on z is inside line
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
  arr x_rel = (~rot)*(x-conv_vec2arr(t.pos)); //point in box coordinates
  arr dim = {dx, dy, dz};
  signs.resize(3);
  signs.setZero();
  closest = x_rel;
  arr del_abs = fabs(x_rel)-dim;
  if(max(del_abs)<0.) { //inside
    uint side=argmax(del_abs); //which side are we closest to?
    //in positive or neg direction?
    if(x_rel(side)>0) { closest(side) = dim(side);  signs(side)=+1.; }
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

double SDF_ssBox::f(arr& g, arr& H, const arr& x) {
  arr rot = pose.rot.getArr();
  arr x_rel = (~rot)*(x-conv_vec2arr(pose.pos)); //point in box coordinates
  arr box = .5*size;
  if(r) box -= r;

  arr closest = x_rel;
  arr del_abs = fabs(x_rel)-box;
  bool inside=true;
  //-- find closest point on box
  if(max(del_abs)<0.) { //inside
    uint side=argmax(del_abs); //which side are we closest to?
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
      if(min(del_abs)>0.) { //outside on all 3 axis
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

//===========================================================================

double SDF_ssSomething::f(arr& g, arr& H, const arr& x){
  return (*something)(g, H, x)-r;
}

//===========================================================================

double interpolate1D(double v0, double v1, double x){
  return v0*(1.-x) + v1*x;
}

double interpolate2D(double v00, double v10, double v01, double v11, double x, double y){
  double s = interpolate1D(v00, v10, x);
  double t = interpolate1D(v01, v11, x);
  return interpolate1D(s, t, y);
}

double interpolate3D(double v000, double v100, double v010, double v110, double v001, double v101, double v011, double v111, double x, double y, double z) {
  double s = interpolate2D(v000, v100, v010, v110, x, y);
  double t = interpolate2D(v001, v101, v011, v111, x, y);
  return interpolate1D(s, t, z);
}

SDF_GridData::SDF_GridData(SDF& f, const arr& _lo, const arr& _up, const uintA& res)
  : lo(_lo), up(_up) {
  //compute grid data
  arr samples = ::grid(lo, up, res);
  arr values = f.eval(samples);
  copy(gridData, values);
  gridData.reshape({res(0)+1, res(1)+1, res(2)+1});
}

double SDF_GridData::f(arr& g, arr& H, const arr& x){
  arr rot = pose.rot.getArr();
  arr x_rel = (~rot)*(x-conv_vec2arr(pose.pos)); //point in box coordinates

  arr gBox, HBox;
  boolA clipped = {false, false, false};
  double fBox=0.;
  double eps=.001;
  if(!boundCheck(x_rel, lo+eps, up-eps, 0., false)){ //check outside box
//    boundClip(x_rel, lo+eps, up-eps);
    //clip -- and memorize which are clipped!
    for(uint i=0; i<3; i++) {
      if(x_rel(i)<lo.elem(i)+eps){ x_rel.elem(i) = lo.elem(i)+eps; clipped.elem(i)=true; }
      if(x_rel(i)>up.elem(i)-eps){ x_rel.elem(i) = up.elem(i)-eps; clipped.elem(i)=true; }
    }
    arr size = up - lo - 2.*eps;
    arr center = .5*(up+lo);
    rai::Transformation boxPose=pose;
    boxPose.addRelativeTranslation(center);
    SDF_ssBox B(boxPose, size);
    fBox = B.f(gBox, HBox, x);
    CHECK(fBox>=0., "");
  }

  arr res = arr{(double)gridData.d0-1, (double)gridData.d1-1, (double)gridData.d2-1};
  res /= (up-lo);
  arr fidx = (x_rel-lo) % res;

  arr frac(3), idx(3);
  for(uint i=0;i<3;i++) frac(i) = modf(fidx(i), &idx(i));

  int _x = idx(0);
  int _y = idx(1);
  int _z = idx(2);
  double dx = frac(0);
  double dy = frac(1);
  double dz = frac(2);

  if(_x+1==(int)gridData.d0 && dx<1e-10){ _x--; dx=1.; }
  if(_y+1==(int)gridData.d1 && dy<1e-10){ _y--; dy=1.; }
  if(_z+1==(int)gridData.d2 && dz<1e-10){ _z--; dz=1.; }

  double v000 = gridData(_x+0,_y+0,_z+0);
  double v100 = gridData(_x+1,_y+0,_z+0);
  double v010 = gridData(_x+0,_y+1,_z+0);
  double v110 = gridData(_x+1,_y+1,_z+0);
  double v001 = gridData(_x+0,_y+0,_z+1);
  double v101 = gridData(_x+1,_y+0,_z+1);
  double v011 = gridData(_x+0,_y+1,_z+1);
  double v111 = gridData(_x+1,_y+1,_z+1);

#if 1
  double f = interpolate3D(v000, v100, v010, v110, v001, v101, v011, v111,
                           dx,dy,dz);
#else
  arr wx = {1.-dx, dx};
  arr wy = {1.-dy, dy};
  arr wz = {1.-dz, dz};
  arr coeffs = (wx ^ wy) ^ wz;
  arr values = { v000, v100, v010, v110, v001, v101, v011, v111 };
  double f = scalarProduct(coeffs, values);
#endif

  if(!!g){
    g.resize(3).setZero();
    if(!clipped(0)) g(0) = interpolate2D(v100,v110,v101,v111, dy,dz) - interpolate2D(v000,v010,v001,v011, dy,dz);
    if(!clipped(1)) g(1) = interpolate2D(v010,v110,v011,v111, dx,dz) - interpolate2D(v000,v100,v001,v101, dx,dz);
    if(!clipped(2)) g(2) = interpolate2D(v001,v101,v011,v111, dx,dy) - interpolate2D(v000,v100,v010,v110, dx,dy);
    g *= res;
    g = rot*g;
  }

  if(!!H){
    H.resize(3,3).setZero();
  }

  if(fBox){
    double boxFactor=1.;
    f += boxFactor * fBox;
    if(!!g) g += boxFactor * gBox;
    if(!!H) H += boxFactor * HBox;
  }

  return f;
}

void SDF_GridData::write(std::ostream& os) const {
  lo.writeTagged(os,"lo");
  up.writeTagged(os,"up");
  gridData.writeTagged(os, "sdf", true);
}

void SDF_GridData::read(std::istream& is){
  lo.readTagged(is,"lo");
  up.readTagged(is,"up");
  gridData.readTagged(is, "sdf");
}

//===========================================================================

double SDF_SuperQuadric::f(arr& g, arr& H, const arr& x) {
  double fx=0;
  if(!!g) g.resize(3).setZero();
  if(!!H) H.resize(3,3).setZero();
  for(uint i=0;i<3;i++){
    double s=size.elem(i);
    double z=x.elem(i)/s;
    if(z<0.){ z*=-1.; s*=-1.; }
//    double sign=rai::sign(z);
    fx += pow(z, degree);
    if(!!g) g(i) += degree*pow(z, degree-1.)/s;
    if(!!H) H(i,i) += degree*(degree-1.)*pow(z, degree-2.)/(s*s);
  }

  return fx-1.;
}

//===========================================================================

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
    g({3, 5}) = - signs%(rai::Vector(grad) / t.rot).getArr();
    g(6) = -1.;
    g({10, 13}) = ~grad*crossProduct(t.rot.getJacobian(), (x({0, 2})-t.pos.getArr()));
    g({10, 13}) /= -sqrt(sumOfSqr(x({10, 13}))); //account for the potential non-normalization of q
  }
  return d;
};


double SDF_Torus::f(arr& g, arr& H, const arr& _x){
    double x=_x(0), y=_x(1), z=_x(2);
    double r=sqrt(x*x + y*y);
    return z*z + (1.-r)*(1.-r) - .1;
}
