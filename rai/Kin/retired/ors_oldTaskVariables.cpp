/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "kin.h"
#include "kin_oldTaskVariables.h"

TaskVariable::TaskVariable() {
  active=false;
  type=noneTVT;
  targetType=directTT;
  y_prec=0.; v_prec=0.; Pgain=Dgain=0.; err=derr=0.;
}

DefaultTaskVariable::DefaultTaskVariable():TaskVariable() {
}

DefaultTaskVariable::~DefaultTaskVariable() {
}

DefaultTaskVariable::DefaultTaskVariable(
  const char* _name,
  const rai::Configuration& _ors,
  TVtype _type,
  const char* iname, const char* iframe,
  const char* jname, const char* jframe,
  const arr& _params) {
  set(
    _name, _ors, _type,
    iname  ? (int)_ors.getBodyByName(iname)->index      : -1,
    iframe ? rai::Transformation().setText(iframe) : Transformation_Id,
    jname  ? (int)_ors.getBodyByName(jname)->index      : -1,
    jframe ? rai::Transformation().setText(jframe) : Transformation_Id,
    _params);
}

DefaultTaskVariable::DefaultTaskVariable(
  const char* _name,
  const rai::Configuration& _ors,
  TVtype _type,
  const char* iShapeName,
  const char* jShapeName,
  const arr& _params) {
  rai::Shape* a = iShapeName ? _ors.getShapeByName(iShapeName):nullptr;
  rai::Shape* b = jShapeName ? _ors.getShapeByName(jShapeName):nullptr;
  set(
    _name, _ors, _type,
    a ? (int)a->body->index : -1,
    a ? a->rel : Transformation_Id,
    b ? (int)b->body->index : -1,
    b ? b->rel : Transformation_Id,
    _params);
}

TaskVariable::~TaskVariable() {
}

void DefaultTaskVariable::set(
  const char* _name,
  const rai::Configuration& _ors,
  TVtype _type,
  int _i, const rai::Transformation& _irel,
  int _j, const rai::Transformation& _jrel,
  const arr& _params) {
  type=_type;
  name=_name;
  i=_i;
  irel=_irel;
  j=_j;
  jrel=_jrel;
  if(!!_params) params=_params; else params.clear();
  updateState(_ors);
  y_target=y;
  v_target=v;
}

/*void TaskVariable::set(const char* _name, rai::Configuration& _sl, TVtype _type, const char *iname, const char *jname, const char *reltext){
  set(
    _name, _sl, _type,
    _sl.getBodyByName(iname)->index,
    _sl.getBodyByName(jname)->index,
    rai::Transformation().setText(reltext));
}*/

void TaskVariable::setGains(double pgain, double dgain, bool onReal) {
  if(onReal)  targetType=pdGainOnRealTT;  else  targetType=pdGainOnReferenceTT;
  active=true;
  Pgain=pgain;
  Dgain=dgain;
  if(!y_prec) y_prec=100.;
}

void TaskVariable::setGainsAsNatural(double oscPeriod, double dampingRatio, bool onReal) {
  if(onReal)  targetType=pdGainOnRealTT;  else  targetType=pdGainOnReferenceTT;
  active=true;
  Pgain = rai::sqr(RAI_PI/oscPeriod);
  Dgain = 4.*dampingRatio*sqrt(Pgain);
  if(!y_prec) y_prec=100.;
}

void TaskVariable::setGainsAsAttractor(double decaySteps, double oscillations, bool onReal) {
  if(onReal)  targetType=pdGainOnRealTT;  else  targetType=pdGainOnReferenceTT;
  active=true;
  Dgain=2./decaySteps;
  Pgain=Dgain*Dgain*(RAI_PI*RAI_PI*oscillations*oscillations + .25);
  if(!y_prec) y_prec=100.;
}

//compute an y_trajectory and y_prec_trajectory which connects y with y_target and 0 with y_prec
void TaskVariable::setTrajectory(uint T, double funnelsdv, double funnelvsdv) {
  OPS;
  targetType=trajectoryTT;
  active=true;
  uint t;
  double a;
  y_trajectory.resize(T, y.N);
  y_prec_trajectory.resize(T);
  v_trajectory.resize(T, y.N);
  v_prec_trajectory.resize(T);
  for(t=0; t<T; t++) {
    a = (double)t/(T-1);
    y_trajectory[t]()  = ((double)1.-a)*y + a*y_target;
    y_prec_trajectory(t) = (double)1./rai::sqr(sqrt((double)1./y_prec) + ((double)1.-a)*funnelsdv);

    v_trajectory[t]()  = ((double)1.-a)*v + a*v_target;
    v_prec_trajectory(t) = (double)1./rai::sqr(sqrt((double)1./v_prec) + ((double)1.-a)*funnelvsdv);
  }
}

//compute an y_trajectory and y_prec_trajectory which connects y with y_target and 0 with y_prec
void TaskVariable::setConstantTargetTrajectory(uint T) {
  OPS;
  targetType=trajectoryTT;
  active=true;
  uint t;
  y_trajectory.resize(T+1, y.N);
  v_trajectory.resize(T+1, y.N);
  for(t=0; t<=T; t++) {
    y_trajectory[t]()  = y_target;
    v_trajectory[t]()  = v_target;
  }
}

//compute an y_trajectory and y_prec_trajectory which connects y with y_target and 0 with y_prec
void TaskVariable::setInterpolatedTargetTrajectory(uint T) {
  OPS;
  targetType=trajectoryTT;
  active=true;
  uint t;
  double a;
  y_trajectory.resize(T, y.N);
  v_trajectory.resize(T, y.N);
  for(t=0; t<T; t++) {
    a = (double)t/(T-1);
    y_trajectory[t]()  = ((double)1.-a)*y + a*y_target;
    v_trajectory[t]()  = ((double)1.-a)*v + a*v_target;
  }
}

void TaskVariable::setInterpolatedTargetsEndPrecisions(uint T, double mid_y_prec, double final_y_prec, double mid_v_prec, double final_v_prec) {
  targetType=trajectoryTT;
  active=true;
  uint t;
  double a;
  y_trajectory.resize(T+1, y.N);  y_prec_trajectory.resize(T+1);
  v_trajectory.resize(T+1, y.N);  v_prec_trajectory.resize(T+1);
  for(t=0; t<=T; t++) {
    a = (double)t/T;
    y_trajectory[t]() = ((double)1.-a)*y + a*y_target;
    v_trajectory[t]() = ((double)1.-a)*v + a*v_target;
  }
  for(t=0; t<T; t++) {
    y_prec_trajectory(t) = mid_y_prec;
    v_prec_trajectory(t) = mid_v_prec;
  }
  y_prec_trajectory(T) = final_y_prec;
  v_prec_trajectory(T) = final_v_prec;
}

void TaskVariable::setInterpolatedTargetsConstPrecisions(uint T, double y_prec, double v_prec) {
  targetType=trajectoryTT;
  active=true;
  uint t;
  double a;
  y_trajectory.resize(T+1, y.N);  y_prec_trajectory.resize(T+1);
  v_trajectory.resize(T+1, y.N);  v_prec_trajectory.resize(T+1);
  for(t=0; t<=T; t++) {
    a = (double)t/T;
    y_trajectory[t]() = ((double)1.-a)*y + a*y_target;
    v_trajectory[t]() = ((double)1.-a)*v + a*v_target;
  }
  for(t=0; t<=T; t++) {
    y_prec_trajectory(t) = y_prec;
    v_prec_trajectory(t) = v_prec;
  }
}

void TaskVariable::setConstTargetsConstPrecisions(uint T, double y_prec, double v_prec) {
  targetType=trajectoryTT;
  active=true;
  uint t;
  y_trajectory.resize(T+1, y.N);  y_prec_trajectory.resize(T+1);
  v_trajectory.resize(T+1, y.N);  v_prec_trajectory.resize(T+1);
  for(t=0; t<=T; t++) {
    y_trajectory[t]() = y_target;
    v_trajectory[t]() = v_target;
    y_prec_trajectory(t) = y_prec;
    v_prec_trajectory(t) = v_prec;
  }
}

void TaskVariable::appendConstTargetsAndPrecs(uint T) {
  targetType=trajectoryTT;
  active=true;
  uint t, t0=y_trajectory.d0;
  CHECK(t0, "");
  y_trajectory.resizeCopy(T+1, y.N);  y_prec_trajectory.resizeCopy(T+1);
  v_trajectory.resizeCopy(T+1, y.N);  v_prec_trajectory.resizeCopy(T+1);
  for(t=t0; t<=T; t++) {
    y_trajectory[t]() = y_trajectory[t0-1];
    v_trajectory[t]() = v_trajectory[t0-1];
    y_prec_trajectory(t) = y_prec_trajectory(t0-1);
    v_prec_trajectory(t) = v_prec_trajectory(t0-1);
  }
}

void TaskVariable::setInterpolatedTargetsEndPrecisions(uint T, double mid_y_prec, double mid_v_prec) {
  setInterpolatedTargetsEndPrecisions(T, mid_y_prec, y_prec, mid_v_prec, v_prec);
}

void TaskVariable::setInterpolatedTargetsConstPrecisions(uint T) {
  setInterpolatedTargetsConstPrecisions(T, y_prec, v_prec);
}

void TaskVariable::setConstTargetsConstPrecisions(uint T) {
  setConstTargetsConstPrecisions(T, y_prec, v_prec);
}
//compute an y_trajectory and y_prec_trajectory which connects y with y_target and 0 with y_prec
void TaskVariable::setPrecisionTrajectoryFinal(uint T, double intermediate_prec, double final_prec) {
  OPS;
  active=true;
  uint t;
  y_prec_trajectory.resize(T);
  for(t=0; t<T-1; t++) y_prec_trajectory(t) = intermediate_prec;
  y_prec_trajectory(T-1) = final_prec;
}

//compute an y_trajectory and y_prec_trajectory which connects y with y_target and 0 with y_prec
void TaskVariable::setPrecisionTrajectoryConstant(uint T, double const_prec) {
  OPS;
  active=true;
  y_prec_trajectory.resize(T);
  y_prec_trajectory = const_prec;
}

void TaskVariable::setPrecisionVTrajectoryFinal(uint T, double intermediate_v_prec, double final_v_prec) {
  OPS;
  active=true;
  uint t;
  v_prec_trajectory.resize(T);
  for(t=0; t<T-1; t++) v_prec_trajectory(t) = intermediate_v_prec;
  v_prec_trajectory(T-1) = final_v_prec;
}

//compute an y_trajectory and y_prec_trajectory which connects y with y_target and 0 with y_prec
void TaskVariable::setPrecisionVTrajectoryConstant(uint T, double const_prec) {
  OPS;
  active=true;
  v_prec_trajectory.resize(T);
  v_prec_trajectory = const_prec;
}

//set velocity and position precisions splitting the T-step-trajectory into as
//much intervals as y_precs given.
void TaskVariable::setIntervalPrecisions(uint T, arr& y_precs, arr& v_precs) {
  CHECK(y_precs.nd==1 && v_precs.nd==1 && y_precs.N>0 && v_precs.N>0
        && y_precs.N<=T+1 && y_precs.N<=T+1,
        "number of intervals needs to be in [1, T+1].");

  uint t;
  active=true;

  v_prec_trajectory.resize(T+1);
  y_prec_trajectory.resize(T+1);

  for(t=0; t<=T; ++t) {
    y_prec_trajectory(t) = y_precs(t * y_precs.N/(T+1));
    v_prec_trajectory(t) = v_precs(t * v_precs.N/(T+1));
  }
}

void TaskVariable::shiftTargets(int offset) {
  if(!y_trajectory.N) return;
  uint n=y_trajectory.d1, T=y_trajectory.d0;
  y_trajectory.shift(offset*n, false);  y_prec_trajectory.shift(offset, false);
  v_trajectory.shift(offset*n, false);  v_prec_trajectory.shift(offset, false);
#if 1
  uint L = T+offset-1; //last good value before shift
  for(uint t=T+offset; t<T; t++) {
    y_trajectory[t] = y_trajectory[L];  y_prec_trajectory(t) = .5*y_prec_trajectory(L); //reduce precision more and more...
    v_trajectory[t] = v_trajectory[L];  v_prec_trajectory(t) = .5*v_prec_trajectory(L);
  }
#endif
}

void DefaultTaskVariable::updateState(const rai::Configuration& ors, double tau) {
  arr q, qd, p;
  rai::Vector pi, pj, c;
  arr zi, zj, Ji, Jj, JRj;
  rai::Transformation f, fi, fj;
  rai::Vector vi, vj, r, jk;
  uint k, l;
  rai::Body* bi = ors.bodies(i);
  rai::Body* bj = ors.bodies(j);

  v_old=v;
  y_old=y;

  //get state
  switch(type) {
    case posTVT:
      if(j==-1) {
        ors.kinematicsPos(y, J, bi, irel.pos);
        break;
      }
      pi = bi->X.pos + bi->X.rot * irel.pos;
      pj = bj->X.pos + bj->X.rot * jrel.pos;
      c = bj->X.rot / (pi-pj);
      y = conv_vec2arr(c);
      ors.kinematicsPos(NoArr, Ji, bi, irel.pos);
      ors.kinematicsPos(NoArr, Jj, bj, jrel.pos);
      ors.axesMatrix(JRj, bj);
      J.resize(3, Jj.d1);
      for(k=0; k<Jj.d1; k++) {
        vi.set(Ji(0, k), Ji(1, k), Ji(2, k));
        vj.set(Jj(0, k), Jj(1, k), Jj(2, k));
        r .set(JRj(0, k), JRj(1, k), JRj(2, k));
        jk =  bj->X.rot / (vi - vj);
        jk -= bj->X.rot / (r ^(pi - pj));
        J(0, k)=jk.x; J(1, k)=jk.y; J(2, k)=jk.z;
      }

      break;
    case zoriTVT:
      if(j==-1) {
        vi = irel.rot.getZ();
        ors.kinematicsVec(y, J, bi, vi);
        break;
      }
      //relative
      RAI_MSG("warning - don't have a correct Jacobian for this TVType yet");
      fi = bi->X; fi.appendTransformation(irel);
      fj = bj->X; fj.appendTransformation(jrel);
      f.setDifference(fi, fj);
      y = conv_vec2arr(f.rot.getZ());
      NIY; //TODO: Jacobian?
      break;
    case rotTVT:       y.resize(3);  ors.axesMatrix(J, bi);  y.setZero(); break; //the _STATE_ of rot is always zero... the Jacobian not... (hack)
    case qItselfTVT:   ors.getJointState(q, qd);    y = q;   J.setId(q.N);  break;
    case qLinearTVT:   ors.getJointState(q, qd);    y = params * q;   J=params;  break;
    case qSquaredTVT:
      ors.getJointState(q, qd);
      y.resize(1);  y(0) = scalarProduct(params, q, q);
      J = params * q;
      J *= (double)2.;
      J.reshape(1, q.N);
      break;
    case qSingleTVT:
      ors.getJointState(q, qd);
      y.resize(1);  y(0)=q(-i);
      J.resize(1, ors.getJointStateDimension());
      J.setZero();
      J(0, -i) = 1.;
      break;
    case qLimitsTVT:   ors.kinematicsLimitsCost(y, J, params);  break;
    case comTVT:       ors.getCenterOfMass(y);     y.resizeCopy(2); ors.getComGradient(J);  J.resizeCopy(2, J.d1);  break;
    case collTVT:      ors.kinematicsProxyCost(y, J, params(0));  break;
    case colConTVT:    ors.kinematicsContactConstraints(y, J); break;
    case skinTVT:
      y.resize(params.N);
      y.setZero();
      J.clear();
      for(k=0; k<params.N; k++) {
        l=(uint)params(k);
        ors.kinematicsPos(NoArr, Ji, ors.bodies(l));
        vi = -ors.bodies(l)->X.rot.getY();
        vi *= -1.;
        zi = conv_vec2arr(vi);
        J.append(~zi*Ji);
      }
      J.reshape(params.N, J.N/params.N);
      break;
    case zalignTVT:
      vi = irel.rot.getZ();
      ors.kinematicsVec(zi, Ji, bi, vi);
      if(j==-1) {
        rai::Vector world_z;
        if(params.N==3) world_z.set(params.p); else world_z=Vector_z;
        zj = conv_vec2arr((jrel*world_z));
        Jj.resizeAs(Ji);
        Jj.setZero();
      } else {
        vj = jrel.rot.getZ();
        ors.kinematicsVec(zj, Jj, bj, vj);
      }
      y.resize(1);
      y(0) = scalarProduct(zi, zj);
      J = ~zj * Ji + ~zi * Jj;
      J.reshape(1, ors.getJointStateDimension());
      break;
    case userTVT:
      userUpdate(ors);
      break;
    default:  HALT("no such TVT");
  }
  transpose(Jt, J);

  if(y_old.N!=y.N) {
    y_old=y;
    v.resizeAs(y); v.setZero();
    v_old=v;
  }

  //v = .5*v + .5*(y - y_old);
  v = (y - y_old)/tau; //TODO: the velocity should be evaluated from the joint angle velocity (J*dq) to be consistent with the whole soc code!

  if(y_target.N==y.N) {
    err=length(y - y_target);
    derr=err - length(y_old - y_target);
  }
}

void DefaultTaskVariable::getHessian(const rai::Configuration& ors, arr& H) {
  switch(type) {
    case posTVT:
      if(j==-1) { kin.hessianPos(H, ors.bodies(i), &irel.pos); break; }
    default:  NIY;
  }
}

void TaskVariable::updateChange(int t, double tau) {
  CHECK(y.N, "variable needs to be updated before!");
  arr yt, vt;
  if(t!=-1) {
    yt.referToDim(y_trajectory, t);
    vt.referToDim(v_trajectory, t);
    //y_prec     = y_prec_trajectory(t);
  } else {
    yt.referTo(y_target);
    vt.referTo(v_target);
  }
  CHECK_EQ(yt.N, y.N, "targets have wrong dimension -- perhaps need to be set before");
  CHECK_EQ(vt.N, v.N, "targets have wrong dimension -- perhaps need to be set before");
  switch(targetType) {
    case trajectoryTT:
    case directTT: {
      y_ref = yt;
      v_ref = vt;
      break;
    }
    case positionGainsTT: {
      y_ref = y + Pgain*(yt - y) + Dgain*(vt - v);
      v_ref = v;
      break;
    }
    case pdGainOnRealTT: {
      v_ref = v + tau*(Pgain*(yt - y) + Dgain*(vt - v));
      y_ref = y + tau*v_ref; //``Euler integration''
      //v_ref /= tau;  //TaskVariable measures vel in steps; here we meassure vel in double time
      break;
    }
    case pdGainOnReferenceTT: {
      if(y_ref.N!=y.N) { y_ref=y; v_ref=v; }
      v_ref = v_ref + tau*(Pgain*(yt - y_ref) + Dgain*(vt - v_ref));
      y_ref = y_ref + tau*v_ref; //``Euler integration''
      //v_ref /= tau;  //TaskVariable measures vel in steps; here we meassure vel in double time
      static ofstream fil("refs");
      fil <<y_ref <<v_ref <<yt <<vt <<y <<v <<' ' <<Pgain <<' ' <<Dgain <<endl;
      break;
    }
    default:
      HALT("needs a target type! set targets before!");
  }
}

/*  switch(type){
  case zoriTVT:
    dx.resize(3);
    normal.set(y.p);
    d.set(y_change.p);
    d.makeNormal(normal);
    dx.setCarray(d.v, 3);
    break;
  default:
    dx = y_change;
  default:
    break;
  }
    */

void TaskVariable::write(ostream& os, const rai::Configuration& ors) const {
  os <<"TaskVariable '" <<name <<'\'';
  os
      <<"\n  y=" <<y
      <<"\t  v=" <<v
      <<"\n  y_target=" <<y_target
      <<"\t  v_target=" <<v_target
      <<"\n  y_ref="  <<y_ref
      <<"\t  v_ref=" <<v_ref
      <<"\n  y_prec=" <<y_prec
      <<"\t  v_prec=" <<v_prec
      <<"\n  Pgain=" <<Pgain
      <<"\t  Dgain=" <<Dgain
      <<"\n  y_error=" <<sqrDistance(y, y_target)
      <<"\t  v_error=" <<sqrDistance(v, v_target)
      <<"\t  error="  <<y_prec* sqrDistance(y, y_target)+v_prec* sqrDistance(v, v_target)
      <<endl;
}

void DefaultTaskVariable::write(ostream& os, const rai::Configuration& ors) const {
  TaskVariable::write(os);
  return;
  rai::Body* bi = ors.bodies(i);
  switch(type) {
    case posTVT:     os <<"  (pos " <<bi->name <<")"; break;
    //case relPosTVT:  os <<"  (relPos " <<bi->name <<'-' <<bj->name <<")"; break;
    case zoriTVT:    os <<"  (zori " <<bi->name <<")"; break;
    case rotTVT:     os <<"  (rot " <<bi->name <<")"; break;
    case qLinearTVT: os <<"  (qLinear " <<sum(params) <<")"; break;
    case qSquaredTVT:os <<"  (qSquared " <<sum(params) <<")"; break;
    case qSingleTVT: os <<"  (qSingle " <<ors.joints(-i)->from->name <<'-' <<ors.joints(-i)->to->name <<")"; break;
    case qLimitsTVT: os <<"  (qLimitsTVT " <<sum(params) <<")"; break;
    case qItselfTVT: os <<"  (qItselfTVT)"; break;
    case comTVT:     os <<"  (COM)"; break;
    case collTVT:    os <<"  (COLL)"; break;
    case colConTVT:  os <<"  (colCon)"; break;
    case zalignTVT:  os <<"  (zalign " <<ors.bodies(i)->name <<'-' <<(j==-1?"-1":STRING("" <<ors.bodies(j)->name).p) <<"); params:" <<params; break;
    case userTVT:    os <<"  (userTVT)"; break;
    default: HALT("CV::write - no such TVT");
  }
}

ProxyTaskVariable::ProxyTaskVariable(const char* _name,
                                     rai::Configuration& ors,
                                     CTVtype _type,
                                     uintA _shapes,
                                     double _margin,
                                     bool _linear) {
  type=_type;
  name=_name;
  shapes=_shapes;
  margin=_margin;
  linear=_linear;
  updateState(ors);
  y_target=y;
  v_target=v;
}

#if 0
void addAContact(double& y, arr& J, const rai::Proxy* p, const rai::Configuration& ors, double margin, bool linear) {
  double d;
  rai::Shape* a, *b;
  rai::Vector arel, brel;
  arr Ja, Jb, dnormal;

  a=ors.shapes(p->a); b=ors.shapes(p->b);
  d=1.-p->d/margin;

  if(!linear) y += d*d;
  else        y += d;

  arel.setZero();  arel=a->X.rot/(p->posA-a->X.pos);
  brel.setZero();  brel=b->X.rot/(p->posB-b->X.pos);

  if(!!J) {
    CHECK(p->normal.isNormalized(), "proxy normal is not normalized");
    dnormal.referTo(&p->normal.x, 3); dnormal.reshape(1, 3);
    if(!linear) {
      ors.jacobianPos(Ja, a->body->index, &arel); J -= (2.*d/margin)*(dnormal*Ja);
      ors.jacobianPos(Jb, b->body->index, &brel); J += (2.*d/margin)*(dnormal*Jb);
    } else {
      ors.jacobianPos(Ja, a->body->index, &arel); J -= (1./margin)*(dnormal*Ja);
      ors.jacobianPos(Jb, b->body->index, &brel); J += (1./margin)*(dnormal*Jb);
    }
  }
}
#endif

void ProxyTaskVariable::updateState(const rai::Configuration& ors, double tau) {
  v_old=v;
  y_old=y;

  y.resize(1);  y.setZero();
  J.resize(1, ors.getJointStateDimension());  J.setZero();

  switch(type) {
    case allCTVT:
      for(rai::Proxy* p: ors.proxies)  if(p->d<margin) {
          ors.kinematicsProxyCost(y, J, p, margin, linear, true);
          p->colorCode = 1;
        }
      break;
    case allListedCTVT:
      for(rai::Proxy* p: ors.proxies)  if(p->d<margin) {
          if(shapes.contains(p->a) && shapes.contains(p->b)) {
            ors.kinematicsProxyCost(y, J, p, margin, linear, true);
            p->colorCode = 2;
          }
        }
    case allExceptListedCTVT:
      for(rai::Proxy* p: ors.proxies)  if(p->d<margin) {
          if(!shapes.contains(p->a) && !shapes.contains(p->b)) {
            ors.kinematicsProxyCost(y, J, p, margin, linear, true);
            p->colorCode = 3;
          }
        }
      break;
    case bipartiteCTVT:
      for(rai::Proxy* p: ors.proxies)  if(p->d<margin) {
          if((shapes.contains(p->a) && shapes2.contains(p->b)) ||
              (shapes.contains(p->b) && shapes2.contains(p->a))) {
            ors.kinematicsProxyCost(y, J, p, margin, linear, true);
            p->colorCode = 4;
          }
        }
    case pairsCTVT: {
      shapes.reshape(shapes.N/2, 2);
      // only explicit paris in 2D array shapes
      uint j;
      for(rai::Proxy* p: ors.proxies)  if(p->d<margin) {
          for(j=0; j<shapes.d0; j++) {
            if((shapes(j, 0)==(uint)p->a && shapes(j, 1)==(uint)p->b) || (shapes(j, 0)==(uint)p->b && shapes(j, 1)==(uint)p->a))
              break;
          }
          if(j<shapes.d0) { //if a pair was found
            ors.kinematicsProxyCost(y, J, p, margin, linear, true);
            p->colorCode = 5;
          }
        }
    } break;
    case allExceptPairsCTVT: {
      shapes.reshape(shapes.N/2, 2);
      // only explicit paris in 2D array shapes
      uint j;
      for(rai::Proxy* p: ors.proxies)  if(p->d<margin) {
          for(j=0; j<shapes.d0; j++) {
            if((shapes(j, 0)==(uint)p->a && shapes(j, 1)==(uint)p->b) || (shapes(j, 0)==(uint)p->b && shapes(j, 1)==(uint)p->a))
              break;
          }
          if(j==shapes.d0) { //if a pair was not found
            ors.kinematicsProxyCost(y, J, p, margin, linear, true);
            p->colorCode = 5;
          }
        }
    } break;
    case vectorCTVT: {
      //outputs a vector of collision meassures, with entry for each explicit pair
      shapes.reshape(shapes.N/2, 2);
      y.resize(shapes.d0, 1).setZero();
      J.resize(shapes.d0, J.d1).setZero();
      uint j;
      for(rai::Proxy* p: ors.proxies)  if(p->d<margin) {
          for(j=0; j<shapes.d0; j++) {
            if((shapes(j, 0)==(uint)p->a && shapes(j, 1)==(uint)p->b) || (shapes(j, 0)==(uint)p->b && shapes(j, 1)==(uint)p->a))
              break;
          }
          if(j<shapes.d0) {
            ors.kinematicsProxyCost(y[j](), J[j](), p, margin, linear, true);
            p->colorCode = 5;
          }
        }
      y.reshape(shapes.d0);
    } break;
    default: NIY;
  }
  transpose(Jt, J);

  if(y_old.N!=y.N) {
    y_old=y;
    v.resizeAs(y); v.setZero();
    v_old=v;
  }
  v = (y - y_old)/tau; //TODO: the velocity should be evaluated from the joint angle velocity (J*dq) to be consistent with the whole soc code!

  if(y_target.N==y.N) {
    err=length(y - y_target);
    derr=err - length(y_old - y_target);
  }
}

//===========================================================================
//
// TaskVariableList functions
//

void reportAll(TaskVariableList& CS, ostream& os, bool onlyActives) {
  for(uint i=0; i<CS.N; i++) if(!onlyActives || CS(i)->active) {
      os <<'[' <<i <<"] " <<*CS(i);
    }
}

void reportNames(TaskVariableList& CS, ostream& os, bool onlyActives) {
  uint i, j, n=1;
  os <<"CVnames = {";
  for(i=0; i<CS.N; i++) if(!onlyActives || CS(i)->active) {
      for(j=0; j<CS(i)->y.N; j++) {
        os <<"'" <<n <<'-' <<CS(i)->name <<j <<"' ";
        n++;
      }
    }
  os <<"};" <<endl;
}

void reportState(TaskVariableList& CS, ostream& os, bool onlyActives) {
  ;
  uint i;
  rai::IOraw=true;
  for(i=0; i<CS.N; i++) if(!onlyActives || CS(i)->active) {
      os <<CS(i)->y;
    }
  os <<endl;
}

void reportErrors(TaskVariableList& CS, ostream& os, bool onlyActives, int t) {
  uint i;
  double e, E=0.;
  for(i=0; i<CS.N; i++) if(!onlyActives || CS(i)->active) {
      if(t!=-1)
        if(t) e=length(CS(i)->y - CS(i)->y_trajectory[t-1]);
        else  e=0.;
      else
        e=length(CS(i)->y - CS(i)->y_target);
      os <<e <<' ';
      E += e; //*CS(i)->y_prec;
    }
  os <<E <<endl;
}

void activateAll(TaskVariableList& CS, bool active) {
  for(uint i=0; i<CS.N; i++) CS(i)->active=active;
}

void shiftTargets(TaskVariableList& CS, int offset) {
  for(uint i=0; i<CS.N; i++) CS(i)->shiftTargets(offset);
}

void updateState(TaskVariableList& CS, const rai::Configuration& ors) {
  for(uint i=0; i<CS.N; i++) {
    CS(i)->updateState(ors);
  }
}

void updateChanges(TaskVariableList& CS, int t) {
  for(uint i=0; i<CS.N; i++) if(CS(i)->active) {
      CS(i)->updateChange(t);
    }
}

void getJointJacobian(TaskVariableList& CS, arr& J) {
  uint i, n=0;
  J.clear();
  for(i=0; i<CS.N; i++) if(CS(i)->active) {
      NIY; //TODO: do I have to do updateState?
      //CS(i)->updateJacobian();
      J.append(CS(i)->J);
      n=CS(i)->J.d1;
    }
  J.reshape(J.N/n, n);
}

void bayesianControl(TaskVariableList& CS, arr& dq, const arr& W) {
  uint n=W.d0;
  dq.resize(n);
  dq.setZero();
  uint i;
  arr a(n), A(n, n), Ainv(n, n);
  //arr Q, JQ;
  A=W;
  a.setZero();
  arr w(3);
  for(i=0; i<CS.N; i++) if(CS(i)->active) {
      a += CS(i)->y_prec * CS(i)->Jt * (CS(i)->y_ref-CS(i)->y);
      A += CS(i)->y_prec * CS(i)->Jt * CS(i)->J;
    }
  inverse_SymPosDef(Ainv, A);
  dq = Ainv * a;
}

//===========================================================================
//
// task variable table
//

#if 0

void TaskVariableTable::init(const rai::Configuration& ors) {
  uint i, j, k, m=0, T=0, t, qdim;
  //count the total task dimension, q-d
  for_list(TaskVariable, v, list) {
    v->updateState(ors);
    if(v->active) {
      m+=y.N;
      if(!T) T=y_trajectory.d0;
      else CHECK(T=y_trajectory.d0);
      qdim=J.d1;
    }
  }
  //resize everything
  y.resize(T, m);
  phi.resize(T, m);
  J.resize(T, m, qdim);
  rho.resize(T, m);

  updateState(0, ors, true);
}

//recompute all phi in time slice t using the pose in ors
void TaskVariableTable::updateTimeSlice(uint t, const rai::Configuration& ors, bool alsoTargets) {
  uint i, j, k, m=0;
  for_list(TaskVariable, v, list) {
    v->updateState(ors);
    if(v->active) {
      for(j=0; j<v->y.N; j++) {
        phi(t, m+j) = v->y(j);
        for(k=0; k<J.d2; k++) J(t, m+j, k) = v->J(j, k);
        if(alsoTargets) {
          y(t, m+j) = v->y_trajectory(t, j);
          rho(t, m+j) = v->y_prec_trajectroy(t, j);
        }
      }
      m+=j;
    }
    CHECK_EQ(m, y.d1, "");
  }
}

double TaskVariableTable::totalCost() {
  CHECK(y.N==phi.N && y.N == rho.N, "");
  double C = 0;
  for(uint i=0; i<y.N; i++) {
    C += rho.elem(i)*sqrDistance(y.elem(i), phi.elem(i));
  }
}

#endif

/*void getJointXchange(TaskVariableList& CS, arr& y_change){
  uint i;
  y_change.clear();
  for(i=0;i<CS.N;i++) if(CS(i)->active){
    y_change.append(CS(i)->y_change);
  }
  y_change.reshape(y_change.N);
}

double getCost_obsolete(TaskVariableList& CS, const arr& W, int t){
  uint i;
  double e, C=0.;
  for(i=0;i<CS.N;i++) if(CS(i)->active){
    if(t!=-1){
      e=sumOfSqr(CS(i)->y - CS(i)->y_trajectory[t]);
    }else{
      e=sumOfSqr(CS(i)->y - CS(i)->y_target);
    }
    C += e*CS(i)->y_prec;
    //cout <<"cost(" <<CS(i)->name <<") = " <<e <<", " <<e*CS(i)->y_prec <<endl;
  }
  return C;
}

void getCostGradient_obsolete(TaskVariableList& CS, arr& dCdq, const arr& W, int t){
  uint i, n=W.d0;
  dCdq.resize(n);
  dCdq.setZero();
  arr e, J, Jt, dx;
  for(i=0;i<CS.N;i++) if(CS(i)->active){
    CS(i)->updateJacobian();
    if(t!=-1){
      e=CS(i)->y - CS(i)->y_trajectory[t];
    }else{
      e=CS(i)->y - CS(i)->y_target;
    }
    dCdq += CS(i)->Jt * (e*((double)2.*CS(i)->y_prec));
  }
}

void hierarchicalControl_obsolete(TaskVariableList& CS, arr& dq, const arr& W){
  uint i, n=W.d0;
  dq.resize(n);
  dq.setZero();
  arr Jhat, Jhatinv, N;
  N.setId(n);
  arr Winv;
  inverse_SymPosDef(Winv, W);
  for(i=0;i<CS.N;i++) if(CS(i)->active){
    CS(i)->updateJacobian();

    Jhat = CS(i)->J * N;
    pseudoInverse(Jhatinv, Jhat, Winv, 1e-5);
    dq += Jhatinv * (CS(i)->y_change - CS(i)->J * dq);
    N  -= Jhatinv * Jhat;
  }
}

void bayesianIterateControl_obsolete(TaskVariableList& CS,
                            arr& qt, const arr& qt_1, const arr& W, double eps, uint maxIter){
  uint j;
  qt=qt_1;
  arr dq;
  for(j=0;j<maxIter;j++){
    top.setq(qt);
    bayesianIKControl(top, dq, W);
    if(j<3) qt+=dq;
    //else if(j<10) qt+=.8*dq;
    else qt+=.8*dq;
    if(dq.absMax()<eps) break;
  }
  if(j==maxIter) HALT("warning: IK didn't converge (|last step|=" <<dq.absMax() <<")");
  else cout <<"IK converged after steps=" <<j <<endl;
}

void additiveControl_obsolete(TaskVariableList& CS, arr& dq, const arr& W){
  dq.resize(W.d0);
  dq.setZero();
  uint i, n=0;
  arr Jinv;
  arr Winv;
  inverse_SymPosDef(Winv, W);
  for(i=CS.N;i--;) if(CS(i)->active){
    CS(i)->updateJacobian();
    pseudoInverse(Jinv, CS(i)->J, Winv, 0.);
    dq += Jinv * CS(i)->y_change;
    n++;
  }
  dq/=(double)n;
}
*/
/*OLD
void bayesianPlanner_obsolete(rai::Configuration *ors, TaskVariableList& CS, SwiftInterface *swift, OpenGL *gl,
                     arr& q, uint T, const arr& W, uint iterations,
                     std::ostream* os, int display, bool repeat){
  //FOR THE OLD VERSION, SEE SMAC.CPP IN THE DEPOSIT
  uint n=W.d0, i;
  arr J, Jt, phiHatQ, dx;
  arr q0, qv0, Winv;
  inverse_SymPosDef(Winv, W);

  byteA img(300, 500, 3);

  ors->getJointState(q0, qv0);

  arr tmp,
    a(T, n), Ainv(T, n, n),
    z(T, n), Zinv(T, n, n),
    b(T, n), B(T, n, n), Binv(T, n, n),
    r(T, n), R(T, n, n),
    hatq(T, n);
  a[0]=q0;
  Ainv[0].setDiag(1e10);
  b[0]=q0;
  B[0].setDiag(1e-10);
  Binv[0].setDiag(1e10);
  r[0]=0.;
  R[0]=0.;
  z.setZero();
  Zinv.setZero();

  rai::timerStart();

  uint k, t, dt, t0;
  for(k=0;k<iterations;k++){
    if(!(k&1)){ dt=1; t0=1; }else{ dt=(uint)-1; t0=T-1; }
    for(t=t0;t<T && t>0;t+=dt){
      //compute (a, A)
      inverse_SymPosDef(tmp, Ainv[t-1] + R[t-1]);
      a[t] = tmp * (Ainv[t-1]*a[t-1] + r[t-1]);
      inverse_SymPosDef(Ainv[t](), Winv + tmp);

      //cout <<"a\n" <<a[t] <<endl <<Ainv[t] <<endl;

      //compute (z, Z)
      if(k && t<T-1){
        inverse_SymPosDef(tmp, Zinv[t+1] + R[t+1]);
        z[t] = tmp * (Zinv[t+1]*z[t+1] + r[t+1]);
        inverse_SymPosDef(Zinv[t](), Winv + tmp);
      }
      if(k && t==T-1){
        z[t] = b[t];
        Zinv[t].setDiag(1e10); //fixes the end posture!, use 1e-5 otherwise
      }

      //cout <<"z\n" <<z[t] <<endl <<Zinv[t] <<endl;

      //compute (r, R)
      //if(k) hatq[t]()=.2*b[t]+.8*hatq[t]; else hatq[t]()=a[t];
      if(k) hatq[t]()=b[t]; else hatq[t]()=a[t];
      ors->setJointState(hatq[t]);
      ors->calcNodeFramesFromEdges();
      computeProxiesUsingSwift(*ors, *swift, false);
      //slGetProxies(*ors, *ode);
      r[t].setZero();
      R[t].setZero();
      for(i=0;i<CS.N;i++) if(CS(i)->active){
        CS(i)->updateState();    phiHatQ=CS(i)->y;
        CS(i)->updateJacobian(); J=CS(i)->J; Jt=CS(i)->Jt;
        dx = CS(i)->y_trajectory[t] - phiHatQ;
        r[t]() += CS(i)->y_prec_trajectory(t) * Jt * dx;
        R[t]() += CS(i)->y_prec_trajectory(t) * Jt * J;
      }
      r[t]() += R[t] * hatq[t];

      //cout <<"r\n" <<r[t] <<endl <<R[t] <<endl;

      //compute (b, B);
      Binv[t] = Ainv[t] + Zinv[t] + R[t];
      //cout <<"Binv\n" <<Binv[t] <<endl;
      inverse_SymPosDef(B[t](), Binv[t]);
      b[t] = B[t] * (Ainv[t]*a[t] + Zinv[t]*z[t] + r[t]);

      //cout <<"b\n" <<b[t] <<endl <<B[t] <<endl;

      //display
      if(display>0){
        ors->setJointState(b[t]);
        ors->calcNodeFramesFromEdges();
        //if(t==1 || !(t%display)){ gl->text.clear() <<k <<':' <<t; gl->update(); }
        //glGrabImage(img); write_ppm(img, STRING("imgs/plan_" <<std::setfill('0') <<std::setw(3) <<k <<std::setfill('0') <<std::setw(3) <<((k&1)?T-t:t) <<".ppm"), true);
      }

      if(repeat){
        //meassure offset
        double off=sqrDistance(W, b[t], hatq[t]);
        //cout <<"off = " <<off <<endl;
        if(false && k>0 && off>.05){
          //cout <<t <<" REPEAT: off = " <<off <<endl;
          t-=dt;
        }
      }
    }

    //evaluate trajectory
    //cout <<"variances over time = ";
    //for(t=0;t<T;t++) cout <<' ' <<trace(B[t]);
    double cost_t, cost1=.0, cost2=.0, length=0.;
    for(t=0;t<T;t++){
      ors->setJointState(b[t]);
      ors->calcNodeFramesFromEdges();
      computeProxiesUsingSwift(*ors, *swift, false);
      //slGetProxies(*ors, *ode);
      updateState(CS);
      if(t>0) cost2 += sqrDistance(W, b[t-1], b[t]);
      if(t>0) length += metricDistance(W, b[t-1], b[t]);
      cost1 += cost_t = getCost(CS, W, t);  //cout <<"cost = " <<cost_t <<endl;
    }
    *os <<std::setw(3) <<k
        <<"  time " <<rai::timerRead(false)
        <<"  cost1 " <<cost1
        <<"  cost2 " <<cost2
        <<"  length " <<length
        <<"  total-cost " <<cost1+cost2 <<endl;
  }

  q = b;
}
*/

#if 0
void SMAC::readCVdef(std::istream& is) {
  char c;
  TaskVariable* cv;
  rai::String name, ref1, ref2;
  rai::Transformation f;
  uint i, j, k;
  arr mat;
  rai::String::readSkipSymbols=" \n\r\t";
  rai::String::readStopSymbols=" \n\r\t";
  for(;;) {
    rai::skip(is);
    is.get(c);
    if(!is.good()) break;
    cv=&CVs.append();
    switch(c) {
      case 'p':
        is >>name >>ref1 >>PARSE("<") >>f >>PARSE(">");
        cv->initPos(name, *ors, ors->getBodyByName(ref1)->index, f);
        break;
      case 'j':
        is >>name >>ref1;
        cv->initQSingle(name, *ors, ors->getBodyByName(ref1)->firstIn->index);
        break;
      case 'l':
        is >>name >>ref1 >>ref2;
        i=ors->getBodyByName(ref1)->firstIn->index;
        j=ors->getBodyByName(ref2)->firstIn->index;
        mat.resize(j-i+1, ors->getJointStateDimension());
        mat.setZero();
        for(k=0; k<=j-i; k++) mat(k, i+k)=1.;
        cv->initQLinear(name, *ors, mat);
        break;
      case 's':
        is >>name >>ref1 >>ref2;
        i=ors->getBodyByName(ref1)->firstIn->index;
        j=ors->getBodyByName(ref2)->firstIn->index;
        mat.resize(j-i+1, ors->getJointStateDimension());
        mat.setZero();
        for(k=0; k<=j-i; k++) mat(k, i+k)=1.;
        cv->initQSquared(name, *ors, mat);
        break;
      case 'c':
        is >>name >>ref1;
        cv->initContact(name, *ors, ors->getBodyByName(ref1)->index);
        break;
      case 'X':
        is >>name >>ref1;
        cv->initGrip(name, *ors, ors->getBodyByName(ref1)->index);
        break;
      case 'o':
        is >>name >>ref1 >>ref2 >>PARSE("<") >>f >>PARSE(">");
        if(ref2=="*")
          cv->initOri(name, *ors, ors->getBodyByName(ref1)->index, -1, f);
        else
          cv->initOri(name, *ors, ors->getBodyByName(ref1)->index, ors->getBodyByName(ref2)->index, f);
        break;
      case 'M':
        is >>name;
        cv->initCom(name, *ors);
        break;
      case 'C':
        is >>name;
        cv->initCollision(name, *ors);
        break;
      default:
        NIY;
    }
    if(is.fail()) HALT("error reading `" <<c <<"' variable in smac");
  }
  is.clear();
}

void SMAC::CVclear() {
  uint i;
  for(i=0; i<CS.N; i++)
    CS(i)->y.clear();
}

void SMAC::plotCVs() {
  plotData.points.resize(2*CS.N);
  uint i;
  for(i=0; i<CS.N; i++) {
    plotData.points(2*i) = CS(i)->y;
    plotData.points(2*i+1) = CS(i)->y_target;
  }
}

#endif
