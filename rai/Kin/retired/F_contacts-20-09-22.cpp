struct TM_ContactConstraints_Vel : Feature {
  TM_ContactConstraints_Vel(){ order=1; }
  void phi2(arr& y, arr& J, const FrameL& F);
  uint dim_phi2(const FrameL& F);
};

void TM_ContactConstraints_Vel::phi2(arr& y, arr& J, const FrameL& F) {
  CHECK_EQ(order, 1, "");

  rai::ForceExchange* ex = getContact(F(0,0), F(1,0));

  arr poa, poaJ;
  ex->kinPOA(poa, poaJ);

  Value p1 = F_Position().eval({&ex->a});
  Value p2 = F_Position().eval({&ex->b});
  Value v1 = F_Position().setOrder(1).eval({F(0,0), F(1,0)});
  Value v2 = F_Position().setOrder(1).eval({F(0,1), F(1,1)});
  Value w1 = F_AngVel().eval({{2,1}, {F(0,0), F(1,0)}});
  Value w2 = F_AngVel().eval({{2,1}, {F(0,1), F(1,1)}});

  arr vc1 = v1.y - crossProduct(w1.y, poa - p1.y);
  arr Jvc1 = v1.J - skew(w1.y) * (poaJ - p1.J) + skew(poa-p1.y) * w1.J;
  arr vc2 = v2.y - crossProduct(w2.y, poa - p2.y);
  arr Jvc2 = v2.J - skew(w2.y) * (poaJ - p2.J) + skew(poa-p2.y) * w2.J;

  y = vc1 - vc2;
  if(!!J) J = Jvc1 - Jvc2;
}

uint TM_ContactConstraints_Vel::dim_phi2(const FrameL& F) {
  return 3;
}

//===========================================================================

struct F_Wrench : Feature {
  rai::Vector vec;
  double gravity=9.81;
  bool torqueOnly=false;
  F_Wrench(const arr& _vec, bool _torqueOnly=false);
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi2(const FrameL& F) {  if(torqueOnly) return 3;  return 6;  }
};

F_Wrench::F_Wrench(const arr& _vec, bool _torqueOnly) : vec(_vec), torqueOnly(_torqueOnly) {
  order=2;
  gravity = rai::getParameter<double>("TM_Wrench/gravity", 9.81);
}

void F_Wrench::phi2(arr& y, arr& J, const FrameL& F) {
  CHECK_EQ(order, 2, "");

  //get linear acceleration
  arr acc, Jacc;
  F_LinVel pos;
  pos.order=2;
  pos.impulseInsteadOfAcceleration=false;
  pos.phi2(acc, Jacc, F);

  acc(2) += gravity;

  //get relative vector
  arr v, Jv;
  F(-2)->C.kinematicsVec(v, Jv, F(-2), vec);

  //torque
  arr torque = crossProduct(v, acc);
  arr Jtorque;
  if(!!J) Jtorque = skew(v) * Jacc - skew(acc) * Jv;

  //compose
  if(torqueOnly) {
    y = torque;
    if(!!J) J = Jtorque;
  } else {
    NIY
  }
}

//===========================================================================

struct F_StaticStability : Feature {
  double margin;
  F_StaticStability(double _margin=.01) : margin(_margin) {}
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi2(const FrameL& F) { return 4; }
};


void F_StaticStability::phi2(arr& y, arr& J, const FrameL& F) {
  NIY;
#if 0
  //get shapes above
  rai::Frame* a = K.frames(i);
  FrameL aboves = getShapesAbove(a);
//  cout <<"ABOVES="<<endl; listWrite(aboves);

  //get average center of all shapes
  arr cog(3), J_cog(3, K.getJointStateDimension());
  cog.setZero(); J_cog.setZero();
  double M=0.;
  for(rai::Frame* b:aboves) if(b!=a) {
      double mass=0.;
      if(b->shape) mass=1.;
      if(b->inertia) mass=b->inertia->mass;
      arr y, J;
      K.kinematicsPos(y, J, b);
      cog += mass*y;
      J_cog += mass*J;
      M += mass;
    }
  CHECK(M>0., "");
  cog  /= M;
  J_cog /= M;

  //align avg with object center
  K.kinematicsPos(y, J, a);
  y = (y-cog)({0, 1});
  if(!!J) J=(J-J_cog)({0, 1});

#if 1
  CHECK(a->shape, "");
  CHECK_EQ(a->shape->type(), rai::ST_ssBox, "the supporting shape needs to be a box");
  arr range = { .5*a->shape->size(0)-margin, .5*a->shape->size(1)-margin };
  arr pos=y, posJ=J;

  y.resize(4);
  y(0) =  pos(0) - range(0);
  y(1) = -pos(0) - range(0);
  y(2) =  pos(1) - range(1);
  y(3) = -pos(1) - range(1);
  if(!!J) {
    J.resize(4, posJ.d1);
    J[0] =  posJ[0];
    J[1] = -posJ[0];
    J[2] =  posJ[1];
    J[3] = -posJ[1];
  }
#endif
#endif
}
