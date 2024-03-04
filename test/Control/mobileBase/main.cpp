#include <Kin/kin.h>
#include <Kin/F_pose.h>
#include <Kin/F_qFeatures.h>

#include <Control/control.h>

//===========================================================================

struct WheelConstraint : Feature{
  WheelConstraint() {
    scale = arr{1e1};
    order = 1;
  }

  virtual void phi2(arr& y, arr& J, const FrameL& F){
    CHECK_EQ(F.nd, 2, "");
//    std::shared_ptr<Feature> f = symbols2feature(FS_position, {wheelCenter}, *Ctuple(-1), {}, {}, 1);
    auto centerVel = F_Position()
                     .setOrder(1)
                     .eval(FrameL({2,1}, {F(0,0), F(1,0)}));

//    std::shared_ptr<Feature> f2 = symbols2feature(FS_vectorX, {wheelCenter}, *Ctuple(-1));
    auto centerX = F_Vector(Vector_x)
                   .eval({F(1,0)});

//    std::shared_ptr<Feature> f3 = symbols2feature(FS_qItself, {wheelJoint}, *Ctuple(-1), {}, {}, 1);
    auto jointVel = F_qItself()
                    .setOrder(1)
                    .eval(FrameL({2,1},{F(0,1), F(1,1)}));

    y = centerVel - centerX * (.05 * jointVel.scalar());
    if(!!J){
      J = centerVel.J() - (centerX.J() * (.05 * jointVel.scalar()) + (centerX.reshape(3,1) * (.05 * jointVel.J())));
    }

  }

  virtual uint dim_phi2(const FrameL& F){ return 3; }
};

//===========================================================================

void controlMobile(){

  rai::Configuration C;
  C.addFile("../../../../rai-robotModels/holoDrive/sketchBase.g");

  C.view(true);

  arr q_last = C.getJointState();

  double tau = .02;

  CtrlSet CS;
  auto c_acc = CS.addControlObjective(2, 1e-4*sqrt(tau), C);
  auto c_vel = CS.addControlObjective(1, 1e-1*sqrt(tau), C);

  auto co = CS.addObjective(make_feature(FS_qItself, {"mobileBase"}, C, {1e1}, {}, 1), OT_sos);

  auto w1 = CS.addObjective(make_feature<WheelConstraint>({"W1_center", "W1_wheelJoint"}, C), OT_eq);
  auto w2 = CS.addObjective(make_feature<WheelConstraint>({"W2_center", "W2_wheelJoint"}, C), OT_eq);
  auto w3 = CS.addObjective(make_feature<WheelConstraint>({"W3_center", "W3_wheelJoint"}, C), OT_eq);
  auto w4 = CS.addObjective(make_feature<WheelConstraint>({"W4_center", "W4_wheelJoint"}, C), OT_eq);

//  ctrl.maxVel=1e1;
  CtrlSolver ctrl(C, tau, 2);

  ofstream fil("z.path");
  for(uint k=0;k<20;k++){
    arr vstar = randn(3);
    vstar(2) *= 5.;
    co->feat->setTarget(vstar);

    for(uint t=0;t<20;t++){
      ctrl.set(CS);
      arr q = C.getJointState();
      ctrl.update(q, {}, C);
      q = ctrl.solve();
      C.setJointState(q);

      C.view(false, STRING("t:" <<t));
      rai::wait(.05);
      arr err = ((C.q-q_last)({0,2})/tau - vstar);
      err.write(fil, 0, 0, "  "); fil<<endl;
      q_last = C.q;

//      ctrl.report();
    }
  }

  C.view(true);
}

//===========================================================================

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  controlMobile();

  return 0;
}

