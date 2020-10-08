#include <Kin/kin.h>
#include <Kin/viewer.h>
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

    y = centerVel.y - centerX.y * (.05 * jointVel.y.scalar());
    if(!!J){
      J = centerVel.J - (centerX.J * (.05 * jointVel.y.scalar()) + (centerX.y.reshape(3,1) * (.05 * jointVel.J)));
    }

  }

  virtual uint dim_phi2(const FrameL& F){ return 3; }
};

//===========================================================================

void controlMobile(){

  rai::Configuration C;
  C.addFile("../../../../rai-robotModels/scenarios/mobileBase.g");

  rai::ConfigurationViewer V;
  V.setConfiguration(C, "", true);

  arr q_last = C.getJointState();

  double tau = .02;

  CtrlProblem ctrl(C, tau, 2);
  auto c_acc = ctrl.add_qControlObjective(2, 1e-4);
  auto c_vel = ctrl.add_qControlObjective(1, 1e-1);

  auto co = ctrl.addObjective(FS_qItself, {"mobileBase"}, OT_sos, {1e1}, {}, 1);

  auto w1 = ctrl.addObjective(make_shared<WheelConstraint>(), {"W1_center", "W1_wheelJoint"}, OT_eq);
  auto w2 = ctrl.addObjective(make_shared<WheelConstraint>(), {"W2_center", "W2_wheelJoint"}, OT_eq);
  auto w3 = ctrl.addObjective(make_shared<WheelConstraint>(), {"W3_center", "W3_wheelJoint"}, OT_eq);
  auto w4 = ctrl.addObjective(make_shared<WheelConstraint>(), {"W4_center", "W4_wheelJoint"}, OT_eq);

//  ctrl.maxVel=1e1;

  ofstream fil("z.path");
  for(uint k=0;k<20;k++){
    arr vstar = randn(3);
    vstar(2) *= 5.;
    co->feat->setTarget(vstar);

    for(uint t=0;t<20;t++){
      ctrl.update(C);
      arr q = ctrl.solve();
      C.setJointState(q);

      V.setConfiguration(C, STRING("t:" <<t), false);
      rai::wait(.05);
      arr err = ((C.q-q_last)({0,2})/tau - vstar);
      err.write(fil, 0, 0, "  "); fil<<endl;
      q_last = C.q;

//      ctrl.report();
    }
  }

  V.setConfiguration(C,"", true);
}

//===========================================================================


int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  controlMobile();

  return 0;
}

