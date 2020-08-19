/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifndef GRAVITYCOMPENSATION_H
#define GRAVITYCOMPENSATION_H

#include <map>

#include "../Core/array.h"
#include "../Kin/kin.h"
#include "../Kin/TM_default.h"

struct GravityCompensation {

  struct CV;

  rai::Configuration world;

  arr TLeftArm, TRightArm, THead;
  StringA leftJoints = {"l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint", "l_elbow_flex_joint",
                        "l_forearm_roll_joint", "l_wrist_flex_joint"
                       };

  StringA rightJoints = {"r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_upper_arm_roll_joint", "r_elbow_flex_joint",
                         "r_forearm_roll_joint", "r_wrist_flex_joint"
                        };

  StringA headJoints = {"head_tilt_joint"};

  std::map<rai::String, arr> betasGC;
  arr beta_l_shoulder_pan_joint, beta_l_shoulder_lift_joint, beta_l_upper_arm_roll_joint, beta_l_elbow_flex_joint, beta_l_forearm_roll_joint, beta_l_wrist_flex_joint;
  arr betaFTL, betaFTR;

  void learnGCModel();

  arr compensate(const arr& q, const arr& qSign, const StringA& joints);

  void learnFTModel();
  arr compensateFTL(const arr& q);
  arr compensateFTR(const arr& q);

  GravityCompensation(const rai::Configuration& world);

  arr featuresGC(arr q, arr qSign, const rai::String& joint);

  arr featuresFT(arr q, rai::String endeff);
  arr generateTaskMapFeature(TM_Default map, arr Q);

  void testForLimits();
  void removeLimits();

#if 0
  enum RobotPart {
    leftArm,
    rightArm,
    head
  };

  arr TLeftArm, TRightArm, THead;

  arr betaLeftArm, betaRightArm, betaHead;

  bool modelLearned  = false;

  StringA leftJoints = {"l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint", "l_elbow_flex_joint",
                        "l_forearm_roll_joint", "l_wrist_flex_joint"
                       };

  StringA rightJoints = {"r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_upper_arm_roll_joint", "r_elbow_flex_joint",
                         "r_forearm_roll_joint", "r_wrist_flex_joint"
                        };

  StringA headJoints = {"head_tilt_joint"};//,"head_pan_joint"};

  arr features(arr Q, const RobotPart robotPart);
  void learnModels(bool verbose);
  void saveBetas();
  void loadBetas();
  arr compensate(arr q, bool compensateLeftArm, bool compensateRightArm, bool compensateHead);
  arr compensate(arr q, StringA joints);

  GravityCompensation(const rai::Configuration& world);

  //for debugging
  void testForLimits();

  //========= Helper functions, just convenience and good practices
  arr makeQMatrix(arr Q, uint jointIndex);
  arr generateTaskMapFeature(TM_Default map, arr Q);
  void generatePredictionsOnDataSet(const arr& Q, const arr& U, const StringA& joints);
#endif
};

#endif // GRAVITYCOMPENSATION_H
