/*  ---------------------------------------------------------------------
    Copyright 2014 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */

#include "teleop2tasks.h"
#include <Motion/taskMaps.h>
#include <Hardware/gamepad/gamepad.h>

Teleop2Tasks::Teleop2Tasks(TaskController& _MP):fmc(_MP){
  effPosR = fmc.addPDTask("MoveEffTo_endeffR", .2, 1.8,new TaskMap_Default(posTMT, fmc.world,"endeffR",NoVector,"base_footprint"));
  effPosR->y_ref = {0.8, -.5, 1.};

  effPosL = fmc.addPDTask("MoveEffTo_endeffL", .2, 1.8,new TaskMap_Default(posTMT,fmc.world,"endeffL",NoVector,"base_footprint"));
  effPosL->y_ref = {0.8, .5, 1.};

  fc = fmc.addPDTask("fc_endeffL", .2, 1.8,new TaskMap_Default(posTMT,fmc.world, "endeffForceL",NoVector,"base_footprint"));
  fc->y_ref ={0.8,0.5,1.};
  fc->f_ref = {15.,15.,15.};
  fc->f_alpha = .075;
  fc->active = true;

  int jointID = fmc.world.getJointByName("r_gripper_joint")->qIndex;
  gripperR = fmc.addPDTask("gripperR", .3, 1.8, new TaskMap_qItself(jointID, fmc.world.q.N));
  gripperR->setTarget({0.01});
    //gripperR->y_ref = {.08};  // open gripper 8cm

  jointID = fmc.world.getJointByName("l_gripper_joint")->qIndex;
  gripperL = fmc.addPDTask("gripperL", .3, 1.8, new TaskMap_qItself(jointID, fmc.world.q.N));
  gripperL->setTarget({0.01});
  //gripperL->y_ref = {.08};  // open gripper 8cm

  effOrientationR = fmc.addPDTask("orientationR", .2, 1.8,new TaskMap_Default(quatTMT,fmc.world, "endeffR"));
  effOrientationR->y_ref = {1., 0., 0., 0.};
  effOrientationR->flipTargetSignOnNegScalarProduct = true;

  effOrientationL = fmc.addPDTask("orientationL", .2,1.8,new TaskMap_Default(quatTMT,fmc.world, "endeffL"));
  effOrientationL->y_ref = {1., 0., 0., 0.};
  effOrientationL->flipTargetSignOnNegScalarProduct = true;

  base = fmc.addPDTask("basepos", .2,.8,new TaskMap_qItself(fmc.world, "worldTranslationRotation"));
  base->y_ref={0.,0.,0.};
  base->active =false;
}

mlr::Array<CtrlTask*> Teleop2Tasks::getTasks(){
  return { effPosR, gripperR, effOrientationR, effPosL, gripperL, effOrientationL, base }; //, fc
}

void Teleop2Tasks::deactivateTasks(){
  effPosR->active = false;
  effPosL->active = false;
  effOrientationR->active = false;
  effOrientationL->active = false;
  gripperL->active = false;
  gripperR->active = false;
  fc->active = false;
  base->active =false;

}


void Teleop2Tasks::updateMovement(floatA& cal_pose, arr& old_pos, arr& old_effpos, CtrlTask *effPos){
  arr pos, pos_div;

  //get positiondata
  copy(pos, cal_pose.sub(0,2));
  pos += ARR(0.6, 0., 1.);


  //calculate difference in position
  copy(pos_div,pos);
  pos_div -= old_pos;

  //reset positions if no movement on the axis allowed
  if(!move_x){
    pos_div += ARR(-pos_div(0),0,0);
  }
  if(!move_y){
    pos_div += ARR(0,-pos_div(1),0);
  }
  if(!move_z){
    pos_div += ARR(0,0,-pos_div(2));
  }
  old_effpos +=pos_div;

  if(effPos){
    effPos->setTarget(old_effpos);
//    effPos->setCompliance( c_in_this_direction = 1., c_in_other_directions=.1, direction=ARR(.7,.7,0) );
  }
  copy(old_pos, pos);
}


void Teleop2Tasks::updateTasks(floatA cal_pose_rh, floatA cal_pose_lh, float calibrated_gripper_lh, float calibrated_gripper_rh, arr drive, int button){

  effPosR->active = true;
  effPosL->active = false;
  effOrientationR->active = true;
  effOrientationL->active = false;
  gripperL->active = false;
  gripperR->active = true;
  fc->active = true;
  base->active =false;



  //check if button has changed
  if(button != old_button){
    //toggle movement restrictions
    if(button & BTN_Y){
      move_y = !move_y;
      printf("Move along Y-Axis: %s\n", move_y ? "allowed" : "not allowed");
    }else if(button & BTN_B){
      move_x = !move_x;
      printf("Move along X-Axis: %s\n", move_x ? "allowed" : "not allowed");
    }else if(button & BTN_A){
      move_z = !move_z;
      printf("Move along Z-Axis: %s\n", move_z ? "allowed" : "not allowed");
    }else if(button & BTN_X){
      rotate = !rotate;
      printf("Hand rotation: %s\n", rotate ? "allowed" : "not allowed");
    }
   old_button=button;
  }

  // set hand position
  arr pos, quat, pos_div;

  //set an inital value to all of the old_* variables
  if(!initialised){
    copy(old_pos_rh, cal_pose_rh.sub(0,2));
    copy(old_pos_lh, cal_pose_lh.sub(0,2));
    old_pos_rh += ARR(0.6, 0., 1.);
    old_pos_lh += ARR(0.6, 0., 1.);
    copy(old_effpos_r, old_pos_rh);
    copy(old_effpos_l, old_pos_lh);
    initialised = true;
  }

  ors::Quaternion orsquats = fmc.world.getShapeByName("endeffBase") -> X.rot;
//  ors::Joint *trans = fmc.world.getJointByName("worldTranslationRotation");
//  orsquats.setRad( q(trans->qIndex+2),{0.,0.,1.} );
  ors::Quaternion orsquatsacc;

  //update the movement of the right Hand
  updateMovement(cal_pose_rh, old_pos_rh, old_effpos_r, effPosR);

  //orientation
  orsquatsacc.set(
      (double)cal_pose_rh(3),
      (double)cal_pose_rh(4),
      (double)cal_pose_rh(5),
      (double)cal_pose_rh(6));
  quat = conv_quat2arr(orsquats * orsquatsacc);
  if(rotate){
    if(effOrientationR) effOrientationR->setTarget(quat);
  }



  //update the movement of the left Hand
  updateMovement(cal_pose_lh, old_pos_lh, old_effpos_l, effPosL);

  // orientation
  orsquatsacc.set(
      (double)cal_pose_lh(3),
      (double)cal_pose_lh(4),
      (double)cal_pose_lh(5),
      (double)cal_pose_lh(6));
  quat = conv_quat2arr(orsquats * orsquatsacc);
  if(effOrientationL) effOrientationL->setTarget(quat);

  //gripper
  double cal_gripper;
  cal_gripper =  calibrated_gripper_rh;
  if(gripperR) gripperR->setTarget({cal_gripper});
  cal_gripper =  calibrated_gripper_lh;
  if(gripperL) gripperL->setTarget({cal_gripper});

  //base movement
  arr drive_des;
  double y_c,x_c,phi_c;
  ors::Joint *trans = fmc.world.getJointByName("worldTranslationRotation");
  x_c = base->y_ref(trans->qIndex+0);
  y_c = base->y_ref(trans->qIndex+1);
  phi_c = base->y_ref(trans->qIndex+2);

  if(false) //drive indicator
  {
      drive_des = drive;
      x_c += drive_des(0)*cos(phi_c) - drive_des(1)*sin(phi_c);
      y_c += drive_des(0)*sin(phi_c) + drive_des(1)*cos(phi_c);
      phi_c += drive_des(2);
  }


  base->setTarget({x_c,y_c,phi_c});
  fmc.qNullCostRef.y_ref(trans->qIndex+0) = base->y_ref(trans->qIndex+0);
  fmc.qNullCostRef.y_ref(trans->qIndex+1) = base->y_ref(trans->qIndex+1);
  fmc.qNullCostRef.y_ref(trans->qIndex+2) = base->y_ref(trans->qIndex+2);

}

