/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include <Core/util.h>
#include "frame.h"
#include "taskMap.h"

struct PairCollision;

namespace rai {

///Description of a Contact
struct Contact : GLDrawer {
  Frame &a, &b;

  PairCollision *__coll=0;

  uint dim=3;
  uint qIndex=UINT_MAX;

//  arr a_pts, b_pts;          // points on the core mesh that define the contact simplices
  rai::Vector a_rel, b_rel;    // contact point RELATIVE to the frames
  rai::Vector a_norm, b_norm;    // contact point RELATIVE to the frames
  double a_rad, b_rad;         // the radii for sphere-swept objects: the contact points are on the cvx CORE, not the surface!
  uint a_type, b_type;
//  rai::Vector a_line, b_line;  // when of line type, these are the line/axis directions RELATIVE to the frame

  arr force;

  double y=0.;                 // place to store the constraint value (typically: neg distance) when the taskmap is called
//  double lagrangeParameter=0.; // place to store the respective lagrange parameter after an optimization

  Contact(Frame &a, Frame &b, Contact *copyContact=NULL);
  ~Contact();

  void setZero();
  uint qDim() { return dim; }
  void calc_F_from_q(const arr& q, uint n);
  arr calc_q_from_F() const { return force; }

  PairCollision *coll();

  void setFromPairCollision(PairCollision& col);
//  rai::Vector get_pa() const{ return a.X * (a_rel + a_rad*a_norm); }
//  rai::Vector get_pb() const{ return b.X * (b_rel + b_rad*b_norm); }
//  rai::Vector get_norm() const{ return .5 * (a.X.rot * a_norm + b.X.rot * b_norm); }
//  double get_pDistance() const{ return (get_pa()-get_pb()).length(); } // get distance between the FIXED contacted points p_a and p_b
//  double get_tangentDistance() const{
//    arr n = get_norm().getArr();
//    return length((eye(3) - (n^n)) * (get_pa()-get_pb()).getArr());
//  }

  double getDistance() const; // get normal(!) distance (projected onto contact normal), by calling the TM_ContactNegDistance()
  TaskMap* getTM_Friction() const;
  TaskMap* getTM_ContactNegDistance() const;
  void glDraw(OpenGL&);
  void write(ostream& os) const;
};
stdOutPipe(Contact)

struct TM_ContactNegDistance : TaskMap {
  const Contact& C;
  
  TM_ContactNegDistance(const Contact& contact) : C(contact) {}
  
  void phi(arr& y, arr& J, const rai::KinematicWorld& K);
  virtual uint dim_phi(const rai::KinematicWorld& K) { return 1; }
  virtual rai::String shortTag(const rai::KinematicWorld& K) { return STRING("ContactNegDistance-"<<C.a.name<<'-'<<C.b.name); }
};

} //rai
