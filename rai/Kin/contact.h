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

namespace mlr{

///Description of a Contact
struct Contact : GLDrawer {
  Frame &a, &b;
  arr a_pts, b_pts;          // points on the core mesh that define the contact simplices
//  mlr::Vector a_rel, b_rel;    // contact point RELATIVE to the frames
  double a_rad, b_rad;         // the radii for sphere-swept objects: the contact points are on the cvx CORE, not the surface!
//  uint a_type, b_type;
//  mlr::Vector a_line, b_line;  // when of line type, these are the line/axis directions RELATIVE to the frame

  double y=0.;                 // place to store the constraint value (typically: neg distance) when the taskmap is called
  double lagrangeParameter=0.; // place to store the respective lagrange parameter after an optimization

  Contact(Frame &a, Frame &b)
      : a(a), b(b) {
    CHECK(&a != &b,"");
    a.contacts.append(this);
    b.contacts.append(this);
  }
  ~Contact(){
    a.contacts.removeValue(this);
    b.contacts.removeValue(this);
  }

//  mlr::Vector get_pa() const{ return a.X * (a_rel + a_rad*a_norm); }
//  mlr::Vector get_pb() const{ return b.X * (b_rel + b_rad*b_norm); }
//  mlr::Vector get_norm() const{ return .5 * (a.X.rot * a_norm + b.X.rot * b_norm); }
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

  TM_ContactNegDistance(const Contact& contact) : C(contact){}

  void phi(arr& y, arr& J, const mlr::KinematicWorld& K, int t=-1);
  virtual uint dim_phi(const mlr::KinematicWorld& K){ return 1; }
  virtual mlr::String shortTag(const mlr::KinematicWorld& K){ return STRING("ContactNegDistance-"<<C.a.name<<'-'<<C.b.name); }
};

} //mlr
