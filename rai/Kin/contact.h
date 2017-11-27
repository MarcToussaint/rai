#pragma once

#include <Core/util.h>
#include "frame.h"
#include "taskMap.h"

namespace mlr{

///Description of a Contact
struct Contact : GLDrawer {
  Frame &a, &b;
  mlr::Vector a_rel, b_rel;   // contact point RELATIVE to the frames
  mlr::Vector a_norm, b_norm; // normals RELATIVE to the frames, pointing AWAY from the object
  double a_rad, b_rad;        // the radii for sphere-swept objects: the contact points are on the cvx CORE, not the surface!
  uint a_type, b_type;
  mlr::Vector a_line, b_line; // when of line type, these are the line/axis directions RELATIVE to the frame

  double y;                   // place to store the constraint value (typically: neg distance) when the taskmap is called
  double lagrangeParameter;   // place to store the respective lagrange parameter after an optimization

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

  double getDistance(); //calls the task map!
  TaskMap* getTM_Friction();
  TaskMap* getTM_ContactNegDistance();
  void glDraw(OpenGL&);
};

struct TM_ContactNegDistance : TaskMap {
  Contact& C;

  TM_ContactNegDistance(Contact& contact) : C(contact){}

  void phi(arr& y, arr& J, const mlr::KinematicWorld& K, int t=-1);
  virtual uint dim_phi(const mlr::KinematicWorld& K){ return 1; }
  virtual mlr::String shortTag(const mlr::KinematicWorld& K){ return STRING("ContactNegDistance-"<<C.a.name<<'-'<<C.b.name); }
};

} //mlr
