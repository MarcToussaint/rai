/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "PhysXThread.h"
#include "kin_physx.h"
#include "kinViewer.h"

struct PhysXThread : Thread {
  Var<rai::Configuration> modelWorld;
  Var<rai::Configuration> physxWorld;
  Var<arr> ctrl_q_ref;
  PhysXInterface* px;
  KinViewer* view;
  OpenGL* gl;

  PhysXThread() : Thread("PhysX", .03), px(nullptr), view(nullptr), gl(nullptr) {
    threadLoop(true);
  }

  ~PhysXThread() {
    threadClose();
  }

  void open() {
#if 0
    physxWorld.writeAccess();
    physxWorld() = modelWorld.get();
    for(uint i=physxWorld().joints.N; i--;) {
      rai::Joint* j = physxWorld().joints.elem(i);
      if(j->type==rai::JT_rigid) {
        LOG(0) <<"removing fixed joint '" <<j->type <<':' <<j->from->name <<'-' <<j->to->name <<"' (assuming it is not articulated)";
        delete j;
      }
    }
    physxWorld.deAccess();
#endif
    px = new PhysXInterface(physxWorld.get());
    px->setArticulatedBodiesKinematic(physxWorld.get());
    view = new KinViewer(Var<rai::Configuration>()); NIY //("physxWorld", .1);
    view->threadLoop();
  }

  void step() {
    physxWorld.writeAccess();
    physxWorld().setJointState(ctrl_q_ref.get());
    px->step();
    physxWorld.deAccess();
    if(gl) if(!(step_count%10)) gl->update(nullptr, true);
  }

  void close() {
    if(gl) delete gl;
    if(view) delete view;
    delete px;
  }

  void showInternalOpengl() {
    if(!gl) {
      stepMutex.lock(RAI_HERE);
      gl = new OpenGL("Internal PhyesX display");
      gl->add(glStandardScene);
      gl->add(*px);
      gl->camera.setDefault();
      stepMutex.unlock();
    }
  }
};

Thread* newPhysXThread() { return new PhysXThread(); }

