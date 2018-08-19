/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "PhysXThread.h"
#include <Kin/kin_physx.h>
#include <Kin/kinViewer.h>

struct PhysXThread : Thread {
  VAR(rai::KinematicWorld, modelWorld)
  VAR(rai::KinematicWorld, physxWorld)
  VAR(arr, ctrl_q_ref)
  PhysXInterface *px;
  KinViewer *view;
  OpenGL *gl;
  
  PhysXThread() : Thread("PhysX", .03), px(NULL), view(NULL), gl(NULL) {
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
      rai::Joint *j = physxWorld().joints.elem(i);
      if(j->type==rai::JT_rigid) {
        LOG(0) <<"removing fixed joint '" <<j->type <<':' <<j->from->name <<'-' <<j->to->name <<"' (assuming it is not articulated)";
        delete j;
      }
    }
    physxWorld.deAccess();
#endif
    px = new PhysXInterface(physxWorld.set());
    px->setArticulatedBodiesKinematic();
    view = new KinViewer("physxWorld", .1);
    view->threadLoop();
  }
  
  void step() {
    physxWorld.writeAccess();
    physxWorld().setJointState(ctrl_q_ref.get());
    px->step();
    physxWorld.deAccess();
    if(gl) if(!(step_count%10)) gl->update(NULL, false, false, true);
  }
  
  void close() {
    if(gl) delete gl; gl=NULL;
    if(view) delete view; view=NULL;
    delete px; px=NULL;
  }
  
  void showInternalOpengl() {
    if(!gl) {
      stepMutex.lock();
      gl = new OpenGL("Internal PhyesX display");
      gl->add(glStandardScene);
      gl->add(*px);
      gl->camera.setDefault();
      stepMutex.unlock();
    }
  }
};

Thread* newPhysXThread() { return new PhysXThread(); }

