#include "PhysXThread.h"
#include <Kin/kin_physx.h>
#include <Kin/kinViewer.h>

struct PhysXThread : Thread{
  ACCESS(mlr::KinematicWorld, modelWorld)
  ACCESS(mlr::KinematicWorld, physxWorld)
  ACCESS(arr, ctrl_q_ref)
  PhysXInterface *px;
  OrsViewer *view;
  OpenGL *gl;

  PhysXThread() : Thread("PhysX", .03), px(NULL), view(NULL), gl(NULL){
    threadLoop(true);
  }

  ~PhysXThread(){
    threadClose();
  }

  void open(){
    physxWorld.writeAccess();
    physxWorld() = modelWorld.get();
    for(uint i=physxWorld().joints.N;i--;){
      mlr::Joint *j = physxWorld().joints.elem(i);
      if(j->type==mlr::JT_rigid){
        LOG(0) <<"removing fixed joint '" <<j->tag() <<"' (assuming it is not articulated)";
        delete j;
      }
    }
    physxWorld.deAccess();
    px = new PhysXInterface(physxWorld.set());
    px->setArticulatedBodiesKinematic();
    view = new OrsViewer("physxWorld", .1);
    view->threadLoop();
   }

  void step(){
    physxWorld.writeAccess();
    physxWorld().setJointState(ctrl_q_ref.get());
    px->step();
    physxWorld.deAccess();
    if(gl) if(!(step_count%10)) gl->update(NULL, false, false, true);
  }

  void close(){
    if(gl) delete gl; gl=NULL;
    if(view) delete view; view=NULL;
    delete px; px=NULL;
  }

  void showInternalOpengl(){
    if(!gl){
      stepMutex.lock();
      gl = new OpenGL("Internal PhyesX display");
      gl->add(glStandardScene);
      gl->add(*px);
      gl->camera.setDefault();
      stepMutex.unlock();
    }
  }
};

Thread* newPhysXThread(){ return new PhysXThread(); }

