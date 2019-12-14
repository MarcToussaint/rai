/*  ------------------------------------------------------------------
    Copyright (c) 2019 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include <Kin/kin.h>

class btRigidBody;

struct BulletInterface {
  struct BulletInterface_self* self=0;

  BulletInterface(bool verbose=false);
  BulletInterface(rai::Configuration& K, bool verbose=false);
  ~BulletInterface();

  btRigidBody* addGround();
  btRigidBody* addFrame(rai::Frame* f, bool verbose);
  void addFrames(FrameL& frames, bool verbose);
  void defaultInit(rai::Configuration& K, bool verbose);

  void step(double tau=.01);
  void pushFullState(const FrameL& frames, const arr& vel);
  void pushKinematicStates(const FrameL& frames);
  void pullDynamicStates(FrameL& frames, arr& vel=NoArr);

  void saveBulletFile(const char* filename);
};
