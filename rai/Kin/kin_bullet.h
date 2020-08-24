/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "kin.h"

class btRigidBody;

struct BulletInterface {
  struct BulletInterface_self* self=0;

  BulletInterface(rai::Configuration& C, int verbose=1);
  ~BulletInterface();

  void step(double tau=.01);

  void pushKinematicStates(const FrameL& frames);
  void pushFullState(const FrameL& frames, const arr& frameVelocities=NoArr);
  void pullDynamicStates(FrameL& frames, arr& frameVelocities=NoArr);

  void changeObjectType(rai::Frame* f, int _type);

  void saveBulletFile(const char* filename);
};
