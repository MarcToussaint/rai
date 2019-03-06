#include <Kin/kin.h>

class btRigidBody;

struct BulletInterface{
  struct sBulletInterface *self=0;

  BulletInterface();
  BulletInterface(const rai::KinematicWorld& K);
  ~BulletInterface();

  btRigidBody* addGround();
  btRigidBody* addFrame(const rai::Frame *f);
  void addFrames(const FrameL& frames);
  void defaultInit(const rai::KinematicWorld& K);

  void step(double tau=.01);
  void pushFullState(FrameL& frames, arr& vel);
  void pushKinematicStates(FrameL& frames);
  arr pullDynamicStates(FrameL& frames);

  void saveBulletFile(const char* filename);
};
