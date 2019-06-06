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
  void pushFullState(const FrameL& frames, const arr& vel);
  void pushKinematicStates(const FrameL& frames);
  void pullDynamicStates(FrameL& frames, arr& vel=NoArr);

  void saveBulletFile(const char* filename);
};
