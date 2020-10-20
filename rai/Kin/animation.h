#include "kin.h"
#include "viewer.h"

namespace rai {

struct Animation{
    struct AnimationPart{
      uintA frameIDs;
      arr X;
      void write(ostream& os) const;
      void read(istream& is);
    };

  rai::Array<AnimationPart> A;

  void write(ostream& os) const;
  void read(istream& is);

  uint getT(){
      uint T=0;
      for(auto& a: A) if(a.X.d0>T) T=a.X.d0;
      return T;
  }

  void set_t(rai::Configuration& C, uint t){
    for(auto& a: A){
      if(a.X.d0>t) C.setFrameState(a.X[t], C.getFrames(a.frameIDs));
    }
  }

  void play(rai::Configuration& C){
    rai::ConfigurationViewer V;
    V.setConfiguration(C);
    uint T=getT();

    for(uint t=0;t<T;t++){
      set_t(C, t);
      V.setConfiguration(C, STRING("Animation t:" <<t));
      rai::wait(.1);
    }
  }
};
stdPipes(Animation::AnimationPart)
stdPipes(Animation)

}
