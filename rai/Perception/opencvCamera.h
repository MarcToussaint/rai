#include <Core/thread.h>

struct OpencvCamera : Thread {
  ptr<struct sOpencvCamera> s;
  Var<byteA> rgb;
  std::map<int,double> properties; bool set(int prop, double status);
  OpencvCamera(const Var<byteA>& _rgb);
  ~OpencvCamera();
  void open();
  void step();
  void close();
};
