#include <Optim/MathematicalProgram.h>

struct IpoptInterface {
  MathematicalProgram_Logged P;
//  MathematicalProgram& P;

  IpoptInterface(MathematicalProgram& P) : P(P){}

  void solve();
};

