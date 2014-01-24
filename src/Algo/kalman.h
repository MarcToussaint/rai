#include <Core/array.h>

struct Kalman{
  arr b_mean, b_var;
  void initialize(const arr& _b_mean, const arr& _b_var){ b_mean=_b_mean, b_var=_b_var; }
  void stepPredict(const arr& A, const arr& a, const arr& Q);
  void stepObserve(const arr& y, const arr& C, const arr& c, const arr& W);
  void step(const arr& A, const arr& a, const arr& Q, const arr& y, const arr& C, const arr& c, const arr& W);
};

