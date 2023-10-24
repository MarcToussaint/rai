/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

struct ParticleAroundWalls : KOrderMarkovFunction {
  //options of the problem
  uint T, k, n;
  bool useKernel;
  arr x;

  ParticleAroundWalls():
    T(rai::getParameter<uint>("opt/ParticleAroundWalls/T", 1000)),
    k(rai::getParameter<uint>("opt/ParticleAroundWalls/k", 2)),
    n(rai::getParameter<uint>("opt/ParticleAroundWalls/n", 3)),
    useKernel(false) {}

  //implementations of the kOrderMarkov virtuals
  void set_x(const arr& _x) { x=_x; x.reshape(T+1, n); }
  void phi_t(arr& phi, arr& J, ObjectiveTypeA& tt, uint t);
  uint get_T() { return T; }
  uint get_k() { return k; }
  uint dim_x(uint t) { return n; }
  uint dim_phi(uint t);
  uint dim_g(uint t);

  bool hasKernel() { return useKernel; }
  double kernel(uint t0, uint t1) {
    //if(t0==t1) return 1e3;
    return 1e0*::exp(-.001*rai::sqr((double)t0-t1));
  }
};
