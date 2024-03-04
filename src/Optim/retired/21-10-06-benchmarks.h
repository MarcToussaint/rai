/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

struct ParticleAroundWalls2 : KOMO_Problem {
  //options of the problem
  uint T, k, n;
  bool useKernel;
  arr x;

  ParticleAroundWalls2():
    T(rai::getParameter<uint>("opt/ParticleAroundWalls/T", 1000)),
    k(rai::getParameter<uint>("opt/ParticleAroundWalls/k", 2)),
    n(rai::getParameter<uint>("opt/ParticleAroundWalls/n", 3)),
    useKernel(false) {}

  //implementations of the kOrderMarkov virtuals
  uint get_T() { return T; }
  uint get_k() { return k; }
  void getStructure(uintA& variableDimensions, uintA& featureTimes, ObjectiveTypeA& featureTypes);

  void phi(arr& phi, arrA& J, arrA& H, uintA& featureTimes, ObjectiveTypeA& tt, const arr& x);
};
