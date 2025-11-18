#include "m_CMA.h"

extern "C" {
#include "CMA/cmaes_interface.h" //by Nikolaus Hansen
}

namespace rai {

struct CMA_self {
  cmaes_t evo;
};

CMAES::CMAES(shared_ptr<NLP> P, const arr& x_init, shared_ptr<OptOptions> opt) : GenericBO("cmaes", P, x_init, opt) {//EvolutionStrategy("cmaes", f, x_init, opt) {
  self = make_unique<CMA_self>();
  arr startDev = rai::consts<double>(sigmaInit, best_x.N);
  int seed = rnd.uni_int(1, INT_MAX);
  cmaes_init(&self->evo, x_init.N, x_init.p, startDev.p, seed, lambda, nullptr);

  if(opt->verbose>0) cout <<"--cmaes-- " <<evals <<" (seeded with: " <<seed <<")" <<std::endl;
}

CMAES::~CMAES() {
  cmaes_exit(&self->evo);
  rai::system("rm -f errcmaes.err actparcmaes.par");
}

arr CMAES::generateSamples(){
  CHECK_EQ(best_x.N, uint(self->evo.sp.N), "")
  double* const* rgx = cmaes_SamplePopulation(&self->evo);

  // arr samples(self->evo.sp.lambda, self->evo.sp.N);
  // for(uint i=0; i<samples.d0; i++) samples[i].setCarray(rgx[i], samples.d1);

  arr samples;
  samples.setCarray(rgx, self->evo.sp.lambda, self->evo.sp.N);

  return samples;
}

void CMAES::overwriteSamples(const arr& Xnew){
  double **sam = self->evo.rgrgx;
  CHECK_EQ((int)Xnew.d0, self->evo.sp.lambda, "");
  for(uint i=0;i<Xnew.d0;i++) for(uint j=0;j<Xnew.d1;j++) sam[i][j] = Xnew(i,j);
}

void CMAES::overwriteMean(const arr& x){
  arr ref(self->evo.rgxmean, self->evo.sp.N, true);
  ref = x;
}

void CMAES::update(arr& X, arr& Phi, arr& F){
  CHECK_EQ((int)F.N, self->evo.sp.lambda, "")
  cmaes_UpdateDistribution(&self->evo, F.p);
}

arr CMAES::getBestEver() {
  return arr(self->evo.rgxbestever, self->evo.sp.N, false);
}

arr CMAES::getCurrentMean() {
  return arr(self->evo.rgxmean, self->evo.sp.N, false);
}

double CMAES::getSigma() {
  return self->evo.sigma;
}

}
