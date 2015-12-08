#include "modelEnsemble.h"

ModelEnsemble::ModelEnsemble(){
  vert={0.0477157, -0.617799, -0.784887};
}

ModelEnsemble::~ModelEnsemble(){
  listDelete(models);
}

bool ModelEnsemble::addNewRegionGrowingModel(DataNeighbored& data){
  MinEigModel *model = new MinEigModel(data);

//  if(models.N>5) return false;

  //-- find a random seed
  uint i;
  for(uint k=20;k--;){
    i=rnd(data.n());
    if(data.valid(i) && data.weights(i)*10.<rnd.uni()) break;
    if(!k){
      LOG(0) <<"no unused data points found";
      return false;
    }
  }

  //-- initialize with neighborhood of size 400
  model->setPoints(data.getKneighborhood(i, 400));
  model->calc(false);

  //-- expand until fringe is empty
  for(uint k=0;;k++){
    model->expand(10);
    if(!model->fringe.N) break;
    model->calc(true);
  }

  //-- check success
  model->calcDensity();
  model->report();
  if(model->density<.1){
    delete model;
    return false;
  }

  for(uint i:model->pts) if(data.weights(i)<model->weights(i)) data.weights(i)=model->weights(i);
  model->label=models.N;
  models.append(model);
  return true;
}

void ModelEnsemble::reoptimizeModels(DataNeighbored& data){
  data.weights.setZero();
  for(MinEigModel *model:models){
    model->fringe = model->pts;
    model->expand(10);
    model->setWeightsToZero();
    model->reweightWithError(model->pts);
    model->calc(true);
    model->calcDensity();
    if(model->density<.1){ model->setWeightsToZero(); model->density=0.; continue; }
    for(uint i:model->pts) if(data.weights(i)<model->weights(i)) data.weights(i)=model->weights(i);
  }
  for(uint i=models.N;i--;) if(!models(i)->density){ delete models(i);  models.remove(i); }
}

void ModelEnsemble::reestimateVert(){
  for(auto m:models){
    double sp = scalarProduct(m->eig.x_lo,vert);
    if(sp<0.){ m->eig.x_lo *= -1.; sp *=-1; }
    if(sp>.95) m->label=1;
  }

  double n=0.;
  arr mu=zeros(3);
  for(auto m:models) if(m->label==1){
    n += ::sqrt(m->stat_n);
    mu += ::sqrt(m->stat_n) * m->eig.x_lo;
  }
  vert = mu/n;
  vert /= length(vert);
  for(auto m:models) if(m->label==1) m->bias_xx = -1e-1 * (vert^vert);
}

void ModelEnsemble::glDraw(OpenGL&){
  for(MinEigModel *m:models){
    glColor(m->label);
    m->glDraw();
  }
}

void ModelEnsemble::report(ostream& os, bool mini){
  uint c=0;
  for(MinEigModel* m:models){ os <<c++ <<' '; m->report(os, mini); }
  os <<"vert=" <<vert <<endl;
}
