#include "modelEnsemble.h"

ModelEnsemble::ModelEnsemble(){
  vert={0.0477157, -0.617799, -0.784887};
}

bool ModelEnsemble::addNewRegionGrowingModel(DataNeighbored& D){
  MinEigModel *model = new MinEigModel(D);

  //-- find a random seed
  uint i;
  for(;;){
    i=rnd(D.n());
    if(D.ok(i) && D.weights(i)*10.<rnd.uni()) break;
//    if(D.ok(i) && D.weights(i)==0.) break;
  }

  //-- initialize with neighborhood of size 400
  model->setPoints(D.getKneighborhood(i, 400));
  model->calc(false);

  //-- expand until fringe is empty
  for(uint k=0;;k++){
    model->expand(10);
    if(!model->fringe.N) break;
    model->calc(true);
    //      model->colorPixelsWithWeights(cols);
    //      gl.update();
  }

  //-- check success
  model->calcDensity();
  model->report();
  if(model->density<40000.){
    delete model;
    return false;
  }

  for(uint i:model->pts) if(D.weights(i)<model->weights(i)) D.weights(i)=model->weights(i);
  models.append(model);
  model->label=models.N;
  return true;
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

void ModelEnsemble::reoptimizeModels(DataNeighbored& D){
  D.weights.setZero();
  for(MinEigModel *m:models){
    m->fringe = m->pts;
    m->expand(10);
    m->setWeightsToZero();
    m->reweightWithError(m->pts);
    m->calc(true);
    m->calcDensity();
    for(uint i:m->pts) if(D.weights(i)<m->weights(i)) D.weights(i)=m->weights(i);
  }
}

void ModelEnsemble::glDraw(OpenGL&){
  for(MinEigModel *m:models){
    glColor(m->label);
    m->glDraw();
  }
}

void ModelEnsemble::report(ostream& os){
  uint c=0;
  for(MinEigModel* m:models){ os <<c++ <<' '; m->report(os, true); }
  os <<"vert=" <<vert <<endl;
}
