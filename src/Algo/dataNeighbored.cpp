#include "dataNeighbored.h"

DataNeighbored::DataNeighbored(const arr& pts):X(pts){
  //decide on ok
  ok.resize(X.d0);
  for(uint i=0;i<X.d0;i++) if(pts(i,2)>=0) ok(i)=true; else ok(i)=false;
  weights.resize(X.d0);
  weights.setZero();
}

uint DataNeighbored::n() const{ return X.d0; }

uint DataNeighbored::d() const{ return X.d1; }

void DataNeighbored::setGridNeighborhood(uint height, uint width){
  CHECK_EQ(width*height, X.d0, "");
  N.resize(X.d0);
  for(uint y=0;y<height;y++) for(uint x=0;x<width;x++){
    uint i=y*width + x, j;
    if(!ok(i)) continue;
    if(y){          j=(y-1)*width+(x  ); if(ok(j)) N(i).append(j); }
    if(x){          j=(y  )*width+(x-1); if(ok(j)) N(i).append(j); }
    if(y<height-1){ j=(y+1)*width+(x  ); if(ok(j)) N(i).append(j); }
    if(x<width-1){  j=(y  )*width+(x+1); if(ok(j)) N(i).append(j); }
  }
}

void DataNeighbored::removeNonOk(){
  uintA index(X.d0);
  index = X.d0;
  int s=0;
  for(uint i=0;i<X.d0;i++) if(ok(i)){ index(i)=s; s++; } //assign new indeces to each point
  idx2pixel.resize(s);
  for(uint i=0;i<X.d0;i++) if(ok(i)){
    uintA& Ni = N(i);
    for(uint& j:Ni) j=index(j); //use new indices in neighborhoods
    Ni.sort();                  //sort neighborhoods
    while(Ni.N && Ni.last()==X.d0) Ni.resizeCopy(Ni.N-1); //remove those, pointing to !ok (==X.d0 index)
  }
  for(uint i=0;i<X.d0;i++) if(ok(i)){
    X[index(i)] = X[i];
    N(index(i)) = N(i);
    idx2pixel(index(i)) = i;
    weights(index(i)) = weights(i);
  }
  X.resizeCopy(s,X.d1);
  N.resizeCopy(s);
  weights.resizeCopy(s);
}

void DataNeighbored::initFringe(uintA& fringe, uintA& pts, boolA& included, uint i){
  fringe.clear();
  fringe.append(i);
  pts = fringe;
  included.resize(X.d0);
  included.setZero();
  included(i) = true;
}

void DataNeighbored::expandFringe(uintA& fringe, uintA& pts, boolA& included){
  uintA newfringe;
  for(uint i:fringe) for(uint j:N(i)){
    if(!included(j)){
      newfringe.append(j);
      pts.append(j);
      included(j)=true;
    }
  }
  fringe = newfringe;
}

uintA DataNeighbored::getKneighborhood(uint i, uint k){
  uintA fringe, pts;
  boolA included;
  initFringe(fringe, pts, included, i);
  uintA Nk;

  for(;fringe.N;){
    if(Nk.N+fringe.N<=k){
      Nk.append(fringe);
      if(Nk.N==k) return Nk;
    }else for(uint j:fringe){
      Nk.append(j);
      if(Nk.N==k) return Nk;
    }

    expandFringe(fringe, pts, included);
  }
  return Nk;
}
