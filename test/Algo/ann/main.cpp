#include <Core/util.h>
#include <Algo/ann.h>

void TEST(ANN) {
  uint N=1000,dim=2;

  ANN ann;
  arr x(dim),X(N,dim),Y;
  uintA idx;
  arr dists;

  rndUniform(X,0.,1.,false);

  rai::timerStart();
  ann.setX(X);
  rndUniform(x,0.,1.,false); //x=.5;
  ann.getkNN(dists,idx,x,10,.0, true);
  Y.resize(idx.N,x.N);
  for(uint i=0;i<Y.d0;i++) Y[i] = X[idx(i)];
  std::cout <<"build time (#" <<ann.X.N <<") = " <<rai::timerRead() <<"sec" <<std::endl;
  FILE("z.data") <<X.modRaw();
  FILE("z.query") <<x.reshape(1,x.N).modRaw();
  FILE("z.neighbors") <<Y.modRaw();
  gnuplot("set size square; plot 'z.data' w p,'z.query' w p,'z.neighbors' w p");
  rai::wait();
}

void TEST(ANNIncremental) {
  uint N=1000,dim=2;

  ANN ann;
  ann.bufferSize=20;
  arr x(dim),q(dim),Q;
  intA idx;
  
  rndUniform(q,0.,1.,false); //constant query point
  for(uint i=0;i<N;i++){
    rndUniform(x,0.,1.,false);
    ann.append(x);
    if(i>10){
      ann.getkNN(Q,q,10,.0, true);
      //rai::wait();
    }
  }
}

/*void TEST(ANNregression){
  arr X,Y,Z;
  uint i,j;
  X.setGrid(1,-3.,3.,100);
  Y.resize(X.d0,1);
  for(i=0;i<X.N;i++) Y(i,0)=::sin(X(i,0));
  write(X,Y,"z.Y");
  gnuplot("plot 'z.Y'");

  ANN ann;
  arr x,y;

  for(i=0;i<500;i++){
    j=rnd(X.N);
    x=X[j]; rndGauss(x,.01,true);
    y=Y[j]; rndGauss(y,.1,true);
    ann.learn(x,y);
    //ann.map(X,Z);
    if(!(i%20)){
      ann.map(X,Z);
      write(X,Z,"z.Z");
      ann.X >>FILE("z.X");
      gnuplot("plot 'z.X' with points,'z.Y','z.Z'");
    }
  }

}*/

int MAIN(int argc,char** argv){
  rai::initCmdLine(argc, argv);

  testANN();
  testANNIncremental();
  //testANNregression();

  return 0;
}
