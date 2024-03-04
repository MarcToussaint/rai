#include <Algo/spline.h>
#include <Gui/plot.h>
#include <Kin/kin.h>

//==============================================================================

void plotIt(rai::BSpline& S){
  ofstream fil("z.test");
  for(double t=S.begin()-.1;t<=S.end()+.1;t+=.001){
    arr x, xDot, xDDot;
    S.eval(x, xDot, xDDot, t);
    fil <<t <<' ' <<x.modRaw() <<' ' <<xDot.modRaw() <<' ' <<xDDot.modRaw() <<endl;
//    fil <<t <<' ' <<S.eval(t,2).modRaw() <<endl;
  }
  fil.close();
  gnuplot("set size square; set grid; plot [:][-.2:1.2] 'z.test' us 1:2, 3-x, 1+x, x-.5");
  rai::wait();
}

void TEST(Basics){
  rai::BSpline S;
  arr X = {1., 0., 0., 1.};  X.reshape(-1,1);
  arr T = {0., .5, .5, 1.};
  arr vel = {1.};
  S.set(3, X, T);

  cout <<"\ntimes = " <<S.knotTimes <<endl;
  cout <<"points = " <<~S.ctrlPoints <<endl;
  plotIt(S);

  S.append(arr{0., 1.}.reshape(-1,1), {.5, 1.}, true);
  cout <<"\ntimes = " <<S.knotTimes <<endl;
  cout <<"points = " <<~S.ctrlPoints <<endl;
  plotIt(S);

  S.setDoubleKnotVel(-1, vel);
  S.setDoubleKnotVel(1, vel);
  S.setDoubleKnotVel(5, -vel);

  cout <<"\ntimes = " <<S.knotTimes <<endl;
  cout <<"points = " <<~S.ctrlPoints <<endl;
  plotIt(S);

}

//==============================================================================

void TEST(Basis){
  rai::BSpline S;

  uint n=300;
  for(uint deg=0;deg<5;deg++){
    if(deg) S.setUniform(deg, deg+deg%2);
    else{ S.degree=0; S.knotTimes={0., 1.}; S.ctrlPoints=zeros(1,1); }
    //S.knotTimes = ::range(0.,1.,S.knotTimes.N-1);
    arr B = S.getGridBasis(n);

    FILE("z.dat") <<B.modRaw() <<endl;

    rai::String cmd;
    cmd <<"set style data lines;\n";
    cmd <<"set key off;\n";
    cmd <<"set xtics (";
    for(uint i=0;i<S.knotTimes.N;i++) cmd <<"\"t_{" <<i <<"}\" " <<S.knotTimes(i) <<",";
    cmd <<");\n";
    cmd <<"plot [-.05:1.05][0:1.05] 'z.dat' us ($0/" <<n <<"):1";
    for(uint i=1;i<B.d1;i++) cmd <<", '' us ($0/" <<n <<"):" <<i+1;
    cout <<cmd <<endl;
    gnuplot(cmd);
    rai::wait();
  }

}

//==============================================================================

void TEST(Speed){

  uint N=1000000, n=2;
  arr X = randn(N, n);
  arr T = integral(rand(N)+0.1);
//  cout <<X <<endl <<T <<endl;

  rai::BSpline S;
  S.set(2, X, T);

  rai::timerStart();
  double a=S.begin()-1., b=S.end()+1.;
  for(double t=a;t<b;t+= 1e-3*(b-a)){
//    cout <<t <<" : " <<S.eval(t,2) <<endl;
    S.eval(t,2);
  }
  cout <<"time: " <<rai::timerRead() <<endl;
}

//==============================================================================

void TEST(Path){
  arr X(11,1);
  rndUniform(X,-1,1,false);

  rai::Path P(X,2);
  cout <<"times = " <<P.knotTimes
      <<"\npoints= " <<P.ctrlPoints <<endl;

  //-- gradient check of velocity
  fct Test = [&P](const arr& x) -> arr {
    CHECK_EQ(x.N,1,"");
    arr y = P.getPosition(x(0));
    y.J() = P.getVelocity(x(0));
    return y;
  };
  for(uint k=0;k<10;k++){
    arr x(1);
    x(0) = rnd.uni();
    checkJacobian(Test, x, 1e-4);
  }

  //-- write spline
  rai::arrayBrackets="  ";
  FILE("z.points") <<X;

  ofstream fil("z.test");
  for(uint t=0;t<=1000;t++){
    double time=(double)t/1000;
    fil <<time <<' ' <<P.getPosition(time) <<' ' <<P.getVelocity(time) <<endl;
  }
  fil.close();
  gnuplot("plot 'z.test' us 1:2 t 'pos', '' us 1:3 t 'vel', 'z.points' us ($0/10):1 w p", true, true);

}

//==============================================================================

void testDiff(){
  uint T=100;
  arr X(T,7);
  rndUniform(X,-1,1,false);

  rai::BSpline S;
  S.set(3, X, grid(1, 0., 1., X.d0-1).reshape(-1));

  //-- test w.r.t. times
  double teval;
  fct evalFromKnotTimes = [&S, &teval](const arr& knotTimes) -> arr {
    S.knotTimes = knotTimes;
    arr y;
    y = S.eval2(teval, 0, NoArr, y.J());
    return y;
  };

  double time = -rai::cpuTime();
  for(uint k=0;k<100;k++){
    teval = rnd.uni(-.1, 1.1);
    arr knotTimes = S.knotTimes;
    arr x = S.eval(teval);
    arr y = S.eval2(teval);
    arr z = evalFromKnotTimes(knotTimes);
    cout <<teval <<' ' <<x <<' ' <<y <<' ' <<x-y <<' ' <<x-z.noJ() <<endl;
    checkJacobian(evalFromKnotTimes, knotTimes, 1e-4);
    S.knotTimes = knotTimes;
  }
  time += rai::cpuTime();
  cout <<"time: " <<time <<endl;

  //-- test w.r.t. points
  fct evalFromKnotPoints= [&S, &teval](const arr& knotPoints) -> arr {
    S.ctrlPoints = knotPoints;
    arr y;
    y = S.eval2(teval, 0, y.J(), NoArr);
    return y;
  };

  time = -rai::cpuTime();
  for(uint k=0;k<100;k++){
    teval = rnd.uni(-.1, 1.1);
    arr knotPoints = S.ctrlPoints;
    arr x = S.eval(teval);
    arr y = S.eval2(teval);
    arr z = evalFromKnotPoints(knotPoints);
    cout <<teval <<' ' <<x <<' ' <<y <<' ' <<x-y <<' ' <<x-z.noJ() <<endl;
    checkJacobian(evalFromKnotPoints, knotPoints, 1e-4);
    S.ctrlPoints = knotPoints;
  }
  time += rai::cpuTime();
  cout <<"time: " <<time <<endl;
}

//==============================================================================

int MAIN(int argc,char** argv){
  rai::initCmdLine(argc, argv);

  testBasics();
  testBasis();
//  testSpeed();

//  testPath();
//  testDiff();

  return 0;
}
