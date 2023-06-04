#include <Algo/spline.h>
#include <Gui/plot.h>
#include <Kin/kin.h>

//==============================================================================

void TEST(Basics){
  rai::BSpline S;
  arr X = {1., 0., 0., 1.};  X.reshape(-1,1);
  arr T = {0., .5, .5, 1.};
  arr vel = {1.};
  S.set(2, X, T);

  cout <<"\ntimes = " <<S.knotTimes <<endl;
  cout <<"points = " <<~S.knotPoints <<endl;

  S.append(arr{0., 1.}.reshape(-1,1), {.5, 1.});

  cout <<"\ntimes = " <<S.knotTimes <<endl;
  cout <<"points = " <<~S.knotPoints <<endl;

  S.setDoubleKnotVel(-1, vel);
  S.setDoubleKnotVel(1, vel);
  S.setDoubleKnotVel(5, -vel);

  cout <<"\ntimes = " <<S.knotTimes <<endl;
  cout <<"points = " <<~S.knotPoints <<endl;

  ofstream fil("z.test");
  for(double t=S.begin()+.1;t<=S.end()+.1;t+=.001){
    arr x, xDot, xDDot;
    S.eval(x, xDot, xDDot, t);
    fil <<t <<' ' <<x.modRaw() <<' ' <<xDot.modRaw() <<' ' <<xDDot.modRaw() <<endl;
//    fil <<t <<' ' <<S.eval(t,2).modRaw() <<endl;
  }
  fil.close();
  gnuplot("set size square; plot  'z.test' us 1:2, x", true);
//  gnuplot("plot 'z.test' us 1:2, '' us 1:3, '' us 1:4, x", true);
  rai::wait();
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
      <<"\npoints= " <<P.knotPoints <<endl;

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

  double teval;
  fct Test = [&S, &teval](const arr& knots) -> arr {
    S.knotTimes = knots;
    arr y;
    y = S.eval2(teval, 0, y.J());
    return y;
  };

  double time = -rai::cpuTime();
  for(uint k=0;k<100;k++){
    teval = rnd.uni(-.1, 1.1);
    arr knots = S.knotTimes;
    arr x = S.eval(teval);
    arr y = S.eval2(teval);
    arr z = Test(knots);
    cout <<teval <<' ' <<x <<' ' <<y <<' ' <<x-y <<' ' <<x-z.noJ() <<endl;
    checkJacobian(Test, knots, 1e-4);
    S.knotTimes = knots;
  }
  time += rai::cpuTime();
  cout <<"time: " <<time <<endl;
}

//==============================================================================


int MAIN(int argc,char** argv){
  rai::initCmdLine(argc, argv);

//  testBasics();
//  testSpeed();

//  testPath();
  testDiff();

  return 0;
}
