#include <Algo/spline.h>
#include <Gui/plot.h>
#include <Kin/kin.h>

//==============================================================================

void plotIt(rai::BSpline& S){
  ofstream fil("z.test");
  for(double t=S.begin()-.1;t<=S.end()+.1;t+=.001){
    arr x, xDot, xDDot;
    S.eval2(x, xDot, xDDot, t);
    fil <<t <<' ' <<x.modRaw() <<' ' <<xDot.modRaw() <<' ' <<xDDot.modRaw() <<endl;
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

void TEST(BasisMatrix){
  rai::BSpline S;
  rai::BSplineCore SC;

  uint n=300;
  for(uint deg=0;deg<5;deg++){
    if(deg) S.setUniform(deg, deg+deg%2);
    else{ S.degree=0; S.knotTimes={0., 1.}; S.ctrlPoints=zeros(1,1); }
    //S.knotTimes = ::range(0.,1.,S.knotTimes.N-1);
    arr B = S.getGridBasis(n);

    {
      SC.setUniformKnots(deg, deg+deg%2+1);
      //cout <<"==========\n" <<SC.knots <<endl <<S.knotTimes <<endl;
      CHECK_ZERO(::maxDiff(SC.knots, S.knotTimes), 1e-10, "");
      arr BB = SC.getBmatrix(n+1, deg);
      //cout <<"==========\n" <<B <<endl <<BB <<endl;
      CHECK_ZERO(::maxDiff(B, BB), 1e-10, "");
    }

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

void TEST(Fitting){
  uint N = 10, n=1, z=10, deg=3;
  arr X = randn(N, n);
  arr Z = rai::BSpline_path2ctrlPoints(X, z, deg, false);
  rai::BSpline S;
#if 1
  rai::BSplineCore SC;
  SC.setUniformKnots(deg, z);
  S.degree = deg;
  S.knotTimes = SC.knots;
  S.ctrlPoints = Z;
  // S.setPoints(Z);
#else
  S.set(deg, Z, ::range(0., 1., Z.d0-1));
#endif
  CHECK_EQ(S.ctrlPoints.d0, S.knotTimes.N - deg - 1, "");

  FILE("z.X.dat") <<rai::catCol(~~::range(0.,1.,X.d0-1), X).modRaw();
  FILE("z.Z.dat") <<rai::catCol(~~::range(0.,1.,Z.d0-1), Z).modRaw();
  arr t = ::range(0.,1.,100.);
  FILE("z.S.dat") <<rai::catCol(t, S.eval(t)).modRaw();

  gnuplot("plot 'z.S.dat' us 1:2 w l, 'z.Z.dat' us 1:2 w p, 'z.X.dat' us 1:2 w lp", true);
}

//==============================================================================

void TEST(Speed){

  uint N=1000000, n=2, deg=3;
  arr X = randn(N, n);
  arr T = integral(rand(N)+0.1);
//  cout <<X <<endl <<T <<endl;

  rai::BSpline S;
  S.set(deg, X, T);

  rai::timerStart();
  double a=S.begin()-1., b=S.end()+1.;
  for(double t=a; t<b; t+=1e-5*(b-a)){
    S.eval(t);
  }
  cout <<"time: " <<rai::timerRead() <<endl;
}

//==============================================================================

void testDiff(){
  uint T=100, deg=3;
  arr X(T,7);
  rndUniform(X,-1,1,false);

  rai::BSpline S;
  S.set(deg, X, ::range(0., 1., X.d0-1));

  //-- test w.r.t. knots
  double teval, time;
  fct evalFromKnots = [&S, &teval](const arr& knots) -> arr {
    S.knotTimes = knots;
    arr y;
    S.eval2(y, NoArr, NoArr, teval, NoArr, y.J());
    return y;
  };

  time = -rai::cpuTime();
  for(uint k=0;k<100;k++){
    teval = rnd.uni(-.1, 1.1);
    arr knots = S.knotTimes;
    arr x = S.eval(teval);
    arr y = evalFromKnots(knots);
    CHECK_ZERO(maxDiff(x,y), 1e-10, "");
    cout <<teval <<' ' <<x <<' ' <<y.noJ() <<' ' <<maxDiff(x, y) <<endl;
    checkJacobian(evalFromKnots, knots, 1e-4);
    S.knotTimes = knots;
  }
  time += rai::cpuTime();
  cout <<"time: " <<time <<endl;

  //-- test w.r.t. ctrl points
  fct evalFromCtrlPoints= [&S, &teval](const arr& ctrlPoints) -> arr {
    S.ctrlPoints = ctrlPoints;
    arr y;
    S.eval2(y, NoArr, NoArr, teval, y.J(), NoArr);
    return y;
  };

  time = -rai::cpuTime();
  for(uint k=0;k<100;k++){
    teval = rnd.uni(-.1, 1.1);
    arr ctrlPoints = S.ctrlPoints;
    arr x = S.eval(teval);
    arr y = evalFromCtrlPoints(ctrlPoints);
    CHECK_ZERO(maxDiff(x,y), 1e-10, "");
    cout <<teval <<' ' <<x <<' ' <<y.noJ() <<' ' <<maxDiff(x,y) <<endl;
    checkJacobian(evalFromCtrlPoints, ctrlPoints, 1e-4);
    S.ctrlPoints = ctrlPoints;
  }
  time += rai::cpuTime();
  cout <<"time: " <<time <<endl;

  //-- test time derivatives along spline
  fct evalFromTime = [&S](const arr& t) -> arr {
    arr y;
    y = S.eval(t.elem(), 1);
    y.J() = ~S.eval(t.elem(), 2);
    return y;
  };

  time = -rai::cpuTime();
  for(uint k=0;k<100;k++){
    arr t = arr{rnd.uni(-.1, 1.1)};
    arr x = S.eval(t.elem(), 1);
    arr y = evalFromTime(t);
    cout <<t <<' ' <<x <<' ' <<y.noJ() <<' ' <<maxDiff(x,y) <<endl;
    checkJacobian(evalFromTime, t, 1e-4);
  }
  time += rai::cpuTime();
  cout <<"time: " <<time <<endl;

}

//==============================================================================

int MAIN(int argc,char** argv){
  rai::initCmdLine(argc, argv);

  testBasics();
  testBasisMatrix();
  testFitting();
  testSpeed();

  // testPath();
  testDiff();

  return 0;
}
