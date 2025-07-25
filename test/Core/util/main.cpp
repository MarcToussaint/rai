#include <Core/util.h>
#include <Core/graph.h>
#include <math.h>
#include <iomanip>

void TEST(Rnd){
  uint n=100;
  arr X(n,1);
  for(uint i=0;i<n;i++){
    X.elem(i) = rnd.uni();
  }
  FILE("z.dat") <<X.modRaw() <<endl;
  gnuplot("plot 'z.dat' us 0:1 w p", true);

  std::map<int, int> hist;
  for (int n = 0; n != 10000; ++n)
    ++hist[std::round(rnd.gauss(2, 3))];

  std::cout << "Normal distribution around " << mean << ":\n"
            << std::fixed << std::setprecision(1);
  for (auto [x, y] : hist)
    std::cout << std::setw(2) << x << ' ' << std::string(y / 100, '*') << '\n';
}

void TEST(String){
  //-- basic IO
  rai::String s("4.123, ");                     // create the string
  rai::String t1,t2;
  t1 <<s <<"length=" <<s.N <<'|' <<endl;   // piping into a string (including the endl '\n')
  t2 <<s <<"length=7|" <<endl;
  cout <<t1 <<t2;                          // outputting a string
  CHECK_EQ(t1,t2,"");

  //-- parsing from a string
  rai::String tmp;
  double a,b;
  s="a=1.2, b=3.4, blabla";
  cout <<s <<'|' <<endl;
  s >>"a=" >>a >>", b=" >>b;  // read things from string
  s >>"," >>tmp;      // read string from string (starting at current scan position)
  CHECK_ZERO(a-1.2, 1e-10, "");
  CHECK_ZERO(b-3.4, 1e-10, "");
  CHECK_EQ(tmp,"blabla", "");
  cout <<"a=" <<a <<", b=" <<b <<tmp <<'|' <<endl;

  s.resetIstream();   // reset the istream pointer of source string before...
  s >>tmp;            // ...reading string from string (starting at beginning of string)
  CHECK_EQ(tmp,"a=1.2", "");
  cout <<tmp <<'|' <<endl;

  s.resetIstream();   // reset the istream pointer of source string before...
  tmp.read(s,"", " ,;:\n\r"); //stop symbols
  CHECK_EQ(tmp,"a=1.2", "");
  cout <<tmp <<'|' <<endl;

  for(uint i=0;i<100;i++){
    tmp.setRandom();
    tmp <<"3";
  }
}

void TEST(Parameter){
  rai::String p1 = rai::getParameter<rai::String>("par", rai::String("default1"));
  CHECK_EQ(p1,"default1","");

  rai::String p2 = rai::getParameter<rai::String>("h", rai::String("def2"));
  CHECK_EQ(p2,"def2","");

  double d = rai::getParameter<double>("number");

  cout <<p1 <<endl <<p2 <<endl <<d <<endl;
}

void TEST(Wait){
  rai::wait();
}

void TEST(Timer){
  for(uint t=0;t<10;t++){
    cout <<"now=" <<rai::date() <<" clockTime=" <<std::setprecision(14) <<rai::clockTime() <<" realTime=" <<std::setprecision(14) <<rai::realTime() <<" cpuTime=" <<std::setprecision(14) <<rai::cpuTime() <<endl;
    rai::wait(.001);
  }

  double realTime = -rai::realTime();
  double cpuTime = -rai::cpuTime();

  for(uint i=0;i<4;i++){
    cout <<"i=" <<i <<std::flush;
    for(uint j=0;j<100000;j++){ j+=10; j-=10; } //do something stupid
    rai::wait(.5);
    cout <<" cpu timer reads " <<rai::cpuTime()+cpuTime <<"sec" <<endl;
  }
  cpuTime += rai::cpuTime();
  realTime += rai::realTime();
  CHECK_ZERO(realTime-2., .01, "wait failed");
  CHECK(cpuTime>=0. && cpuTime<1.,"no cpu time measured");
}

void TEST(Logging){
  rai::LogObject _log("Test");
  LOG(-1) <<"HALLO";
//  LOG(-3) <<"bye";
//  rai::log() <<"bla" <<endl;
}

void TEST(Exception){
  try{
    CHECK_EQ(2,1,"two is not equal to one (INTENDED EXCEPTION TEST)")
  }catch(const std::runtime_error& err){
    LOG(0) <<"Exception caught: " <<err.what();
  }
}

void TEST(Paths){
  std::cout <<rai::raiPath("here") <<endl;
}

void TEST(Inotify){
  rai::Inotify I(".");
  for(uint i=0;i<3;i++){
    rai::wait(1.);
    I.poll(false, true);
    cout <<i <<"sec" <<endl;
  }
}

int MAIN(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  uint double_size=sizeof(double);
  uint long_int_size=sizeof(long);
  cout <<"double size: " <<double_size <<"\nlong int size: " <<long_int_size <<endl;

  testRnd(); return 0;
  testPaths();
  testString();
  testParameter();
  testWait();
  testTimer();
  testLogging();
  testException();
  testInotify();

  return 0;
}
