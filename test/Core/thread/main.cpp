#include <Core/thread.h>

//===========================================================================

void testMetronome(){
  rai::Metronome tic(.1);
  for(;;){
    tic.waitForTic();
    cout <<"tic: " <<tic.tics <<" \t time:" <<rai::realTime() <<endl;
    if(tic.tics>30) break;
  }
}

//===========================================================================

struct MyThread : rai::Thread {
  rai::Var<double>& x;
  uint n;
  MyThread(rai::Var<double>& _x, uint n, double beat)
    : rai::Thread(STRING("MyThread_"<<n), beat), x(_x), n(n){
    threadOpen();
  }
  void step(){
    if(n<=1) x.set()++; //only thread 1 should modify x!
    COUT <<"Thread " <<n <<": " <<rai::realTime() <<"sec  - is counting:" <<x.get() <<endl;
  }
};

void TEST(Listening){
  rai::Var<double> x;
  x.set() = 0.;
  MyThread t1(x, 1, .5), t2(x, 2, -1);

  t1.threadLoop();
  t2.listenTo(x); //whenever t1 modifies x, t2 is stepped
  
  rai::wait(3.);

  t1.threadClose();
  t2.threadClose();

  CHECK(x.get()>=5. && x.get()<=7.,"");
}


//===========================================================================
//
// test with a pair-wise distributed sorter
//

struct PairSorter : rai::Thread{
  rai::Var<int>& a;
  rai::Var<int>& b;
  PairSorter(const char* name, rai::Var<int>& _a, rai::Var<int>& _b)
    : rai::Thread(name),
      a(_a),
      b(_b){}
  ~PairSorter(){
    threadClose();
  }
  void step(){
    if(a.get()>b.get()){  //swap numbers
      auto a_set = a.set();
      auto b_set = b.set();
      if(a_set.data>b_set.data){
        a_set.data = b_set.data;
        b_set.data = a_set.data;
      }
    }
  }
};

//==============================================================================

void TEST(Sorter){
  uint N=20;

  rai::Array<shared_ptr<rai::Var<int>>> x(N);
  rai::Array<shared_ptr<PairSorter>> ps(N-1);
  for(uint i=0;i<N;i++) x(i) = make_shared<rai::Var<int>>();
  for(uint i=0;i<N-1;i++) ps(i) = make_shared<PairSorter>(STRING("PS_" <<i <<"_" <<i+1), *x(i), *x(i+1));

  {
    for(auto& s:ps) s->threadOpen();

    for(uint i=0;i<N;i++) x(i)->set() = rnd(100);

    for(uint k=0;k<20;k++){
      //if(moduleShutdown()->getStatus()) break;
      for(uint i=0;i<N;i++) cout <<x(i)->get() <<' ';
      cout <<endl;
      for(auto& s:ps) s->threadStep();
      rai::wait(.1);
    }

    for(auto& s:ps) s->threadClose();
  }

}

//==============================================================================
//
// condition grabber
//

struct RndWriter : rai::Thread {
  rai::Var<int>& x;
  RndWriter(const char* name, rai::Var<int>& x) : rai::Thread(name, .001), x(x) { threadLoop(); }
  ~RndWriter() { threadClose(); }
  void step(){ x.set() = rnd.uni_int(0, 100); }
};

void testCondition(){
  rai::Var<int> x(100);
  rai::Array<shared_ptr<RndWriter>> writers(20);
  for(uint i=0;i<writers.N;i++) writers(i) = make_shared<RndWriter>(STRING("thread_" <<i), x);

  for(uint k=0;k<20;k++){
    auto x_get = x.waitForEqAndGet(k); //wait for x to be equal k and then grab it as getter
    cout <<"k: " <<k <<", grabbed: " <<x_get.data <<", var revision: " <<x_get.var.revision <<endl;
  }
}


//==============================================================================
//
// logging with threads
//

// Normal Thread struct
struct MyLogThread : rai::Thread{
  uint n;
  MyLogThread(uint n, double beatIntervalSec=0.) : rai::Thread(STRING("MyThread_"<<n), beatIntervalSec), n(n){}
  void open(){}
  void close(){}
  void step(){
    LOG(0) <<rai::realTime() <<"sec Thread " <<n;
  }
};

void TEST(Logging){
  MyLogThread t1(1, .5), t2(2, .25);

  t1.threadLoop();
  t2.threadLoop();

  LOG(0) <<"starting to wait";
  rai::wait(3.);

  LOG(0) <<"done with wait";

  t1.threadClose();
  t2.threadClose();
}

//===========================================================================

int MAIN(int argc,char** argv){
  rai::initCmdLine(argc, argv);

  testMetronome();
  testListening();
  testSorter();
  testCondition();

  testLogging();

  return 0;
}
