#include <Geo/geo.h>
#include <Core/array.h>
#include <Core/util.h>

//===========================================================================
//
// test very basics
//

#define TEST_DIFF_ZERO(expr) { CHECK_ZERO((expr).diffZero(), 1e-6, "TEST FAILED"); cout <<"Success: " <<(expr).diffZero() <<" == " <<#expr <<endl; }

#define TEST_ZERO(expr) { CHECK_ZERO((expr), 1e-6, "TEST FAILED"); cout <<"Success: " <<(expr) <<" == " <<#expr <<endl; }

void TEST(Basics){
  for(uint k=0;k<10;k++){
    rai::Quaternion A,B,C;
    A.setRandom();
    B.setRandom();
    C.setRandom();
    TEST_DIFF_ZERO(Quaternion_Id);
    TEST_DIFF_ZERO(A/A);
    TEST_DIFF_ZERO(A*B/B/A);
    TEST_ZERO( ((A/B).getRad()) - ((B/A).getRad()) );
  }

  for(uint k=0;k<10;k++){
    rai::Transformation A,B,C;
    A.setRandom();
    B.setRandom();
    C.setDifference(A,B);
    TEST_DIFF_ZERO(A/A);
    TEST_DIFF_ZERO(A*A/A/A);
    TEST_DIFF_ZERO(A*C/B);
  }
}

//===========================================================================

void TEST(QuaternionJacobian){
  for(uint k=0;k<1;k++){
    rai::Vector z;
    z.setRandom();
    VectorFunction f = [&z](const arr& x) -> arr{
//      double l = length(x);
      rai::Quaternion q(x);
      arr y = conv_vec2arr(q*z);
      y.J() = ~(q.getMatrixJacobian() * conv_vec2arr(z));
      return y;
    };

    arr x = randn(4);
    x /= length(x);
    checkJacobian(f, x, 1e-4, true);
  }
}

//===========================================================================

int MAIN(int argc,char **argv){
  rai::initCmdLine(argc, argv);

  testBasics();
  testQuaternionJacobian();

  return 0;
}
