#include <Geo/geo.h>
#include <Core/array.h>
#include <Core/util.h>
#include <math.h>

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
    TEST_DIFF_ZERO(A * -A);
    TEST_DIFF_ZERO(A*B * -B * -A);
    TEST_ZERO( ((A*-B).getRad()) - ((B*-A).getRad()) );
  }

  for(uint k=0;k<10;k++){
    rai::Transformation A,B,C;
    A.setRandom();
    B.setRandom();
    C.setRelative(A,B);
    TEST_DIFF_ZERO(A/A);
    TEST_DIFF_ZERO(A*A/A/A);
    TEST_DIFF_ZERO(A*C/B);
  }
}

//===========================================================================

void testQuaternions(){
  for(uint k=0;k<20;k++){
    rai::Quaternion a,b,c,d;
    rai::Vector v,w;
    rai::Matrix R,S;

    a.setRandom();
    b.setRandom();
    v.setRandom();

    //log - exp
    w = a.getLog();
    c.setExp(w);
    CHECK(c.isNormalized(), "");
    CHECK_ZERO(c.sqrDiff(a), 1e-12, "");

    //via matrix
    w = a.getLog();
    R.setExponential(w);
    a.getMatrix(&S.m00);
    CHECK_ZERO((R+(-1.*S)).diffZero(), 1e-12, "");
    b.setMatrix(&R.m00);
    CHECK_ZERO(b.sqrDiff(a), 1e-12, "");

    //relative
    c = b * -a;
    CHECK(c.isNormalized(), "");
    CHECK_ZERO(b.sqrDiff(c*a), 1e-12, "");

    //mult and inverse
    c = (a * b) * (-b);
    CHECK(c.isNormalized(), "");
    CHECK_ZERO(c.sqrDiff(a), 1e-12, "");

    //this works only for t=1!!
    c.setInterpolateProper(1., a, b);
    d.setInterpolateEmbedded(1., a, b);
    CHECK_ZERO(c.sqrDiff(d), 1e-12, "");

    //matrix
    c.setMatrix(a.getMatrix());
    CHECK_ZERO(c.sqrDiff(a), 1e-12, "");

    //matrix vs quat multiplication
    c.setMatrix(a.getMatrix() * b.getMatrix());
    CHECK_ZERO(c.sqrDiff(a*b), 1e-12, "");

    //matrix vs quat multiplication
    w = a.getMatrix() * v.getArr();
    CHECK_ZERO(sqrDistance(w, a * v), 1e-12, "");

    //association
    w = (a * b) * v;
    v = a * (b * v);
    CHECK_ZERO(sqrDistance(w, v), 1e-12, "");

  }
}


//===========================================================================

void TEST(QuaternionJacobian){
  for(uint k=0;k<10;k++){
    rai::Vector z;
    z.setRandom();
    VectorFunction f = [&z](const arr& x) -> arr{
//      double l = length(x);
      rai::Quaternion q(x);
      arr y = conv_vec2arr(q*z);
      y.J() = ~(q.getMatrixJacobian() * z.getArr());
      return y;
    };

    arr x = randn(4);
    // x /= length(x);
    checkJacobian(f, x, 1e-4, true);
  }

  for(uint k=0;k<10;k++){
    rai::Vector z;
    z.setRandom();
    VectorFunction f = [&z](const arr& x) -> arr{
      rai::Quaternion q(x);
      double n = q.sqrNorm();
      q.normalize();
      arr y = conv_vec2arr(q*z);
      arr R = q.getMatrix();
      // y.J() = crossProduct(R*q.getJacobian(), y);
      y.J() = crossProduct(q.getJacobian(), y);
      y.J() /= ::sqrt(n);
      return y;
    };

    arr x = randn(4);
    // x /= length(x);
    checkJacobian(f, x, 1e-4, true);
  }
}

//===========================================================================

int MAIN(int argc,char **argv){
  rai::initCmdLine(argc, argv);

  testBasics();
  testQuaternions();
  testQuaternionJacobian();

  return 0;
}
