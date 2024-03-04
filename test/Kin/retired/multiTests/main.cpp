#define RAI_REDEFINE_TESTS

#define TEST(name) test##name()

#undef MAIN
#define MAIN test_array_main
#include "../../Core/array/main.cpp"

#undef MAIN
#define MAIN test_komo_main
#include "../../KOMO/komo/main.cpp"

#undef MAIN
#define MAIN test_simulation_main
#include "../../Kin/simulation/main.cpp"

//===========================================================================

int main(int argc, char **argv){
  rai::setRaiPath("/home/mtoussai/git/rai");
  rai::initCmdLine(argc, argv);

//  testBasics();
//  testIterators();
//  testCheatSheet();
//  testInitializationList();
//  testSimpleIterators();
//  testSorted();
//  testRowsAndColumsAccess();
//  testStdVectorCompat();
//  testAutodiff();
//  testMatlab();
//  testException();
//  testMemoryBound();
//  testBinaryIO();
//  testExpression();
//  testPermutation();
//  testGnuplot();
//  testDeterminant();
//  testEigenValues();;
//  testRowShifted();
//  testSparseVector();
//  testSparseMatrix();
//  testInverse();
//  testMM();
//  testSVD();
//  testPCA();
//  testTensor();
//  testGaussElimintation();

  testEasy();
  testAlign();
  testThin();
  testPR2();
  testThreading();

  testRndScene();
  testFriction();
  testStackOfBlocks();
  testCompound();
  testPushes();
  testOpenClose();
  testGrasp();

  cout <<"\n ** total memory still allocated = " <<rai::globalMemoryTotal <<endl;
  //CHECK_ZERO(rai::globalMemoryTotal, 0, "memory not released");
  
  return 0;
}
