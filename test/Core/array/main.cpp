#include <Core/array.h>

using namespace std;

bool DoubleComp(const double& a,const double& b){ return a<b; }

//===========================================================================

void TEST(CheatSheet) {
  // CHEAT SHEET for rai::Array
  
  cout << "##### CREATING MATRICES" << endl;
  // Create rai::Array<double> with the 'arr' macro
  arr A;
  // or rai::Array<int> with 'intA'
  intA B; 
  // or rai::Array<uint> with 'intA'
  uintA C; 

  // Use matlab-like function to create matrices
  cout << "\n##### MATLAB FUNCTIONS" << endl;
  A = randn(2);
  A = zeros(3, 5);
  A = ones(3,3);
  arr I = eye(4);
  // or fill the matrix manually with an initialization list
  A = {1., 3., 7., 8.};

  cout << "\n##### MISC" << endl;
  // you can reshape matrices
  A.reshape(2, 2);
  // print matrices
  cout << A << endl;

  // query information about the array dimentions
  cout << "number of elements total:    " << A.N << endl;
  cout << "number of elements in dim 0: " << A.d0 << endl;
  cout << "number of elements in dim 1: " << A.d1 << endl;

  cout << "\n##### ACCESS" << endl;
  // access single elements
  cout << A(0, 0) << endl;
  // overwriting single elements
  A(0, 0) += 1;
  cout << "A(0, 0) = " << A(0, 0) << endl;
  // access rows
  cout <<"I.row(0): " <<I.row(0) <<endl;
  cout <<"I.rows(0, 2):\n" <<I.rows(0, 2) <<endl;
  // access columns
  cout <<"I.col(3): " <<I.col(3) <<endl;
  cout <<"I.rows(1, 4):\n" <<I.rows(1, 4) <<endl;
  // iterate through all elements
  for (const auto& elem : A) cout << elem << endl;
  for (auto& elem : A) elem += 4.;
  cout <<"A+4: " <<A <<endl;

  cout << "\n##### MATRIX OPERATIONS" << endl;
  // Work as you'd expect
  A = randn(2, 2);
  I = eye(2);
  CHECK_EQ(A * I , A, "");
  CHECK_EQ(A + A , A * 2., "");
  // transpose
  CHECK_EQ(A , ~(~A), "A = A^T^T");
  // inplace transpose
  cout <<"before inplace transpose:\n" <<A <<endl;
  transpose(A);
  cout <<"after inplace transpose:\n" <<A <<endl;
  transpose(A);
  cout <<"back to normal/after second inplace transpose:\n" <<A <<endl;
  // inverse
  cout <<"\nA * A.I:\n" <<A * inverse(A) <<endl;
  cout <<"\nA.I * A:\n" <<inverse(A) * A <<endl;
  CHECK_ZERO(maxDiff(A * inverse(A),I),1e-6, "");
}

//===========================================================================

void TEST(Basics){
  cout <<"\n*** basic manipulations\n";
  arr a;     //'arr' is a macro for rai::Array<double>
  intA ints; //a macro for rai::Array<int>

  a.resize(7,5);
  double *ap=a.p, *astop=ap+a.N;
  for(; ap!=astop; ap++) *ap=ap-a.p; //assign pointer offsets to entries
  cout <<"\narray filled with pointer offsets (-> memory is linear):\n" <<a <<endl;
  cout <<"\nsubarray (of the original) [2:4,:] (in MATLAB notation)\n" <<a.sub(2,4,0,-1) <<endl;
  CHECK_EQ(a.last(),a.N-1,"");

  //reshape
  cout <<a.copy().reshape(5,7) <<endl;
  cout <<a <<endl;

  //easier looping:
  cout <<"\neasier looping:\n";
  for(double& e: a) e++;
  cout <<a <<endl;

  //fancy writing:
  cout <<"\nfancy writing with dimensionality tag:" <<endl;
  a.write(cout,"-"," NEWLINE\n","{}",true);
  
  //deleting rows/columns
  a.delRows(1);
  cout <<"\n\nrow 1 deleted:\n" <<a <<endl;
  a.delColumns(1,2);
  cout <<"\n2 columns deleted at 1:\n" <<a <<endl;
  a.insColumns(1,3);
  cout <<"\n3 columns inserted at 1:\n" <<a <<endl;
  CHECK_EQ(a.d1,6,"");
  CHECK_EQ(a(0,1),0,"non-zeros inserted");

  //access:
  cout <<"\n3rd line:\n" <<a[2] <<endl; //gets a const-version of the []-subarray
  a[2](1)=7.; //same as a(2,1)=7 (but much slower)
  a[3] += 1.; //use operator() to get a non-const &-version of the []-subarray
  a[1] = a[2];
  cout <<"\nrows manipulated:\n" <<a <<endl;
  CHECK_EQ(a(2,1),7.,"");
  CHECK_EQ(a[1],a[2],"");

  //range access:
  cout <<"\nall rows:\n" <<a({0,-1}) <<endl;
  cout <<"\nrow 3:\n" <<a({3,3}) <<endl;
  cout <<"\nrow 3:\n" <<a(3, {0,-1}) <<endl;
  cout <<"\nrows 1-3:\n" <<a({1,3}) <<endl;
  cout <<"\nentries 1-4 of row 3:\n" <<a(3, {1,4}) <<endl;

  ints.setStraightPerm(8);
  ints.reshape(2,2,2);
  cout <<"\nint:\n" <<ints <<endl;
  cout <<"\nentries (1,0,{}):\n" <<ints(1, {0,0}) <<endl;
  cout <<"\nentries (1,0,{}):\n" <<ints(1, 0, {}) <<endl;
//  cout <<"\nentries (1,0,{}):\n" <<ints(1, 0, {0,-1}) <<endl;

  //access (copy and reference) of subarrays
  cout <<"\n({2,4}) =\n" <<a({2,4}) <<endl;
  a({2,4}) *= 10.;
  cout <<"\nrows manipulated:\n" <<a <<endl;

  //setting arrays ``by hand''
  a = ARR(0, 1, 2, 3, 4); //ARR(...) is equivalent to rai::Array<double>({ ... })
  cout <<"\nset by hand:\n" <<a <<endl;
  ints = { 0, -1, -2, -3, -4 };
  cout <<"\nset by hand:\n" <<ints <<endl;
  copy(a, ints); //copying between different types
  CHECK_EQ(a(2),-2,"");

  //using initialization lists within expressions
  //arr b = a + {2,2,2,2,2}; //does not compile
  a += {2,2,2,2,2}; //here it is considered as direct argument
  cout <<"\nadded {2,2,2,2,2}:\n" <<a <<endl;

  //inverse
  arr A = randn(3,3);
  cout <<"\nA:\n" <<A <<endl;
  cout <<"\n(1/A):\n" <<(1/A) <<endl;
  cout <<"\n(1/A)*A:\n" <<(1/A)*A <<endl;
  cout <<"\n(1/A)*A:\n" <<(A|A) <<endl;

  //concatenation
  a = {1.,2,3};
  arr b = {4.,5,6};
  cout <<"\na,b: " <<(a,b,b,a,b) <<endl;
//  cout <<"\na <<b: " <<(a.copy()() <<b) <<endl;
  cout <<"\na: " <<a <<endl;

  //TRY DEBUGGING with GDB:
  //set a breakpoint here
  //in gdb (or a watch console of your IDE) type 'print gdb(a)' and 'print gdb(ints)'
  
  //randomization
  rndUniform(a,-1.,1,false); //same as   forall(i,a) *i=rnd.uni(-1,1);
  cout <<"\nrandom double array:\n" <<a <<endl;

  //sorting
  a.sort(DoubleComp);
  //std::sort(a.p,a.pstop,DoubleComp);
  cout <<"\n sorting: sorted double array:\n" <<a <<endl;
  a.insertInSorted(.01,DoubleComp);
  cout <<"\n sorted insert: .01 added:\n" <<a <<endl;
  for(uint i=0;i<a.N-1;i++) CHECK_LE(a(i), a(i+1),"not sorted!");

  //commuting I/O-operators:
  a.resize(3,7,2);
  rndInteger(a,1,9,false);
  cout <<"\nbefore save/load:\n " <<a <<endl;

  FILE("z.tmp") <<a;

  FILE("z.tmp") >>b;

  cout <<"\nafter saved and loaded from a file: " <<b <<endl;
  CHECK_ZERO(maxDiff(a,b), 1e-4, "non-exact save load");
}

//===========================================================================

void TEST(Iterations) {
  arr x(10);
  x.setStraightPerm();
  x += 10.;
  cout <<x <<endl;

  for(auto& e:x.enumerated()){
    cout <<e.i <<' ' <<e() <<endl;
  }
}

//===========================================================================

void TEST(StdVectorCompat) {
  //-- plain conversion
  std::vector<double> x(3);
  x[0]=1.;
  cout <<"std::vector to arr:" <<arr(x, true) <<endl;

  arr y(10);
  y.setStraightPerm(10);
  x=y.vec();
  cout <<"arr -> std::vector -> arr = " <<arr(x, true) <<endl;

  //-- real interoperability

  intA ints(3,4);
  ints.setStraightPerm();
  cout <<ints <<endl;
}

//===========================================================================

void TEST(SimpleIterators) {
  // This test shows how to use the iterators

  cout << "*** Iterate linearly through the memory of an array (1D) - const" << endl;
  arr A = randn(9, 1);
  // notice: we're using const here
  for (const auto& elem : A) {
    cout << elem << endl;
  }

  cout << "*** increment each element" << endl;
  // notice: we DON'T use const here
  for (auto& elem : A) { elem += 1; }

  cout << "*** Iterate linearly through the memory of an array (2D)" << endl;
  A.reshape(3, 3);
  for (const auto& elem : A) {
    cout << elem << endl;
  }

  cout << "*** Iterate linearly through the memory of an array (3D)" << endl;
  arr C = randn(1, 8);
  C.reshape(TUP(2, 2, 2));
  for (const auto& elem : C) {
    cout << elem << endl;
  }
}

void TEST(InitializationList) {
  cout << "Use c++11 initialization list to initialize arrays by hand" << endl;
  arr a = {1., 3., 2., 5.};
  cout << a << endl;
}

//===========================================================================

void TEST(RowsAndColumsAccess) {
  // access rows and columns easily
  arr A = eye(3);

  cout << "\nAccessing single rows" << endl;
  cout << A.row(0) << endl;
  cout << A.row(1) << endl;
  cout << A.row(2) << endl;

  cout << "\nAccessing single columns" << endl;
  cout << A.col(0) << endl;
  cout << A.col(1) << endl;
  cout << A.col(2) << endl;

  cout << "\nAccessing multiple rows" << endl;
  cout << A.rows(0, 2) << endl;
  cout << A.rows(1, 3) << endl;

  cout << "\nAccessing multiple columns" << endl;
  cout << A.cols(0, 2) << endl;
  cout << A.cols(1, 3) << endl;
}

//===========================================================================

void TEST(Matlab){
  arr x = randn(5);
  cout <<"\nrandn(5)" <<x <<endl;

  x = eye(5);
  cout <<"\neye(5)" <<x <<endl;
  for(uint i=0;i<x.d0;i++) CHECK_EQ(x(i,i),1.,"is not eye");

  uintA p = randperm(5);
  cout <<"\nrandperm(5)" <<p <<endl;

  arr A = ARR(1,2,3,4);  A.reshape(2,2);
  arr B = repmat(A,2,3);
  cout <<"\nA=" <<A <<endl;
  cout <<"\nrepmat(A,2,3)" <<B <<endl;
}

//===========================================================================

void TEST(Exception){
  cout <<"\n*** exception handling\n";
  arr A;
  A.append(10);
  cout <<"accessing out of range..." <<endl;
  bool caught=false;
  try{
    cout <<A(2);
  }catch(const std::runtime_error& err){
    caught=true;
    cout <<"exception caught `" <<err.what() <<"'" <<endl;
  }
  CHECK(caught,"exception not caught");
}

void TEST(MemoryBound){
  cout <<"\n*** memory bound\n";
  rai::globalMemoryBound=1ull<<20;
  rai::globalMemoryStrict=true;
  arr A;
  try{
    A.resize(1000,1000,1000);
  }catch(...){
    cout <<"caught memory restriction exception..." <<endl;
  }
  A.resize(1000);
  cout <<"total memory allocated = " <<rai::globalMemoryTotal <<endl;
  rai::globalMemoryBound=1ull<<30;
}

//===========================================================================

void TEST(BinaryIO){
  cout <<"\n*** acsii and binary IO\n";
  arr a,b; a.resize(1000,100); rndUniform(a,0.,1.,false);

  ofstream fout("z.ascii"),bout("z.bin",ios::binary);
  ifstream fin("z.ascii") ,bin("z.bin",ios::binary);

  rai::timerStart();
  a.write(fout," ","\n","[]",true,false);
  cout <<"ascii write time: " <<rai::timerRead() <<"sec" <<endl;
  fout.close();

  rai::timerStart();
  b.read(fin);
  cout <<"ascii read time: " <<rai::timerRead() <<"sec" <<endl;
  fin.close();

  CHECK_ZERO(maxDiff(a,b), 1e-4, "ascii write-read error");

  rai::timerStart();
  a.write(bout,nullptr,nullptr,nullptr,true,true);
  cout <<"binary write time: " <<rai::timerRead() <<"sec" <<endl;
  bout.close();

  rai::timerStart();
  b.read(bin);
  cout <<"binary read time: " <<rai::timerRead() <<"sec" <<endl;
  bin.close();

  CHECK_EQ(a,b,"binary IO failed!");
  cout <<"binary IO exactly restores double array and is much faster" <<endl;
}

//===========================================================================

void TEST(Expression){
  cout <<"\n*** matrix expressions\n";
  arr a(2,3),b(3,2),c(3),d;
  rndInteger(a,-5,5,false);
  rndInteger(b,-5,5,false);
  rndInteger(c,-5,5,false);
  cout <<"\nmatrix A\n" <<a;
  cout <<"\nmatrix B\n" <<b;
  cout <<"\nmatrix product A*B\n" <<a*b;
  cout <<"\nvector c\n" <<c <<endl;
  cout <<"\ntranspose of c\n" <<~c;
  cout <<"\ntensor product c^c\n" <<(c^c);
  cout <<"\ncoupled unitary\n" <<1. + .5 * (2.*a) - 1.;
  cout <<"\nlonger expression\n" <<2.*a + 3.*a;
  cout <<"\nlonger expression\n" <<2.*a + ~b;
}

//===========================================================================

void TEST(Permutation){
  cout <<"\n*** permutation\n";
  uintA p;
  rnd.seed(3);

  p.setStraightPerm(10);
  cout <<"\ninit:\n" <<p;
  CHECK(p(2)==2 && p(5)==5,"");

  p.permute(2,5);
  cout <<"\npermute(2,5):\n" <<p;
  CHECK(p(2)==5 && p(5)==2,"");

  p.setRandomPerm();
  cout <<"\nrandom:\n" <<p <<endl;
  for(uint i=0;i<p.N;i++) cout <<i <<":" <<p(i) <<"\n";
}

//===========================================================================

void TEST(Sorted){
  cout <<"\n*** sorted\n";
  uintA x;
  rnd.seed(3);

  for(uint k=0;k<100;k++){
    uint y=rnd(99);
    if(!x.N || y>x.last()) x.insertInSorted(y, rai::greater<uint>);
    if(x.N>33) x.popLast();
    cout <<x <<endl;
    CHECK(x.isSorted(rai::greaterEqual<uint>),"");
  }
}

//===========================================================================

void TEST(Gnuplot){
  cout <<"\n*** gnuplot\n";
  uint i,j;
  arr X(30,30);
  for(i=0;i<X.d0;i++) for(j=0;j<X.d1;j++) X(i,j)=sin(.2*i)*sin(.1*j);
  gnuplot(X);
  rai::wait(.5);

  X.resize(100);
  for(i=0;i<X.d0;i++) X(i)=sin(.3*i);
  gnuplot(X);
  rai::wait(.5);

  X.resize(100,2);
  for(i=0;i<X.d0;i++){ X(i,0)=RAI_PI*(2./(X.d0-1)*i-1.); X(i,1)=sin(X(i,0)); }
  gnuplot(X);
  rai::wait(.5);
}

//===========================================================================

void TEST(Determinant){
  cout <<"\n*** determinant computation\n";
  arr a = ARR(1,1,2,1,1,0,0,-2,3);
  a.reshape(3,3);
  double d=determinant(a);
  double c00=cofactor(a,0,0);
  double c11=cofactor(a,1,1);
  double c22=cofactor(a,2,2);
  cout <<a <<"det=" <<d <<endl;
  cout <<"co00=" <<c00 <<endl;
  cout <<"co11=" <<c11 <<endl;
  cout <<"co22=" <<c22 <<endl;
  //  CHECK(fabs(d-c00*a(0,0))<1e-10,"");
  //  CHECK(fabs(d-c11*a(1,0))<1e-10,"");
}

//===========================================================================

void TEST(MM){
  cout <<"\n*** matrix multiplication speeds\n";
  uint M=3000,N=100,O=100;
  arr A(M,N),B(N,O),C,D;
  rndUniform(A,-1,1,false);
  rndUniform(B,-1,1,false);

  cout <<"speed test: " <<M <<'x' <<N <<'x' <<O <<" matrix multiplication..." <<endl;

  rai::useLapack=false; 
  rai::timerStart();
  innerProduct(D,A,B);
  double t_native=rai::timerRead();
  cout <<"native time = " <<t_native <<endl;

  if(!rai::lapackSupported){
    cout <<"LAPACK not installed - only native algorithms" <<endl;
    return;
  }

  rai::useLapack=true;
  rai::timerStart();
  blas_MM(C,A,B);
  rai::timerRead();
  cout <<"blas time = " <<rai::timerRead() <<endl;

  CHECK_ZERO(maxDiff(C,D), 1e-10, "blas MM is not equivalent to native matrix multiplication");
//  CHECK(t_blas < t_native,"blas MM is slower than native");
}

//===========================================================================

void TEST(SVD){
  cout <<"\n*** singular value decomposition\n";
  uint m=300,n=100,r=2,svdr;
  arr L(m,r),R(r,n),A,U,d,V,D;
  rndUniform(L,-1,1,false);
  rndUniform(R,-1,1,false);
  A=L*R;
  
  cout <<"speed test: " <<m <<'x' <<n <<" (rank=" <<r <<") SVD decomposition..." <<endl;

  rai::useLapack=false;
  rai::timerStart();
  svdr=svd(U,d,V,A);
  double t_native = rai::timerRead();
  cout <<"native SVD time = " <<t_native <<flush;
  D.setDiag(d);
  cout <<" error = " <<maxDiff(A, U*D*~V) <<" rank = " <<svdr <<"("<<r<<")"<<endl;
  CHECK_ZERO(maxDiff(A, U*D*~V), 1e-10, "native SVD failed");

  if(!rai::lapackSupported){
    cout <<"LAPACK not installed - only native algorithms" <<endl;
    return;
  }

  rai::useLapack=true;
  rai::timerStart();
  svdr=svd(U,d,V,A);
  double t_lapack = rai::timerRead();
  cout <<"lapack SVD time = " <<t_lapack <<flush;
  D.setDiag(d);
  cout <<" error = " <<maxDiff(A, U*D*~V) <<" rank = " <<svdr <<"("<<r<<")" <<endl;
  CHECK_ZERO(maxDiff(A, U*D*~V), 1e-10, "Lapack SVD failed");
}

//===========================================================================

void TEST(PCA) {
  // TODO: not really checking values automatically, just visualizing them
  // (and they are ok).

  cout <<"\n*** principal component analysis\n";
  arr x = { 1., -2., 1., -1., 1., 1., 1., 2. };
  x.reshape(4, 2);
  cout << "x = " << x << endl;

  arr xp, v, w;
  pca(xp, v, w, x, 1);

  cout << "xp = " << xp << endl;
  cout << "v = " << v << endl;
  cout << "w = " << w << endl;

  arr y = { 1., 1., -1., 1., -1., -1., 1., -1. };
  y.reshape(4, 2);
  cout << "y = " << y << endl;

  arr yp = y * w;
  cout << "yp = " << yp << endl;
}

//===========================================================================

void TEST(Inverse){
  cout <<"\n*** matrix inverse\n";
  uint m=300,n=300,svdr;
  arr A(m,n),invA,I;
  rndUniform(A,-1,1,false);
  I.setId(m);
  
  cout <<"speed test: " <<m <<'x' <<n <<" inversion..." <<endl;

  rai::useLapack=false;
  rai::timerStart();
  svdr=inverse_SVD(invA,A);
  double t_native = rai::timerRead();
  cout <<"native SVD inverse time = " <<t_native <<flush;
  cout <<" error = " <<maxDiff(A*invA,I) <<" rank = " <<svdr <<endl;
  CHECK_ZERO(maxDiff(A*invA,I), 1e-10, "native matrix inverse failed");

  /*rai::timerStart();
  rai::inverse_LU(invA,A);
  cout <<"native LU  inverse time = " <<rai::timerRead(); cout.flush();
  cout <<" error = " <<maxDiff(invA*A,I) <<endl;*/
  
  if(!rai::lapackSupported){
    cout <<"LAPACK not installed - only native algorithms" <<endl;
    return;
  }

  rai::useLapack=true;
  rai::timerStart();
  svdr=inverse_SVD(invA,A);
  double t_lapack = rai::timerRead();
  cout <<"lapack SVD inverse time = " <<t_lapack <<flush;
  cout <<" error = " <<maxDiff(A*invA, I) <<" rank = " <<svdr <<endl;
  CHECK_ZERO(maxDiff(A*invA, I), 1e-10, "lapack matrix inverse failed");

  /*rai::timerStart();
    rai::inverse_LU(invA,A);
    cout <<"lapack LU  inverse time = " <<rai::timerRead(); cout.flush();
    cout <<" error = " <<length(invA*A - I) <<endl;*/
  
  cout <<"\n*** symmetric matrix inverse\n";
  A.resize(m,m);
  rndUniform(A,-1,1,false);
  A=A*~A;
  I.setId(m);
  
  cout <<"speed test: " <<m <<'x' <<m <<" symmetric inversion..." <<endl;

  rai::timerStart();
  lapack_inverseSymPosDef(invA,A);
  double t_symPosDef = rai::timerRead();
  cout <<"lapack SymDefPos inverse time = " <<t_symPosDef <<flush;
  cout <<" error = " <<maxDiff(A*invA, I) <<endl;
  CHECK_ZERO(maxDiff(A*invA, I), 1e-6, "lapack SymDefPos inverse failed");

  CHECK(t_lapack < t_native, "lapack matrix inverse slower than native");
  CHECK(t_symPosDef < t_lapack, "symposdef matrix inverse slower than general");
}

//===========================================================================

void TEST(GaussElimintation) {
  cout << "\n*** Gaussian elimination with partial pivoting \n";
  if (rai::lapackSupported) {
    arr A = arr({3,3}, {7., 2., 4., 2., 6., 5., 5., 3., 7.});
    cout <<"A=\n" <<A << endl;

    arr b = arr({3,2}, {9., 5., 2., 1., 2., 3.});
    cout <<"b=\n" <<b << endl;

    arr X;
    lapack_mldivide(X, A, b);
    cout <<"X=\n" <<X << endl;
    cout <<"A*X=\n" <<A*X << endl;
  }
}

//===========================================================================

void TEST(Tensor){
  cout <<"\n*** tensor manipulations\n";

  arr A,B,C,D;
  uint k;
  for(k=0;k<100;k++){
    //element-wise multiplication
    A.resize(TUP(rnd(3)+1,rnd(3)+1));
    B.resize(A.d1,A.d0);
    rndUniform(A,0.,1.,false);
    rndUniform(B,0.,1.,false);
    C=A;
    tensorMultiply(C,B,TUP(1,0));
    CHECK_EQ(C,A%~B,"");
    C=A;
    tensorMultiply_old(C,B,TUP(C.d0,C.d1),TUP(1,0));
    CHECK_EQ(C,A%~B,"");
    tensorEquation(C,A,TUP(0,1),B,TUP(1,0),0);
    CHECK_EQ(C,A%~B,"");

    //matrix product
    C.resize(A.d0,A.d0);
    tensorEquation(C,A,TUP(0,2),B,TUP(2,1),1);
    CHECK_EQ(C,A*B,"");

    C.resize(A.d1,A.d1);
    tensorEquation(C,A,TUP(2,0),B,TUP(1,2),1);
    CHECK_EQ(C,~A*~B,"");
  }
  cout <<"\n... tensor tests successful\n";

  //test permutations:
  A.resize(2,3,4);
  rndInteger(A,0,1,false);
  tensorPermutation(B, A, TUP(0,1,2));
  cout <<A <<endl <<B <<endl;
}

//===========================================================================

void TEST(RowShifted){
  cout <<"\n*** RowShifted\n";

//  rnd.clockSeed();
  
  arr J;
  rai::RowShifted& J_ = J.rowShifted();
  J_.resize(10,12,4);
  rndInteger(J,0,9);
  for(uint i=0;i<J.d0;i++) J_.rowShift(i) = i/3;
  J_.computeColPatches(false);
  cout <<J.rowShifted();

  cout <<J_.At().rowShifted() <<endl;

  //constructor compressing an array
  arr K = unpack(J);
  K.rowShifted().reshift();
  cout <<K.rowShifted();
  
  cout <<"-----------------------" <<endl;

  //-- randomized checks
  for(uint k=0;k<100;k++){
    arr X(1+rnd(5),1+rnd(5));
    rndInteger(X,0,1);
    X *= rand(X.d0, X.d1);
    arr Y = X;
    Y.rowShifted().reshift();
    arr Yt = comp_At(Y);
//    Yt.rowShifted().computeColPatches(false);
//    cout <<"-----------------------" <<endl;
//    cout <<X <<endl;
//    cout <<Y.rowShifted() <<endl <<Yt.rowShifted() <<endl;
//    cout <<"***\n" <<~X*X <<endl <<comp_At_A(Y).rowShifted() <<endl;

    arr x(X.d0);   rndGauss(x, 1.); //Integer(x,0,9);
    arr x2(X.d1);  rndGauss(x2, 1.); //Integer(x2,0,9);
    arr Z(1+rnd(5), X.d0);
    arr Z2(X.d1, 1+rnd(5));
    cout <<"errors = " <<maxDiff(X,unpack(Y))
        <<' ' <<maxDiff(~X,unpack(Yt))
        <<' ' <<maxDiff(~X*X, unpack(comp_At_A(Y)))
       <<' ' <<maxDiff(X*~X, unpack(comp_A_At(Y)))
      <<' ' <<maxDiff(X*x2, comp_A_x(Y,x2))
      <<' ' <<maxDiff(~X*x, comp_At_x(Y,x))
      <<' ' <<maxDiff(~X*x, comp_A_x(Yt,x))
     <<' ' <<maxDiff(~X*X, unpack(comp_A_At(Yt)))
    <<' ' <<maxDiff(Z*X, unpack(Z*Y))
    <<' ' <<maxDiff(X*Z2, unpack(Y*Z2))
    <<endl;
    CHECK_ZERO(maxDiff(X, unpack(Y)), 1e-10, "");
    CHECK_ZERO(maxDiff(~X, unpack(Yt)), 1e-10, "");
    CHECK_ZERO(maxDiff(~X*X, unpack(comp_At_A(Y))), 1e-10, "");
//    arr tmp =comp_A_At(Y);
//    //write(*castRowShifted(tmp));
//    cout <<X*~X <<endl <<unpack(comp_A_At(Y)) <<endl;
    CHECK_ZERO(maxDiff(X*~X, unpack(comp_A_At(Y))), 1e-10, "");
    CHECK_ZERO(maxDiff(X*x2, comp_A_x(Y,x2)), 1e-10, "");
    CHECK_ZERO(maxDiff(~X*x, comp_At_x(Y,x)), 1e-10, "");
    CHECK_ZERO(maxDiff(~X*x, comp_A_x(Yt,x)), 1e-10, "");
    CHECK_ZERO(maxDiff(~X*X, unpack(comp_A_At(Yt))), 1e-10, "");
    CHECK_ZERO(maxDiff(Z*X, unpack(Z*Y)), 1e-10, "");
    CHECK_ZERO(maxDiff(X*Z2, unpack(Y*Z2)), 1e-10, "");

    //cholesky:
    arr H = comp_A_At(Y);
    addDiag(H, 1.);
    arr Hchol;
    lapack_choleskySymPosDef(Hchol, H);
    CHECK_ZERO(maxDiff(comp_At_A(Hchol), H), 1e-10, "");
    CHECK_ZERO(maxDiff(unpack(comp_At_A(Hchol)), unpack(H)), 1e-10, "");
  }
}

//===========================================================================

void sparseProduct(arr& y, arr& A, const arr& x);

void TEST(SparseMatrix){
  cout <<"\n*** SparseMatrix\n";

  arr A(5,10), B(10);
  rndInteger(A,0,3);
  rndInteger(B,0,3);
  cout <<"A=\n" <<A <<"\nB=\n" <<B <<"\nA*B=\n" <<A*B <<endl;

  cout <<"A sparsity=" <<A.sparsity() <<endl;
  cout <<"B sparsity=" <<B.sparsity() <<endl;

  A.sparse();
  B.sparseVec();

  cout <<"A=\n" <<A <<"\nB=\n" <<B <<endl;
//  cout <<"\nA*B=\n" <<A*B <<endl;
  arr y;
  sparseProduct(y, A, B);
  y = y.sparseVec().unsparse();
  cout <<"A*B=" <<y <<endl;

  arr x = eigen_Ainv_b(A, y);
  cout <<"Ainv*A*B=" <<~x <<endl;

  for(uint k=0;k<100;k++){
    arr A(10,20);
    arr B(20);
    rndInteger(A,0,3);
    rndInteger(B,0,3);
    arr C = A*B;
    A.sparse();
//    B.makeSparse();
    arr D;
    sparseProduct(D, A, B);
    CHECK_EQ(C, D, "");
  }
}

//===========================================================================

void TEST(SparseVector){
  cout <<"\n*** SparseVector\n";

  arr a(10), b(10);
  rndInteger(a,0,3);
  rndInteger(b,0,3);
  cout <<"A=\n" <<a <<"\nB=\n" <<b <<"\nA*B=\n" <<scalarProduct(a, b) <<endl;

  cout <<"A sparsity=" <<a.sparsity() <<endl;
  cout <<"B sparsity=" <<b.sparsity() <<endl;

  a.sparseVec();
  b.sparseVec();

  cout <<"A=\n" <<a <<"\nB=\n" <<b <<"\nA*B=\n" <<scalarProduct(a, b) <<endl;

  for(uint k=0;k<100;k++){
    arr a(20);
    arr b(20);
    rndInteger(a,0,3);
    rndInteger(b,0,3);
    double c = scalarProduct(a,b);
    a.sparseVec();
    b.sparseVec();
    double d = scalarProduct(a,b);
    CHECK_EQ(c, d, "");
  }
}

//===========================================================================

void TEST(EigenValues){
  rnd.clockSeed();
  arr C(30,8);
  rndUniform(C,-1., 1.);
  arr H=~C*C;

  //lapack:
  arr lambda, x;
  lapack_EigenDecomp(H, lambda, x);
  cout <<"eigwenvalues=" <<lambda <<endl;
  cout <<"eigenvectors=" <<x <<endl;

  //power method
  arr x_hi = 2.*rand(H.d0)-1.;  x_hi/=length(x_hi);
  arr x_lo = 2.*rand(H.d0)-1.;  x_lo/=length(x_lo);
  arr I = eye(H.d0);
  double lambda_hi, lambda_lo;
  for(uint k=0;k<10;k++){
    x_hi = H*x_hi;
    lambda_hi=length(x_hi);
    x_hi /= lambda_hi;

    x_lo = (H-lambda_hi*I) * x_lo;
    lambda_lo=length(x_lo);
    x_lo /= lambda_lo;
    lambda_lo = lambda_hi - lambda_lo;
  }
  //flip signs
  if(x_hi.last()*x.last()<0.) x_hi *= -1.;
  if(x_lo.elem(0)*x.elem(0)<0.) x_lo *= -1.;
  cout <<"power method:" <<endl;
  cout <<lambda_hi <<" : " <<x_hi <<endl;
  cout <<lambda_lo <<" : " <<x_lo <<endl;
  cout <<"errors:" <<endl;
  cout <<fabs(lambda_hi-lambda.last()) <<" " <<sqrDistance(x_hi, x[x.d0-1]) <<endl;
  cout <<fabs(lambda_lo-lambda(0)) <<" " <<sqrDistance(x_lo, x[0]) <<endl;
}

//===========================================================================

int MAIN(int argc, char **argv){
  rai::initCmdLine(argc, argv);

  testBasics();
  testIterations();
  testCheatSheet();
  testInitializationList();
  testSimpleIterators();
  testSorted();
  testRowsAndColumsAccess();
  testStdVectorCompat();
  testMatlab();
  testException();
//  testMemoryBound();
  testBinaryIO();
  testExpression();
  testPermutation();
  testGnuplot();
  testDeterminant();
  testEigenValues();;
  testRowShifted();
  testSparseVector();
  testSparseMatrix();
  testInverse();
  testMM();
  testSVD();
  testPCA();
  testTensor();
  testGaussElimintation();
  
  cout <<"\n ** total memory still allocated = " <<rai::globalMemoryTotal <<endl;
  CHECK_ZERO(rai::globalMemoryTotal, 0, "memory not released");
  
  return 0;
}
