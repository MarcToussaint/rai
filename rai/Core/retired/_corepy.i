// Read DOCSTRING to get an idea of corepy!
%define DOCSTRING_COREPY
"
This is a SWIG wrapper to be able to use the mlr Core within python

Note:
- tested with python.

TODO
- DONE map MT::Array to pylist/ndarray
- memory management sometimes fails
- DONE Interfaces for PhysX not implemented
- integrate some docstrings:
  http://www.swig.org/Doc1.3/Python.html#Python_nn65
- pointers sometimes need to be handled differently:
  http://www.swig.org/Doc1.3/Python.html#Python_nn47
  http://www.swig.org/Doc1.3/Python.html#Python_nn18
- TODO run unittests with Jenkins


author: Stefan Otte and Johannes Kulick

created: <2013-03-20 Wed>
"
%enddef
%module(docstring=DOCSTRING_COREPY) corepy

%feature("autodoc", "1");
%include "std_string.i"

%include "array_typemaps.i"


//===========================================================================
%pythoncode %{
import os
def get_mlr_path():
    """
    Return the path of the MLR code.
    The path is used to locate the libs and similar stuff.
    You can set he env var MLR_PATH if MLR is not in the default location.
    """
    return os.environ.get("MLR_PATH", os.path.expanduser("~/git/mlr/"))
%}

//===========================================================================
%module corepy
%{
  #include "Core/geo.h"
  #include "Core/array.h"
  #include "Core/util_t.h"
  #include "Core/array_t.h"
  // #include "Core/algos.h"
  #include "Gui/opengl.h"
  #include <sstream>
%}


//===========================================================================
// we need to map uint. Otherwise python complains about the uint type
%inline %{
  typedef unsigned int uint;
%}

//===========================================================================

// ugly, but we need to get the numpy type number from the actual C(++) type
%fragment("getNP_TYPE"{double}, "header") %{
  int numpy_type_double() { return NPY_DOUBLE; }
%}
%fragment("getNP_TYPE"{int}, "header") %{
  int numpy_type_int() { return NPY_INT; }
%}
%fragment("getNP_TYPE"{uint}, "header") %{
  int numpy_type_uint() { return NPY_UINT; }
%}

/*%Array_Typemap(double)  // arr*/
/*%Array_Typemap(int)     // intA*/
/*%Array_Typemap(uint)    // uintA*/

// we need to typedef array. Otherwise python complains about arr.
%inline %{
  typedef MT::Array<double> arr;
  typedef MT::Array<int> intA;
  typedef MT::Array<uint> uintA;
%}

%List_Typemap(arr)
%List_Typemap(intA)
%List_Typemap(uintA)

%inline %{
  typedef MT::Array<arr*> arrL;
%}
//===========================================================================

%typemap(in) MT::String {
    $1 = PyString_AsString($input);
}
%typemap(in) MT::String & {
    $1 = new MT::String(PyString_AsString($input));
}
%typemap(in) MT::String * {
    $1 = new MT::String(PyString_AsString($input));
}
%typemap(out) MT::String {
    $result = PyString_FromString($1.p);
}

//===========================================================================

%ignore MT::errString;

%include "geo.h"

//===========================================================================
// Map some common array functions
// TODO: do we really need them?

/// return identity matrix
inline arr eye(uint d0, uint d1) { arr z;  z.resize(d0, d1);  z.setId();  return z; }
/// return identity matrix
inline arr eye(uint n) { return eye(n, n); }

/// return matrix of ones
inline arr ones(const uintA& d) {  arr z;  z.resize(d);  z=1.;  return z;  }
/// return matrix of ones
inline arr ones(uint n) { return ones(TUP(n, n)); }
/// return matrix of ones
inline arr ones(uint d0, uint d1) { return ones(TUP(d0, d1)); }

/// return matrix of zeros
inline arr zeros(const uintA& d) {  arr z;  z.resize(d);  z.setZero();  return z; }
/// return matrix of zeros
inline arr zeros(uint n) { return zeros(TUP(n, n)); }
/// return matrix of zeros
inline arr zeros(uint d0, uint d1) { return zeros(TUP(d0, d1)); }

arr repmat(const arr& A, uint m, uint n);

/// return array with random numbers in [0, 1]
arr rand(const uintA& d);
/// return array with random numbers in [0, 1]
inline arr rand(uint n) { return rand(TUP(n, n)); }
/// return array with random numbers in [0, 1]
inline arr rand(uint d0, uint d1) { return rand(TUP(d0, d1)); }

/// return array with normal (Gaussian) random numbers
arr randn(const uintA& d);
/// return array with normal (Gaussian) random numbers
inline arr randn(uint n) { return randn(TUP(n, n)); }
/// return array with normal (Gaussian) random numbers
inline arr randn(uint d0, uint d1) { return randn(TUP(d0, d1)); }

inline double max(const arr& x) { return x.max(); }
inline double min(const arr& x) { return x.min(); }
inline uint argmax(const arr& x) { return x.maxIndex(); }
inline uint argmin(const arr& x) { return x.minIndex(); }

inline uintA randperm(uint n) {  uintA z;  z.setRandomPerm(n);  return z; }

//===========================================================================


namespace MT {
  template<class T> T getParameter(const char* tag);
  template<class T> T getParameter(const char* tag, const T& Default);
}

%template(getDoubleParameter) MT::getParameter<double>;
%template(getIntParameter) MT::getParameter<int>;
%template(getArrayParameter) MT::getParameter<arr>;
%template(getIntAParameter) MT::getParameter<intA>;
%template(getStringParameter) MT::getParameter<MT::String>;
%template(getBoolParameter) MT::getParameter<bool>;

//===========================================================================

%extend ors::Vector {
  std::string __str__() {
    std::ostringstream oss(std::ostringstream::out);
    oss << (*$self);
    return oss.str();
  }
  Vector __add__(const Vector& other) { return *$self + other; }
  Vector __sub__(const Vector& other) { return *$self - other; }
  Vector __mul__(const double& other) { return *$self * other; }
  bool __eq__(const Vector& other) { return *$self == other; }
  bool __ne__(const Vector& other) { return *$self != other; }
} 

%extend ors::Matrix {
  Matrix __add__(const Matrix& other) { return *$self + other; };
  bool __eq__(const Matrix& other) { return *$self == other; }
  bool __ne__(const Matrix& other) { return *$self != other; }

  %pythoncode %{
    def set(self, lst):
      assert(len(lst) >= 9)
      self.m00 = lst[0]
      self.m01 = lst[1]
      self.m02 = lst[2]
      self.m10 = lst[3]
      self.m11 = lst[4]
      self.m12 = lst[5]
      self.m20 = lst[6]
      self.m21 = lst[7]
      self.m22 = lst[8]
  %}
}

%extend ors::Quaternion {
  bool __eq__(const Quaternion& other) { return *$self == other; }
  bool __ne__(const Quaternion& other) { return *$self != other; }
}

%extend ors::Transformation {
  std::string __str__() {
    std::ostringstream oss(std::ostringstream::out);
    oss << (*$self);
    return oss.str();
  }
  bool __eq__(const Transformation& other) { return $self->pos == other.pos &&
$self->rot == other.rot; }
  bool __ne__(const Transformation& other) { return $self->pos != other.pos ||
$self->rot != other.rot; }
} 

//===========================================================================
// vim: ft=swig
