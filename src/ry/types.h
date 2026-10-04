/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#ifdef RAI_PYBIND

#include "../Core/array.h"
#include "../Core/graph.h"
#include "../Geo/geo.h"
#include "../Kin/frame.h"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <pybind11/pytypes.h>

pybind11::dict graph2dict(const rai::Graph& G, const rai::NodeL& parents={}, bool parentsInKeys=true);
rai::Graph dict2graph(const pybind11::dict& dict);

pybind11::list graph2list(const rai::Graph& G);

pybind11::tuple uintA2tuple(const uintA& tup);

arr vecvec2arr(const std::vector<std::vector<double>>& X);

template<class T> std::vector<uint> vecdim(const rai::Array<T>& x) {
  uintA dim = x.dim();
  return std::vector<uint>(dim.p, dim.p+dim.N);
}

template<class T> std::vector<T> Array2vec(const rai::Array<T>& x) {
  std::vector<T> y(x.N);
  for(uint i=0; i<x.N; i++) y[i] = x.elem(i);
  return y;
}

template<class T> rai::Array<T> vec2Array(const std::vector<T>& x) {
  rai::Array<T> y;
  y.setCarray(&x.front(), x.size());
  return y;
}

template<class T> pybind11::array_t<T> Array2numpy(const rai::Array<T>& x) {
  if(!x.N) return pybind11::array_t<T>();
  return pybind11::array_t<T>(vecdim(x), x.p);
}

inline pybind11::array_t<double> arr2numpy(const arr& x) {
  //default!
  if(!isSparse(x)) return Array2numpy<double>(x);
  //sparse!
  arr triplets = x.sparse().getTriplets();
  return Array2numpy<double>(triplets);
}

template<class T> rai::Array<T> numpy2Array(const pybind11::array_t<T>& X) {
  rai::Array<T> Y;
  uintA dim(X.ndim());
  for(uint i=0; i<dim.N; i++) dim(i)=X.shape()[i];
  Y.resize(dim);
  auto ref = X.unchecked();
  if(Y.nd==0) {
    Y.clear();
    return Y;
  } else if(Y.nd==1) {
    for(uint i=0; i<Y.d0; i++) Y(i) = ref(i);
    return Y;
  } else if(Y.nd==2) {
    for(uint i=0; i<Y.d0; i++) for(uint j=0; j<Y.d1; j++) Y(i, j) = ref(i, j);
    return Y;
  } else if(Y.nd==3) {
    for(uint i=0; i<Y.d0; i++) for(uint j=0; j<Y.d1; j++) for(uint k=0; k<Y.d2; k++) Y(i, j, k) = ref(i, j, k);
    return Y;
  }
  NIY;
  return Y;
}

template<class T> pybind11::array ArrArr2numpy(const rai::Array<rai::Array<T>>& X) {
  pybind11::array Y = pybind11::array(pybind11::dtype("O"), pybind11::array::ShapeContainer{X.N});
  auto ref = Y.mutable_unchecked<pybind11::array_t<T>>();
  for(uint i=0;i<X.N;i++) ref(i) = Array2numpy<T>(X.elem(i));
  return Y.reshape(X.dim());
}

template<class T> rai::Array<rai::Array<T>> numpy2ArrArr(const pybind11::array& X) {
  rai::Array<rai::Array<T>> Y;
  uintA dim(X.ndim());
  for(uint i=0; i<dim.N; i++) dim(i)=X.shape()[i];
  Y.resize(dim);
  auto ref = X.unchecked<pybind11::array_t<T>>();
  if(Y.nd==0) { Y.clear(); }
  else if(Y.nd==1) { for(uint i=0; i<Y.d0; i++) Y(i) = numpy2Array<T>(ref(i)); }
  else if(Y.nd==2) { for(uint i=0; i<Y.d0; i++) for(uint j=0; j<Y.d1; j++) Y(i, j) = numpy2Array<T>(ref(i, j)); }
  else if(Y.nd==3) { for(uint i=0; i<Y.d0; i++) for(uint j=0; j<Y.d1; j++) for(uint k=0; k<Y.d2; k++) Y(i, j, k) = numpy2Array<T>(ref(i, j, k)); }
  else HALT("not implemented: array dimensionality " <<X.ndim());
  return Y;

  // if(X.ndim()==0) Y.clear();
  // else if(X.ndim()==1) Y.resize(X.shape()[0]);
  // else if(X.ndim()==2) Y.resize(X.shape()[0], X.shape()[1]);
  // else if(X.ndim()==3) Y.resize(X.shape()[0], X.shape()[1], X.shape()[2]);
  // else HALT("not implemented: array dimensionality " <<X.ndim());
  // auto ref = X.unchecked<pybind11::array_t<T>>();
  // for(uint i=0; i<Y.N; i++) Y.elem(i) = numpy2Array<T>(ref[i]);
  // return Y;
}

#if 0

using py=pybind11;

py::array return_array_of_arrays() {
  // 1. Prepare your inner NumPy arrays (e.g., jagged arrays of different lengths)
  py::array_t<double> arr1({3});
  auto r1 = arr1.mutable_unchecked<1>();
  r1(0) = 1.1; r1(1) = 2.2; r1(2) = 3.3;

  py::array_t<double> arr2({2});
  auto r2 = arr2.mutable_unchecked<1>();
  r2(0) = 4.4; r2(1) = 5.5;

  std::vector<py::array_t<double>> source_vectors = {arr1, arr2};

  // 2. Instantiate a base py::array with the object dtype "O"
  size_t size = source_vectors.size();
  py::array outer_array = py::array(py::dtype("O"), {size});

  // 3. Request a mutable buffer layout to write directly into it
  py::buffer_info buf = outer_array.request();

  // cast the raw pointer to a pointer of PyObject*
  PyObject** ptr = static_cast<PyObject**>(buf.ptr);

  // 4. Fill the outer array and manually manage the Python reference counts
  for (size_t i = 0; i < size; ++i) {
    // .ptr() extracts the underlying raw PyObject*
    PyObject* inner_ptr = source_vectors[i].ptr();

    // Safely increment the reference counter since the outer array now holds it
    Py_INCREF(inner_ptr);
    ptr[i] = inner_ptr;
  }

  return outer_array;
}

PYBIND11_MODULE(my_module, m) {
  m.def("get_array_of_arrays", &return_array_of_arrays);
}
#endif

template<class T> rai::Array<T> list2arr(const pybind11::list& X) {
  rai::Array<T> Y(X.size());
  for(uint i=0; i<Y.N; i++) Y.elem(i) = X[i].cast<T>();
  return Y;
}

template<class T> pybind11::list arr2list(const rai::Array<T>& X) {
  pybind11::list Y(X.N);
  for(uint i=0; i<X.N; i++) Y[i] = X.elem(i);
  return Y;
}

template<class T> rai::Array<T*> list2ArrayL(const pybind11::list& X) {
  rai::Array<T*> Y(X.size());
  for(uint i=0; i<Y.N; i++) Y.elem(i) = X[i].cast<shared_ptr<T>>().get();
  return Y;
}

template<class T> pybind11::list ArrayL2list(const rai::Array<T*>& X) {
  pybind11::list Y(X.N);
  for(uint i=0; i<X.N; i++) Y[i] = shared_ptr<T>(X.elem(i), [](T*){});  //giving it a nil deleter
  return Y;
}

template<class T> rai::Vector numpy2Vector(const pybind11::array_t<T>& X) {
  CHECK_EQ(X.ndim(), 1, "");
  CHECK_EQ(X.shape()[0], 3, "");
  auto ref = X.unchecked();
  return rai::Vector(ref(0), ref(1), ref(2));
}

inline pybind11::array_t<double> Vector2numpy(const rai::Vector v) {
  return pybind11::array_t<double>({3}, &v.x);
}

inline StringA strvec2StringA(const std::vector<std::string>& x) {
  StringA y(x.size());
  for(uint i=0; i<y.N; i++) y(i) = x[i];
  return y;
}

inline std::vector<std::string> StringA2strvec(const StringA& x) {
  std::vector<std::string> y;
  for(const rai::String& s:x) y.push_back(s.p);
  return y;
}

inline pybind11::list StringA2list(const StringA& x) {
  pybind11::list y(x.N);
  for(uint i=0; i<x.N; i++) y[i] = pybind11::str(x.elem(i).p, x.elem(i).N);
  return y;
}

inline pybind11::list StringAA2list(const StringAA& x) {
  pybind11::list y(x.N);
  for(uint i=0; i<x.N; i++) y[i] = StringA2list(x.elem(i));
  return y;
}

inline StringA list2StringA(const pybind11::list& X) {
  StringA Y(X.size());
  for(uint i=0; i<Y.N; i++) Y.elem(i) = X[i].cast<std::string>();
  return Y;
}

inline arrA npvec2arrA(const std::vector<pybind11::array_t<double>>& x) {
  arrA y(x.size());
  for(uint i=0; i<y.N; i++) y(i) = numpy2Array<double>(x[i]);
  return y;
}

inline std::vector<pybind11::array_t<double>> arrA2npvec(const arrA& x) {
  std::vector<pybind11::array_t<double>> y;
  for(const arr& z:x) y.push_back(arr2numpy(z));
  return y;
}

inline pybind11::list arrA2nplist(const arrA& x) {
  pybind11::list y(x.N);
  for(uint i=0; i<x.N; i++) y[i] = arr2numpy(x.elem(i));
  return y;
}

inline rai::Graph map2Graph(const std::map<std::string, std::string>& x) {
  return rai::Graph(x);
}

//inline ry::I_arr I_conv(const arr& x) {
//  ry::I_arr y;
//  y.first = vecdim(x);
//  y.second = x.vec();
//  return y;
//}

//inline arr I_conv(const ry::I_arr& x) {
//  arr y;
//  y = x.second;
//  uintA dim;
//  dim.referTo(&x.first.front(), x.first.size());
//  y.reshape(dim);
//  return y;
//}

namespace pybind11 {
namespace detail {

//== String -- std::string
template <> struct type_caster<rai::String> {
  PYBIND11_TYPE_CASTER(rai::String, _("rai::String"));

  bool load(pybind11::handle src, bool) {
    value = src.cast<std::string>();
    return !PyErr_Occurred();
  }

  static handle cast(const rai::String& src, return_value_policy, handle) {
    return pybind11::str(src.p, src.N).release();
  }
};

//== StringA -- list<std::string>
template <> struct type_caster<StringA> {
  PYBIND11_TYPE_CASTER(StringA, _("StringA"));

  bool load(pybind11::handle src, bool) {
    value = strvec2StringA(src.cast<std::vector<std::string>>());
    return !PyErr_Occurred();
  }

  static handle cast(const StringA& src, return_value_policy, handle) {
    return StringA2list(src).release();
  }
};

//== FrameL -- list<shared_ptr<Frame>>
template <> struct type_caster<FrameL> {
  PYBIND11_TYPE_CASTER(FrameL, _("FrameL"));

  bool load(pybind11::handle src, bool) {
    value = list2ArrayL<rai::Frame>(src.cast<pybind11::list>());
    return !PyErr_Occurred();
  }

  static handle cast(const FrameL& src, return_value_policy, handle) {
    return ArrayL2list<rai::Frame>(src).release();
  }
};

//== StringAA -- list<list<std::string>>
template <> struct type_caster<StringAA> {
  PYBIND11_TYPE_CASTER(StringAA, _("StringAA"));

  bool load(pybind11::handle src, bool) {
    NIY;
    //    value = strvec2StringA(src.cast<std::vector<std::string>>());
    return !PyErr_Occurred();
  }

  static handle cast(const StringAA& src, return_value_policy, handle) {
    return StringAA2list(src).release();
  }
};

//== arr -- numpy
template <> struct type_caster<arr> {
  PYBIND11_TYPE_CASTER(arr, _("arr"));

  bool load(pybind11::handle src, bool) {
    auto buf = pybind11::array_t<double>::ensure(src);
    if(!buf) return false;
    value = numpy2Array<double>(buf);
    return !PyErr_Occurred();
  }

  static handle cast(const arr& src, return_value_policy, handle) {
    pybind11::array_t<double> ret = arr2numpy(src);
    return ret.release();
  }
};

//== uintA -- numpy
template <> struct type_caster<uintA> {
  PYBIND11_TYPE_CASTER(uintA, _("uintA"));

  bool load(pybind11::handle src, bool) {
    auto buf = pybind11::array_t<uint>::ensure(src);
    if(!buf) return false;
    value = numpy2Array<uint>(buf);
    return !PyErr_Occurred();
  }

  static handle cast(const uintA& src, return_value_policy, handle) {
    pybind11::array_t<uint> ret = Array2numpy<uint>(src);
    return ret.release();
  }
};

//== Array<T> -- numpy<T>
template <class T> struct type_caster<rai::Array<T>> {
  PYBIND11_TYPE_CASTER(rai::Array<T>, _("Array<T>"));

  bool load(pybind11::handle src, bool) {
    auto buf = pybind11::array_t<T>::ensure(src);
    if(!buf) return false;
    value = numpy2Array<T>(buf);
    return !PyErr_Occurred();
  }

  static handle cast(const rai::Array<T>& src, return_value_policy, handle) {
    pybind11::array_t<T> ret = Array2numpy<T>(src);
    return ret.release();
  }
};

//== Array<Array<T>> <--> numpy<numpy<T>>
template <class T> struct type_caster<rai::Array<rai::Array<T>>> {
  PYBIND11_TYPE_CASTER(rai::Array<rai::Array<T>>, _("Array<Array<T>>"));

  bool load(pybind11::handle src, bool) {
    auto buf = pybind11::array::ensure(src);
    if(!buf) return false;
    value = numpy2ArrArr<T>(buf);
    return !PyErr_Occurred();
  }

  static handle cast(const rai::Array<rai::Array<T>>& src, return_value_policy, handle) {
    return ArrArr2numpy<T>(src).release();
  }
};

//== Vector -- numpy
template <> struct type_caster<rai::Vector> {
  PYBIND11_TYPE_CASTER(rai::Vector, _("Vector"));

  bool load(pybind11::handle src, bool) {
    auto buf = pybind11::array_t<double>::ensure(src);
    if(!buf) return false;
    value = numpy2Vector<double>(buf);
    return !PyErr_Occurred();
  }

  static handle cast(const rai::Vector& src, return_value_policy, handle) {
    pybind11::array_t<double> ret = Vector2numpy(src);
    return ret.release();
  }
};

//== rai::Graph -- pybind11::dict
template <> struct type_caster<rai::Graph> {
  PYBIND11_TYPE_CASTER(rai::Graph, _("rai::Graph"));

  bool load(pybind11::handle src, bool) {
    value = dict2graph(src.cast<pybind11::dict>());
    return !PyErr_Occurred();
  }

  static handle cast(const rai::Graph& G, return_value_policy, handle) {
    return graph2dict(G, rai::NodeL{}).release();
  }
};

}
}

#endif
