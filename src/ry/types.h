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

template<class T> rai::Array<T> numpy2arr(const pybind11::array_t<T>& X) {
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

template<class T> rai::Array<T> list2arr(const pybind11::list& X) {
  rai::Array<T> Y(X.size());
  for(uint i=0; i<Y.N; i++) Y.elem(i) = X[i].cast<T>();
  return Y;
}

template<class T> rai::Array<T> arr2list(const rai::Array<T>& X) {
  pybind11::list Y(X.N);
  for(uint i=0; i<X.N; i++) Y[i] = X.elem(i);
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
  for(uint i=0; i<y.N; i++) y(i) = numpy2arr<double>(x[i]);
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

//== arrA <--> list<numpy>
template <> struct type_caster<arrA> {
  PYBIND11_TYPE_CASTER(arrA, _("arrA"));

  bool load(pybind11::handle src, bool) {
    value = npvec2arrA(src.cast<std::vector<pybind11::array_t<double>>>());
    return !PyErr_Occurred();
  }

  static handle cast(const arrA& src, return_value_policy, handle) {
    return arrA2nplist(src).release();
  }
};

//== arr -- numpy
template <> struct type_caster<arr> {
  PYBIND11_TYPE_CASTER(arr, _("arr"));

  bool load(pybind11::handle src, bool) {
    auto buf = pybind11::array_t<double>::ensure(src);
    if(!buf) return false;
    value = numpy2arr<double>(buf);
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
    value = numpy2arr<uint>(buf);
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
    value = numpy2arr<T>(buf);
    return !PyErr_Occurred();
  }

  static handle cast(const rai::Array<T>& src, return_value_policy, handle) {
    pybind11::array_t<T> ret = Array2numpy<T>(src);
    return ret.release();
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
