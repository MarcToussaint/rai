/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PYBIND

#include "types.h"
#include "../Geo/mesh.h"

template<> pybind11::array_t<double> arr2numpy(const rai::Array<double>& x){
  //default!
  if(!x.isSparse()) return pybind11::array_t<double>(x.dim(), x.p);
  //sparse!
  arr triplets = x.sparse().getTriplets();
  return pybind11::array_t<double>(triplets.dim(), triplets.p);
}

pybind11::dict graph2dict(const rai::Graph& G) {
  pybind11::dict dict;
  for(rai::Node* n:G) {
    rai::String key;
    if(n->key.N) key=n->key;
    else key <<n->index;

    //-- write value
    if(n->isGraph()) {
      dict[key.p] = graph2dict(n->get<rai::Graph>());
    } else if(n->isOfType<rai::String>()) {
      dict[key.p] = n->get<rai::String>().p;
    } else if(n->isOfType<arr>()) {
      dict[key.p] = n->get<arr>().vec();
    } else if(n->isOfType<arrA>()) {
      dict[key.p] = n->get<arrA>().vec();
    } else if(n->isOfType<intA>()) {
      dict[key.p] = n->get<intA>().vec();
    } else if(n->isOfType<uintA>()) {
      dict[key.p] = n->get<uintA>().vec();
    } else if(n->isOfType<boolA>()) {
      dict[key.p] = n->get<boolA>().vec();
    } else if(n->isOfType<double>()) {
      dict[key.p] = n->get<double>();
    } else if(n->isOfType<int>()) {
      dict[key.p] = n->get<int>();
    } else if(n->isOfType<uint>()) {
      dict[key.p] = n->get<uint>();
    } else if(n->isOfType<bool>()) {
      dict[key.p] = n->get<bool>();
    } else if(n->isOfType<rai::Enum<rai::ShapeType>>()) {
      dict[key.p] = n->get<rai::Enum<rai::ShapeType>>().name();
    } else {
      LOG(-1) <<"can't convert node of type " <<n->type.name() <<" to dictionary";
    }
  }
  return dict;
}

pybind11::list graph2list(const rai::Graph& G) {
  pybind11::list list;
  for(rai::Node* n:G) {
    //-- write value
    if(n->isGraph()) {
      list.append(graph2dict(n->get<rai::Graph>()));
    } else if(n->isOfType<rai::String>()) {
      list.append(n->get<rai::String>().p);
    } else if(n->isOfType<arr>()) {
      list.append(n->get<arr>().vec());
    } else if(n->isOfType<double>()) {
      list.append(n->get<double>());
    } else if(n->isOfType<int>()) {
      list.append(n->get<int>());
    } else if(n->isOfType<uint>()) {
      list.append(n->get<uint>());
    } else if(n->isOfType<bool>()) {
      list.append(n->get<bool>());
    } else {
    }
  }
  return list;
}

pybind11::tuple uintA2tuple(const uintA& tup) {
  pybind11::tuple tuple;
  for(uint i=0; i<tup.N; i++) tuple[i] = tup(i);
  return tuple;
}

arr numpy2arr(const pybind11::array& X) {
  arr Y;
  uintA dim(X.ndim());
  for(uint i=0; i<dim.N; i++) dim(i)=X.shape()[i];
  Y.resize(dim);
  auto ref = X.unchecked<double>();
  if(Y.nd==1) {
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

byteA numpy2arr(const pybind11::array_t<byte>& X) {
  byteA Y;
  uintA dim(X.ndim());
  for(uint i=0; i<dim.N; i++) dim(i)=X.shape()[i];
  Y.resize(dim);
  auto ref = X.unchecked<>();
  if(Y.nd==1) {
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

arr vecvec2arr(const std::vector<std::vector<double>>& X) {
  CHECK(X.size()>0, "");
  arr Y(X.size(), X[0].size());
  for(uint i=0; i<Y.d0; i++) for(uint j=0; j<Y.d1; j++) Y(i, j) = X[i][j];
  return Y;
}

#endif
