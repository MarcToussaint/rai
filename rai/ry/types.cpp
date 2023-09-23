/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PYBIND

#include "types.h"
#include "../Geo/mesh.h"


pybind11::dict graph2dict(const rai::Graph& G) {
  pybind11::dict dict;
  for(rai::Node* n:G) {
    rai::String key;
    if(n->key.N) key=n->key;
    else key <<n->index;

    //-- write value
    if(n->is<rai::Graph>()) {
      dict[key.p] = graph2dict(n->as<rai::Graph>());
    } else if(n->is<double>()) {
      dict[key.p] = n->as<double>();
    } else if(n->is<int>()) {
      dict[key.p] = n->as<int>();
    } else if(n->is<uint>()) {
      dict[key.p] = n->as<uint>();
    } else if(n->is<bool>()) {
      dict[key.p] = n->as<bool>();
    } else if(n->is<rai::String>()) {
      dict[key.p] = n->as<rai::String>().p;
    } else if(n->is<arr>()) {
      dict[key.p] = n->as<arr>().vec();
    } else if(n->is<arrA>()) {
      dict[key.p] = Array2vec(n->as<arrA>());
    } else if(n->is<intA>()) {
      dict[key.p] = Array2vec(n->as<intA>());
    } else if(n->is<uintA>()) {
      dict[key.p] = Array2vec(n->as<uintA>());
    } else if(n->is<boolA>()) {
      dict[key.p] = Array2vec(n->as<boolA>());
    } else if(n->is<rai::Enum<rai::ShapeType>>()) {
      dict[key.p] = n->as<rai::Enum<rai::ShapeType>>().name();
    } else {
      LOG(-1) <<"can't convert node of type " <<n->type.name() <<" to dictionary";
    }
  }
  return dict;
}

rai::Graph dict2graph(const pybind11::dict& dict) {
  rai::Graph G;
  for(auto item:dict) {
    rai::String key = item.first.cast<std::string>().c_str();
    pybind11::handle value = item.second;

    if(pybind11::isinstance<pybind11::float_>(value)) {
      G.add<double>(key, value.cast<double>());
    }else if(pybind11::isinstance<pybind11::int_>(value)) {
      G.add<int>(key, value.cast<int>());
    }else if(pybind11::isinstance<pybind11::bool_>(value)) {
      G.add<bool>(key, value.cast<bool>());
    }else if(pybind11::isinstance<pybind11::str>(value)) {
        G.add<rai::String>(key, value.cast<std::string>().c_str());
    } else {
      LOG(-1) <<"can't convert dict entry '" <<key <<"' of type " <<value.get_type() <<" to graph";
    }
  }
  return G;
}

pybind11::list graph2list(const rai::Graph& G) {
  pybind11::list list;
  for(rai::Node* n:G) {
    //-- write value
    if(n->is<rai::Graph>()) {
      list.append(graph2dict(n->as<rai::Graph>()));
    } else if(n->is<rai::String>()) {
      list.append(n->as<rai::String>().p);
    } else if(n->is<arr>()) {
      list.append(n->as<arr>().vec());
    } else if(n->is<double>()) {
      list.append(n->as<double>());
    } else if(n->is<int>()) {
      list.append(n->as<int>());
    } else if(n->is<uint>()) {
      list.append(n->as<uint>());
    } else if(n->is<bool>()) {
      list.append(n->as<bool>());
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
