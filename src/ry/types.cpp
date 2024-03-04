/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PYBIND

#include "types.h"
#include "../Geo/mesh.h"
#include "../Kin/frame.h"

pybind11::dict graph2dict(const rai::Graph& G) {
  pybind11::dict dict;
  for(rai::Node* n:G) {
    str key;
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
    } else if(n->is<str>()) {
      dict[key.p] = n->as<str>().p;
    } else if(n->is<rai::FileToken>()) {
      dict[key.p] = n->as<rai::FileToken>().autoPath().p;
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
    } else if(n->is<StringA>()) {
      dict[key.p] = StringA2list(n->as<StringA>());
    } else if(n->is<rai::Enum<rai::ShapeType>>()) {
      dict[key.p] = n->as<rai::Enum<rai::ShapeType>>().name();
    } else if(n->is<rai::Enum<rai::JointType>>()) {
      dict[key.p] = n->as<rai::Enum<rai::JointType>>().name();
    } else {
      LOG(-1) <<"can't convert node of type " <<n->type.name() <<" to dictionary";
    }
  }
  return dict;
}

rai::Graph dict2graph(const pybind11::dict& dict) {
  rai::Graph G;
  for(auto item:dict) {
    str key = item.first.cast<std::string>().c_str();
    pybind11::handle value = item.second;

    if(pybind11::isinstance<pybind11::bool_>(value)) {
//      LOG(0) <<"converting bool";
      G.add<bool>(key, value.cast<bool>());

    } else if(pybind11::isinstance<pybind11::float_>(value)) {
//      LOG(0) <<"converting float";
      G.add<double>(key, value.cast<double>());

    } else if(pybind11::isinstance<pybind11::int_>(value)) {
//      LOG(0) <<"converting int";
      G.add<int>(key, value.cast<int>());

    } else if(pybind11::isinstance<pybind11::list>(value)) {
//      LOG(0) <<"converting list";
      pybind11::list L = value.cast<pybind11::list>();
      if(pybind11::isinstance<pybind11::float_>(L[0])
          || pybind11::isinstance<pybind11::int_>(L[0])) {
//        LOG(0) <<"number list";
        G.add<arr>(key, list2arr<double>(L));
      } else if(pybind11::isinstance<pybind11::str>(L[0])) {
//        LOG(0) <<"string list";
        G.add<StringA>(key, list2StringA(L));
      } else LOG(-1) <<"can't convert dict entry '" <<key <<"' of type " <<value.get_type() <<" to graph";

    } else if(pybind11::isinstance<pybind11::array_t<double>>(value)
              || pybind11::isinstance<pybind11::array_t<float>>(value)
              || pybind11::isinstance<pybind11::array_t<int>>(value)) {
//      LOG(0) <<"converting arr";
      auto val = value.cast<pybind11::array_t<double>>();
      G.add<arr>(key, numpy2arr(val));

    } else if(pybind11::isinstance<pybind11::str>(value)) {
      G.add<str>(key, value.cast<std::string>().c_str());

    } else if(pybind11::isinstance<pybind11::dict>(value)) {
      G.addSubgraph(key) = dict2graph(value.cast<pybind11::dict>());

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
    } else if(n->is<str>()) {
      list.append(n->as<str>().p);
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
