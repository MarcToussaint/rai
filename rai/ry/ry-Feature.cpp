#ifdef RAI_PYBIND

#include "ry-Feature.h"
#include "ry-Config.h"
#include "types.h"

#include "../Kin/feature.h"

void init_Feature(pybind11::module &m) {
pybind11::class_<ry::RyFeature>(m, "Feature")
.def("eval", [](ry::RyFeature& self, ry::Config& K) {
  arr y, J;
  self.feature->__phi(y, J, K.get());
  pybind11::tuple ret(2);
  ret[0] = pybind11::array(y.dim(), y.p);
  ret[1] = pybind11::array(J.dim(), J.p);
  return ret;
})
.def("eval", [](ry::RyFeature& self, pybind11::tuple& Kpytuple) {
  ConfigurationL Ktuple;
  for(uint i=0; i<Kpytuple.size(); i++) {
    ry::Config& K = Kpytuple[i].cast<ry::Config&>();
    Ktuple.append(&K.set()());
  }

  arr y, J;
  self.feature->order=Ktuple.N-1;
  self.feature->__phi(y, J, Ktuple);
  cout <<"THERE!!" <<J.dim() <<endl;
  pybind11::tuple ret(2);
  ret[0] = pybind11::array(y.dim(), y.p);
  ret[1] = pybind11::array(J.dim(), J.p);
  return ret;
})
.def("description", [](ry::RyFeature& self, ry::Config& K) {
  std::string s = self.feature->shortTag(K.get()).p;
  return s;
})
;
}

#endif
