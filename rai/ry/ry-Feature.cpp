/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PYBIND

#include "ry-Feature.h"
#include "ry-Config.h"
#include "types.h"

#include "../Kin/feature.h"

void init_Feature(pybind11::module& m) {
  pybind11::class_<Feature, shared_ptr<Feature>>(m, "Feature")
      .def("setOrder", &Feature::setOrder)
      .def("setScale", &Feature::setScale)
      .def("setTarget", &Feature::setTarget)
//      .def("setFrameIDs", &Feature::setFrameIDs)
//      .def("setDiffInsteadOfVel", &Feature::setDiffInsteadOfVel)


  .def("eval", [](shared_ptr<Feature>& self, shared_ptr<rai::Configuration>& C) {
    arr y, J;
    self->__phi(y, J, *C);
    pybind11::tuple ret(2);
    ret[0] = pybind11::array(y.dim(), y.p);
    ret[1] = pybind11::array(J.dim(), J.p);
    return ret;
  })
//  .def("eval", [](shared_ptr<Feature>& self, pybind11::tuple& Kpytuple) {
//    ConfigurationL Ktuple;
//    for(uint i=0; i<Kpytuple.size(); i++) {
//      shared_ptr<rai::Configuration>& K = Kpytuple[i].cast<shared_ptr<rai::Configuration>& >();
//      Ktuple.append(&K.set()());
//    }

//    arr y, J;
//    self.feature->order=Ktuple.N-1;
//    self.feature->__phi(y, J, Ktuple);
//    cout <<"THERE!!" <<J.dim() <<endl;
//    pybind11::tuple ret(2);
//    ret[0] = pybind11::array(y.dim(), y.p);
//    ret[1] = pybind11::array(J.dim(), J.p);
//    return ret;
//  })
  .def("description", [](shared_ptr<Feature>& self, shared_ptr<rai::Configuration>& C) {
    std::string s = self->shortTag(*C).p;
    return s;
  })
  ;
}

#endif
