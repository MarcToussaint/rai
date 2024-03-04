/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PYBIND

#include "py-LGP_Tree.h"
#include "types.h"

#include "../Kin/viewer.h"
#include "../LGP/LGP_tree.h"

void init_LGP_Tree(pybind11::module& m) {
  pybind11::class_<rai::LGP_Tree, std::shared_ptr<rai::LGP_Tree>>(m, "LGP_Tree")
  .def("walkToNode", [](rai::LGP_Tree& self, const char* seq) {
    self.walkToNode(seq);
  })

  .def("walkToRoot", [](rai::LGP_Tree& self) {
    self.focusNode = self.root;
  })

  .def("walkToParent", [](rai::LGP_Tree& self) {
    self.focusNode = self.focusNode->parent;
  })

  .def("walkToDecision", [](rai::LGP_Tree& self, uint decision) {
    rai::LGP_Node* focusNode = self.focusNode;
    if(!focusNode->isExpanded) focusNode->expand();
    self.focusNode = focusNode->children(decision);
  })

  .def("getDecisions", [](rai::LGP_Tree& self) {
    rai::LGP_Node* focusNode = self.focusNode;
    if(!focusNode->isExpanded) focusNode->expand();
    StringA decisions(focusNode->children.N);
    uint c=0;
    for(rai::LGP_Node* a:focusNode->children) {
      decisions(c++) <<*a->decision;
    }
    return I_conv(decisions);
  })

  .def("nodeInfo", [](rai::LGP_Tree& self) {
    rai::Graph G = self.focusNode->getInfo();
    LOG(0) <<G;
    return graph2dict(G);
  })

  .def("viewTree", [](rai::LGP_Tree& self) {
    self.displayTreeUsingDot();
  })

  .def("optBound", [](rai::LGP_Tree& self, rai::BoundType bound, bool collisions) {
    rai::ConfigurationViewer V;
    self.focusNode->optBound(bound, collisions);
    self.focusNode->displayBound(V, bound); //(.02, false, false);
  })

//  .def("getKOMOforBound", [](rai::LGP_Tree& self, rai::BoundType bound) {
//    return self.focusNode->komoProblem(bound);
//  })

  .def("addTerminalRule", [](rai::LGP_Tree& self, const char* precondition) {
    self.fol.addTerminalRule(precondition);
  })

  .def("run", [](rai::LGP_Tree& self, int verbose) {
    self.displayBound = rai::BD_seqPath;
    self.LGP_Tree::verbose=verbose;
    self.run();
  })

  .def("stop", [](rai::LGP_Tree& self) {
//    self.threadStop();
  })

  .def("numSolutions", [](rai::LGP_Tree& self) {
    return self.solutions.get()->N;
  })

//  .def("getReport", [](rai::LGP_Tree& self, uint solution, rai::BoundType bound) {
//    rai::Graph G = self.getReport(solution, bound);
//    return graph2list(G);
//  })

//  .def("getKOMO", [](rai::LGP_Tree& self, uint solution, rai::BoundType bound) {
//    const auto& komo = self.getKOMO(solution, bound);
//    return komo;
//  })
  ;
}

#endif
