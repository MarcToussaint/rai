/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PYBIND

#include "ry-LGP_Tree.h"
#include "types.h"

#include "../LGP/LGP_tree.h"

void init_LGP_Tree(pybind11::module& m) {
  pybind11::class_<ry::RyLGP_Tree>(m, "LGP_Tree")
  .def("walkToNode", [](ry::RyLGP_Tree& self, const char* seq) {
    self.lgp->walkToNode(seq);
  })

  .def("walkToRoot", [](ry::RyLGP_Tree& self) {
    self.lgp->focusNode = self.lgp->root;
  })

  .def("walkToParent", [](ry::RyLGP_Tree& self) {
    self.lgp->focusNode = self.lgp->focusNode->parent;
  })

  .def("walkToDecision", [](ry::RyLGP_Tree& self, uint decision) {
    LGP_Node* focusNode = self.lgp->focusNode;
    if(!focusNode->isExpanded) focusNode->expand();
    self.lgp->focusNode = focusNode->children(decision);
  })

  .def("getDecisions", [](ry::RyLGP_Tree& self) {
    LGP_Node* focusNode = self.lgp->focusNode;
    if(!focusNode->isExpanded) focusNode->expand();
    StringA decisions(focusNode->children.N);
    uint c=0;
    for(LGP_Node* a:focusNode->children) {
      decisions(c++) <<*a->decision;
    }
    return I_conv(decisions);
  })

  .def("nodeInfo", [](ry::RyLGP_Tree& self) {
    Graph G = self.lgp->focusNode->getInfo();
    LOG(0) <<G;
    return graph2dict(G);
  })

  .def("viewTree", [](ry::RyLGP_Tree& self) {
    self.lgp->displayTreeUsingDot();
  })

  .def("optBound", [](ry::RyLGP_Tree& self, BoundType bound, bool collisions) {
    self.lgp->focusNode->optBound(bound, collisions);
    if(bound == BD_seqPath) {
      self.lgp->focusNode->komoProblem(bound)->displayTrajectory(.02, false, false);
    } else {
      self.lgp->focusNode->komoProblem(bound)->displayTrajectory(.1, false, false);
    }
  })

  .def("getKOMOforBound", [](ry::RyLGP_Tree& self, BoundType bound) {
    return self.lgp->focusNode->komoProblem(bound);
  })

  .def("addTerminalRule", [](ry::RyLGP_Tree& self, const char* precondition) {
    self.lgp->fol.addTerminalRule(precondition);
  })

  .def("run", [](ry::RyLGP_Tree& self, int verbose) {
    self.lgp->displayBound = BD_seqPath;
    self.lgp->LGP_Tree::verbose=verbose;
    self.lgp->threadLoop();
  })

  .def("stop", [](ry::RyLGP_Tree& self) {
    self.lgp->threadStop();
  })

  .def("numSolutions", [](ry::RyLGP_Tree& self) {
    return self.lgp->numSolutions();
  })

  .def("getReport", [](ry::RyLGP_Tree& self, uint solution, BoundType bound) {
    Graph G = self.lgp->getReport(solution, bound);
    return graph2list(G);
  })

  .def("getKOMO", [](ry::RyLGP_Tree& self, uint solution, BoundType bound) {
    const auto& komo = self.lgp->getKOMO(solution, bound);
    return komo;
  })
  ;
}

#endif
