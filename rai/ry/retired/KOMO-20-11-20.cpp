.def("add_StableRelativePose", [](std::shared_ptr<KOMO>& self, const std::vector<int>& confs, const char* gripper, const char* object) {
  for(uint i=1; i<confs.size(); i++)
    self->addObjective(ARR(confs[0], confs[i]), FS_poseDiff, {gripper, object}, OT_eq);
  //  for(uint i=0;i<confs.size();i++) self.self->configurations(self.self->k_order+confs[i]) -> makeObjectsFree({object});
  self->world.makeObjectsFree({object});
}, "", pybind11::arg("confs"),
pybind11::arg("gripper"),
pybind11::arg("object"))

.def("add_StablePose", [](std::shared_ptr<KOMO>& self, const std::vector<int>& confs, const char* object) {
  for(uint i=1; i<confs.size(); i++)
    self->addObjective(ARR(confs[0], confs[i]), FS_pose, {object}, OT_eq);
  //  for(uint i=0;i<confs.size();i++) self.self->configurations(self.self->k_order+confs[i]) -> makeObjectsFree({object});
  self->world.makeObjectsFree({object});
}, "", pybind11::arg("confs"),
pybind11::arg("object"))

.def("add_grasp", [](std::shared_ptr<KOMO>& self, int conf, const char* gripper, const char* object) {
  self->addObjective(ARR(conf), FS_distance, {gripper, object}, OT_eq);
})

.def("add_place", [](std::shared_ptr<KOMO>& self, int conf, const char* object, const char* table) {
  self->addObjective(ARR(conf), FS_aboveBox, {table, object}, OT_ineq);
  self->addObjective(ARR(conf), FS_standingAbove, {table, object}, OT_eq);
  self->addObjective(ARR(conf), FS_vectorZ, {object}, OT_sos, {}, {0., 0., 1.});
})

.def("add_resting", [](std::shared_ptr<KOMO>& self, int conf1, int conf2, const char* object) {
  self->addObjective(ARR(conf1, conf2), FS_pose, {object}, OT_eq);
})

.def("add_restingRelative", [](std::shared_ptr<KOMO>& self, int conf1, int conf2, const char* object, const char* tableOrGripper) {
  self->addObjective(ARR(conf1, conf2), FS_poseDiff, {tableOrGripper, object}, OT_eq);
})

.def("addSkeleton", [](std::shared_ptr<KOMO>& self, const pybind11::list& L) {
  Skeleton S = list2skeleton(L);
  cout <<"SKELETON: " <<S <<endl;
  self->setSkeleton(S);
//    skeleton2Bound(*self.komo, BD_path, S, self->world, self->world, false);
})

//.def("addSkeletonBound", [](std::shared_ptr<KOMO>& self, const pybind11::list& L, BoundType boundType, bool collisions) {
//  Skeleton S = list2skeleton(L);
//  cout <<"SKELETON: " <<S <<endl;
////    self->setSkeleton(S);
//  skeleton2Bound(self.komo, boundType, S, self->world, self->world, collisions);
//})
