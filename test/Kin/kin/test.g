world { }
stem { X: "t(0 0 1)"  shape:capsule mass=.1 size=[0.1 0.1 2 .1] }
arm1 { shape:capsule mass=.1 size=[0.1 0.1 .4 .1] }
arm2 { shape:capsule mass=.1 size=[0.1 0.1 .4 .1] }
arm3 { shape:capsule mass=.1 size=[0.1 0.1 .4 .1] }

(world stem) { joint:transXYPhi }
(stem arm1) { joint:hingeX A: "t(0 0 1) d(90 1 0 0) " B: "t(0 0 .2) "  }
j4 (arm1 arm2) { joint:hingeX A: "t(0 0 0.2) d(45 0 0 1) " B: "t(0 0 .2) "  }
(arm2 arm3) { joint:hingeX A: "t(0 0 0.2) d(45 0 0 1) " B: "t(0 0 .2) "  coupledTo=j4 }

