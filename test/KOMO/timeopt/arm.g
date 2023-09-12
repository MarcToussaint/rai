
stem { X: "t(0 0 1)" shape:capsule size=[0.1 0.1 2 .1] }

arm1 { shape:capsule size=[0.1 0.1 .4 .1] contact:-1, }
arm2 { shape:capsule size=[0.1 0.1 .4 .1] contact:-1, }
arm3 { shape:capsule size=[0.1 0.1 .4 .1] contact:-1, }
arm4 { shape:capsule size=[0.1 0.1 .4 .1] contact:-1, }
arm5 { shape:capsule size=[0.1 0.1 .4 .1] contact:-1, }
arm6 { shape:capsule size=[0.1 0.1 .4 .1] contact:-1, }
arm7 { shape:capsule size=[0.1 0.1 .4 .1] contact:-1, }

(stem arm1) { joint:hingeX A: "t(0 0 1) d(90 1 0 0)" B: "t(0 0 .2)"  Q: "d(1 0 0 0)" }

(arm1 arm2) { joint:hingeX A: "t(0 0 0.2) d(45 0 0 1)" B: "t(0 0 .2)"  Q: "d(1 0 0 0)" }
(arm2 arm3) { joint:hingeX A: "t(0 0 0.2) d(45 0 0 1)" B: "t(0 0 .2)"  Q: "d(1 0 0 0)" }
(arm3 arm4) { joint:hingeX A: "t(0 0 0.2) d(45 0 0 1)" B: "t(0 0 .2)"  Q: "d(1 0 0 0)" }
(arm4 arm5) { joint:hingeX A: "t(0 0 0.2) d(45 0 0 1)" B: "t(0 0 .2)"  Q: "d(1 0 0 0)" }
(arm5 arm6) { joint:hingeX A: "t(0 0 0.2) d(45 0 0 1)" B: "t(0 0 .2)"  Q: "d(1 0 0 0)" }
(arm6 arm7) { joint:hingeX A: "t(0 0 0.2) d(45 0 0 1)" B: "t(0 0 .2)"  Q: "d(1 0 0 0)" }

target { X: "t(.7 -.5 1.2)"  shape:sphere size=[.1 .1 .1 .05] color=[0 .5 0] }

obstacle { X: "t(1. -1.5 1.)"  shape:sphere size=[1. .1 5. .5] color=[1 0 0] contact, }

endeff(arm7){ shape:marker rel: "t(0 0 .3)" size=[.1 .1 .1 0] } # a marker shape at the tip of arm7
