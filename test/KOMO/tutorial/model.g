
stem { X: "t(0 0 1)" shape:capsule size=[0.1 0.1 2 .1] }

arm1 { shape:capsule size=[0.1 0.1 .4 .1] contact, }
arm2 { shape:capsule size=[0.1 0.1 .4 .1] contact, }
arm3 { shape:capsule size=[0.1 0.1 .4 .1] contact, }
arm4 { shape:capsule size=[0.1 0.1 .4 .1] contact, }
arm5 { shape:capsule size=[0.1 0.1 .4 .1] contact, }
arm6 { shape:capsule size=[0.1 0.1 .4 .1] contact, }
arm7 { shape:capsule size=[0.1 0.1 .4 .1] contact, }

(stem arm1) { joint:hingeX A: "t(0 0 1) d(90 1 0 0)" B: "t(0 0 .2)"  limits:[-2 2]}

(arm1 arm2) { joint:hingeX A: "t(0 0 0.2) d(45 0 0 1)" B: "t(0 0 .2)"  limits:[-2 2] }
(arm2 arm3) { joint:hingeX A: "t(0 0 0.2) d(45 0 0 1)" B: "t(0 0 .2)"  limits:[-2 2] }
(arm3 arm4) { joint:hingeX A: "t(0 0 0.2) d(45 0 0 1)" B: "t(0 0 .2)"  limits:[-2 2] }
(arm4 arm5) { joint:quatBall A: "t(0 0 0.2) d(45 0 0 1)" B: "t(0 0 .2)"  limits:[0 1.5 -1 1 -1 1 -1 1] }
(arm5 arm6) { joint:hingeX A: "t(0 0 0.2) d(45 0 0 1)" B: "t(0 0 .2)"  limits:[-2 2] }
(arm6 arm7) { joint:hingeX A: "t(0 0 0.2) d(45 0 0 1)" B: "t(0 0 .2)"  limits:[-2 2] }

#target { X: "t(.5 -1. 2.) q(.939 0 0 .34)"  shape:box size=[.3 .1 .2 .05] color=[0 .5 0] }
target { X: "t(.5 -1. 2.) d(-20 0 1 0) d(20 0 0 1)"  shape:box size=[.3 .1 .2 .05] color=[0 .5 0] }

endeff(arm7){ shape:marker rel: "t(0 0 .3)" size=[.3 .1 .1 0] } # a marker shape at the tip of arm7
