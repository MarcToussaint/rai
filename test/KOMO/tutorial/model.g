
stem { X: "t(0 0 1)" shape:capsule size=[2 .1] }

arm1(j1) { shape:capsule Q:[0 0 .2], size=[.4 .1] contact:-1, }
arm2(j2) { shape:capsule Q:[0 0 .2], size=[.4 .1] contact:-1, }
arm3(j3) { shape:capsule Q:[0 0 .2], size=[.4 .1] contact:-1, }
arm4(j4) { shape:capsule Q:[0 0 .2], size=[.4 .1] contact:-1, }
arm5(j5) { shape:capsule Q:[0 0 .2], size=[.4 .1] contact:-1, }
arm6(j6) { shape:capsule Q:[0 0 .2], size=[.4 .1] contact:-1, }
arm7(j7) { shape:capsule Q:[0 0 .2], size=[.4 .1] contact:-1, }

j1(stem) { joint:hingeX A: "t(0 0 1) d(90 1 0 0)" limits:[-2 2] }
j2(arm1) { joint:hingeX A: "t(0 0 0.2) d(45 0 0 1)" limits:[-2 2] }
j3(arm2) { joint:hingeX A: "t(0 0 0.2) d(45 0 0 1)" limits:[-2 2] }
j4(arm3) { joint:hingeX A: "t(0 0 0.2) d(45 0 0 1)" limits:[-2 2] }
j5(arm4) { joint:quatBall A: "t(0 0 0.2) d(45 0 0 1)" limits:[0 -1 -1 -1 1.5 1 1 1] }
j6(arm5) { joint:hingeX A: "t(0 0 0.2) d(45 0 0 1)" limits:[-2 2] }
j7(arm6) { joint:hingeX A: "t(0 0 0.2) d(45 0 0 1)" limits:[-2 2] }

#target { X: "t(.5 -1. 2.) q(.939 0 0 .34)"  shape:box size=[.3 .1 .2] color=[0 .5 0] }
target { X: "t(.5 -1. 2.) d(-20 0 1 0) d(20 0 0 1)"  shape:box size=[.3 .1 .2] color=[0 .5 0] }

endeff(arm7){ shape:marker pose: "t(0 0 .3)" size=[.3] } # a marker shape at the tip of arm7
