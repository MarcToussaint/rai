
stem { X: "t(0 0 1)" shape:capsule size=[2 .1] }

arm1(j1) { shape:capsule Q:[0 0 .2], size=[.4 .1] contact:-1, }
arm2(j2) { shape:capsule Q:[0 0 .2], size=[.4 .1] contact:-1, }
arm3(j3) { shape:capsule Q:[0 0 .2], size=[.4 .1] contact:-1, }
arm4(j4) { shape:capsule Q:[0 0 .2], size=[.4 .1] contact:-1, }
arm5(j5) { shape:capsule Q:[0 0 .2], size=[.4 .1] contact:-1, }
arm6(j6) { shape:capsule Q:[0 0 .2], size=[.4 .1] contact:-1, }
arm7(j7) { shape:capsule Q:[0 0 .2], size=[.4 .1] contact:-1, }

j1(stem) { joint:hingeX A: "t(0 0 1) d(90 1 0 0)" }
j2(arm1) { joint:hingeX A: "t(0 0 0.2) d(45 0 0 1)" }
j3(arm2) { joint:hingeX A: "t(0 0 0.2) d(45 0 0 1)" }
j4(arm3) { joint:hingeX A: "t(0 0 0.2) d(45 0 0 1)" }
j5(arm4) { joint:hingeX A: "t(0 0 0.2) d(45 0 0 1)" }
j6(arm5) { joint:hingeX A: "t(0 0 0.2) d(45 0 0 1)" }
j7(arm6) { joint:hingeX A: "t(0 0 0.2) d(45 0 0 1)" }

target { X: "t(1. -.5 1.)"  shape: sphere size=[.05] color=[0 .5 0] }

#fileshape { X: "t(-0.25 -3 1.8)"  type: mesh size=[.3 .3 .3 .1] color=[1 0 0] shapefile='c:/home/3dmodel/benchmark/offs/m494.off' swiftfile='c:/home/3dmodel/benchmark/offs/m494.dcp' contact, fixed, }
#ball  { X: "t(.0 -1.0 2.1)"  type: sphere size=[.3 .3 .3 .1] color=[1 0 0] contact, fixed, }
#ball2 { X: "t(.7 -1. 1.5)"  type: sphere size=[.3 .3 .3 .1] color=[1 0 0] contact, fixed, }
#ball3 { X: "t(-.22 -1.5 2.)"  type: sphere size=[.3 .3 .3 .1] color=[1 0 0] contact, fixed, }
obstacle { X: "t(1. -1.5 .8)"  shape: sphere size=[.5] color=[1 0 0] contact }

endeff(arm7){ shape: marker pose: "t(0 0 .3)" size=[.1] } # a marker shape at the tip of arm7
