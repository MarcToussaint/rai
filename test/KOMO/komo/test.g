
stem { X: "t(0 0 1)" type=2 size=[0.1 0.1 2 .1] fixed, }

arm1 { type=2 size=[0.1 0.1 .4 .1] contact, }
arm2 { type=2 size=[0.1 0.1 .4 .1] contact, }
arm3 { type=2 size=[0.1 0.1 .4 .1] contact, }
arm4 { type=2 size=[0.1 0.1 .4 .1] contact, }
arm5 { type=2 size=[0.1 0.1 .4 .1] contact, }
arm6 { type=2 size=[0.1 0.1 .4 .1] contact, }
arm7 { type=2 size=[0.1 0.1 .4 .1] contact, }

(stem arm1) { A: "t(0 0 1) d(90 1 0 0)" B: "t(0 0 .2)"  Q: "d(1 0 0 0)" }

(arm1 arm2) { A: "t(0 0 0.2) d(45 0 0 1)" B: "t(0 0 .2)"  Q: "d(1 0 0 0)" }
(arm2 arm3) { A: "t(0 0 0.2) d(45 0 0 1)" B: "t(0 0 .2)"  Q: "d(1 0 0 0)" }
(arm3 arm4) { A: "t(0 0 0.2) d(45 0 0 1)" B: "t(0 0 .2)"  Q: "d(1 0 0 0)" }
(arm4 arm5) { A: "t(0 0 0.2) d(45 0 0 1)" B: "t(0 0 .2)"  Q: "d(1 0 0 0)" }
(arm5 arm6) { A: "t(0 0 0.2) d(45 0 0 1)" B: "t(0 0 .2)"  Q: "d(1 0 0 0)" }
(arm6 arm7) { A: "t(0 0 0.2) d(45 0 0 1)" B: "t(0 0 .2)"  Q: "d(1 0 0 0)" }

target { X: "t(1. -.5 1.)"  type=1 size=[.1 .1 .1 .05] color=[0 .5 0] fixed, }

#fileshape { X: "t(-0.25 -3 1.8)"  type=3 size=[.3 .3 .3 .1] color=[1 0 0] shapefile='c:/home/3dmodel/benchmark/offs/m494.off' swiftfile='c:/home/3dmodel/benchmark/offs/m494.dcp' contact, fixed, }
#ball  { X: "t(.0 -1.0 2.1)"  type=1 size=[.3 .3 .3 .1] color=[1 0 0] contact, fixed, }
#ball2 { X: "t(.7 -1. 1.5)"  type=1 size=[.3 .3 .3 .1] color=[1 0 0] contact, fixed, }
#ball3 { X: "t(-.22 -1.5 2.)"  type=1 size=[.3 .3 .3 .1] color=[1 0 0] contact, fixed, }
obstacle { X: "t(1. -1.5 .8)"  type=1 size=[1. .1 5. .5] color=[1 0 0] contact, fixed, }

endeff(arm7){ type=5 rel: "t(0 0 .3)" size=[.1 .1 .1 0] } # a marker shape at the tip of arm7
