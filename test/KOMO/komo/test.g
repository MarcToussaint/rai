
body stem { X=<T t(0 0 1)> type=2 size=[0.1 0.1 2 .1] fixed, }

body arm1 { type=2 size=[0.1 0.1 .4 .1] contact, }
body arm2 { type=2 size=[0.1 0.1 .4 .1] contact, }
body arm3 { type=2 size=[0.1 0.1 .4 .1] contact, }
body arm4 { type=2 size=[0.1 0.1 .4 .1] contact, }
body arm5 { type=2 size=[0.1 0.1 .4 .1] contact, }
body arm6 { type=2 size=[0.1 0.1 .4 .1] contact, }
body arm7 { type=2 size=[0.1 0.1 .4 .1] contact, }

joint (stem arm1) { A=<T t(0 0 1) d(90 1 0 0)> B=<T t(0 0 .2)>  Q=<T d(1 0 0 0)> }

joint (arm1 arm2) { A=<T t(0 0 0.2) d(45 0 0 1)> B=<T t(0 0 .2)>  Q=<T d(1 0 0 0)> }
joint (arm2 arm3) { A=<T t(0 0 0.2) d(45 0 0 1)> B=<T t(0 0 .2)>  Q=<T d(1 0 0 0)> }
joint (arm3 arm4) { A=<T t(0 0 0.2) d(45 0 0 1)> B=<T t(0 0 .2)>  Q=<T d(1 0 0 0)> }
joint (arm4 arm5) { A=<T t(0 0 0.2) d(45 0 0 1)> B=<T t(0 0 .2)>  Q=<T d(1 0 0 0)> }
joint (arm5 arm6) { A=<T t(0 0 0.2) d(45 0 0 1)> B=<T t(0 0 .2)>  Q=<T d(1 0 0 0)> }
joint (arm6 arm7) { A=<T t(0 0 0.2) d(45 0 0 1)> B=<T t(0 0 .2)>  Q=<T d(1 0 0 0)> }

body target { X=<T t(1. -.5 1.)>  type=1 size=[.1 .1 .1 .05] color=[0 .5 0] fixed, }

#body fileshape { X=<T t(-0.25 -3 1.8)>  type=3 size=[.3 .3 .3 .1] color=[1 0 0] shapefile='c:/home/3dmodel/benchmark/offs/m494.off' swiftfile='c:/home/3dmodel/benchmark/offs/m494.dcp' contact, fixed, }
#body ball  { X=<T t(.0 -1.0 2.1)>  type=1 size=[.3 .3 .3 .1] color=[1 0 0] contact, fixed, }
#body ball2 { X=<T t(.7 -1. 1.5)>  type=1 size=[.3 .3 .3 .1] color=[1 0 0] contact, fixed, }
#body ball3 { X=<T t(-.22 -1.5 2.)>  type=1 size=[.3 .3 .3 .1] color=[1 0 0] contact, fixed, }
body obstacle { X=<T t(1. -1.5 .8)>  type=1 size=[1. .1 5. .5] color=[1 0 0] contact, fixed, }

shape endeff(arm7){ type=5 rel=<T t(0 0 .3)> size=[.1 .1 .1 0] } # a marker shape at the tip of arm7
