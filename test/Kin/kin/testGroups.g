
body stem { X=<T t(0 0 1)>  type=capsule mass=.1 size=[0.1 0.1 2 .1] fixed, }

body arm1 { type=capsule mass=.1 size=[0.1 0.1 .4 .1] }
body arm2 { type=capsule mass=.1 size=[0.1 0.1 .4 .1] }
body arm3 { type=capsule mass=.1 size=[0.1 0.1 .4 .1] }
body arm4 { type=capsule mass=.1 size=[0.1 0.1 .4 .1] }
body arm5 { type=capsule mass=.1 size=[0.1 0.1 .4 .1] contact }
body arm6 { type=capsule mass=.1 size=[0.1 0.1 .4 .1] contact }
body arm7 { type=capsule mass=.1 size=[0.1 0.1 .4 .1] contact }

shape endeff(arm7){ type=marker rel=<T t(0 0 .3)> size=[.1 0 0 0] }

#body fileshape { X=<T t(-0.25 -3 1.8)>  type=mesh mass=.1 size=[.3 .3 .3 .1] color=[1 0 0] shapefile='c:/home/3dmodel/benchmark/offs/m494.off' swiftfile='c:/home/3dmodel/benchmark/offs/m494.dcp' contact, }
#body ball  { X=<T t(.0 -1.0 2.1)>  type=sphere mass=.1 size=[.3 .3 .3 .1] color=[1 0 0] contact }
body ball2 { X=<T t(.15 -2.5 1.9)>  type=box mass=.1 size=[.6 .1 .6 .3] color=[1 0 0] contact }
#body ball3 { X=<T t(-.22 -1.5 2.)>  type=sphere mass=.1 size=[.3 .3 .3 .1] color=[1 0 0] contact }

joint (stem arm1) { A=<T t(0 0 1) d(90 1 0 0) > B=<T t(0 0 .2) >  limits=[-.1 .1]}

joint (arm1 arm2) { A=<T t(0 0 0.2) d(45 0 0 1) > B=<T t(0 0 .2) > limits=[-.1 .1] }
joint (arm2 arm3) { A=<T t(0 0 0.2) d(45 0 0 1) > B=<T t(0 0 .2) > limits=[-.1 .1] }
joint (arm3 arm4) { A=<T t(0 0 0.2) d(45 0 0 1) > B=<T t(0 0 .2) > limits=[-.1 .1] agent=1 }
joint j4(arm4 arm5) { A=<T t(0 0 0.2) d(45 0 0 1) > B=<T t(0 0 .2) > limits=[-.1 .1] agent=1 }
joint (arm5 arm6) { A=<T t(0 0 0.2) d(45 0 0 1) > B=<T t(0 0 .2) > limits=[-.1 .1] agent=1 }
joint (arm6 arm7) { A=<T t(0 0 0.2) d(45 0 0 1) > B=<T t(0 0 .2) >  }
#joint (arm6 arm7) { A=<T t(0 0 0.2) d(45 0 0 1) > B=<T t(0 0 .2) >  coupledTo=j4 }

