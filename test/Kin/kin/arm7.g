
body stem { X=<T t(0 0 1) d(90 0 0 1)>  shape:capsule mass=.1 size=[0.1 0.1 2 .1] }

body arm1 { shape:capsule mass=.1 size=[0.1 0.1 .4 .1] }
body arm2 { shape:capsule mass=.1 size=[0.1 0.1 .4 .1] }
body arm3 { shape:capsule mass=.1 size=[0.1 0.1 .4 .1] }
body arm4 { shape:capsule mass=.1 size=[0.1 0.1 .4 .1] }
body arm5 { shape:capsule mass=.1 size=[0.1 0.1 .4 .1] contact }
body arm6 { shape:capsule mass=.1 size=[0.1 0.1 .4 .1] contact }
body arm7 { shape:capsule mass=.1 size=[0.1 0.1 .4 .1] contact }

shape endeff(arm7){ shape:marker rel=<T t(0 0 .3)> size=[.1 0 0 0] }

#body fileshape { X=<T t(-0.25 -3 1.8)>  shape:mesh mass=.1 size=[.3 .3 .3 .1] color=[1 0 0] shapefile='c:/home/3dmodel/benchmark/offs/m494.off' swiftfile='c:/home/3dmodel/benchmark/offs/m494.dcp' contact, }
#body ball  { X=<T t(.0 -1.0 2.1)>  shape:sphere mass=.1 size=[.3 .3 .3 .1] color=[1 0 0] contact }
body ball2 { X=<T d(90 0 0 1) t(.15 -2.5 1.9)>  shape:box mass=.1 size=[.6 .1 .6 .3] color=[1 0 0] contact }
#body ball3 { X=<T t(-.22 -1.5 2.)>  shape:sphere mass=.1 size=[.3 .3 .3 .1] color=[1 0 0] contact }

joint (stem arm1) { joint:hingeX A=<T t(0 0 1) d(90 1 0 0) > B=<T t(0 0 .2) >  limits=[-.1 .1]}

joint (arm1 arm2) { joint:hingeX A=<T t(0 0 0.2) d(45 0 0 1) > B=<T t(0 0 .2) > limits=[-.1 .1] }
joint (arm2 arm3) { joint:hingeX A=<T t(0 0 0.2) d(45 0 0 1) > B=<T t(0 0 .2) > limits=[-.1 .1] }
joint (arm3 arm4) { joint:hingeX A=<T t(0 0 0.2) d(45 0 0 1) > B=<T t(0 0 .2) > limits=[-.1 .1] }
joint j4(arm4 arm5) { joint:hingeX A=<T t(0 0 0.2) d(45 0 0 1) > B=<T t(0 0 .2) > limits=[-.1 .1] }
joint (arm5 arm6) { joint:hingeX A=<T t(0 0 0.2) d(45 0 0 1) > B=<T t(0 0 .2) > limits=[-.1 .1] }
joint (arm6 arm7) { joint:hingeX A=<T t(0 0 0.2) d(45 0 0 1) > B=<T t(0 0 .2) >  }
#joint (arm6 arm7) { joint:hingeX A=<T t(0 0 0.2) d(45 0 0 1) > B=<T t(0 0 .2) >  coupledTo=j4 }

