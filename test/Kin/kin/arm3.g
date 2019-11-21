# kinematic graph

body stem { X=<T t(0 0 .5)> shape:capsule mass=.5 size=[0.1 0.1 1 .05] }
body arm1 { shape:capsule mass=1 size=[0.1 0.5 .3 .05] }
body arm2 { shape:capsule mass=1 size=[0.1 0.1 .3 .05] }
body eff { shape:capsule mass=1 size=[0.1 0.1 .3 .05] }

joint (stem arm1) { joint:hingeX A=<T t(0 0 .5) d(90 1 0 0) > Q=<T d(-30 1 0 0) > B=<T t(0 0 .15) > }
joint (arm1 arm2) { joint:hingeX A=<T t(0 0 .15) d(0 0 0 1) > Q=<T d(-10 1 0 0)> B=<T t(0 0 .15) > }
joint (arm2 eff) { joint:hingeX A=<T t(0 0 .15) d(0 0 0 1) > Q=<T d(-10 1 0 0)> B=<T t(0 0 .15) > }

body target { X=<T t(.0 .2 1.7)>  shape:sphere mass=.001 size=[0 0 0 .02] color=[0 0 0] }

