
body stem { X=<T t(0 0 1)> shape:capsule size=[0.1 0.1 2 .1] }

body arm1 { shape:capsule size=[0.1 0.1 .4 .1] contact, }
body arm2 { shape:capsule size=[0.1 0.1 .4 .1] contact, }
body arm3 { shape:capsule size=[0.1 0.1 .4 .1] contact, }
body arm4 { shape:capsule size=[0.1 0.1 .4 .1] contact, }
body arm5 { shape:capsule size=[0.1 0.1 .4 .1] contact, }
body arm6 { shape:capsule size=[0.1 0.1 .4 .1] contact, }
body arm7 { shape:capsule size=[0.1 0.1 .4 .1] contact, }

joint (stem arm1) { joint:hingeX A=<T t(0 0 1) d(90 1 0 0)> B=<T t(0 0 .2)>  Q=<T d(1 0 0 0)> limits:[0 0.01] }

joint (arm1 arm2) { joint:hingeX A=<T t(0 0 0.2) d(45 0 0 1)> B=<T t(0 0 .2)>  Q=<T d(1 0 0 0)> }
joint (arm2 arm3) { joint:hingeX A=<T t(0 0 0.2) d(45 0 0 1)> B=<T t(0 0 .2)>  Q=<T d(1 0 0 0)> }
joint (arm3 arm4) { joint:hingeX A=<T t(0 0 0.2) d(45 0 0 1)> B=<T t(0 0 .2)>  Q=<T d(1 0 0 0)> }
joint (arm4 arm5) { joint:hingeX A=<T t(0 0 0.2) d(45 0 0 1)> B=<T t(0 0 .2)>  Q=<T d(1 0 0 0)> }
joint (arm5 arm6) { joint:hingeX A=<T t(0 0 0.2) d(45 0 0 1)> B=<T t(0 0 .2)>  Q=<T d(1 0 0 0)> }
joint (arm6 arm7) { joint:hingeX A=<T t(0 0 0.2) d(45 0 0 1)> B=<T t(0 0 .2)>  Q=<T d(1 0 0 0)> }

#body target { X=<T t(.5 -1. 2.) q(.939 0 0 .34)>  shape:box size=[.3 .1 .2 .05] color=[0 .5 0] }
body target { X=<T t(.5 -1. 2.) d(-20 0 1 0) d(20 0 0 1)>  shape:box size=[.3 .1 .2 .05] color=[0 .5 0] }

shape endeff(arm7){ shape:marker rel=<T t(0 0 .3)> size=[.3 .1 .1 0] } # a marker shape at the tip of arm7
