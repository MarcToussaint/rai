body world { }
body stem { X=<T t(0 0 1)>  shape:capsule mass=.1 size=[0.1 0.1 2 .1] }
body arm1 { shape:capsule mass=.1 size=[0.1 0.1 .4 .1] }
body arm2 { shape:capsule mass=.1 size=[0.1 0.1 .4 .1] }
body arm3 { shape:capsule mass=.1 size=[0.1 0.1 .4 .1] }

joint (world stem) { joint:transXYPhi }
joint (stem arm1) { joint:hingeX A=<T t(0 0 1) d(90 1 0 0) > B=<T t(0 0 .2) >  }
joint j4 (arm1 arm2) { joint:hingeX A=<T t(0 0 0.2) d(45 0 0 1) > B=<T t(0 0 .2) >  }
joint (arm2 arm3) { joint:hingeX A=<T t(0 0 0.2) d(45 0 0 1) > B=<T t(0 0 .2) >  coupledTo=j4 }

