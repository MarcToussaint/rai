body world { fixed }
body stem { X=<T t(0 0 1)>  type=capsule mass=.1 size=[0.1 0.1 2 .1] }
body arm1 { type=capsule mass=.1 size=[0.1 0.1 .4 .1] }
body arm2 { type=capsule mass=.1 size=[0.1 0.1 .4 .1] }
body arm3 { type=capsule mass=.1 size=[0.1 0.1 .4 .1] }

joint (world stem) { type=transXYPhi }
joint (stem arm1) { A=<T t(0 0 1) d(90 1 0 0) > B=<T t(0 0 .2) >  }
joint j4 (arm1 arm2) { A=<T t(0 0 0.2) d(45 0 0 1) > B=<T t(0 0 .2) >  }
joint (arm2 arm3) { A=<T t(0 0 0.2) d(45 0 0 1) > B=<T t(0 0 .2) >  coupledTo=j4 }

