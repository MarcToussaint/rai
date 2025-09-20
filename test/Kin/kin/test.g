world { }
stem(j0) { X: "t(0 0 1)"  shape:capsule mass=.1 size=[2 .1] }
arm1(j1) { shape:capsule Q:[0 0 .2], mass=.1 size=[.4 .1] }
arm2(j2) { shape:capsule Q:[0 0 .2], mass=.1 size=[.4 .1] }
arm3(j3) { shape:capsule Q:[0 0 .2], mass=.1 size=[.4 .1] }

j0(world) { joint:transXYPhi }
j1(stem) { joint:hingeX A: "t(0 0 1) d(90 1 0 0) " }
j2(arm1) { joint:hingeX A: "t(0 0 0.2) d(45 0 0 1) " }
j3(arm2) { joint:hingeX A: "t(0 0 0.2) d(45 0 0 1) " mimic=j2 }

