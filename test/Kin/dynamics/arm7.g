
stem { X: "t(0 0 1)"  shape:capsule mass=.1 size=[2 .1] }

arm1(joint1) { shape:capsule Q:[0 0 .2], mass=.1 size=[.4 .1] }
arm2(joint2) { shape:capsule Q:[0 0 .2], mass=.1 size=[.4 .1] }
arm3(joint3) { shape:capsule Q:[0 0 .2], mass=.1 size=[.4 .1] }
arm4(joint4) { shape:capsule Q:[0 0 .2], mass=.1 size=[.4 .1] }
arm5(joint5) { shape:capsule Q:[0 0 .2], mass=.1 size=[.4 .1] contact }
arm6(joint6) { shape:capsule Q:[0 0 .2], mass=.1 size=[.4 .1] contact }
arm7(joint7) { shape:capsule Q:[0 0 .2], mass=.1 size=[.4 .1] contact }

endeff(arm7){ shape:marker Q: "t(0 0 .3)" size=[.1] }

#fileshape { X: "t(-0.25 -3 1.8)"  shape:mesh mass=.1 size=[.3 .3 .3 .1] color=[1 0 0] shapefile='c:/home/3dmodel/benchmark/offs/m494.off' swiftfile='c:/home/3dmodel/benchmark/offs/m494.dcp' contact, }
#ball  { X: "t(.0 -1.0 2.1)"  shape:sphere mass=.1 size=[.3 .3 .3 .1] color=[1 0 0] contact }
ball2 { X: "t(.15 -2.5 1.9)"  shape:box mass=.1 size=[.6 .1 .6] color=[1 0 0] contact }
#ball3 { X: "t(-.22 -1.5 2.)"  shape:sphere mass=.1 size=[.3 .3 .3 .1] color=[1 0 0] contact }

joint1 (stem) { joint:hingeX A: "t(0 0 1) d(90 1 0 0) "  limits=[-.1 .1]}
joint2 (arm1) { joint:hingeX A: "t(0 0 0.2) d(45 0 0 1) " limits=[-.1 .1] }
joint3 (arm2) { joint:hingeX A: "t(0 0 0.2) d(45 0 0 1) " limits=[-.1 .1] }
joint4 (arm3) { joint:hingeX A: "t(0 0 0.2) d(45 0 0 1) " limits=[-.1 .1] }
joint5 (arm4) { joint:hingeX A: "t(0 0 0.2) d(45 0 0 1) " limits=[-.1 .1] }
joint6 (arm5) { joint:hingeX A: "t(0 0 0.2) d(45 0 0 1) " limits=[-.1 .1] }
joint7 (arm6) { joint:hingeX A: "t(0 0 0.2) d(45 0 0 1) "  }
#joint7 (arm6) { joint:hingeX A: "t(0 0 0.2) d(45 0 0 1) "  mimic: j4 }

