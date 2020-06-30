stem { X=<t(0 0 .5)>, shape:capsule, mass:1, size:[1 .05] }
arm1 { shape:capsule, mass:1, size:[.3 .05] }
arm2 { shape:capsule, mass:1, size:[.3 .05] }
eff { shape:capsule, mass:1, size:[.3 .05] }

joint1 (stem arm1) { joint:hingeX, A:<t(0 0 .5) d(90 1 0 0)>, Q:<d(-30 1 0 0)>, B:<t(0 0 .15) > }
joint2 (arm1 arm2) { joint:hingeX, A:<t(0 0 .15) d(0 0 0 1)>, Q:<d(-10 1 0 0)>, B:<t(0 0 .15) > }
joint3 (arm2 eff ) { joint:hingeX, A:<t(0 0 .15) d(0 0 0 1)>, Q:<d(-10 1 0 0)>, B:<t(0 0 .15) > }

target { X:<t(.0 .2 1.7)>, shape:sphere, mass:.001, size:[0 0 0 .02], color:[0 0 0] }

