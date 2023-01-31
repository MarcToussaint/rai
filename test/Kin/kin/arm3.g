stem { X=[0 0 1.5], shape:capsule, mass:1, size:[1 .05] }

pre1(stem){ Q:[0 0 .5] }
joint1 (pre1) { joint:hingeX, q:1. }
arm1(joint1){ Q:[0 0 .15] shape:capsule, mass:1, size:[.3 .05] }

pre2(joint1){ Q:[0 0 .3] }
joint2 (pre2) { joint:hingeX, q:-.2 }
arm2(joint2) { Q:[0 0 .15]  shape:capsule, mass:1, size:[.3 .05] }

pre3(joint2) { Q:[0 0 .3] }
joint3(pre3) { joint:hingeX q:-.2 }
eff(joint3) { Q:[0 0 .15] shape:capsule, mass:1, size:[.3 .05] }

#target { X:[.0 .2 1.7], shape:sphere, mass:.001, size:[0 0 0 .02], color:[0 0 0] }

