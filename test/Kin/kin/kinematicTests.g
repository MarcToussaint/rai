body base { X=<T t(0 0 .2)> shape:box size=[.4 .4 .4 0] color=[0 0 0]}

body stem { X=<T t(0 0 1)> shape:capsule size=[0.1 0.1 2 .1] }
joint transRot (base stem){ joint:phiTransXY B=<T t(0 0 .6)> } #phiTransXY=12 #transXYPhi=8

body arm1 { shape:capsule size=[0.1 0.1 .3 .1] }
body arm2 { shape:capsule size=[0.1 0.1 .3 .1] }
body arm3 { shape:capsule size=[0.1 0.1 .3 .1] }
body arm4 { shape:capsule size=[0.1 0.1 .3 .1] }
body arm5 { shape:capsule size=[0.1 0.1 .3 .1] }

joint (stem arm1) { joint:_xya A=<T t(0 0 1) d(90 1 0 0)> B=<T t(0 0 .15)> } #quatBall
joint j2(arm1 arm2) { joint:hingeX A=<T t(0 0 0.15)> B=<T t(0 0 .15)> axis=[0 0 1] }
joint (arm2 arm3) { joint:free A=<T t(0 0 0.15)> B=<T t(0 0 .15)> axis=[1 0 0] } #free
joint (arm3 arm4) { joint:hingeX A=<T t(0 0 0.15)> B=<T t(0 0 .15)> axis=[0 0 -1] mimic=j2 }
joint (arm4 arm5) { joint:hingeX A=<T t(0 0 0.15)> B=<T t(0 0 .15)> axis=[1 0 0] q=.5 }

