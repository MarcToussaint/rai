base { X: "t(0 0 .2)" shape:box size=[.4 .4 .4 0] color=[0 0 0]}

stem { X: "t(0 0 1)" shape:capsule size=[0.1 0.1 2 .1] }
transRot (base stem){ joint:phiTransXY B: "t(0 0 .6)" } #phiTransXY=12 #transXYPhi=8

arm1 { shape:capsule size=[0.1 0.1 .3 .1] }
arm2 { shape:capsule size=[0.1 0.1 .3 .1] }
arm3 { shape:capsule size=[0.1 0.1 .3 .1] }
arm4 { shape:capsule size=[0.1 0.1 .3 .1] }
arm5 { shape:capsule size=[0.1 0.1 .3 .1] }

(stem arm1) { joint:_xya A: "t(0 0 1) d(90 1 0 0)" B: "t(0 0 .15)" } #quatBall
j2(arm1 arm2) { joint:hingeX A: "t(0 0 0.15)" B: "t(0 0 .15)" axis=[0 0 1] }
(arm2 arm3) { joint:free A: "t(0 0 0.15)" B: "t(0 0 .15)" axis=[1 0 0] } #free
(arm3 arm4) { joint:hingeX A: "t(0 0 0.15)" B: "t(0 0 .15)" axis=[0 0 -1] mimic=j2 }
(arm4 arm5) { joint:hingeX A: "t(0 0 0.15)" B: "t(0 0 .15)" axis=[1 0 0] q=.5 }

