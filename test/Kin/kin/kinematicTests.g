body base { X=<T t(0 0 .2)> type=box size=[.4 .4 .4 0] color=[0 0 0] fixed}

body stem { X=<T t(0 0 1)> type=capsule size=[0.1 0.1 2 .1] }
joint transRot (base stem){ B=<T t(0 0 .6)> type=phiTransXY } #phiTransXY=12 #transXYPhi=8

body arm1 { type=capsule size=[0.1 0.1 .3 .1] }
body arm2 { type=capsule size=[0.1 0.1 .3 .1] }
body arm3 { type=capsule size=[0.1 0.1 .3 .1] }
body arm4 { type=capsule size=[0.1 0.1 .3 .1] }
body arm5 { type=capsule size=[0.1 0.1 .3 .1] }

joint (stem arm1) { A=<T t(0 0 1) d(90 1 0 0)> B=<T t(0 0 .15)> type=quatBall } #quatBall
joint j2(arm1 arm2) { A=<T t(0 0 0.15)> B=<T t(0 0 .15)> axis=[0 0 1] }
joint (arm2 arm3) { A=<T t(0 0 0.15)> B=<T t(0 0 .15)> axis=[1 0 0] type=free } #free
joint (arm3 arm4) { A=<T t(0 0 0.15)> B=<T t(0 0 .15)> axis=[0 0 -1] mimic=j2 }
joint (arm4 arm5) { A=<T t(0 0 0.15)> B=<T t(0 0 .15)> axis=[1 0 0] q=.5 }

