base { X: "t(0 0 .2)" shape:box size=[.4 .4 .4] color=[0 0 0]}

stem (j0){ Q: "t(0 0 .1) t(0 0 1)" shape:capsule size=[2 .1] }
j0 (base){ joint:phiTransXY } #phiTransXY=12 #transXYPhi=8

arm1(j1) { shape:capsule Q:"t(0 0 .15)" size=[.3 .1] }
arm2(j2) { shape:capsule Q:"t(0 0 .15)" size=[.3 .1] }
arm3(j3) { shape:capsule Q:"t(0 0 .15)" size=[.3 .1] }
arm4(j4) { shape:capsule Q:"t(0 0 .15)" size=[.3 .1] }
arm5(j5) { shape:capsule Q:"t(0 0 .15)" size=[.3 .1] }

j1(stem) { joint:universal pre:"t(0 0 1) d(90 1 0 0)" } #quatBall
j2(arm1) { joint:hingeX pre:"t(0 0 0.15)" }
j3(arm2) { joint:free pre:"t(0 0 0.15)" } #free
j4(arm3) { joint:hingeX pre: "t(0 0 0.15)" mimic=j2 }
j5(arm4) { joint:hingeX pre:"t(0 0 0.15)" q=.5 }
