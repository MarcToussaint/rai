base { X: "t(0 0 .2)" shape:box size=[.4 .4 .4] color=[.2]}
ref(base) { pose: "t(0 0 1)" shape:box size=[0.1 0.2 .3] color=[1 .7 .7]}
marker(base) { pose: "t(0 0 1)" shape:marker size=[.25] }

hand { shape:box size=[0.1 0.2 .3] }
endeff(hand) { shape:marker size=[.25] }

(base hand) { pre: "t(0 0 1)" joint:quatBall } #quatBall

