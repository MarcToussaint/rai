body base { X=<T t(0 0 .2)> shape:box size=[.4 .4 .4 0] color=[0 0 0]}
shape ref(base) { rel=<T t(0 0 1)> shape:box size=[0.1 0.2 .3 .0] color=[1 .7 .7]}
shape marker(base) { rel=<T t(0 0 1)> shape:marker size=[.25] }

body hand { shape:box size=[0.1 0.2 .3 .0] }
shape endeff(hand) { shape:marker size=[.25] }

joint (base hand) { A=<T t(0 0 1)> joint:quatBall } #quatBall

