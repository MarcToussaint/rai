body base { X=<T t(0 0 .2)> type=box size=[.4 .4 .4 0] color=[0 0 0] fixed}
shape ref(base) { rel=<T t(0 0 1)> type=box size=[0.1 0.2 .3 .0] color=[1 .7 .7]}
shape marker(base) { rel=<T t(0 0 1)> type=marker size=[.25 0 0 0] }

body hand { type=box size=[0.1 0.2 .3 .0] }
shape endeff(hand) { type=marker size=[.25 0 0 0] }

joint (base hand) { A=<T t(0 0 1)> type=quatBall } #quatBall

