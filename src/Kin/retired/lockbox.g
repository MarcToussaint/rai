body lockbox {X=<T d(90 0 0 1) t(0 0 1.)> }
#shape back (lockbox) { type=0 size=[.02 .6 .6 0] rel=<T t(0 0 .3) d(90 0 0 1)> fixed }
shape front (lockbox) { type=0 size=[1 0.02 1 0] color=[0 0 0] }
#shape front_bottom (lockbox) { type=0 size=[.02 .3 .1 0] rel=<T t(0 0 .05) d(90 0 0 1) t(-.2 .15 0)> fixed }
#shape front_top (lockbox) { type=0 size=[.02 .3 .1 0] rel=<T t(0 0 .55) d(90 0 0 1) t(-.2 .15 0)> fixed }
#shape bottom (lockbox) { type=0 size=[.6 .18 .02 0] rel=<T t(0 -.1 .01)> fixed }
#shape top (lockbox) { type=0 size=[.6 .18 .02 0] rel=<T t(0 -.1 .59)> fixed }
#shape left (lockbox) { type=0 size=[.02 .22 .6 0] rel=<T t(-.31 -.10 .3)> fixed }
#shape right (lockbox) { type=0 size=[.02 .22 .6 0] rel=<T t(.31 -.10 .3)> fixed }

body wheel {}
shape wheel (wheel) { type=4 size=[0 0 .02 .15] rel=<T d(-90 1 0 0)> color=[1. .5 .5]}
shape wheelcut (wheel) { type=0 size=[.05 .1 .021 0] rel=<T t(0 0 .1) d(-90 1 0 0)> color=[0 0 0]}
joint lockbox_wheel (lockbox, wheel){ A=<T t(.2 -.04 0)> axis=[0 -1 0] }
#shape wheel_handle (wheel) {type=0 size=[0.03 0.02 0.03 0] rel=<T t(.0 -0.02 0)> color=[.7 .7 .7]}


body door { type=0 size=[.02 .3 .4 0] color=[.6 .6 .6] }
joint lockbox_door (lockbox, door){Q=<T t(0 0 .25)> A=<T d(0 90 0 1) t(-.15 -.2 -.15)> limit=[-.5
.5 0 0 0] B=<T d(90 0 0 1) t(0 0 .2)> agent=99}

body bar {}
shape big_bar (bar) { type=0 size=[.15 .02 .06 0] color=[.7 .7 .7]}
shape small_bar (bar) { type=0 size=[.05 .02 .02 0] rel=<T t(.1 0 0)> color=[.7 .7 .7]}
joint lockbox_bar (lockbox, bar){type=3 A=<T t(0.02 -0.22 .15) d(180 0 0 1)>
Q=<T d(90 0 0 1)> limit=[-.5 .5 0 0 0] B=<T d(90 0 0 1)> }


body bolt {}
shape big_bolt (bolt) { type=0 size=[.02 .06 .1 0] color=[.7 .7 .7]}
shape small_bolt (bolt) { type=0 size=[.02 .02 .03 0] rel=<T t(.0 0 -0.065 )> color=[.7 .7 .7]}
joint lockbox_bolt (lockbox, bolt){type=5 A=<T d(90 0 0 1) t(-0.22 -0.23 .29) >
limit=[-.5 .5 0 0 0] B=<T d(0 0 0 1)>  }

body screw { type=0 size=[.02 .15 .02 0] color=[.7 .7 .7]}
shape head (screw) { type=0 size=[.04 .01 .04 0] rel=<T t(0 -.08 0)> color=[.7 .7 .7]}
joint lockbox_screw (lockbox, screw) {type=4 A=<T d(90 0 0 1) t(-.22 -.25 .355)>
}

body pin{}
shape big_pin (pin) { type=0 size=[.02 .06 .1 0] color=[.7 .7 .7]}
shape small_pin (pin) { type=0 size=[.02 .02 .03 0] rel=<T t(.0 0 -0.065 )> color=[.7 .7 .7]}
joint lockbox_pin (lockbox, pin){type=5 A=<T d(90 0 0 1) t(-0.22 -0.23 .43) >
limit=[-.5 .5 0 0 0] B=<T d(0 0 0 1)> }
