
## torso & arms

waist { X: "t(0 0 .7) d(90 0 1 0)"  shape:capsule mass=1 size=[0. 0. .15 .1] }

back     { shape:capsule mass=1 size=[0. 0. .1 .1] }
chest    { shape:capsule mass=1 size=[0. 0. .2 .1] }
shoulders{ shape:capsule mass=1 size=[0. 0. .2 .1] }
shoulderL{ shape:sphere mass=.1 size=[0. 0. .1 .08]  }
shoulderR{ shape:sphere mass=.1 size=[0. 0. .1 .08]  }
upArmL   { shape:capsule mass=.1 size=[.1 .1 .1 .05]  }
upArmR   { shape:capsule mass=.1 size=[.1 .1 .1 .05]  }
dnArmL   { shape:capsule mass=.1 size=[.1 .1 .1 .05]  }
dnArmR   { shape:capsule mass=.1 size=[.1 .1 .1 .05]  }
upWristL { shape:capsule mass=.1 size=[.1 .1 .1 .045] }
upWristR { shape:capsule mass=.1 size=[.1 .1 .1 .045] }

neck { shape:capsule mass=.1 size=[.0 .0 .1 .05] }
head { shape:sphere mass=.1 size=[0 0 0 .14] }

(waist back) { joint:hingeX A: "d(-90 0 1 0) t(0 0 .05)" B: "t(0 0 .05)"  }
(back chest) { joint:hingeX A: "t(0 0 .05) d(90 0 0 1)" B: "d(-90 0 0 1) t(0 0 .1)"  }
(chest shoulders) { joint:hingeX A: "t(0 0 .1) d(90 0 1 0)" B: "t(-.05 0 0)"  }
(shoulders shoulderL) { joint:hingeX A: "t(-.02 0 .15) d(-90 0 0 1) d(30 1 0 0)" B: "d(90 0 0 1)"  }
(shoulders shoulderR) { joint:hingeX A: "d(180 0 1 0) t(.02 0 .15) d(90 0 0 1) d(30 1 0 0)" B: "d(-90 0 0 1)"  }

(shoulderL upArmL){ joint:hingeX A: "d(90 0 1 0) t(-.02 0 .05)" B: "t(0 0 .05)"  }
(shoulderR upArmR){ joint:hingeX A: "d(-90 0 1 0) t(.02 0 .05)" B: "t(0 0 .05)"  }
(upArmL dnArmL)   { joint:hingeX A: "t(0 0 .05) d(-90 0 1 0) d(30 1 0 0)" B: "d(90 0 1 0) t(0 0 .05)"  }
(upArmR dnArmR)   { joint:hingeX A: "t(0 0 .05) d( 90 0 1 0) d(30 1 0 0)" B: "d(-90 0 1 0) t(0 0 .05)"  }
(dnArmL upWristL) { joint:hingeX A: "t(0 0 .05) d(80 1 0 0)" B: "t(0 0 .05)"  }
(dnArmR upWristR) { joint:hingeX A: "t(0 0 .05) d(80 1 0 0)" B: "t(0 0 .05)"  }

(shoulders neck) { joint:hingeX A: "t(-.05 0 0)" B: "d(-90 0 1 0) t(0 0 .05)"  }
(neck head) { joint:hingeX A: "t(0 0 .05)" B: "t(0 0 .1)"  }


## left & right hand

dnWristR { shape:capsule mass=.01 size=[.1 .1 .1 .04] }
dnWristL { shape:capsule mass=.01 size=[.1 .1 .1 .04] }
ddnWristR{ shape:capsule mass=.01 size=[.5 .5 .04 .03] }
ddnWristL{ shape:capsule mass=.01 size=[.5 .5 .04 .03] }
handR    { shape:box mass=.01 size=[.05 .02 .05 .05] }
handL    { shape:box mass=.01 size=[.05 .02 .05 .05] }

(upWristR dnWristR) { joint:hingeX A: "t(0 0 .05) d( 90 0 1 0) d(140 1 0 0)" B: "d(-90 0 1 0) t(0 0 .05)"  }
(upWristL dnWristL) { joint:hingeX A: "t(0 0 .05) d(-90 0 1 0) d(140 1 0 0)" B: "d( 90 0 1 0) t(0 0 .05)"  }
(dnWristR ddnWristR){ joint:hingeX A: "t(0 0 .05) d( 90 0 0 1)" B: "d(-90 0 0 1) t(0 0 .02)"  }
(dnWristL ddnWristL){ joint:hingeX A: "t(0 0 .05) d(-90 0 0 1)" B: "d( 90 0 0 1) t(0 0 .02)"  }
(ddnWristR handR)   { joint:hingeX A: "t(0 0 .03)" B: "t(0 0 .05)"  }
(ddnWristL handL)   { joint:hingeX A: "t(0 0 .03)" B: "t(0 0 .05)"  }


## legs

lhip { mass=1 size=[.1 .1 .02 .08] shape:capsule }
rhip { mass=1 size=[.1 .1 .02 .08] shape:capsule }
lup  { mass=1 size=[.1 .1 .26 .07] shape:capsule } 
rup  { mass=1 size=[.1 .1 .26 .07] shape:capsule } 
ldn  { mass=1 size=[.1 .1 .3 .06] shape:capsule } 
rdn  { mass=1 size=[.1 .1 .3 .06] shape:capsule } 
lfoot{ mass=1 size=[.1 .3 .05 .05] shape:box } 
rfoot{ mass=1 size=[.1 .3 .05 .05] shape:box }

(waist lhip) { joint:hingeX A: "d(-90 0 1 0) t(-.1 0 -.05) d(90 0 0 1)" B: "d(90 0 0 1) t(0 0 -.01)"  }
(waist rhip) { joint:hingeX A: "d(-90 0 1 0) t(+.1 0 -.05) d(90 0 0 1)" B: "d(90 0 0 1) t(0 0 -.01)"  }
(lhip lup) { joint:hingeX A: "t(0 0 -.01) d(20  1 0 0)" B: "t(0 0 -.13)"  }
(rhip rup) { joint:hingeX A: "t(0 0 -.01) d(20  1 0 0)" B: "t(0 0 -.13)"  }
(lup ldn)  { joint:hingeX A: "t(0 0 -.13) d(-40 1 0 0)" B: "t(0 .01 -.15)"  }
(rup rdn)  { joint:hingeX A: "t(0 0 -.13) d(-40 1 0 0)" B: "t(0 .01 -.15)"  }
(ldn lfoot){ joint:hingeX A: "t(0 0 -.17) d(20  1 0 0)" B: "t(0 .04 -.025)"  }
(rdn rfoot){ joint:hingeX A: "t(0 0 -.17) d(20  1 0 0)" B: "t(0 .04 -.025)"  }


## targets

rightTarget { shape:sphere size=[0 0 0 .03] X: "t(-0.2 -0.4 1.1)" color=[1 0 0] }
leftTarget  { shape:sphere size=[0 0 0 .03] X: "t( 0.2 -0.4 1.1)" color=[1 0 0] }
