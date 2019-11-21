
## torso & arms

body waist { X=<T t(0 0 .7) d(90 0 1 0)>  shape:capsule mass=1 size=[0. 0. .15 .1] }

body back     { shape:capsule mass=1 size=[0. 0. .1 .1] }
body chest    { shape:capsule mass=1 size=[0. 0. .2 .1] }
body shoulders{ shape:capsule mass=1 size=[0. 0. .2 .1] }
body shoulderL{ shape:sphere mass=.1 size=[0. 0. .1 .08]  }
body shoulderR{ shape:sphere mass=.1 size=[0. 0. .1 .08]  }
body upArmL   { shape:capsule mass=.1 size=[.1 .1 .1 .05]  }
body upArmR   { shape:capsule mass=.1 size=[.1 .1 .1 .05]  }
body dnArmL   { shape:capsule mass=.1 size=[.1 .1 .1 .05]  }
body dnArmR   { shape:capsule mass=.1 size=[.1 .1 .1 .05]  }
body upWristL { shape:capsule mass=.1 size=[.1 .1 .1 .045] }
body upWristR { shape:capsule mass=.1 size=[.1 .1 .1 .045] }

body neck { shape:capsule mass=.1 size=[.0 .0 .1 .05] }
body head { shape:sphere mass=.1 size=[0 0 0 .14] }

joint (waist back) { joint:hingeX A=<T d(-90 0 1 0) t(0 0 .05)> B=<T t(0 0 .05)>  }
joint (back chest) { joint:hingeX A=<T t(0 0 .05) d(90 0 0 1)> B=<T d(-90 0 0 1) t(0 0 .1)>  }
joint (chest shoulders) { joint:hingeX A=<T t(0 0 .1) d(90 0 1 0)> B=<T t(-.05 0 0)>  }
joint (shoulders shoulderL) { joint:hingeX A=<T t(-.02 0 .15) d(-90 0 0 1) d(30 1 0 0)> B=<T d(90 0 0 1)>  }
joint (shoulders shoulderR) { joint:hingeX A=<T d(180 0 1 0) t(.02 0 .15) d(90 0 0 1) d(30 1 0 0)> B=<T d(-90 0 0 1)>  }

joint (shoulderL upArmL){ joint:hingeX A=<T d(90 0 1 0) t(-.02 0 .05)> B=<T t(0 0 .05)>  }
joint (shoulderR upArmR){ joint:hingeX A=<T d(-90 0 1 0) t(.02 0 .05)> B=<T t(0 0 .05)>  }
joint (upArmL dnArmL)   { joint:hingeX A=<T t(0 0 .05) d(-90 0 1 0) d(30 1 0 0)> B=<T d(90 0 1 0) t(0 0 .05)>  }
joint (upArmR dnArmR)   { joint:hingeX A=<T t(0 0 .05) d( 90 0 1 0) d(30 1 0 0)> B=<T d(-90 0 1 0) t(0 0 .05)>  }
joint (dnArmL upWristL) { joint:hingeX A=<T t(0 0 .05) d(80 1 0 0)> B=<T t(0 0 .05)>  }
joint (dnArmR upWristR) { joint:hingeX A=<T t(0 0 .05) d(80 1 0 0)> B=<T t(0 0 .05)>  }

joint (shoulders neck) { joint:hingeX A=<T t(-.05 0 0)> B=<T d(-90 0 1 0) t(0 0 .05)>  }
joint (neck head) { joint:hingeX A=<T t(0 0 .05)> B=<T t(0 0 .1)>  }


## left & right hand

body dnWristR { shape:capsule mass=.01 size=[.1 .1 .1 .04] }
body dnWristL { shape:capsule mass=.01 size=[.1 .1 .1 .04] }
body ddnWristR{ shape:capsule mass=.01 size=[.5 .5 .04 .03] }
body ddnWristL{ shape:capsule mass=.01 size=[.5 .5 .04 .03] }
body handR    { shape:box mass=.01 size=[.05 .02 .05 .05] }
body handL    { shape:box mass=.01 size=[.05 .02 .05 .05] }

joint (upWristR dnWristR) { joint:hingeX A=<T t(0 0 .05) d( 90 0 1 0) d(140 1 0 0)> B=<T d(-90 0 1 0) t(0 0 .05)>  }
joint (upWristL dnWristL) { joint:hingeX A=<T t(0 0 .05) d(-90 0 1 0) d(140 1 0 0)> B=<T d( 90 0 1 0) t(0 0 .05)>  }
joint (dnWristR ddnWristR){ joint:hingeX A=<T t(0 0 .05) d( 90 0 0 1)> B=<T d(-90 0 0 1) t(0 0 .02)>  }
joint (dnWristL ddnWristL){ joint:hingeX A=<T t(0 0 .05) d(-90 0 0 1)> B=<T d( 90 0 0 1) t(0 0 .02)>  }
joint (ddnWristR handR)   { joint:hingeX A=<T t(0 0 .03)> B=<T t(0 0 .05)>  }
joint (ddnWristL handL)   { joint:hingeX A=<T t(0 0 .03)> B=<T t(0 0 .05)>  }


## legs

body lhip { mass=1 size=[.1 .1 .02 .08] shape:capsule }
body rhip { mass=1 size=[.1 .1 .02 .08] shape:capsule }
body lup  { mass=1 size=[.1 .1 .26 .07] shape:capsule } 
body rup  { mass=1 size=[.1 .1 .26 .07] shape:capsule } 
body ldn  { mass=1 size=[.1 .1 .3 .06] shape:capsule } 
body rdn  { mass=1 size=[.1 .1 .3 .06] shape:capsule } 
body lfoot{ mass=1 size=[.1 .3 .05 .05] shape:box } 
body rfoot{ mass=1 size=[.1 .3 .05 .05] shape:box }

joint (waist lhip) { joint:hingeX A=<T d(-90 0 1 0) t(-.1 0 -.05) d(90 0 0 1)> B=<T d(90 0 0 1) t(0 0 -.01)>  }
joint (waist rhip) { joint:hingeX A=<T d(-90 0 1 0) t(+.1 0 -.05) d(90 0 0 1)> B=<T d(90 0 0 1) t(0 0 -.01)>  }
joint (lhip lup) { joint:hingeX A=<T t(0 0 -.01) d(20  1 0 0)> B=<T t(0 0 -.13)>  }
joint (rhip rup) { joint:hingeX A=<T t(0 0 -.01) d(20  1 0 0)> B=<T t(0 0 -.13)>  }
joint (lup ldn)  { joint:hingeX A=<T t(0 0 -.13) d(-40 1 0 0)> B=<T t(0 .01 -.15)>  }
joint (rup rdn)  { joint:hingeX A=<T t(0 0 -.13) d(-40 1 0 0)> B=<T t(0 .01 -.15)>  }
joint (ldn lfoot){ joint:hingeX A=<T t(0 0 -.17) d(20  1 0 0)> B=<T t(0 .04 -.025)>  }
joint (rdn rfoot){ joint:hingeX A=<T t(0 0 -.17) d(20  1 0 0)> B=<T t(0 .04 -.025)>  }


## targets

body rightTarget { shape:sphere size=[0 0 0 .03] X=<T t(-0.2 -0.4 1.1)> color=[1 0 0] }
body leftTarget  { shape:sphere size=[0 0 0 .03] X=<T t( 0.2 -0.4 1.1)> color=[1 0 0] }
