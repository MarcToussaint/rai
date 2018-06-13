
## torso & arms

body waist { X=<T t(0 0 .7) d(90 0 1 0)>  type=capsule mass=1 size=[0. 0. .15 .1] }

body back     { type=capsule mass=1 size=[0. 0. .1 .1] }
body chest    { type=capsule mass=1 size=[0. 0. .2 .1] }
body shoulders{ type=capsule mass=1 size=[0. 0. .2 .1] }
body shoulderL{ type=sphere mass=.1 size=[0. 0. .1 .08]  }
body shoulderR{ type=sphere mass=.1 size=[0. 0. .1 .08]  }
body upArmL   { type=capsule mass=.1 size=[.1 .1 .1 .05]  }
body upArmR   { type=capsule mass=.1 size=[.1 .1 .1 .05]  }
body dnArmL   { type=capsule mass=.1 size=[.1 .1 .1 .05]  }
body dnArmR   { type=capsule mass=.1 size=[.1 .1 .1 .05]  }
body upWristL { type=capsule mass=.1 size=[.1 .1 .1 .045] }
body upWristR { type=capsule mass=.1 size=[.1 .1 .1 .045] }

body neck { type=capsule mass=.1 size=[.0 .0 .1 .05] }
body head { type=sphere mass=.1 size=[0 0 0 .14] }

joint (waist back) { A=<T d(-90 0 1 0) t(0 0 .05)> B=<T t(0 0 .05)>  }
joint (back chest) { A=<T t(0 0 .05) d(90 0 0 1)> B=<T d(-90 0 0 1) t(0 0 .1)>  }
joint (chest shoulders) { A=<T t(0 0 .1) d(90 0 1 0)> B=<T t(-.05 0 0)>  }
joint (shoulders shoulderL) { A=<T t(-.02 0 .15) d(-90 0 0 1) d(30 1 0 0)> B=<T d(90 0 0 1)>  }
joint (shoulders shoulderR) { A=<T d(180 0 1 0) t(.02 0 .15) d(90 0 0 1) d(30 1 0 0)> B=<T d(-90 0 0 1)>  }

joint (shoulderL upArmL){ A=<T d(90 0 1 0) t(-.02 0 .05)> B=<T t(0 0 .05)>  }
joint (shoulderR upArmR){ A=<T d(-90 0 1 0) t(.02 0 .05)> B=<T t(0 0 .05)>  }
joint (upArmL dnArmL)   { A=<T t(0 0 .05) d(-90 0 1 0) d(30 1 0 0)> B=<T d(90 0 1 0) t(0 0 .05)>  }
joint (upArmR dnArmR)   { A=<T t(0 0 .05) d( 90 0 1 0) d(30 1 0 0)> B=<T d(-90 0 1 0) t(0 0 .05)>  }
joint (dnArmL upWristL) { A=<T t(0 0 .05) d(80 1 0 0)> B=<T t(0 0 .05)>  }
joint (dnArmR upWristR) { A=<T t(0 0 .05) d(80 1 0 0)> B=<T t(0 0 .05)>  }

joint (shoulders neck) { A=<T t(-.05 0 0)> B=<T d(-90 0 1 0) t(0 0 .05)>  }
joint (neck head) { A=<T t(0 0 .05)> B=<T t(0 0 .1)>  }


## left & right hand

body dnWristR { type=capsule mass=.01 size=[.1 .1 .1 .04] }
body dnWristL { type=capsule mass=.01 size=[.1 .1 .1 .04] }
body ddnWristR{ type=capsule mass=.01 size=[.5 .5 .04 .03] }
body ddnWristL{ type=capsule mass=.01 size=[.5 .5 .04 .03] }
body handR    { type=box mass=.01 size=[.05 .02 .05 .05] }
body handL    { type=box mass=.01 size=[.05 .02 .05 .05] }

joint (upWristR dnWristR) { A=<T t(0 0 .05) d( 90 0 1 0) d(140 1 0 0)> B=<T d(-90 0 1 0) t(0 0 .05)>  }
joint (upWristL dnWristL) { A=<T t(0 0 .05) d(-90 0 1 0) d(140 1 0 0)> B=<T d( 90 0 1 0) t(0 0 .05)>  }
joint (dnWristR ddnWristR){ A=<T t(0 0 .05) d( 90 0 0 1)> B=<T d(-90 0 0 1) t(0 0 .02)>  }
joint (dnWristL ddnWristL){ A=<T t(0 0 .05) d(-90 0 0 1)> B=<T d( 90 0 0 1) t(0 0 .02)>  }
joint (ddnWristR handR)   { A=<T t(0 0 .03)> B=<T t(0 0 .05)>  }
joint (ddnWristL handL)   { A=<T t(0 0 .03)> B=<T t(0 0 .05)>  }


## legs

body lhip { mass=1 size=[.1 .1 .02 .08] type=capsule }
body rhip { mass=1 size=[.1 .1 .02 .08] type=capsule }
body lup  { mass=1 size=[.1 .1 .26 .07] type=capsule } 
body rup  { mass=1 size=[.1 .1 .26 .07] type=capsule } 
body ldn  { mass=1 size=[.1 .1 .3 .06] type=capsule } 
body rdn  { mass=1 size=[.1 .1 .3 .06] type=capsule } 
body lfoot{ mass=1 size=[.1 .3 .05 .05] type=box } 
body rfoot{ mass=1 size=[.1 .3 .05 .05] type=box fixed }

joint (waist lhip) { A=<T d(-90 0 1 0) t(-.1 0 -.05) d(90 0 0 1)> B=<T d(90 0 0 1) t(0 0 -.01)>  }
joint (waist rhip) { A=<T d(-90 0 1 0) t(+.1 0 -.05) d(90 0 0 1)> B=<T d(90 0 0 1) t(0 0 -.01)>  }
joint (lhip lup) { A=<T t(0 0 -.01) d(20  1 0 0)> B=<T t(0 0 -.13)>  }
joint (rhip rup) { A=<T t(0 0 -.01) d(20  1 0 0)> B=<T t(0 0 -.13)>  }
joint (lup ldn)  { A=<T t(0 0 -.13) d(-40 1 0 0)> B=<T t(0 .01 -.15)>  }
joint (rup rdn)  { A=<T t(0 0 -.13) d(-40 1 0 0)> B=<T t(0 .01 -.15)>  }
joint (ldn lfoot){ A=<T t(0 0 -.17) d(20  1 0 0)> B=<T t(0 .04 -.025)>  }
joint (rdn rfoot){ A=<T t(0 0 -.17) d(20  1 0 0)> B=<T t(0 .04 -.025)>  }


## targets

body rightTarget { type=sphere size=[0 0 0 .03] X=<T t(-0.2 -0.4 1.1)> color=[1 0 0] }
body leftTarget  { type=sphere size=[0 0 0 .03] X=<T t( 0.2 -0.4 1.1)> color=[1 0 0] }
