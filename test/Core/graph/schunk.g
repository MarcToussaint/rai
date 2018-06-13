
# kinematic graph

###########
## Qlin file
###########

QlinFile='schunk.g.qlin'

###########
## base
###########

body base { X=<T t(0 0 .25)> }

shape world (base) { contact, type=box, size=[.1 .1 .1 .1], color=[1 0 0], rel=<T t(0 0 -.25)> }
#shape checkerBoard (base) { type=box, color=[0 1 0], rel=<T t(

shape platte (base) { contact, type=box, size=[.7 .8 .03 .01], color=[.9 .85 .8] }

shape wheelR  (base) { type=cylinder, size=[.1 .1 .04 .1], color=[.5 .5 .5], rel=<T t(.35 -.3 -.15) d(90 0 1 0)> }
shape wheelL  (base) { type=cylinder, size=[.1 .1 .04 .1], color=[.5 .5 .5], rel=<T t(-.35 -.3 -.15) d(90 0 1 0)> }
shape wheelB1 (base) { type=cylinder, size=[.1 .1 .04 .1], color=[.5 .5 .5], rel=<T t(.0 .37 -.15) d(90 0 1 0)> }


###########
## camera
###########

shape camera(base){ contact, rel=<T t(.40 .10 .95) d(90 1 0 0) d(165 0 1 0) d(-35 1 0 0)>, type=box, size=[.157 .035 .056 .01], color=[1 0 0] }



###########
## schunk arm
###########

body m3 {}
body m4 {}
body m5 {}
body m6 {}
body m7 {}
body m8 {}
body m9 {}

joint (base m3) { A=<T t(0 .25 0) d(25 0 0 1) d(15 1 0 0) t(0 0 .85) d(180 0 0 1) t(0 0 0.120) d(90 0 1 0)> }
joint (m3 m4)   { A=<T d( 90 0 1 0)  t(-0.175 0 0)   d(-105 1 0 0)> }
joint (m4 m5)   { A=<T t(-0.175 0 0) d(-90 0 1 0)    d(  90 1 0 0)> }
joint (m5 m6)   { A=<T d( 90 0 1 0)  t(-0.1515 0 0)  d( -90 1 0 0)> }# Q=<T d(45 1 0 0)> }
joint (m6 m7)   { A=<T t(-0.1515 0 0) d(-90 0 1 0)   d( -90 1 0 0)> }
joint (m7 m8)   { A=<T d(-90 0 1 0)  t(-0.12575 0 0) d( -90 1 0 0)> }# Q=<T d(-120 1 0 0)> }
joint (m8 m9)   { A=<T t(-0.12575 0 0) d( 90 0 1 0)> }

body knuck1 {}
body knuck2 {}
body knuck3 {}
body fing1  {}
body fing2  {}
body fing3  {}
body tip1   {}
body tip2   {}
body tip3   {}

# 107.4 to the root of the hand, then  98 (-10mm ring) to the finger joints
joint (m9 knuck1) { A=<T d(180 1 0 0) t(0 0 0.1074) d(-90 0 0 1) t(-.038105    0   .086) d(-90 0 1 0)>, B=<T d(90 0 1 0)>, Q=<T d(0 1 0 0)> }
joint (m9 knuck2) { A=<T d(180 1 0 0) t(0 0 0.1074) d(-90 0 0 1) t( .0190525  .033 .086) d(-90 0 1 0)>, B=<T d(90 0 1 0)>, Q=<T d( 5 1 0 0)>  }
joint (m9 knuck3) { A=<T d(180 1 0 0) t(0 0 0.1074) d(-90 0 0 1) t( .0190525 -.033 .086) d(-90 0 1 0)>, B=<T d(90 0 1 0)>, Q=<T d(-5 1 0 0)>  }

### THE ORDER IS IMPORTANT -- SHOULD CORRESPOND TO SCHUNK'S CONVENTION FOR ODERING!
joint (knuck3 fing3) { A=<T d(-90 0 0 1)> }
joint (fing3 tip3) { A=<T t(0 0 .0865)> B=<T t(0 0 0.035)> }

joint (knuck1 fing1) { A=<T d( 90 0 0 1)> }
joint (fing1 tip1) { A=<T t(0 0 .0865)> B=<T t(0 0 0.035)> }

joint (knuck2 fing2) { A=<T d(-90 0 0 1)> }
joint (fing2 tip2) { A=<T t(0 0 .0865)> B=<T t(0 0 0.035)> }



### mesh shapes
shape (base){ rel=<T t(0 .25 0) d(25 0 0 1) d(15 1 0 0) t(0 0 .85) d(180 0 0 1)>, contact, type=mesh, mesh='schunk_3d/3385031017_fus_120_x.tri' }
shape (m3){ rel=<T d(90 0 0 1)>, contact, type=mesh, mesh='schunk_3d/schunk_0306925_prl_12010_x.tri' color=[.5 .5 .5]}
shape (m4){ rel=<T d(90 0 0 1)>, contact, type=mesh, mesh='schunk_3d/schunk_0306925_prl_12010_x.tri' color=[.5 .5 .5] }
shape (m5){ rel=<T d(90 0 0 1)>, contact, type=mesh, mesh='schunk_3d/schunk_0306920_prl_10010_x.tri' color=[.5 .5 .5] }
shape (m6){ rel=<T d(90 0 0 1)>, contact, type=mesh, mesh='schunk_3d/schunk_0306920_prl_10010_x.tri' color=[.5 .5 .5] }
shape (m7){ rel=<T d(90 0 0 1)>, contact, type=mesh, mesh='schunk_3d/schunk_0306915_prl_8010_x.tri' color=[.5 .5 .5] }
shape (m8){ rel=<T d(90 0 0 1)>, contact, type=mesh, mesh='schunk_3d/schunk_0306915_prl_8010_x.tri' color=[.5 .5 .5] }
shape (m9){ rel=<T d(90 0 0 1)>, contact, type=mesh, mesh='schunk_3d/schunk_0306910_prl_6010_x.tri' color=[.5 .5 .5] }

shape (m3){ rel=<T t(0 0 .11)    d(180 0 1 0)>, contact, type=mesh, mesh='schunk_3d/3385031117_vbe_1212_x.tri' }
shape (m4){ rel=<T t(-.08 .0 .0) d(-90 0 1 0)>, contact, type=mesh, mesh='schunk_3d/3385038117_vbe_1210_x.tri' }
shape (m5){ rel=<T t(0 0 .1)     d(180 0 1 0)>, contact, type=mesh, mesh='schunk_3d/3385031317_vbe_1010_x.tri' }
shape (m6){ rel=<T t(-.07 .0 .0) d(-90 0 1 0)>, contact, type=mesh, mesh='schunk_3d/3385038417_vbe_1008_x.tri' }
shape (m7){ rel=<T t(0 0 -.08)    d(180 0 0 1)>, contact, type=mesh, mesh='schunk_3d/3385031517_vbe_0808_x.tri' }
shape (m8){ rel=<T t(-.055 .0 .00) d(90 0 1 0) d(180 1 0 0)>, contact, type=mesh, mesh='schunk_3d/3385038717_vbe_0806_x.tri' }
shape (m9){ rel=<T t(.0 .0 -.08)  d(180 0 0 1)>, contact, type=mesh, mesh='schunk_3d/3385031717_vbe_0606_x.tri' }


shape ring   (m9) { rel=<T d(180 1 0 0) t(0 0 0.1034) d(-90 0 0 1)>, contact, type=cylinder, size=[0 0 .008 .04], color=[.1 .1 .1] }

shape wrist  (m9) { rel=<T d(180 1 0 0) t(0 0 0.1525) d(-90 0 0 1)>, contact, type=mesh, mesh='schunk_3d/SDH_Gehaeuse_x.tri' color=[.55 .55 .55] }

#shape (knuck1) { type=mesh, rel=<T t(0 0  -.0175) d(90 1 0 0) d(90 0 1 0)>, mesh='schunk_3d/SDH_Gelenk_Finger1_x.tri' color=[.5 .5 .5] }
#shape (knuck2) { type=mesh, rel=<T t(0 0  -.0175) d(90 1 0 0) d(90 0 1 0)>, mesh='schunk_3d/SDH_Gelenk_Finger1_x.tri' color=[.5 .5 .5] }
#shape (knuck3) { type=mesh, rel=<T t(0 0  -.0175) d(90 1 0 0) d(90 0 1 0)>, mesh='schunk_3d/SDH_Gelenk_Finger1_x.tri' color=[.5 .5 .5] }

shape (fing1)  { contact, type=mesh, rel=<T t(0 0  -.0175) d(90 1 0 0) d(180 0 1 0)>, mesh='schunk_3d/SDH_Mittelteil_Finger1_x.tri' color=[.55 .55 .55] }
shape (fing2)  { contact, type=mesh, rel=<T t(0 0  -.0175) d(90 1 0 0) d(180 0 1 0)>, mesh='schunk_3d/SDH_Mittelteil_Finger1_x.tri' color=[.55 .55 .55] }
shape (fing3)  { contact, type=mesh, rel=<T t(0 0  -.0175) d(90 1 0 0) d(180 0 1 0)>, mesh='schunk_3d/SDH_Mittelteil_Finger1_x.tri' color=[.55 .55 .55] }

shape tip1Shape(tip1)   { contact, type=mesh, rel=<T t(0 0 -.1385) d(90 1 0 0) d(180 0 1 0)>, mesh='schunk_3d/SDH_Kuppe_Finger1_x.tri'  color=[.5 .5 .5]}
shape tip2Shape(tip2)   { contact, type=mesh, rel=<T t(0 0 -.1385) d(90 1 0 0) d(180 0 1 0)>, mesh='schunk_3d/SDH_Kuppe_Finger1_x.tri'  color=[.5 .5 .5]}
shape tip3Shape(tip3)   { contact, type=mesh, rel=<T t(0 0 -.1385) d(90 1 0 0) d(180 0 1 0)>, mesh='schunk_3d/SDH_Kuppe_Finger1_x.tri' color=[.5 .5 .5] }


#basic shapes
#shape (base){ contact, rel=<T t(0 .25 0) d(25 0 0 1) d(15 1 0 0) t(0 0 .85) d(180 0 0 1) t(0 0 .025) >, type=cylinder, size=[0 0 .05 .10] }
#shape (m3){ contact, rel=<T d(90 0 1 0)>, type=cylinder, size=[0 0 .16 .08] }
#shape (m4){ contact, rel=<T d(90 0 1 0)>, type=cylinder, size=[0 0 .25 .08] }
#shape (m5){ contact, rel=<T d(90 0 1 0)>, type=cylinder, size=[0 0 .14 .07] }
#shape (m6){ contact, rel=<T d(90 0 1 0)>, type=cylinder, size=[0 0 .20 .07] }
#shape (m7){ contact, rel=<T d(90 0 1 0)>, type=cylinder, size=[0 0 .12 .06] }
#shape (m8){ contact, rel=<T d(90 0 1 0)>, type=cylinder, size=[0 0 .16 .06] }
#shape (m9){ contact, rel=<T d(90 0 1 0)>, type=cylinder, size=[0 0 .10 .05] }

#laser 'looks' in positive y direction along x-y plane
#shape lasershape (m8){ contact, rel=<T d(90 0 0 1) d(180 0 1 0) t(.0 .12 .08) d(-90 0 0 1)>, type=box, size=[.05 .08 .02 .06] color=[1 0 0]}

#shape wrist  (m9) { contact, rel=<T d(180 1 0 0) t(0 0 0.1074) d(-90 0 0 1)>, type=cylinder, size=[0 0 .03 .05] }
#shape (knuck1) { rel=<T d(90 1 0 0)>, type=cylinder, size=[.03 .02 .06 .02] }
#shape (knuck2) { rel=<T d(90 1 0 0)>, type=cylinder, size=[.03 .02 .06 .02] }
#shape (knuck3) { rel=<T d(90 1 0 0)>, type=cylinder, size=[.03 .02 .06 .02] }
#shape (fing1)  { contact, rel=<T t(0 0 .05)>, type=box, size=[.03 .02 .06 .02] }
#shape (fing2)  { contact, rel=<T t(0 0 .05)>, type=box, size=[.03 .02 .06 .02] }
#shape (fing3)  { contact, rel=<T t(0 0 .05)>, type=box, size=[.03 .02 .06 .02] }
#shape (tip1)   { contact, rel=<T t(0 0 .035)>, type=box, size=[.03 .02 .06 .02] }
#shape (tip2)   { contact, rel=<T t(0 0 .035)>, type=box, size=[.03 .02 .06 .02] }
#shape (tip3)   { contact, rel=<T t(0 0 .035)>, type=box, size=[.03 .02 .06 .02] }


###########
## grasp references
###########

shape tipNormal1(tip1)   { rel=<T t(0 -.015 -.005) d(90 1 0 0)>, type=cylinder, size=[.01 .0 .0 .0] color = [1 0 0] }
shape tipNormal2(tip2)   { rel=<T t(0 -.015 -.005) d(90 1 0 0)>, type=cylinder, size=[.01 .0 .0 .0] color = [1 0 0]}
shape tipNormal3(tip3)   { rel=<T t(0 -.015 -.005) d(90 1 0 0)>, type=cylinder, size=[.01 .0 .0 .0] color = [1 0 0] }

shape fingNormal1(fing1)   { rel=<T t(0 -.014 .055) d(90 1 0 0)>, type=cylinder, size=[.01 .0 .0 .0] color = [1 0 0] }
shape fingNormal2(fing2)   { rel=<T t(0 -.014 .055) d(90 1 0 0)>, type=cylinder, size=[.01 .0 .0 .0] color = [1 0 0]}
shape fingNormal3(fing3)   { rel=<T t(0 -.014 .055) d(90 1 0 0)>, type=cylinder, size=[.01 .0 .0 .0] color = [1 0 0] }

#shape fingNor(fing2)   { rel=<T t(0 -.016 .04) d(90 1 0 0)>, type=sphere, size=[.0 .0 .05 .003] color = [0 0 0] }

shape graspCenter(m9)     { rel=<T t(0 -.006 -.32)>, type=cylinder, size=[.05 .0 .0 .5] color = [0 1 0] }



###########
## poles
###########

shape pole1 (base){ contact, type=box, size=[.05 .05 .60 .05], rel=<T t(.30 -.25 0) d(25 0 0 1) d(-40 1 0 0) t(0 .0 .3)> }
shape pole2 (base){ contact, type=box, size=[.05 .05 .7 .03], rel=<T t(-.30 -.25 0) d(-45 0 0 1) d(-43 1 0 0) t(0 .0 .35)> }

shape back1 (base) { contact, type=box, size=[.05 .05 .85 .05], rel=<T t(0 .25 0) d(25 0 0 1) d(15 1 0 0) t(-.0565  .0565 0) t(0 0 .425)> }
shape back2 (base) { contact, type=box, size=[.05 .05 .85 .05], rel=<T t(0 .25 0) d(25 0 0 1) d(15 1 0 0) t(-.0565 -.0565 0) t(0 0 .425)> }
shape back3 (base) { contact, type=box, size=[.05 .05 .85 .05], rel=<T t(0 .25 0) d(25 0 0 1) d(15 1 0 0) t( .0565 -.0565 0) t(0 0 .425)> }
shape back4 (base) { contact, type=box, size=[.05 .05 .85 .05], rel=<T t(0 .25 0) d(25 0 0 1) d(15 1 0 0) t( .0565  .0565 0) t(0 0 .425)> }


###########
## old objects
###########

body  OBJECTS{ X=<T t(.0 -.8 .69)> }

shape target1(OBJECTS){ contact, type=box, rel=<T t( .25 .2 .055) d(50 0 0 1) d(30 0 1 0)>, size=[.1 .1 .15 .0], color=[.3 .9 .1] }
shape target2(OBJECTS){ contact, type=cylinder, rel=<T t( .25 -.2 .055) d(50 0 0 1) d(30 0 1 0)>, size=[.0 .0 .108 .0375], color=[.3 .9 .1] }

