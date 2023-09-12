
# kinematic graph

###########
## Qlin file
###########

QlinFile='schunk.g.qlin'

###########
## base
###########

base { X: "t(0 0 .25)" }

world (base) { contact, shape:box, size=[.1 .1 .1 .1], color=[1 0 0], rel: "t(0 0 -.25)" }
#checkerBoard (base) { shape:box, color=[0 1 0], rel=<T t(

platte (base) { contact, shape:box, size=[.7 .8 .03 .01], color=[.9 .85 .8] }

wheelR  (base) { shape:cylinder, size=[.1 .1 .04 .1], color=[.5 .5 .5], rel: "t(.35 -.3 -.15) d(90 0 1 0)" }
wheelL  (base) { shape:cylinder, size=[.1 .1 .04 .1], color=[.5 .5 .5], rel: "t(-.35 -.3 -.15) d(90 0 1 0)" }
wheelB1 (base) { shape:cylinder, size=[.1 .1 .04 .1], color=[.5 .5 .5], rel: "t(.0 .37 -.15) d(90 0 1 0)" }


###########
## camera
###########

camera(base){ contact, rel: "t(.40 .10 .95) d(90 1 0 0) d(165 0 1 0) d(-35 1 0 0)", shape:box, size=[.157 .035 .056 .01], color=[1 0 0] }



###########
## schunk arm
###########

m3 {}
m4 {}
m5 {}
m6 {}
m7 {}
m8 {}
m9 {}

(base m3) { joint:hingeX A: "t(0 .25 0) d(25 0 0 1) d(15 1 0 0) t(0 0 .85) d(180 0 0 1) t(0 0 0.120) d(90 0 1 0)" }
(m3 m4)   { joint:hingeX A: "d( 90 0 1 0)  t(-0.175 0 0)   d(-105 1 0 0)" }
(m4 m5)   { joint:hingeX A: "t(-0.175 0 0) d(-90 0 1 0)    d(  90 1 0 0)" }
(m5 m6)   { joint:hingeX A: "d( 90 0 1 0)  t(-0.1515 0 0)  d( -90 1 0 0)" }# Q: "d(45 1 0 0)" }
(m6 m7)   { joint:hingeX A: "t(-0.1515 0 0) d(-90 0 1 0)   d( -90 1 0 0)" }
(m7 m8)   { joint:hingeX A: "d(-90 0 1 0)  t(-0.12575 0 0) d( -90 1 0 0)" }# Q: "d(-120 1 0 0)" }
(m8 m9)   { joint:hingeX A: "t(-0.12575 0 0) d( 90 0 1 0)" }

knuck1 {}
knuck2 {}
knuck3 {}
fing1  {}
fing2  {}
fing3  {}
tip1   {}
tip2   {}
tip3   {}

# 107.4 to the root of the hand, then  98 (-10mm ring) to the finger joints
(m9 knuck1) { joint:hingeX A: "d(180 1 0 0) t(0 0 0.1074) d(-90 0 0 1) t(-.038105    0   .086) d(-90 0 1 0)", B: "d(90 0 1 0)", Q: "d(0 1 0 0)" }
(m9 knuck2) { joint:hingeX A: "d(180 1 0 0) t(0 0 0.1074) d(-90 0 0 1) t( .0190525  .033 .086) d(-90 0 1 0)", B: "d(90 0 1 0)", Q: "d( 5 1 0 0)"  }
(m9 knuck3) { joint:hingeX A: "d(180 1 0 0) t(0 0 0.1074) d(-90 0 0 1) t( .0190525 -.033 .086) d(-90 0 1 0)", B: "d(90 0 1 0)", Q: "d(-5 1 0 0)"  }

### THE ORDER IS IMPORTANT -- SHOULD CORRESPOND TO SCHUNK'S CONVENTION FOR ODERING!
(knuck3 fing3) { joint:hingeX A: "d(-90 0 0 1)" }
(fing3 tip3)   { joint:hingeX A: "t(0 0 .0865)" B: "t(0 0 0.035)" }

(knuck1 fing1) { joint:hingeX A: "d( 90 0 0 1)" }
(fing1 tip1)   { joint:hingeX A: "t(0 0 .0865)" B: "t(0 0 0.035)" }

(knuck2 fing2) { joint:hingeX A: "d(-90 0 0 1)" }
(fing2 tip2)   { joint:hingeX A: "t(0 0 .0865)" B: "t(0 0 0.035)" }



### mesh shapes
(base){ rel: "t(0 .25 0) d(25 0 0 1) d(15 1 0 0) t(0 0 .85) d(180 0 0 1)", contact, shape:mesh, mesh='schunk_3d/3385031017_fus_120_x.tri' }
(m3){ rel: "d(90 0 0 1)", contact, shape:mesh, mesh='schunk_3d/schunk_0306925_prl_12010_x.tri' color=[.5 .5 .5]}
(m4){ rel: "d(90 0 0 1)", contact, shape:mesh, mesh='schunk_3d/schunk_0306925_prl_12010_x.tri' color=[.5 .5 .5] }
(m5){ rel: "d(90 0 0 1)", contact, shape:mesh, mesh='schunk_3d/schunk_0306920_prl_10010_x.tri' color=[.5 .5 .5] }
(m6){ rel: "d(90 0 0 1)", contact, shape:mesh, mesh='schunk_3d/schunk_0306920_prl_10010_x.tri' color=[.5 .5 .5] }
(m7){ rel: "d(90 0 0 1)", contact, shape:mesh, mesh='schunk_3d/schunk_0306915_prl_8010_x.tri' color=[.5 .5 .5] }
(m8){ rel: "d(90 0 0 1)", contact, shape:mesh, mesh='schunk_3d/schunk_0306915_prl_8010_x.tri' color=[.5 .5 .5] }
(m9){ rel: "d(90 0 0 1)", contact, shape:mesh, mesh='schunk_3d/schunk_0306910_prl_6010_x.tri' color=[.5 .5 .5] }

(m3){ rel: "t(0 0 .11)    d(180 0 1 0)", contact, shape:mesh, mesh='schunk_3d/3385031117_vbe_1212_x.tri' }
(m4){ rel: "t(-.08 .0 .0) d(-90 0 1 0)", contact, shape:mesh, mesh='schunk_3d/3385038117_vbe_1210_x.tri' }
(m5){ rel: "t(0 0 .1)     d(180 0 1 0)", contact, shape:mesh, mesh='schunk_3d/3385031317_vbe_1010_x.tri' }
(m6){ rel: "t(-.07 .0 .0) d(-90 0 1 0)", contact, shape:mesh, mesh='schunk_3d/3385038417_vbe_1008_x.tri' }
(m7){ rel: "t(0 0 -.08)    d(180 0 0 1)", contact, shape:mesh, mesh='schunk_3d/3385031517_vbe_0808_x.tri' }
(m8){ rel: "t(-.055 .0 .00) d(90 0 1 0) d(180 1 0 0)", contact, shape:mesh, mesh='schunk_3d/3385038717_vbe_0806_x.tri' }
(m9){ rel: "t(.0 .0 -.08)  d(180 0 0 1)", contact, shape:mesh, mesh='schunk_3d/3385031717_vbe_0606_x.tri' }


ring   (m9) { rel: "d(180 1 0 0) t(0 0 0.1034) d(-90 0 0 1)", contact, shape:cylinder, size=[0 0 .008 .04], color=[.1 .1 .1] }

wrist  (m9) { rel: "d(180 1 0 0) t(0 0 0.1525) d(-90 0 0 1)", contact, shape:mesh, mesh='schunk_3d/SDH_Gehaeuse_x.tri' color=[.55 .55 .55] }

#(knuck1) { shape:mesh, rel: "t(0 0  -.0175) d(90 1 0 0) d(90 0 1 0)", mesh='schunk_3d/SDH_Gelenk_Finger1_x.tri' color=[.5 .5 .5] }
#(knuck2) { shape:mesh, rel: "t(0 0  -.0175) d(90 1 0 0) d(90 0 1 0)", mesh='schunk_3d/SDH_Gelenk_Finger1_x.tri' color=[.5 .5 .5] }
#(knuck3) { shape:mesh, rel: "t(0 0  -.0175) d(90 1 0 0) d(90 0 1 0)", mesh='schunk_3d/SDH_Gelenk_Finger1_x.tri' color=[.5 .5 .5] }

(fing1)  { contact, shape:mesh, rel: "t(0 0  -.0175) d(90 1 0 0) d(180 0 1 0)", mesh='schunk_3d/SDH_Mittelteil_Finger1_x.tri' color=[.55 .55 .55] }
(fing2)  { contact, shape:mesh, rel: "t(0 0  -.0175) d(90 1 0 0) d(180 0 1 0)", mesh='schunk_3d/SDH_Mittelteil_Finger1_x.tri' color=[.55 .55 .55] }
(fing3)  { contact, shape:mesh, rel: "t(0 0  -.0175) d(90 1 0 0) d(180 0 1 0)", mesh='schunk_3d/SDH_Mittelteil_Finger1_x.tri' color=[.55 .55 .55] }

tip1Shape(tip1)   { contact, shape:mesh, rel: "t(0 0 -.1385) d(90 1 0 0) d(180 0 1 0)", mesh='schunk_3d/SDH_Kuppe_Finger1_x.tri'  color=[.5 .5 .5]}
tip2Shape(tip2)   { contact, shape:mesh, rel: "t(0 0 -.1385) d(90 1 0 0) d(180 0 1 0)", mesh='schunk_3d/SDH_Kuppe_Finger1_x.tri'  color=[.5 .5 .5]}
tip3Shape(tip3)   { contact, shape:mesh, rel: "t(0 0 -.1385) d(90 1 0 0) d(180 0 1 0)", mesh='schunk_3d/SDH_Kuppe_Finger1_x.tri' color=[.5 .5 .5] }


#basic shapes
#(base){ contact, rel: "t(0 .25 0) d(25 0 0 1) d(15 1 0 0) t(0 0 .85) d(180 0 0 1) t(0 0 .025) ", shape:cylinder, size=[0 0 .05 .10] }
#(m3){ contact, rel: "d(90 0 1 0)", shape:cylinder, size=[0 0 .16 .08] }
#(m4){ contact, rel: "d(90 0 1 0)", shape:cylinder, size=[0 0 .25 .08] }
#(m5){ contact, rel: "d(90 0 1 0)", shape:cylinder, size=[0 0 .14 .07] }
#(m6){ contact, rel: "d(90 0 1 0)", shape:cylinder, size=[0 0 .20 .07] }
#(m7){ contact, rel: "d(90 0 1 0)", shape:cylinder, size=[0 0 .12 .06] }
#(m8){ contact, rel: "d(90 0 1 0)", shape:cylinder, size=[0 0 .16 .06] }
#(m9){ contact, rel: "d(90 0 1 0)", shape:cylinder, size=[0 0 .10 .05] }

#laser 'looks' in positive y direction along x-y plane
#lasershape (m8){ contact, rel: "d(90 0 0 1) d(180 0 1 0) t(.0 .12 .08) d(-90 0 0 1)", shape:box, size=[.05 .08 .02 .06] color=[1 0 0]}

#wrist  (m9) { contact, rel: "d(180 1 0 0) t(0 0 0.1074) d(-90 0 0 1)", shape:cylinder, size=[0 0 .03 .05] }
#(knuck1) { rel: "d(90 1 0 0)", shape:cylinder, size=[.03 .02 .06 .02] }
#(knuck2) { rel: "d(90 1 0 0)", shape:cylinder, size=[.03 .02 .06 .02] }
#(knuck3) { rel: "d(90 1 0 0)", shape:cylinder, size=[.03 .02 .06 .02] }
#(fing1)  { contact, rel: "t(0 0 .05)", shape:box, size=[.03 .02 .06 .02] }
#(fing2)  { contact, rel: "t(0 0 .05)", shape:box, size=[.03 .02 .06 .02] }
#(fing3)  { contact, rel: "t(0 0 .05)", shape:box, size=[.03 .02 .06 .02] }
#(tip1)   { contact, rel: "t(0 0 .035)", shape:box, size=[.03 .02 .06 .02] }
#(tip2)   { contact, rel: "t(0 0 .035)", shape:box, size=[.03 .02 .06 .02] }
#(tip3)   { contact, rel: "t(0 0 .035)", shape:box, size=[.03 .02 .06 .02] }


###########
## grasp references
###########

tipNormal1(tip1)   { rel: "t(0 -.015 -.005) d(90 1 0 0)", shape:cylinder, size=[.01 .0 .0 .0] color = [1 0 0] }
tipNormal2(tip2)   { rel: "t(0 -.015 -.005) d(90 1 0 0)", shape:cylinder, size=[.01 .0 .0 .0] color = [1 0 0]}
tipNormal3(tip3)   { rel: "t(0 -.015 -.005) d(90 1 0 0)", shape:cylinder, size=[.01 .0 .0 .0] color = [1 0 0] }

fingNormal1(fing1)   { rel: "t(0 -.014 .055) d(90 1 0 0)", shape:cylinder, size=[.01 .0 .0 .0] color = [1 0 0] }
fingNormal2(fing2)   { rel: "t(0 -.014 .055) d(90 1 0 0)", shape:cylinder, size=[.01 .0 .0 .0] color = [1 0 0]}
fingNormal3(fing3)   { rel: "t(0 -.014 .055) d(90 1 0 0)", shape:cylinder, size=[.01 .0 .0 .0] color = [1 0 0] }

#fingNor(fing2)   { rel: "t(0 -.016 .04) d(90 1 0 0)", shape:sphere, size=[.0 .0 .05 .003] color = [0 0 0] }

graspCenter(m9)     { rel: "t(0 -.006 -.32)", shape:cylinder, size=[.05 .0 .0 .5] color = [0 1 0] }



###########
## poles
###########

pole1 (base){ contact, shape:box, size=[.05 .05 .60 .05], rel: "t(.30 -.25 0) d(25 0 0 1) d(-40 1 0 0) t(0 .0 .3)" }
pole2 (base){ contact, shape:box, size=[.05 .05 .7 .03], rel: "t(-.30 -.25 0) d(-45 0 0 1) d(-43 1 0 0) t(0 .0 .35)" }

back1 (base) { contact, shape:box, size=[.05 .05 .85 .05], rel: "t(0 .25 0) d(25 0 0 1) d(15 1 0 0) t(-.0565  .0565 0) t(0 0 .425)" }
back2 (base) { contact, shape:box, size=[.05 .05 .85 .05], rel: "t(0 .25 0) d(25 0 0 1) d(15 1 0 0) t(-.0565 -.0565 0) t(0 0 .425)" }
back3 (base) { contact, shape:box, size=[.05 .05 .85 .05], rel: "t(0 .25 0) d(25 0 0 1) d(15 1 0 0) t( .0565 -.0565 0) t(0 0 .425)" }
back4 (base) { contact, shape:box, size=[.05 .05 .85 .05], rel: "t(0 .25 0) d(25 0 0 1) d(15 1 0 0) t( .0565  .0565 0) t(0 0 .425)" }


###########
## old objects
###########

 OBJECTS{ X: "t(.0 -.8 .69)" }

target1(OBJECTS){ contact, shape:box, rel: "t( .25 .2 .055) d(50 0 0 1) d(30 0 1 0)", size=[.1 .1 .15 .0], color=[.3 .9 .1] }
target2(OBJECTS){ contact, shape:cylinder, rel: "t( .25 -.2 .055) d(50 0 0 1) d(30 0 1 0)", size=[.0 .0 .108 .0375], color=[.3 .9 .1] }

