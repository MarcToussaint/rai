world {}

table (world){
    Q:<t(0 0. .6)>
    shape:ssBox, size:[2. 2. .1 .02], color:[.3 .3 .3] }

box0 (table){
    joint:rigid, Q:<t(0 -.3 .1)>
    shape:ssBox, size:[.1 .1 .1 .02], color:[.8 .8 .8] }

box1 (table){
    joint:rigid, Q:<t(0 -.1 .1)>
    shape:ssBox, size:[.4 .2 .04 .02], color:[.8 .8 .8] }

box2 (table){
    joint:rigid, Q:<t(0 .1 .1)>
    shape:ssBox, size:[.1 .1 .1 .02], color:[.8 .8 .8] }

box3 (table){
    joint:rigid, Q:<t(0 .3 .1)>
    shape:ssBox, size:[.1 .1 .1 .02], color:[.8 .8 .8] }

stick (table){
    joint:rigid, Q:<t(.6 .5 .08) d(90 1 0 0)>
    shape:ssBox, size:[.05 .05 .6 .02], color:[.8 .8 .8] }

ball (table) {
    joint:rigid, Q:<t(-.8 .8 .09)>
    shape:sphere, size:[.04], color:[.8 .8 .8], contact }

#Include: '../../../../rai-robotModels/panda/panda.g'
#(table panda_link0){ joint:rigid, Q:<t(-.5 -.5 .05)> }
#gripper (joint7){
#    Q:<d(-90 0 1 0) d(135 0 0 1) t(0 0 -.155) t(0 0 -.05)>
#    shape:marker, size:[.03], color:[.9 .9 .5] }

Prefix: "L_"
Include: '../../../../rai-robotModels/scenarios/simpleArm.g'
(table L_base){ joint:rigid, Q:<t(-.5 -.5 .1)> }
#L_gripper (L_endeff) { shape:marker, size:[.03], color:[.9 .9 .5] }


Prefix: "R_"
Include: '../../../../rai-robotModels/scenarios/simpleArm.g'
(table R_base){ joint:rigid, Q:<t( .5 -.5 .1)> }
#R_gripper (R_endeff) { shape:marker, size:[.03], color:[.9 .9 .5] }

Prefix!
