world {}

table (world){
    Q:<t(0 0. .6)>
    shape:ssBox, size:[2. 2. .1 .02], color:[.3 .3 .3] }

box2 (table){
    joint:rigid, Q:<t(.0 -.6 .08) d(60 0 0 1)>,
    shape:ssBox, size:[.2 .06 .06 .02], color:[.8 .8 .8] }

target2 (table){
    Q:<t(-.2 .0 .08) d(-30 0 0 1)>
    shape:ssBox, size:[.2 .06 .06 .02], color:[.3 .6 .3 .6] }

box (table){
    joint:rigid, Q:<t(.3 0 .1)>
    shape:ssBox, size:[.3 .2 .1 .02], color:[.8 .8 .8] }

target (table){
    Q:<t(.5 .0 .1) d(20 0 0 1)>
    shape:ssBox, size:[.3 .2 .1 .02], color:[.3 .6 .3 .6] }

stick (table){
    joint:rigid, Q:<t(0 -.3 .1) d(90 0 1 0)>
    shape:ssBox, size:[.05 .05 .6 .02], color:[.8 .8 .8] }

ball (table) {
    joint:rigid, Q:<t(0 .5 .09)>
    shape:sphere, size:[.04], color:[.8 .8 .8], contact }

target3 (table) {
    Q:<t(.8 .8 .05)>
    shape:ssBox, size:[.3 .3 .04 .01], color:[.3 .6 .3 .6] }

#Include: '../../../../rai-robotModels/panda/panda.g'
#(table panda_link0){ joint:rigid, Q:<t(-.5 -.5 .05)> }
#gripper (joint7){
#    Q:<d(-90 0 1 0) d(135 0 0 1) t(0 0 -.155) t(0 0 -.05)>
#    shape:marker, size:[.03], color:[.9 .9 .5] }

Include: '../../../../rai-robotModels/scenarios/simpleArm.g'
(table base){ joint:rigid, Q:<t(-.5 -.5 .1) d(-90 0 0 1)> }
gripper (endeff) { shape:marker, size:[.03], color:[.9 .9 .5] }

Include: '../../../../rai-robotModels/scenarios/walker.g'
Edit handA(table){ joint:rigid, Q:<t(-.6 .5 .1) d(90 0 0 1)> }

