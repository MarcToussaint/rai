frame table1{ shape:ssBox, X:<t(.8 0 .7)>, size:[2. 3. .2 .02], color:[.3 .3 .3] fixed, contact, logical:{ table } }

### arm

frame stem(table1) {
    joint:rigid Q:<t(0 0 .2)>
    shape:ssBox mass:.5 size:[0.1 0.1 .2 .03], contact:-2 }
frame arm1(stem) {
    joint:quatBall A:<t(0 0 .1)>  B:<d(90 1 0 0) t(0 0 .25) > Q:<d(-60 1 0 0)>
    shape:ssBox mass:1 size:[0.1 0.1 .5 .03], contact:-2 }
frame arm2(arm1) {
    joint:hingeX  A:<t(0 0 .25)> B:<t(0 0 .25) > q:1.
    shape:ssBox mass:1 size:[0.1 0.1 .5 .03], contact:-2 }
frame arm3(arm2) {
    joint:hingeX A:<t(0 0 .25)>  B:<t(0 0 .15) > q:1.
    shape:ssBox mass:1 size:[0.1 0.1 .3 .03], contact:-2 }

frame endeff(arm3) {
    shape:ssBox Q:<t(0 0 .2)> size:[.05 .05 .1 .02] color:[1. 1. 0], contact:-2 }

### ball

frame redBall(table1) { Q:<t(0 0 .1) t(.1 .7 .03)> size:[.06 .06 .06 .02] color:[1 0 0] shape:ssBox contact, logical:{ object } }

### hook

frame stick (table1){
  shape:ssBox size:[.8 .025 .04 .01] color:[.6 .3 0] contact, logical:{ object }
  Q:<t(0 0 .1) t(.5 -.7 .02) d(90 0 0 1)>
  joint:rigid
}

frame stickTip (stick) { Q:<t(.4 .1 0) d(90 0 0 1)> type:ssBox size:[.2 .026 .04 0.01] color:[.6 .3 0], logical:{ object, pusher }, contact }

