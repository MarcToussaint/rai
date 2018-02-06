#body slider1a { type=ST_box size=[.2 .02 .02 0] color=[.5 .5 .5] }
#body slider1b { type=ST_box size=[.2 .02 .02 0] color=[.8 .3 .3] }
#joint slider1Joint(slider1a slider1b){ type=JT_transX ctrl_H=.1 }
#shape (slider1b){ rel=<T t(.1 0 0)> type=5 size=[.1 .1 .1] color=[0 1 0] }

body table1{ type=9, X=<T t(.8 0 .7)>, size=[2. 3. .2 .02], color=[.3 .3 .3] fixed, contact, logical={ table } }

### arm


body stem { Q=<T t(0 0 .2)> type=ST_ssBox mass=.5 size=[0.1 0.1 .2 .03] }
body arm1 { type=ST_ssBox mass=1 size=[0.1 0.1 .5 .03] }
body arm2 { type=ST_ssBox mass=1 size=[0.1 0.1 .5 .03] }
body arm3 { type=ST_ssBox mass=1 size=[0.1 0.1 .3 .03] }

joint (table1 stem){ type=JT_rigid }
joint (stem arm1) { type=JT_quatBall A=<T t(0 0 .1)> B=<T d(90 1 0 0) t(0 0 .25) > Q = <T d(-60 1 0 0)> }
joint (arm1 arm2) { A=<T t(0 0 .25) d(0 0 0 1) > B=<T t(0 0 .25) > q=1.}
joint (arm2 arm3) { A=<T t(0 0 .25) d(0 0 0 1) > B=<T t(0 0 .15) > q=1.}

shape endeff(arm3) { type=ST_ssBox rel=<T t(0 0 .2)> size=[.05 .05 .1 .02] color=[1. 1. 0] }

### ball

body redBall { size=[.06 .06 .06 .02] color=[1 0 0] type=ST_ssBox contact, logical={ object } }
joint (table1 redBall) { from=<T t(0 0 .1) t(.1 .7 .03)> type=JT_rigid }

### hook

frame stick (table1){
  shape=ST_ssBox size=[.8 .025 .04 .01] color=[.6 .3 0] contact, logical={ object }
  joint = JT_rigid Q=<T t(0 0 .1) t(.5 -.7 .02) d(90 0 0 1)>
}

shape stickTip (stick) { rel=<T t(.4 .1 0) d(90 0 0 1)> type=ST_ssBox size=[.2 .026 .04 0.01] color=[.6 .3 0], logical={ object, pusher } }

