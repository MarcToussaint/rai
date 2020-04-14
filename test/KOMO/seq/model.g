Include = '../../../../rai-robotModels/baxter/baxter.g'
#Delete shape visual
#Delete shape collision

Include = '../../../../rai-robotModels/human_simple/human_simple.g'


Edit  waist { X=<T t(1.5 0. 1.) d(-90 0 0 1)> }

body tableC{ shape:ssBox, X=<T t(.7 0 .8)>, size=[1. .8 .04 .02], color=[.3 .3 .5] contact }
body tableL{ shape:ssBox, X=<T t(.2 .7 .8)>, size=[2. .6 .04 .02], color=[.3 .5 .3] contact }
body tableR{ shape:ssBox, X=<T t(.2 -.7 .8)>, size=[2. .6 .04 .02], color=[.3 .5 .3] contact }

shape humanR (handR){ shape:marker size=[.1 0 0 0] color=[1 1 0] rel=<T t(0 0 -.05) d(90 0 0 1)> }
shape humanL (handL){ shape:marker size=[.1 0 0 0] color=[1 1 0] rel=<T t(0 0 -.05) d(90 0 0 1)> }

shape baxterR (right_wrist){ rel=<T d(-90 0 1 0) d(-90 0 0 1) t(0 0 -.26)> shape:marker size=[.1 0 0 0] color=[1 1 0] }
shape baxterL (left_wrist) { rel=<T d(-90 0 1 0) d(-90 0 0 1) t(0 0 -.26)> shape:marker size=[.1 0 0 0] color=[1 1 0] }

body Handle { shape:ssBox size=[.05 .3 .15 .02] contact }
joint (tableC Handle) { from=<T t(-.2 .2 0) t(0 0 .1)> joint:rigid }

body Long1 { shape:ssBox size=[.05 .3 .1 .02] contact }
joint (tableC Long1) { from=<T t(.2 .2 0) t(0 0 .1)> joint:rigid }

body Long2 { shape:ssBox size=[.05 .3 .1 .02] contact }
joint (tableC Long2) { from=<T t(.2 -.2 0) t(0 0 .1)> joint:rigid }

Include = '../../../../rai-robotModels/objects/toolbox/toolbox.ors'

joint (tableL /toolbox/handle) { from=<T t(0 0 .04) t(.0 .0 .12)> joint:rigid }
joint (tableL /toolbox/side_front) { from=<T t(0 0 .04) d(-90 0 0 1) t(0 -.212 .12)> joint:rigid }
joint (tableL /toolbox/side_back) { from=<T t(0 0 .04) d(90 0 0 1) t(0. -.212 .12)> joint:rigid }
joint (tableL /toolbox/side_left) { from=<T t(0 0 .04) t(.0 -.147 .12)> joint:rigid }
joint (tableL /toolbox/side_right) { from=<T t(0 0 .04) d(180 0 0 1) t(.0 -.147 .12)> joint:rigid }
joint (tableL /toolbox/floor_left) { from=<T t(0 0 .04) t(.0 .069 .004) d(90 1 0 0)> joint:rigid }
joint (tableL /toolbox/floor_right) { from=<T t(0 0 .04) d(180 0 0 1) t(.0 .069 .004) d(90 1 0 0)> joint:rigid }

Include = '../../../../rai-robotModels/objects/screwdriver/screwdriver.ors'

joint (tableR screwdriver) { from=<T t(0 0 .06) t(-.5 .0 .0) > joint:rigid }
#joint (tableR screwdriver) { from=<T t(0 0 .06) t(.5 .0 .0) > joint:rigid }
shape screwdriverHandle(screwdriver) { shape:marker rel=<T d(90 0 0 1)> size=[.15 0 0 0] color=[1 1 0] }

body screwbox { shape:ssBox, size=[.05 .1 .04 .02], color=[.8 .3 .3] contact }
joint (tableL screwbox) { from=<T t(.8 0 .08)> joint:rigid }
