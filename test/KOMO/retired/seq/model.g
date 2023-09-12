Include = '../../../../rai-robotModels/baxter/baxter.g'
#Delete shape visual
#Delete shape collision

Include = '../../../../rai-robotModels/human_simple/human_simple.g'


Edit  waist { X: "t(1.5 0. 1.) d(-90 0 0 1)" }

tableC{ shape:ssBox, X: "t(.7 0 .8)", size=[1. .8 .04 .02], color=[.3 .3 .5] contact }
tableL{ shape:ssBox, X: "t(.2 .7 .8)", size=[2. .6 .04 .02], color=[.3 .5 .3] contact }
tableR{ shape:ssBox, X: "t(.2 -.7 .8)", size=[2. .6 .04 .02], color=[.3 .5 .3] contact }

humanR (handR){ shape:marker size=[.1 0 0 0] color=[1 1 0] rel: "t(0 0 -.05) d(90 0 0 1)" }
humanL (handL){ shape:marker size=[.1 0 0 0] color=[1 1 0] rel: "t(0 0 -.05) d(90 0 0 1)" }

baxterR (right_wrist){ rel: "d(-90 0 1 0) d(-90 0 0 1) t(0 0 -.26)" shape:marker size=[.1 0 0 0] color=[1 1 0] }
baxterL (left_wrist) { rel: "d(-90 0 1 0) d(-90 0 0 1) t(0 0 -.26)" shape:marker size=[.1 0 0 0] color=[1 1 0] }

Handle { shape:ssBox size=[.05 .3 .15 .02] contact }
(tableC Handle) { from: "t(-.2 .2 0) t(0 0 .1)" joint:rigid }

Long1 { shape:ssBox size=[.05 .3 .1 .02] contact }
(tableC Long1) { from: "t(.2 .2 0) t(0 0 .1)" joint:rigid }

Long2 { shape:ssBox size=[.05 .3 .1 .02] contact }
(tableC Long2) { from: "t(.2 -.2 0) t(0 0 .1)" joint:rigid }

Include = '../../../../rai-robotModels/objects/toolbox/toolbox.ors'

(tableL /toolbox/handle) { from: "t(0 0 .04) t(.0 .0 .12)" joint:rigid }
(tableL /toolbox/side_front) { from: "t(0 0 .04) d(-90 0 0 1) t(0 -.212 .12)" joint:rigid }
(tableL /toolbox/side_back) { from: "t(0 0 .04) d(90 0 0 1) t(0. -.212 .12)" joint:rigid }
(tableL /toolbox/side_left) { from: "t(0 0 .04) t(.0 -.147 .12)" joint:rigid }
(tableL /toolbox/side_right) { from: "t(0 0 .04) d(180 0 0 1) t(.0 -.147 .12)" joint:rigid }
(tableL /toolbox/floor_left) { from: "t(0 0 .04) t(.0 .069 .004) d(90 1 0 0)" joint:rigid }
(tableL /toolbox/floor_right) { from: "t(0 0 .04) d(180 0 0 1) t(.0 .069 .004) d(90 1 0 0)" joint:rigid }

Include = '../../../../rai-robotModels/objects/screwdriver/screwdriver.ors'

(tableR screwdriver) { from: "t(0 0 .06) t(-.5 .0 .0) " joint:rigid }
#(tableR screwdriver) { from: "t(0 0 .06) t(.5 .0 .0) " joint:rigid }
screwdriverHandle(screwdriver) { shape:marker rel: "d(90 0 0 1)" size=[.15 0 0 0] color=[1 1 0] }

screwbox { shape:ssBox, size=[.05 .1 .04 .02], color=[.8 .3 .3] contact }
(tableL screwbox) { from: "t(.8 0 .08)" joint:rigid }
