Include = '../../../../data/baxter_model/baxter.ors'
Delete shape visual
#Delete shape collision

Include = '../../../../data/man_model.ors'


Edit  waist { X=<T t(1.5 0. 1.) d(-90 0 0 1)> }

body tableC{ type=9, X=<T t(.7 0 .8)>, size=[1. .8 .04 .02], color=[.3 .3 .5] fixed, contact }
body tableL{ type=9, X=<T t(.2 .7 .8)>, size=[2. .6 .04 .02], color=[.3 .5 .3] fixed, contact }
body tableR{ type=9, X=<T t(.2 -.7 .8)>, size=[2. .6 .04 .02], color=[.3 .5 .3] fixed, contact }

shape humanR (handR){ type=5 size=[.1 0 0 0] color=[1 1 0] rel=<T t(0 0 -.05) d(90 0 0 1)> }
shape humanL (handL){ type=5 size=[.1 0 0 0] color=[1 1 0] rel=<T t(0 0 -.05) d(90 0 0 1)> }

shape baxterR (right_wrist){ rel=<T d(-90 0 1 0) d(-90 0 0 1) t(0 0 -.26)> type=5 size=[.1 0 0 0] color=[1 1 0] }
shape baxterL (left_wrist) { rel=<T d(-90 0 1 0) d(-90 0 0 1) t(0 0 -.26)> type=5 size=[.1 0 0 0] color=[1 1 0] }

body Handle { type=9 size=[.05 .3 .15 .02] contact }
joint (tableC Handle) { from=<T t(-.2 .2 0) t(0 0 .1)> type=10 }

body Long1 { type=9 size=[.05 .3 .1 .02] contact }
joint (tableC Long1) { from=<T t(.2 .2 0) t(0 0 .1)> type=10 }

body Long2 { type=9 size=[.05 .3 .1 .02] contact }
joint (tableC Long2) { from=<T t(.2 -.2 0) t(0 0 .1)> type=10 }

Include = '../../../../data/toolbox/toolbox.ors'

joint (tableL /toolbox/handle) { from=<T t(0 0 .04) t(.0 .0 .12)> type=10 }
joint (tableL /toolbox/side_front) { from=<T t(0 0 .04) d(-90 0 0 1) t(0 -.212 .12)> type=10 }
joint (tableL /toolbox/side_back) { from=<T t(0 0 .04) d(90 0 0 1) t(0. -.212 .12)> type=10 }
joint (tableL /toolbox/side_left) { from=<T t(0 0 .04) t(.0 -.147 .12)> type=10 }
joint (tableL /toolbox/side_right) { from=<T t(0 0 .04) d(180 0 0 1) t(.0 -.147 .12)> type=10 }
joint (tableL /toolbox/floor_left) { from=<T t(0 0 .04) t(.0 .069 .004) d(90 1 0 0)> type=10 }
joint (tableL /toolbox/floor_right) { from=<T t(0 0 .04) d(180 0 0 1) t(.0 .069 .004) d(90 1 0 0)> type=10 }

Include = '../../../../data/screwdriver/screwdriver.ors'
joint (tableR screwdriver) { from=<T t(0 0 .06) t(-.5 .0 .0) > type=10 }
#joint (tableR screwdriver) { from=<T t(0 0 .06) t(.5 .0 .0) > type=10 }
shape screwdriverHandle(screwdriver) { type=5 rel=<T d(90 0 0 1)> size=[.15 0 0 0] color=[1 1 0] }

body screwbox { type=9, size=[.05 .1 .04 .02], color=[.8 .3 .3] fixed, contact }
joint (tableL screwbox) { from=<T t(.8 0 .08)> type=10 }
