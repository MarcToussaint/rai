#shape { type=ST_marker size=[.5 0 0 0] }

body world { fixed }
body transX {}
body transY {}
body transPhi {}

body base { type=ST_box size=[.15 .02 .6 0] color=[.1 .1 .1] }

joint (world transX) { type=JT_transX }
joint (transX transY) { type=JT_transY }
joint (transY transPhi) { type=JT_hingeZ }
joint (transPhi base) { A=<T t(0 0 .05) > B=<T t(0 0 .3) >  Q=<T d(10 1 0 0)> }

body wheelL{ type=ST_cylinder size=[0 0 .02 .05] }
body wheelR{ type=ST_cylinder size=[0 0 .02 .05] }
joint (base wheelL) { from=<T t(-.1 0 -.3)> to=<T d(90 0 1 0)> }
joint (base wheelR) { from=<T t(.1 0 -.3)> to=<T d(90 0 1 0)> }

