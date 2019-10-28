#shape { shape:marker size=[.5 0 0 0] }

body world { }
body transX {}
body transY {}
body transPhi {}

body base { shape:box size=[.15 .02 .6 0] color=[.1 .1 .1] }

joint (world transX) { joint:transX }
joint (transX transY) { joint:transY }
joint (transY transPhi) { joint:hingeZ }
joint (transPhi base) { joint:hingeX A=<T t(0 0 .05) > B=<T t(0 0 .3) >  Q=<T d(10 1 0 0)> }

body wheelL{ shape:cylinder size=[0 0 .02 .05] }
body wheelR{ shape:cylinder size=[0 0 .02 .05] }
joint (base wheelL) { joint:hingeX from=<T t(-.1 0 -.3)> to=<T d(90 0 1 0)> }
joint (base wheelR) { joint:hingeX from=<T t(.1 0 -.3)> to=<T d(90 0 1 0)> }

